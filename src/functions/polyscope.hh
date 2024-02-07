#pragma once
#include <types/point_cloud.hh>
#include <types/CellComplex.hh>
// #include <typed-geometry/types/objects/aabb.hh>
// #include <typed-geometry/types/pos.hh>

#include <functions/polyscope_helpers.hh>
#include <functions/constuct_adjacency.hh>
#include <functions/color.hh>

#include <valarray>

#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>
#include <polyscope/surface_mesh.h>


namespace polyscope  {


    static void myinit(){
        polyscope::init();

        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
        polyscope::view::setUpDir(polyscope::UpDir::ZUp);

    }

    static void display(linkml::point_cloud const & cloud, std::string name = "Cloud"){
        auto pcd = polyscope::registerPointCloud(name, cloud.pts);
        pcd->setPointRadius(0.001);

        if (cloud.colors.size() == cloud.pts.size()){
            auto pcd_color =  pcd->addColorQuantity("RGB", cloud.colors);
            pcd_color->setEnabled(true);
        }

    }
    static void display(linkml::CellComplex & cw){


        auto ps_mesh = polyscope::registerSurfaceMesh("Mesh", ps_helpers::vertecies(cw, cw.pos), ps_helpers::faces(cw));
        auto face_color = ps_mesh->addFaceColorQuantity("Face Color", cw.faces().map([&](pm::face_handle h){ 
            auto c = cw.plane_colors[h]; 
            return cc::array<float,3>{c.r,c.g,c.b};
        }).to_vector());
        auto facet_color = ps_mesh->addFaceColorQuantity("Facets Color", cw.faces().map([&](pm::face_handle h){ 
            auto c = cw.facet_colors[h]; 
            return cc::array<float,3>{c.r,c.g,c.b};
        }).to_vector());
        facet_color->setEnabled(true);
        ps_mesh->addFaceScalarQuantity("Coverage", cw.faces().map([&](pm::face_handle h){
            return cw.coverage[h];
        }).to_vector());
    }
    static void display(tg::aabb3 const & box ){

        //  Drawing of cube with numbered vertecies.
        // 
        //         (6) +----------------+(7)
        //            /|       6       /|
        //         7 / |            5 / |
        //          /  |             /  |
        //         /   |    4       /   |
        //    (4) +----------------+ (5)| 10
        //        |    | 11        |    |
        //        |    |           |    |
        //        |    |      2    |    |
        //        | (2)+-----------|----+(3)
        //     8  |   /          9 |   /
        //        |  / 3           |  / 1
        //        | /              | /
        //        |/        0      |/
        //        +----------------+
        //    (0)                    (1)

        auto const p0 = tg::pos3(box.min.x,box.min.y, box.min.z); // 0
        auto const p1 = tg::pos3(box.max.x,box.min.y, box.min.z); // 1
        auto const p2 = tg::pos3(box.min.x,box.max.y, box.min.z); // 2
        auto const p3 = tg::pos3(box.max.x,box.max.y, box.min.z); // 3
        auto const p4 = tg::pos3(box.min.x,box.min.y, box.max.z); // 4
        auto const p5 = tg::pos3(box.max.x,box.min.y, box.max.z); // 5
        auto const p6 = tg::pos3(box.min.x,box.max.y, box.max.z); // 6
        auto const p7 = tg::pos3(box.max.x,box.max.y, box.max.z); // 7

        std::vector<tg::pos3> const points = { p0, p1, p2, p3, p4, p5, p6, p7};
        std::vector<std::vector<int>> const edges = { {0,1},{1,3},{3,2},{2,0}, {4,5},{5,7},{7,6},{6,4}, {0,4},{1,5},{3,7},{2,6}};
        auto cn = polyscope::registerCurveNetwork("BBOX",points, edges );
        cn->setRadius(0.00070);

    }
    static void display(linkml::Adjacency adj){

        // auto points = std::vector<tg::pos3>();
        // auto edges = std::vector<std::array<int, 2>>();

        std::vector<tg::pos3> points{};
        std::vector<std::array<int, 2>> edges{};
        std::vector<tg::color3> colors{};


        int i = 0;
        for (auto &super_edge : adj){

            points.push_back(super_edge.s);
            points.push_back(super_edge.t);

            colors.push_back(linkml::get_color_forom_angle(linkml::sample_circle(i)));

            edges.push_back({i,i+1});
            i+=2;
        }

        auto cn = polyscope::registerCurveNetwork("Adjacency", points, edges);
        cn->setRadius(0.00050);
        cn->addEdgeColorQuantity("Identity", colors)->setEnabled(true);

    }
    static void display(linkml::PointCloud & cloud, std::string const name = "Cloud"){

        auto  points = std::vector<std::array<float, 3>>();
        points.resize(cloud.points.size());

        auto colors = std::vector<std::array<float, 3>>();
        colors.resize(cloud.points.size());

        auto confideces = std::vector<int>();
        confideces.resize(cloud.points.size());

        auto lables = std::vector<int>();
        lables.resize(cloud.points.size());

        auto sematic_lables = std::vector<int>();
        sematic_lables.resize(cloud.points.size());

        auto sematic_colors = std::vector<tg::color3>();
        sematic_colors.resize(cloud.points.size());

        auto instance_lables = std::vector<int>();
        instance_lables.resize(cloud.points.size());
        
        auto normals = std::vector<std::array<float, 3>>();
        normals.resize(cloud.points.size());

        auto normal_colors = std::vector<std::array<float, 3>>();
        normal_colors.resize(cloud.points.size());

        #pragma omp parallel for
        for (size_t i = 0; i < cloud.points.size(); i++){

            points[i] = {cloud.points[i].x, cloud.points[i].y, cloud.points[i].z};

            colors[i] = {
                static_cast<float>(cloud.points[i].r)/256,
                static_cast<float>(cloud.points[i].g)/256,
                static_cast<float>(cloud.points[i].b)/256};

            normals[i] = {
                cloud.points[i].normal_x,
                cloud.points[i].normal_y,
                cloud.points[i].normal_z};

            // remap normals to 0-1
            normal_colors[i] = {
                (cloud.points[i].normal_x + 1.0f)/2.0f,
                (cloud.points[i].normal_y + 1.0f)/2.0f,
                (cloud.points[i].normal_z + 1.0f)/2.0f};

            confideces[i] = cloud.points[i].confidence;
            lables[i] = cloud.points[i].label;
            sematic_lables[i] = cloud.points[i].semantic;
            sematic_colors[i] = linkml::get_color_forom_angle(linkml::sample_circle(cloud.points[i].semantic));
            instance_lables[i] = cloud.points[i].instance;
        }

        auto pcd = polyscope::registerPointCloud(name, points);
        pcd->setPointRadius(0.001);
        auto pcd_color =  pcd->addColorQuantity("RGB", colors);
        // pcd_color->setEnabled(true);

        pcd->addColorQuantity("Normal Colors", normal_colors);
        pcd->addVectorQuantity("Normals", normals);
        pcd->addScalarQuantity("Confidence", confideces);
        pcd->addScalarQuantity("Lables", lables);
        pcd->addScalarQuantity("Sematic Lables", sematic_lables);
        auto pcd_sematic = pcd->addColorQuantity("Sematic Colors", sematic_colors);
        pcd_sematic->setEnabled(true);

        pcd->addScalarQuantity("Instance Lables", instance_lables);
    }
}
