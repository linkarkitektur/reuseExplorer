#include "functions/polyscope.hh"
#include <iostream>


namespace polyscope  {

    //// Define a function to display a error message in case the display function is not defined
    template <typename T>
    void display(T, const std::optional<const std::string>){
        static_assert(sizeof(T) == -1, "Display function not implemented for this type.");
    }


    template <>
    void display(tg::aabb3 const& box, std::optional<const std::string> name){

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
        auto cn = polyscope::registerCurveNetwork((name)? name.value() : "AABB",points, edges );
        cn->setRadius(0.00070);

    }


    template <>
    void display( pcl::PointCloud<PointT> const& cloud, std::optional<const std::string> name){


        auto pcd = polyscope::registerPointCloud((name)? name.value() : "Cloud", PolyscopeMap<Field::Points>(cloud));
        pcd->setPointRadius(0.001);

        auto normal_vectors = pcd->addVectorQuantity("Normals", PolyscopeMap<Field::Normal>(cloud));
        normal_vectors->setVectorRadius(0.00050);
        normal_vectors->setVectorLengthScale(0.0050);

        pcd->addColorQuantity("RGB", PolyscopeMap<Field::RGB>(cloud));
        pcd->addColorQuantity("Normal Colors", PolyscopeMap<Field::Normal_color>(cloud));

        pcd->addScalarQuantity("Confidence", PolyscopeMap<Field::Confidence>(cloud));
        pcd->addScalarQuantity("Lables", PolyscopeMap<Field::Lables>(cloud));
        pcd->addScalarQuantity("Sematic Lables", PolyscopeMap<Field::Semantic>(cloud));
        pcd->addScalarQuantity("Instance Lables", PolyscopeMap<Field::Instance>(cloud));
        pcd->addScalarQuantity("Importance", PolyscopeMap<Field::Importance>(cloud));

        pcd->addColorQuantity("Confidence Colors", PolyscopeMap<Field::Confidence_color>(cloud));
        pcd->addColorQuantity("Lables Colors", PolyscopeMap<Field::Lables_color>(cloud));
        pcd->addColorQuantity("Sematic Colors", PolyscopeMap<Field::Semantic_color>(cloud));
        pcd->addColorQuantity("Instance Colors", PolyscopeMap<Field::Instance_color>(cloud));

        pcd->setPointRadiusQuantity("Importance", true);

        pcd->quantities["Lables Colors"]->setEnabled(true);

    }

    template <>
    void display(linkml::Surface_mesh const& mesh, std::optional<const std::string> name ){

        std::vector<std::array<size_t,3>> faces;
        std::vector<std::array<double,  3>> vertecies;

        std::unordered_map<linkml::Surface_mesh::Vertex_index, size_t> index_map;

        size_t i = 0;
        for (auto idx: mesh.vertices()){
            auto p = mesh.point(idx);
            vertecies.push_back({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
            index_map[idx] = i;
            i++;
        }
        for (auto face_index: mesh.faces()){

            CGAL::Vertex_around_face_circulator<linkml::Surface_mesh> vcirc(mesh.halfedge(face_index), mesh), done(vcirc);
            std::vector<linkml::Surface_mesh::Vertex_index> indices;
            do {
                indices.push_back(*vcirc++);
            } while (vcirc != done);

            faces.push_back({index_map[indices[0]], index_map[indices[1]], index_map[indices[2]]});
        }

        std::cout << "Mesh: " << vertecies.size() << " " << faces.size() << std::endl;

        polyscope::registerSurfaceMesh((name)? name.value() : "Mesh", vertecies, faces);
    }

    template <>
    void display( linkml::Surface const& surface, std::optional<const std::string> name){

        auto mesh = surface.GetMesh();
        mesh.collect_garbage();

        std::vector<std::array<size_t,4>> faces;
        std::vector<std::array<double,  3>> vertecies;


        for (auto v: mesh.vertices()){
            auto p = mesh.point(v);
            vertecies.push_back({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
        }

        for (auto f: mesh.faces()){
            std::vector<size_t> face;
            for (auto v: mesh.vertices_around_face(mesh.halfedge(f)))
                face.push_back(v.id());

            faces.push_back({face[0], face[1], face[2], face[3]});
        }

        auto ps_mesh = polyscope::registerSurfaceMesh((name)? name.value() : "Surface", vertecies, faces);
        ps_mesh->setBackFacePolicy(BackFacePolicy::Cull);

        // PMP::triangle_topology(mesh);
        // display<linkml::Surface_mesh const&>(mesh , (name) ? name.value() : "Surface" );
        // auto ps_mesh = polyscope::getSurfaceMesh((name) ? name.value() : "Surface");

        std::vector<double> num_supporting_points = std::vector<double>(mesh.num_faces(), 0.0);
        for (auto f: mesh.faces())
            num_supporting_points[f.id()] = surface.GetPoints(f).size();
        auto quantity = ps_mesh->addFaceScalarQuantity("Number of Supporting Points", num_supporting_points );
        quantity->setEnabled(true);
        quantity->setColorMap("coolwarm");
        quantity->setMapRange({0, 50});

    }


    template <>
    void display( linkml::Dataset const& dataset, std::optional<const std::string> name){


        std::vector<Eigen::Vector4f> path_node;
        std::vector<Eigen::Vector3f> x_axis;
        std::vector<Eigen::Vector3f> y_axis;
        std::vector<Eigen::Vector3f> z_axis;
        std::vector<int> seq;
        std::vector<std::array<size_t, 2>> path_edges;

        path_node.resize(dataset.size());
        x_axis.resize(dataset.size());
        y_axis.resize(dataset.size());
        z_axis.resize(dataset.size());
        seq.resize(dataset.size());
        path_edges.resize(dataset.size()-1);

        auto odometry =  dataset.get_odometry();

        #pragma omp parallel for shared(odometry, path_node, path_edges)
        for (size_t i = 0; i < dataset.size(); i++){


            auto pos = odometry.block<1,3>(i,2);
            auto quat = odometry.block<1,4>(i,5);

            path_node[i] = Eigen::Vector4f(pos(0), pos(1), pos(2), 1);

            tg::pos3 p(pos(0), pos(1), pos(2));
            tg::dquat q(quat(0), quat(1), quat(2), quat(3));
            auto mat = static_cast<tg::dmat3x3>(q);
            // mat.set_col(3, tg::dvec4(p, 1));

            auto x_vec = mat.col(0);
            auto y_vec = mat.col(1);
            auto z_vec = mat.col(2);


            x_axis[i] = Eigen::Vector3f(x_vec.x, x_vec.y, x_vec.z);
            y_axis[i] = Eigen::Vector3f(y_vec.x, y_vec.y, y_vec.z);
            z_axis[i] = Eigen::Vector3f(z_vec.x, z_vec.y, z_vec.z);

            seq[i] = i;

            if(i > 0){
                path_edges[i-1] = {i-1, i};
            }
        }

        auto ps_path = polyscope::registerCurveNetwork("Path", path_node, path_edges);
        ps_path->addNodeScalarQuantity("Sequence", seq);
        
        auto ps_Xaxis = ps_path->addNodeVectorQuantity("XAxis", x_axis);
        auto ps_Yaxis = ps_path->addNodeVectorQuantity("YAxis", y_axis);
        auto ps_Zaxis = ps_path->addNodeVectorQuantity("ZAxis", z_axis);

        ps_Xaxis->setVectorColor({1, 0, 0});
        ps_Yaxis->setVectorColor({0, 1, 0});
        ps_Zaxis->setVectorColor({0, 0, 1});

        ps_Xaxis->setVectorRadius(0.001);
        ps_Yaxis->setVectorRadius(0.001);
        ps_Zaxis->setVectorRadius(0.001);

        ps_Xaxis->setVectorLengthScale(0.02);
        ps_Yaxis->setVectorLengthScale(0.02);
        ps_Zaxis->setVectorLengthScale(0.02);

        ps_Xaxis->setEnabled(true);
        ps_Yaxis->setEnabled(true);
        ps_Zaxis->setEnabled(true);


    }

    static void display(const tg::plane3 & plane, const tg::aabb3 & bbox, std::optional<const std::string> name){
        linkml::Surface_mesh m;
        linkml::crop_plane_with_aabb(m, bbox, plane);
        display<linkml::Surface_mesh const&>(m, (name) ? name.value() : "Plane" );
    }


}
