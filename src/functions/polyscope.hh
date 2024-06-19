#pragma once
#include <types/PointCloud.hh>
#include <types/CellComplex.hh>
#include <types/Surface_Mesh.hh>
#include <typed-geometry/types/objects/aabb.hh>
#include <functions/crop_plane_with_aabb.hh>
// #include <typed-geometry/types/objects/aabb.hh>
// #include <typed-geometry/types/pos.hh>

#include <functions/polyscope_helpers.hh>
#include <functions/constuct_adjacency.hh>
#include <functions/color.hh>
#include <algorithms/surface_reconstruction.hh>

#include <valarray>

#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>
#include <polyscope/surface_mesh.h>

static bool polyscope_enabled = false;


namespace polyscope  {


    static void myinit(){

        if (polyscope_enabled) return;

        polyscope::init();
        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
        // polyscope::view::setUpDir(polyscope::UpDir::ZUp);
        polyscope::view::setUpDir(polyscope::UpDir::YUp);

        polyscope_enabled = true;


    }

    static void myshow(){
        polyscope::show();
    }


    //// Define a function to display a error message in case the display function is not defined
    template <typename T>
    static void display(T, const std::string &){
        static_assert(sizeof(T) == -1, "Display function not implemented for this type.");
    }


    template <>
    static void display(linkml::CellComplex const& cw, std::string const& name){

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


    template <>
    static void display(tg::aabb3 const& box, const std::string & name){

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

    template <>
    static void display(linkml::Adjacency const& adj, const  std::string & name ){

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

    enum class Field { 
        Points,
        RGB,
        Normal,
        Normal_color,
        Confidence,
        Confidence_color,
        Lables,
        Lables_color,
        Semantic,
        Semantic_color,
        Instance,
        Instance_color,
        Importance
    };
    
    template <Field F, typename Enable = void>
    struct FieldType;

    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Points ||
            F == Field::RGB ||
            F == Field::Normal ||
            F == Field::Normal_color
        >::type
    > {
        using type = std::array<float, 3>;
    };

    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Lables ||
            F == Field::Semantic ||
            F == Field::Instance
        >::type
    > {
        using type = int;
    };
    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Confidence ||
            F == Field::Importance
        >::type> {
        using type = float;
    };

    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Confidence_color ||
            F == Field::Lables_color ||
            F == Field::Semantic_color || 
            F == Field::Instance_color
        >::type> {
        using type = tg::color3;
    }; 

    template <Field F>
    class PolyscopeMap{
        private:
            pcl::PointCloud<PointT> const& cloud;
        public:
            PolyscopeMap(pcl::PointCloud<PointT> const& cloud) : cloud(cloud) {}

            size_t size() const { return cloud.points.size(); }
            typename FieldType<F>::type operator[](size_t idx) const {

                if constexpr (F == Field::Points) {
                    return std::array<float, 3>{cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z};
                } else if constexpr (F == Field::RGB){
                    return std::array<float, 3>{
                        static_cast<float>(cloud.points[idx].r)/256,
                        static_cast<float>(cloud.points[idx].g)/256,
                        static_cast<float>(cloud.points[idx].b)/256};
                } else if constexpr (F == Field::Normal){
                    return std::array<float, 3>{
                        cloud.points[idx].normal_x,
                        cloud.points[idx].normal_y,
                        cloud.points[idx].normal_z};
                } else if constexpr (F == Field::Normal_color){
                    return std::array<float, 3>{
                        static_cast<float>(cloud.points[idx].normal_x + 1.0f)/2.0f,
                        static_cast<float>(cloud.points[idx].normal_y + 1.0f)/2.0f,
                        static_cast<float>(cloud.points[idx].normal_z + 1.0f)/2.0f};
                } else if constexpr (F == Field::Confidence){
                    return cloud.points[idx].confidence;
                } else if constexpr (F == Field::Confidence_color){
                    switch (cloud.points[idx].confidence)
                    {
                    case 0:
                        return tg::color3(1.0, 0.0, 0.0);
                        break;
                    case 1:
                        return tg::color3(1.0, 1.0, 0.0);
                        break;
                    case 2:
                        return tg::color3(0.0, 1.0, 0.0);
                        break;
                    default:
                        break;
                    }
                } else if constexpr (F == Field::Lables){
                    return cloud.points[idx].label;
                } else if constexpr (F == Field::Lables_color){
                    return linkml::get_color_forom_angle(linkml::sample_circle(cloud.points[idx].label));
                } else if constexpr (F == Field::Semantic){
                    return cloud.points[idx].semantic;
                } else if constexpr (F == Field::Semantic_color){
                    return linkml::get_color_forom_angle(linkml::sample_circle(cloud.points[idx].semantic));
                } else if constexpr (F == Field::Instance){
                    return static_cast<int>(cloud.points[idx].instance);
                } else if constexpr (F == Field::Instance_color){
                    return linkml::get_color_forom_angle(linkml::sample_circle(cloud.points[idx].instance));
                } else if constexpr (F == Field::Importance){
                    return (cloud.points[idx].confidence+0.01f)
                                *
                                ( (cloud.points[idx].label == 0 ? 0.1f : 1.0f)
                                + (cloud.points[idx].semantic == 0 ? 0.1f : 2.0f)
                                + (cloud.points[idx].instance == 0 ? 0.1f : 3.0f));
            }
        }
    };


    template <>
    static void display( pcl::PointCloud<PointT> const& cloud, const std::string & name){


        auto pcd = polyscope::registerPointCloud(name, PolyscopeMap<Field::Points>(cloud));
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
    static void display(Surface_mesh const& mesh, const std::string & name ){

        std::vector<std::array<size_t,3>> faces;
        std::vector<std::array<double,  3>> vertecies;

        std::unordered_map<Surface_mesh::Vertex_index, size_t> index_map;

        size_t i = 0;
        for (auto & idx: mesh.vertices()){
            auto p = mesh.point(idx);
            vertecies.push_back({p.x(), p.y(), p.z()});
            index_map[idx] = i;
            i++;
        }
        for (auto & face_index: mesh.faces()){

            CGAL::Vertex_around_face_circulator<Surface_mesh> vcirc(mesh.halfedge(face_index), mesh), done(vcirc);
            std::vector<Surface_mesh::Vertex_index> indices;
            do {
                indices.push_back(*vcirc++);
            } while (vcirc != done);

            faces.push_back({index_map[indices[0]], index_map[indices[1]], index_map[indices[2]]});
        }

        polyscope::registerSurfaceMesh(name, vertecies, faces);
    }


    static void display(const tg::plane3 & plane, const tg::aabb3 & bbox, const std::string & name = "Plane"){
        Surface_mesh m;
        linkml::crop_plane_with_aabb(m, bbox, plane);
        display<Surface_mesh const&>(m, name);
    }


}
