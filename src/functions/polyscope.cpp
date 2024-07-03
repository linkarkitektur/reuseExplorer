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

        polyscope::registerSurfaceMesh((name)? name.value() : "Mesh", vertecies, faces);
    }


    static void display(const tg::plane3 & plane, const tg::aabb3 & bbox, std::optional<const std::string> name){
        linkml::Surface_mesh m;
        linkml::crop_plane_with_aabb(m, bbox, plane);
        display<linkml::Surface_mesh const&>(m, (name) ? name.value() : "Plane" );
    }


}
