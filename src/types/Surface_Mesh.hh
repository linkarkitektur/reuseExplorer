#pragma once
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/number_utils.h>

#include <typed-geometry/types/objects/aabb.hh>

namespace linkml {
    using Kernel =  CGAL::Exact_predicates_inexact_constructions_kernel;
    
    using Point_3 = Kernel::Point_3;
    using Vector_3 = Kernel::Vector_3;
    using Direction_3 = Kernel::Direction_3;
    using Surface_mesh = CGAL::Surface_mesh<Point_3>;
    using FT = Kernel::FT;


    class LinkMesh: public Surface_mesh
    {
    public:

        using Base = Surface_mesh;
        using PointT = Surface_mesh::Point;

        Base mesh = ((Base)*this);


        LinkMesh() : Surface_mesh() {}
        LinkMesh(const Surface_mesh& mesh) : Surface_mesh(mesh) {}
        LinkMesh(Surface_mesh&& mesh) : Surface_mesh(std::move(mesh)) {}
        LinkMesh(const LinkMesh& mesh) : Surface_mesh((Base)mesh) {}
        LinkMesh(LinkMesh&& mesh) : Surface_mesh(std::move((Base)mesh)) {}


        double volume() const;
        double area() const;
        tg::aabb3 get_bbox() const;
        std::vector<tg::pos3> get_vertices() const;
        std::vector<int> get_faces() const;
        std::vector<int> get_colors() const;
        std::vector<float> get_textrueCoords() const;
    };
}