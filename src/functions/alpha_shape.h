#pragma once
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/Triangulation_hierarchy_2.h>
#include <CGAL/point_generators_2.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <list>
#include <vector>

#include <clean-core/vector.hh>
#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/triangle.hh>
// #include <typed-geometry/types/objects/segment.hh>


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef K::FT                                               FT;
typedef K::Point_2                                          Point;
typedef K::Segment_2                                        Segment;
typedef CGAL::Alpha_shape_vertex_base_2<K>                  Vb;
typedef CGAL::Alpha_shape_face_base_2<K>                    Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>         Tds;
typedef CGAL::Delaunay_triangulation_2<K,Tds>               Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                Alpha_shape_2;
typedef Alpha_shape_2::Alpha_shape_vertices_iterator        Alpha_shape_vertices_iterator;
typedef Alpha_shape_2::Finite_faces_iterator                Finite_faces_iterator;

typedef CGAL::Triangulation_vertex_base_2<K>                Vbb;
typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>    THVb;
typedef CGAL::Triangulation_face_base_2<K>                  THFb;
typedef CGAL::Triangulation_data_structure_2<THVb,THFb>     THTds;
typedef CGAL::Delaunay_triangulation_2<K,THTds>             Dt;
typedef CGAL::Triangulation_hierarchy_2<Dt>                 Triangulation;
typedef Triangulation::Point                                PointT;
typedef Triangulation::Face_handle                          Face_handle;
// typedef CGAL::Creator_uniform_2<double,Point>            Creator;

template <class OutputIterator>
void alpha_vertecies( const Alpha_shape_2& A, OutputIterator out)
{
  Alpha_shape_vertices_iterator it = A.alpha_shape_vertices_begin(),
                                end = A.alpha_shape_vertices_end();

    for( ; it!=end; ++it){
        *out++ = A.point(*it);

        // auto pt = A.point(*it);
        // *out++ = tg::pos2(pt.x(), pt.y());
    }
}


namespace linkml {

    // Reads a list of points and returns a list of segments
    // corresponding to the Alpha shape.
    std::vector<tg::triangle2> alpha_shape(cc::vector<tg::pos2> points_in)
    {
        std::list<Point> points;
        for (auto & p : points_in)
            points.emplace_back(Point(p.x, p.y));

        Alpha_shape_2 A(points.begin(), points.end(),
                        FT(10000),
                        Alpha_shape_2::GENERAL);

        std::vector<Point> points_sel;
        alpha_vertecies(A, std::back_inserter(points_sel));

        std::vector<PointT> points_t;
        std::transform(points_sel.begin(), points_sel.end(), std::back_inserter(points_t), [](Point p){
            return PointT(p.x(), p.y());
        });

        Triangulation t;
        std::copy(points_t.begin(), points_t.end(), std::back_inserter(t));
        // assert(t.is_valid(true));

        auto traingles = std::vector<tg::triangle2>();
        for (auto face_it = t.finite_faces_begin(); face_it != t.finite_faces_end(); ++face_it) {
            // Access vertices of the current face
            Triangulation::Vertex_handle v0 = face_it->vertex(0);
            Triangulation::Vertex_handle v1 = face_it->vertex(1);
            Triangulation::Vertex_handle v2 = face_it->vertex(2);

            // Now, you can access the coordinates of the vertices
            PointT point0 = v0->point();
            PointT point1 = v1->point();
            PointT point2 = v2->point();

            // Create a triangle
            traingles.emplace_back(
                tg::triangle2(
                    tg::pos2(point0.x(), point0.y()),
                    tg::pos2(point1.x(), point1.y()),
                    tg::pos2(point2.x(), point2.y())));
        }

        return traingles;
    }
}




