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

template <class OutputIterator>
static void alpha_vertecies( const Alpha_shape_2& A, OutputIterator out)
{
  Alpha_shape_vertices_iterator it = A.alpha_shape_vertices_begin(),
                                end = A.alpha_shape_vertices_end();

    for( ; it!=end; ++it){
        *out++ = A.point(*it);

        // auto pt = A.point(*it);
        // *out++ = tg::pos2(pt.x(), pt.y());
    }
}

template <class OutputIterator>
static void alpha_indecies( const Alpha_shape_2& A, OutputIterator out, std::map<Point, std::size_t> const & pointIndexMap) {
  Alpha_shape_vertices_iterator it = A.alpha_shape_vertices_begin(),
                                end = A.alpha_shape_vertices_end();
    for( ; it!=end; ++it){
        *out++ = pointIndexMap.at(A.point(*it));
    }
}

namespace linkml {

    // Reads a list of points and returns a list of segments
    // corresponding to the Alpha shape.
    static std::vector<size_t> alpha_shape(std::vector<tg::pos2> points_in)
    {
        std::vector<Point> points;
        for (auto & p : points_in)
            points.emplace_back(Point(p.x, p.y));

        // Maintain a mapping between points and their indices
        std::map<Point, std::size_t> pointIndexMap;
        for (std::size_t i = 0; i < points.size(); ++i)
            pointIndexMap[points[i]] = i;

        Alpha_shape_2 A(points.begin(), points.end(),
                        FT(10000),
                        Alpha_shape_2::GENERAL);

        std::vector<size_t> indices;
        alpha_indecies(A, std::back_inserter(indices), pointIndexMap);


        return indices;
    }
}




