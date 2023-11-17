#include "linkml.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/algorithm.h>
#include <fstream>
#include <iostream>
#include <list>
#include <vector>

#include <typed-geometry/tg.hh>
#include <typed-geometry/detail/operators.hh>



namespace linkml {


typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef K::FT                                                FT;
typedef K::Point_2                                           Point;
typedef K::Segment_2                                         Segment;
typedef CGAL::Alpha_shape_vertex_base_2<K>                   Vb;
typedef CGAL::Alpha_shape_face_base_2<K>                     Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>          Tds;
typedef CGAL::Delaunay_triangulation_2<K,Tds>                Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;
typedef Alpha_shape_2::Alpha_shape_edges_iterator            Alpha_shape_edges_iterator;



tg::mat3x3 plane_matric(Plane const p)
{
    tg::vec3 x = tg::normalize(tg::cross(p.normal, tg::vec3(1,0,0)));
    tg::vec3 y = tg::normalize(tg::cross(p.normal, x));

    tg::mat3x3 m = tg::mat3x3();

    m.set_col(0, x);
    m.set_col(1, y);
    m.set_col(2, p.normal);

    return m;
}

template <class OutputIterator>
void alpha_edges( const Alpha_shape_2& A, OutputIterator out)
{
    Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin(),
        end = A.alpha_shape_edges_end();
    for( ; it!=end; ++it)
        *out++ = A.segment(*it);
}



// Reads a list of points and returns a list of segments
// corresponding to the Alpha shape.
int aplatest(point_cloud const &cloud, Plane const &plane, std::vector<int> const &indecies)
{

    auto pts = std::vector<tg::pos3>();
    for (auto& i : indecies)
        pts.push_back(project(cloud.pts.at(i), plane));


    std::list<Point> points;
    for (auto & pt : pts)
        points.push_back(Point(pt.x,pt.y));



    Alpha_shape_2 A(points.begin(), points.end(),
                    FT(10000),
                    Alpha_shape_2::GENERAL);
    std::vector<Segment> segments;
    alpha_edges(A, std::back_inserter(segments));
    auto r = *A.find_optimal_alpha(1);

}


}
