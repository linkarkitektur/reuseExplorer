#pragma once

#include <clean-core/vector.hh>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/property_map.h>

#include <typed-geometry/types/pos.hh>

struct Convex_hull_traits_2_alt
{
    typedef tg::pos2                                                        Point_2;
    typedef const std::function<bool(Point_2, Point_2)>                     Less_xy_2;
    typedef const std::function<bool(Point_2, Point_2)>                     Less_yx_2;
    typedef const std::function<bool(Point_2, Point_2, Point_2, Point_2)>   Compare_signed_distance_to_line_2;
    typedef const std::function<bool(Point_2, Point_2, Point_2)>            Less_rotate_ccw_2;
    typedef const std::function<bool(Point_2, Point_2, Point_2)>            Left_turn_2;
    typedef const std::function<bool(Point_2, Point_2)>                     Equal_2;
    typedef const std::function<CGAL::Sign(Point_2, Point_2, Point_2)>      Orientation_2;
    
    Less_xy_2 less_xy_2_object () const {
        // Binary predicate object type comparing Point_2s lexicographically. 
        // Must provide bool operator()(Point_2 p, Point_2 q) where true is returned iff p <xy q. 
        // We have p<xyq, iff px < qx or px = qx and py < qy, where px and py denote x and y coordinate of point p resp. 
        return [](Point_2 const p, Point_2 const  q){return p.x <= q.x and p.y < q.y;};
    }
    Less_yx_2 less_yx_2_object() const {
        return [](Point_2 const  p, Point_2 const  q){return p.y <= q.y and p.x < q.x; };
    };
    Compare_signed_distance_to_line_2 compare_signed_distance_to_line_2_object() const {
        // Predicate object type that must provide bool operator()(Point_2 p, Point_2 q, Point_2 r,Point_2 s), 
        // which compares the signed distance of r and s to the directed line lpq through p and q. 
        // It is used to compute the point right of a line with maximum unsigned distance to the line.

        return [](Point_2 const p, Point_2 const q, Point_2 const r,Point_2 const s){
            auto line = tg::line2(p,tg::normalize(p-q));
            auto r_d = tg::distance(r, line);
            auto s_d = tg::distance(s, line);
            return s_d < r_d;
        };
    }
    Less_rotate_ccw_2 less_rotate_ccw_2_object() const {
        // Predicate object type that must provide bool operator()(Point_2 e, Point_2 p,Point_2 q), 
        // where true is returned iff a tangent at e to the point set {e,p,q} 
        // hits p before q when rotated counterclockwise around e.
        // Ties are broken such that the point with larger distance to e is smaller!

        return [](Point_2 const e, Point_2 const p,Point_2 const q){
            double orientation = (p.x - e.x) * (q.y - e.y) - (p.y - e.y) * (q.x - e.x);
            return orientation < 0;
        };

    }
    Left_turn_2 left_turn_2_object () const {
        // Predicate object type that must provide bool operator()(Point_2 p,Point_2 q,Point_2 r), 
        // which returns true iff r lies to the left of the oriented line through p and q.

        return [](Point_2 const p,Point_2 const q,Point_2 const r){
            double crossProduct = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
            return crossProduct > 0;  // Returns true if it's a left turn
        };
    }
    Equal_2 equal_2_object()const {
        // Binary predicate object type comparing Point_2s.
        // Must provide bool operator()(Point_2 p, Point_2 q) where true is returned iff p==xyq, false otherwise.
        return [](Point_2 const p, Point_2 const q){return p == q;};
    }
    Orientation_2 orientation_2_object() const {
        // Predicate object type that must provide Orientation operator()(Point_2 e, Point_2 p,Point_2 q), 
        // that returns CGAL::LEFT_TURN, if r lies to the left of the oriented line l defined by p and q, 
        // returns CGAL::RIGHT_TURN if r lies to the right of l, and returns CGAL::COLLINEAR if r lies on l.
        return [](Point_2 const e, Point_2 const p,Point_2 const q){
            double crossProduct = (p.x - e.x) * (q.y - e.y) - (p.y - e.y) * (q.x - e.x);

            if (crossProduct > 0) return CGAL::LEFT_TURN;
            if (crossProduct < 0) return CGAL::RIGHT_TURN;
            return CGAL::COLLINEAR;
        };
    }
};


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Convex_hull_traits_adapter_2<K,
          CGAL::Pointer_property_map<Point_2>::type > Convex_hull_traits_2;


namespace linkml {
    
    static std::vector<std::size_t> convex_hull(cc::vector<tg::pos2> points_in){

        auto points = std::vector<Point_2>();
        for (auto& point : points_in)
            points.push_back(Point_2(point.x, point.y)); 


        std::vector<std::size_t> indices(points.size()), out;

        std::iota(indices.begin(), indices.end(),0);


        //   CGAL::ch_graham_andrew( in_start, in_end, out );
        CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                            Convex_hull_traits_2(CGAL::make_property_map(points)));


        return out;
        
    }

    static std::vector<std::size_t> convex_hull(std::vector<tg::pos2> points_in){

        auto points = std::vector<Point_2>();
        for (auto& point : points_in)
            points.push_back(Point_2(point.x, point.y)); 


        std::vector<std::size_t> indices(points.size()), out;

        std::iota(indices.begin(), indices.end(),0);


        //   CGAL::ch_graham_andrew( in_start, in_end, out );
        CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                            Convex_hull_traits_2(CGAL::make_property_map(points)));


        return out;
        
    }


}
