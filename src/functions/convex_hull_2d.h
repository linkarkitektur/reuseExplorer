#pragma once

#include <clean-core/vector.hh>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/property_map.h>

#include <typed-geometry/types/pos.hh>



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

}
