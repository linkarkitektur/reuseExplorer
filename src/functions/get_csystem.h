#pragma once

#include <clean-core/vector.hh>
#include <clean-core/tuple.hh>

#include <typed-geometry/types/mat.hh>
#include <typed-geometry/types/objects/plane.hh>


namespace linkml {

    static cc::tuple<tg::mat3, tg::pos3> get_csystem(cc::vector<tg::pos3> const& points, tg::plane3 const& plane){
            auto center = tg::average(points);
            auto point =  *points.begin();
            auto mat = tg::mat3();
            auto aX = tg::normalize_safe(point-center);
            auto aY = tg::normalize_safe(tg::cross(plane.normal, aX));
            mat.set_col(0, aX);
            mat.set_col(1, aY);
            mat.set_col(2, plane.normal);
            return cc::tuple(mat, center); 
    }
}