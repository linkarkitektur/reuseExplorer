#pragma once
#include <vector>

#include <typed-geometry/tg-lean.hh>

#include <types/plane.hh>
#include <types/point_cloud.hh>
#include <types/result_fit_planes.hh>


namespace linkml {

    struct refinement_parameters
    {
        tg::angle angle_threashhold = tg::degree(25);
        float distance_threshhold = 0.5; //0.1
    };
    

    result_fit_planes refine(point_cloud &cloud, result_fit_planes & rs,  refinement_parameters const & param);
}