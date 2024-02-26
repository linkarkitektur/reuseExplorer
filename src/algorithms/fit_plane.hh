#pragma once
#include <vector>
#include <types/point_cloud.hh>
#include <types/plane_fit_parameters.hh>
#include <types/result_fit_plane.hh>


namespace linkml {
    
    result_fit_plane fit_plane(
        point_cloud const &cloud,
        plane_fitting_parameters const &params,
        std::vector<int> const processed,
        int initial_point_idx);

    result_fit_plane fit_plane(
        point_cloud const &cloud,
        plane_fitting_parameters const &params,
        std::vector<int> const proccessed);


    result_fit_plane fit_plane(
        point_cloud const &cloud,
        plane_fitting_parameters const &params
        );


    result_fit_plane fit_plane(
        point_cloud const &cloud,
        plane_fitting_parameters const &params,
        int initial_point_idx);

}