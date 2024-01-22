#pragma once

namespace linkml {
    struct plane_fitting_parameters{
        float cosalpha = 0.96592583;
        float normal_distance_threshhold = 0.05;
        float distance_threshhold = 0.15;
        int plane_size_threshhold = 500;


        plane_fitting_parameters(){};
        plane_fitting_parameters(float cos, float norm_dist, float dist_threshold, int min_size)
        {
            cosalpha = cos;
            normal_distance_threshhold = norm_dist;
            distance_threshhold = dist_threshold;
            plane_size_threshhold = min_size;
        }
    };
}