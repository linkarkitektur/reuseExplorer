#pragma once
#include <vector>

#include <typed-geometry/tg-lean.hh>

#include <types/Plane.hh>
#include <types/PointCloud.hh>


namespace linkml {

    struct refinement_parameters
    {
        tg::angle angle_threashhold = tg::degree(25);
        float distance_threshhold = 0.5; //0.1
    };
    
    static std::vector<pcl::PointIndices> refine(PointCloud::Ptr const cloud, std::vector<pcl::PointIndices> const & clusters, refinement_parameters const & param);
}