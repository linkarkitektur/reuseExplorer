#pragma once
#include <vector>

#include <typed-geometry/tg-lean.hh>

#include <types/Plane.hh>
#include <types/PointCloud.hh>


namespace linkml {

    
    std::vector<pcl::PointIndices> refine(
        PointCloud::Cloud::Ptr const cloud, 
        std::vector<pcl::PointIndices> const & clusters,
        tg::angle angle_threashhold = tg::degree(25),
        float distance_threshhold = 0.5
        );
}