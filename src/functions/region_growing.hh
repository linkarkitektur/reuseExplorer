#pragma once
#include <types/PointCloud.hh>


namespace linkml {
    PointCloud::Ptr region_growing(
        linkml::PointCloud::Ptr cloud, 
        int minClusterSize = 2*(1/0.02)*(1/0.02), // ca 2sqm in 2cm resolution of point cloud
        int numberOfNeighbours = 30,
        float smoothnessThreshold =  3.0 / 180.0 * M_PI,
        float curvatureThreshold = 0.1
    );
} // namespace linkml
