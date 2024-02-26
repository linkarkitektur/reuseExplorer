#pragma once
#include <string>
#include <types/point_cloud.hh>
#include <pcl/io/pcd_io.h>

namespace linkml{
    PointCloud::Ptr load(std::string file){
        PointCloud::Ptr cloud(new PointCloud);
        pcl::io::loadPCDFile<PointCloud::PointType> (file, *cloud);
        return cloud;
    }
}