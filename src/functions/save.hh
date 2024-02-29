#pragma once
#include <types/PointCloud.hh>
#include <pcl/io/pcd_io.h>

namespace linkml
{
    void save(PointCloud const & cloud, std::string const & file_name, bool binary = true){
        if (binary)
            pcl::io::savePCDFileBinary(file_name, cloud);
        else
            pcl::io::savePCDFileASCII(file_name, cloud);
    }
} // namespace linkml
