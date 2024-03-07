#pragma once
#include <vector>
#include <types/PointCloud.hh>
#


namespace linkml{

    void create_cell_complex(linkml::PointCloud& cloud, std::vector<pcl::PointIndices> const & clusters );
}
