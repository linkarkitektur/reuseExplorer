#pragma once
#include <vector>
#include <types/PointCloud.hh>
#include <types/result_fit_planes.hh>


namespace linkml{
    void create_cell_complex(linkml::PointCloud& cloud, linkml::result_fit_planes& results);
}
