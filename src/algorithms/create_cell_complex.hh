#pragma once
#include <vector>
#include <types/point_cloud.hh>
#include <types/result_fit_planes.hh>


namespace linkml{
    void create_cell_complex(linkml::point_cloud& cloud, linkml::result_fit_planes& results);
}
