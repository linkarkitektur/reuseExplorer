#pragma once
#include <vector>
#include <types/point_cloud.h>
#include <types/result_fit_planes.h>


namespace linkml{
    void create_cell_complex(linkml::point_cloud& cloud, linkml::result_fit_planes& results);
}
