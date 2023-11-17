#pragma once
#include <vector>
#include <types/point_cloud.h>
#include <types/plane.h>
#include <types/cell.h>

namespace linkml{

    std::vector<Cell *> create_cell_complex(
        point_cloud const &cloud, 
        std::vector<Plane> const &planes);
}
