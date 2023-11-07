#pragma once

#include <linkml.h>

namespace linkml {

    struct Cell
    {
        tg::aabb3 box = tg::aabb3();
        std::vector<tg::triangle3> triangels = std::vector<tg::triangle3>();
    };

    std::vector<Cell> CreateCellComplex(
        linkml::point_cloud const &cloud, 
        std::vector<linkml::Plane> const &planes);
}