#pragma once
#include <vector>
#include <types/Plane.hh>


namespace linkml {
    struct  result_fit_planes{
        result_fit_planes() {}
        std::vector<Plane> planes = std::vector<Plane>();
        std::vector<std::vector<int>> indecies = std::vector<std::vector<int>>();
    };

}