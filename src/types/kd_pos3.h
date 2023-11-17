#pragma once
#include <vector>
#include <iostream>
#include <typed-geometry/types/pos.hh>

struct kd_pos3
{
    std::vector<tg::pos3> pts = std::vector<tg::pos3>();

    kd_pos3(){};

    kd_pos3(std::vector<tg::pos3> &points){
        pts = points;
    }

    inline std::size_t kdtree_get_point_count() const { return pts.size(); }

    inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const
    {
        if (dim == 0)
            return pts.at(idx).x;
        if (dim == 1)
            return pts.at(idx).y;
        if (dim == 2)
            return pts.at(idx).z;
        throw std::out_of_range("This only suppors to look up of 3D point cloud");
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};