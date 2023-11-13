#pragma once

#include <linkml.h>

#include <typed-geometry/tg.hh>

#include <polymesh/pm.hh>
#include <polymesh/Mesh.hh>

namespace linkml {

    struct Cell
    {
        std::vector<std::array<int, 3>> faces();
        std::vector<std::array<float, 3>> vertecies();
        auto box(){ return tg::aabb3(this->pos.aabb().min, this->pos.aabb().max ); }

        Cell() : m(pm::Mesh()){}

        pm::Mesh m;
        pm::vertex_attribute<tg::pos3> pos = pm::vertex_attribute<tg::pos3>(m);
    };

    std::vector<Cell *> CreateCellComplex(
        linkml::point_cloud const &cloud, 
        std::vector<linkml::Plane> const &planes);
}