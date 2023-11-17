#pragma once
#include <vector>
#include <array>

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/aabb.hh>

#include <polymesh/Mesh.hh>
#include <polymesh/attributes.hh>

typedef std::vector<std::array<int, 3>> Faces;
typedef std::vector<std::array<float, 3>> Verts;

typedef tg::pos3 Pos;
typedef pm::vertex_attribute<Pos> PosH;

namespace linkml{
    
    struct Cell
    {

        pm::Mesh m;
        PosH pos = pm::vertex_attribute<tg::pos3>(m);

        Cell() : m(pm::Mesh()){}

        Faces faces();
        Verts vertecies();
        tg::aabb3 box();
    };
}
