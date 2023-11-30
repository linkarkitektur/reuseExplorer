#pragma once
#include <vector>
#include <array>

#include <types/plane.h>

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/aabb.hh>

#include <polymesh/Mesh.hh>
#include <polymesh/attributes.hh>

typedef std::vector<std::array<int, 3>> Faces;
typedef std::vector<std::array<float, 3>> Verts;

typedef tg::pos3 Pos;
typedef pm::vertex_attribute<Pos> PosH;
typedef pm::face_attribute<linkml::Plane> SPlaneH;
typedef pm::face_attribute<tg::color3> ColorH;

namespace linkml{
    
    struct CellComplex
    {

        pm::Mesh m;
        PosH pos = PosH(m);
        SPlaneH supporting_plans = SPlaneH(m);
        ColorH colors = ColorH(m);
        ColorH facets_colors = ColorH(m);


        CellComplex() : m(pm::Mesh()){}

        Faces faces();
        Verts vertecies();
        tg::aabb3 box();
    };
}
