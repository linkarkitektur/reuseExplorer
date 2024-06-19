#pragma once

#include <polymesh/Mesh.hh>
#include <typed-geometry/tg.hh>

#include <clean-core/array.hh>
#include <clean-core/vector.hh>


namespace ps_helpers{

    static std::vector<tg::pos3> vertecies(pm::Mesh const & m, pm::vertex_attribute<tg::pos3>const & pos){
        return m.vertices().map([&](pm::vertex_handle h){return pos[h];}).to_vector();
    }
    static std::vector<cc::array<int,3>> faces(pm::Mesh const & m){
        return m.faces().map([&](pm::face_handle f){
            auto vx = f.vertices().to_vector();
            cc::array<int, 3> indexis{int(vx[0]),int(vx[1]),int(vx[2])};
            return indexis;
        }).to_vector();
    }
}