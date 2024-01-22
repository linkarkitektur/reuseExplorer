#pragma once

#include <types/plane.h>
#include <typed-geometry/tg.hh>
#include <typed-geometry/types/objects/plane.hh>
#include <typed-geometry/types/mat.hh>


namespace linkml{
    static tg::mat4 get_matrix_from_plane(tg::plane3 pl){

        auto o = tg::project(tg::pos3(0,0,0),pl);
        auto vec = 1 - tg::abs(tg::dot( tg::vec3(0,0,1), pl.normal)) > 0.5 ?  tg::vec3(0,0,1):  tg::vec3(1,0,0);

        auto X = tg::normalize( tg::project(o+vec, pl)- o);
        auto Y = tg::normalize(tg::cross(X, pl.normal));

        auto mat = tg::mat4::ones;
        mat.set_col(0, (tg::vec4)X);
        mat.set_col(1, (tg::vec4)Y);
        mat.set_col(2, (tg::vec4)pl.normal);
        
        return mat;

    }
}