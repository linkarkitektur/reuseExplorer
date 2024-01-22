#pragma once

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/plane.hh>

#include <functions/get_csystem.h>
#include <functions/get_matrix_from_plane.h>


namespace linkml
{
    static cc::vector<tg::pos2> project_2d(cc::vector<tg::pos3> const& pts, tg::plane3  const& pl){

        auto mat = get_matrix_from_plane(pl);

        // auto [mat, center] = get_csystem(pts, pl);
        mat = tg::inverse(mat);

        cc::vector<tg::pos2> out;
        for (auto& p: pts){
            auto p_ = mat * (p-(pl.normal*pl.dis));
            out.emplace_back(tg::pos2(p_.x, p_.y));
        }

        // Move all points closer to origin
        auto center = (tg::vec2)tg::average(out);
        std::transform(out.begin(), out.end(),out.begin(), [&](tg::pos2 & p){ return p - center;});

        return out;

    }
    static cc::vector<tg::pos2> project_2d(std::vector<tg::pos3> const& pts, tg::plane3  const& pl){

        auto mat = get_matrix_from_plane(pl);

        // auto [mat, center] = get_csystem(pts, pl);
        mat = tg::inverse(mat);

        cc::vector<tg::pos2> out;
        for (auto& p: pts){
            auto p_ = mat * (p-(pl.normal*pl.dis));
            out.emplace_back(tg::pos2(p_.x, p_.y));
        }

        // Move all points closer to origin
        auto center = (tg::vec2)tg::average(out);
        std::transform(out.begin(), out.end(),out.begin(), [&](tg::pos2 & p){ return p - center;});

        return out;

    }
}
