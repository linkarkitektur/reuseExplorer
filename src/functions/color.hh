#pragma once

#include <typed-geometry/feature/colors.hh>
#include <typed-geometry/types/angle.hh>
#include <typed-geometry/tg.hh> // for tg::pi since just the others is not enough

#define PHI 1.618033988749894

namespace linkml {
    static tg::angle sample_circle(int i){
        return i * PHI * tg::pi<float>;
    }

    static tg::color3 get_color_forom_angle(tg::angle ang){
        float rad = ang.radians();
        float pi_2 = (2 * tg::pi<float>).radians();
        float val = tg::mod(rad,pi_2) / pi_2;
        return tg::to_linear_color(tg::hsl(val,1.0,0.5));
    }
}
