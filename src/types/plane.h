#pragma once

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/plane.hh>
#include <typed-geometry/tg-std.hh>


namespace linkml {

    struct Plane: tg::plane3
    {
        tg::pos3 origin = tg::pos3(0,0,0);

        Plane() :origin(){};

        Plane(float A, float B, float C, float D)
        {   
            auto norm = tg::normalize_safe(tg::vec3(A, B, C));

            normal.x = norm.x;
            normal.y = norm.y;
            normal.z = norm.z;

            dis = -D;
        }
        Plane(float A, float B, float C, float D, float x, float y, float z)
        {

            auto norm = tg::normalize_safe(tg::vec3(A, B, C));

            normal.x = norm.x;
            normal.y = norm.y;
            normal.z = norm.z;

            dis = -D;
            origin = tg::pos3(x,y,z);
        }
    };

}
