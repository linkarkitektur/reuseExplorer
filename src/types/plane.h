#pragma once

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/plane.hh>


namespace linkml {

    struct Plane: tg::plane3
    {
        tg::pos3 origin = tg::pos3(0,0,0);

    Plane() :origin(){};

        Plane(float A, float B, float C, float D)
        {
            normal.x = A;
            normal.y = B;
            normal.z = C;
            dis = D;
        }
        Plane(float A, float B, float C, float D, float x, float y, float z)
        {
            normal.x = A;
            normal.y = B;
            normal.z = C;
            dis = D;
            origin = tg::pos3(x,y,z);
        }
    };

}
