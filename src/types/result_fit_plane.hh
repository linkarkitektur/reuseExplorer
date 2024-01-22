#pragma once
#include <vector>
#include <types/plane.hh>

namespace linkml{

    struct result_fit_plane{

        bool valid;
        Plane plane;
        int index;
        std::vector<int> indecies;

        result_fit_plane( int _index){
            valid = false;
            index = _index;
        }

        result_fit_plane( Plane _plane, std::vector<int> _incecies){
            valid = true;
            plane = _plane;
            indecies = _incecies;
        }
    };
}
