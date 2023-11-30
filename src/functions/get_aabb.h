#pragma onec

#include <typed-geometry/tg.hh>


namespace linkml{
    static tg::aabb3 get_aabb(const std::vector<tg::pos3>& points){

        assert(points.size() >= 1 );

        auto p_min = points[0];
        auto p_max = points[0];

        for (int i = 1; i<points.size(); i++){
            if (points[i].x < p_min.x) p_min.x = points[i].x;
            if (points[i].y < p_min.y) p_min.y = points[i].y;
            if (points[i].z < p_min.z) p_min.z = points[i].z;

            if (points[i].x > p_max.x) p_max.x = points[i].x;
            if (points[i].y > p_max.y) p_max.y = points[i].y;
            if (points[i].z > p_max.z) p_max.z = points[i].z;

        }

        return tg::aabb3(p_min, p_max);


    }
}
