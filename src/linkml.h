#pragma once

#include <vector>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <optional>
#include <typed-geometry/tg.hh>
#include "../../../External Repos/nanoflann/include/nanoflann.hpp"




namespace linkml
{

    struct point_cloud
    {
        using coord_t = float;  //!< The type of each coordinate

        std::vector<tg::pos3> pts;
        std::vector<tg::vec3> norm;

               // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return pts.size(); }

               // Returns the dim'th component of the idx'th point in the class:
               // Since this is inlined and the "dim" argument is typically an immediate
               // value, the
               //  "if/else's" are actually solved at compile time.
        inline float kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            return pts[idx][dim];
        }

               // Optional bounding-box computation: return false to default to a standard
               // bbox computation loop.
               //   Return true if the BBOX was already computed by the class and returned
               //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
               //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /* bb */) const
        {
            return false;
        }
    };

    struct reg
    {
        reg(int size)
        {
            mask.reserve(size);
            mask.assign(size,0);
        }
        std::vector<int> mask;
        std::vector<int> indecies;
    };

    struct Plane: tg::plane3
        {
            tg::pos3 origin = tg::pos3(0,0,0);

            Plane();
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
                origin = tg::pos3(x,z,z);
            }
        };

    struct plane_fit_resutl{

        bool valid;
        Plane plane;
        int index;
        std::vector<int> indecies;

        plane_fit_resutl( int _index){
            valid = false;
            index = _index;
        }

        plane_fit_resutl( Plane _plane, std::vector<int> _incecies){
            valid = true;
            plane = _plane;
            indecies = _incecies;
        }
    };

    struct plane_fitting_parameters{
        float const cosalpha;
        float const normal_distance_threshhold;
        float const distance_threshhold;
        int const plane_size_threshhold;
    };

    plane_fit_resutl fit_plane(
        nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float /* Distance */,point_cloud /* Data */> /* Distance */, point_cloud /*Data*/,3 /* Dim */, int /*Index*/> &tree,
        point_cloud &cloud,
        reg &processed_reg,
        const linkml::plane_fitting_parameters params,
        int initial_point_idx = -1
        );
}

class Linkml
{
public:
    Linkml() = default;
};


//struct vec_3
//{
//    float& operator[](size_t i){
//        if(i == 0)
//            return x;
//        if(i == 1)
//            return y;
//        if(i == 2)
//            return z;
//    }
//    float const& operator[](size_t i) const{
//        if(i == 0)
//            return x;
//        if(i == 1)
//            return y;
//        if(i == 2)
//            return z;
//    }
//};
