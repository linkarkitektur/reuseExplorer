#pragma once

#include <vector>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <optional>
#include <typed-geometry/tg.hh>
#include <nanoflann.hpp>
#include <eigen3/Eigen/Core>

typedef Eigen::MatrixXf Matrix;



class Linkml
{
public:
    Linkml() = default;
};



struct KDPos3
{
    std::vector<tg::pos3> pts = std::vector<tg::pos3>();

    KDPos3(){};

    KDPos3(std::vector<tg::pos3> &points){
        pts = points;
    }

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts.at(idx).x;
        if (dim == 1)
            return pts.at(idx).y;
        if (dim == 2)
            return pts.at(idx).z;
        throw std::out_of_range("This only suppors to look up of 3D point cloud");
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};


namespace linkml
{

    struct point_cloud
    {
        typedef nanoflann::L2_Simple_Adaptor<float, KDPos3 > DistaceMatric;
        typedef nanoflann::KDTreeSingleIndexAdaptor<DistaceMatric, KDPos3,3, int> KDTree;

        using coord_t = float;  //!< The type of each coordinate

        std::vector<tg::pos3> pts;
        std::vector<tg::vec3> norm;





        std::vector<int> radiusSearch(int index, float radius) const {

            //KDTree radius search
            std::vector<nanoflann::ResultItem<int, float>> ret_matches;

            tg::pos3 pt = pts.at(index);
            float query_pt[3] ={pt.x,pt.y,pt.z};


            tree_index.radiusSearch(&query_pt[0], radius, ret_matches);

            //Extract indecies
            std::vector<int> indecies = std::vector<int>();
            for (size_t i = 0; i < ret_matches.size(); i++)
                indecies.push_back(ret_matches.at(i).first);

            return indecies;


        };

        point_cloud(const point_cloud &cloud): pts(cloud.pts), norm(cloud.norm), pos(cloud.pos), tree_index(3, pos, nanoflann::KDTreeSingleIndexAdaptorParams(10)){
            tree_index.buildIndex();
        };
        point_cloud(const std::vector<tg::pos3> & points, const std::vector<tg::vec3> & normals ): pts(points), norm(normals), pos(pts), tree_index(3, pos, nanoflann::KDTreeSingleIndexAdaptorParams(10)){
            tree_index.buildIndex();
        };

        private:
            KDPos3 pos;
            KDTree tree_index;
    };

    struct reg
    {
        reg(reg &r){
            mask.reserve(r.mask.size());
            mask.assign(r.mask.size(),0);
        }
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

        Plane() :origin(){};
//            Plane(const Plane &p){
//                origin = p.origin;
//                dis = p.dis;
//                normal.x = p.normal.x;
//                normal.y = p.normal.y;
//                normal.z = p.normal.z;
//            };
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

    struct  fit_planes_resutl{
        fit_planes_resutl() {}
        std::vector<Plane> planes = std::vector<Plane>();
        std::vector<std::vector<int>> indecies = std::vector<std::vector<int>>();
    };

    struct plane_fitting_parameters{
        float cosalpha = 0.96592583;
        float normal_distance_threshhold = 0.05;
        float distance_threshhold = 0.15;
        int plane_size_threshhold = 500;


        plane_fitting_parameters(){};
        plane_fitting_parameters(float cos, float norm_dist, float dist_threshold, int min_size)
        {
            cosalpha = cos;
            normal_distance_threshhold = norm_dist;
            distance_threshhold = dist_threshold;
            plane_size_threshhold = min_size;
        }
    };

    plane_fit_resutl fit_plane(
        point_cloud const &cloud,
        plane_fitting_parameters const &params,
        std::vector<int> const processed = std::vector<int>(),
        int initial_point_idx = -1
        );

    fit_planes_resutl fit_planes(
        point_cloud const &cloud,
        plane_fitting_parameters const &params
        );

}
