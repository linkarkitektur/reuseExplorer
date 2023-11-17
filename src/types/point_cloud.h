#pragma once

#include <vector>

#include <types/kd_pos3.h>

#include <nanoflann.hpp>

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/vec.hh>


namespace linkml{

    typedef nanoflann::L2_Simple_Adaptor<float, kd_pos3 > DistaceMatric;
    typedef nanoflann::KDTreeSingleIndexAdaptor<DistaceMatric, kd_pos3,3, int> KDTree;


    struct point_cloud
    {

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
            kd_pos3 pos;
            KDTree tree_index;
    };
}