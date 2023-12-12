#pragma once

#include <vector>

#include <nanoflann.hpp>

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/vec.hh>
#include <typed-geometry/feature/colors.hh>


namespace linkml{

    struct point_cloud
    {
        typedef nanoflann::L2_Simple_Adaptor<float, point_cloud >                           DistanceMatrix;
        typedef nanoflann::KDTreeSingleIndexAdaptor<DistanceMatrix, point_cloud,3, int>     KDTree;

        using coord_t = float;  //!< The type of each coordinate

        std::vector<tg::pos3> pts = std::vector<tg::pos3>();
        std::vector<tg::vec3>  norm = std::vector<tg::vec3> ();
        std::vector<tg::color3> colors = std::vector<tg::color3>();


        std::vector<int> radiusSearch(tg::pos3 &pt, float radius) const {

            assert(tree_index != NULL && "Tree has to be initialised using buildIndex().");

            //KDTree radius search
            std::vector<nanoflann::ResultItem<int, float>> ret_matches;

            float query_pt[3] ={pt.x,pt.y,pt.z};


            tree_index->radiusSearch(&query_pt[0], radius, ret_matches);

            //Extract indecies
            std::vector<int> indecies = std::vector<int>();
            for (size_t i = 0; i < ret_matches.size(); i++)
                indecies.push_back(ret_matches.at(i).first);

            return indecies;


        };
        std::vector<int> radiusSearch(int index, float radius) const {

            //KDTree radius search
            std::vector<nanoflann::ResultItem<int, float>> ret_matches;

            tg::pos3 pt = pts.at(index);

            return this->radiusSearch(pt, radius);
    
        };


        point_cloud(){};
        ~point_cloud(){
            delete tree_index;
        }
        point_cloud(const std::vector<tg::pos3> & points, const std::vector<tg::vec3> & normals ): 
            pts(points), 
            norm(normals){};


        tg::aabb3 get_bbox() const {
            auto it = pts.begin();
            auto v = *it;
            auto bbox = tg::aabb3(v,v);

            while (it != pts.end())
            {
                auto vv = *it;
                bbox.min.x = tg::min(bbox.min.x, vv.x);
                bbox.min.y = tg::min(bbox.min.y, vv.y);
                bbox.min.z = tg::min(bbox.min.z, vv.z);
                bbox.max.x = tg::max(bbox.max.x, vv.x);
                bbox.max.y = tg::max(bbox.max.y, vv.y);
                bbox.max.z = tg::max(bbox.max.z, vv.z);
                it++;
            }

            return bbox;
        }

        void buildIndex(){
            if (tree_index != NULL) delete tree_index;
            tree_index = new KDTree(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10));
            tree_index->buildIndex();
        }

        // Must return the number of data poins
        inline std::size_t kdtree_get_point_count() const { return pts.size(); }

        // Must return the dim'th component of the idx'th point in the class:
        inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const
        {
            if (dim == 0)
                return pts.at(idx).x;
            if (dim == 1)
                return pts.at(idx).y;
            if (dim == 2)
                return pts.at(idx).z;
            throw std::out_of_range("This only suppors to look up of 3D point cloud");
        }

        // Optional bounding-box computation: return false to default to a standard bbox computation loop.
        //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
        //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& bb) const
        {
            auto bbox = get_bbox();

            bb[0].low = bbox.min.x; bb[0].high = bbox.max.x;  // 0th dimension limits
            bb[1].low = bbox.min.y; bb[1].high = bbox.max.y;  // 1st dimension limits
            bb[2].low = bbox.min.z; bb[2].high = bbox.max.z;  // 1st dimension limits

            return true;
        }
        
        private:
            KDTree* tree_index = NULL;
    };
}