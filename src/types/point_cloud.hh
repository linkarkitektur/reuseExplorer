#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>

#include <pcl/registration/icp.h>
#include <pcl/registration/impl/icp.hpp>

#include <pcl/registration/icp_nl.h>
#include <pcl/registration/impl/icp_nl.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

#include <pcl/common/transforms.h>
#include <pcl/common/impl/transforms.hpp>


#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <nanoflann.hpp>

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/vec.hh>
#include <typed-geometry/feature/colors.hh>

#include <vector>

struct EIGEN_ALIGN16 PointT
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union EIGEN_ALIGN16 {
        std::uint8_t data_i[4];
        struct{
            std::uint8_t confidence;
            std::uint8_t semantic;
            std::uint8_t instance;
            std::uint8_t label;
        };
    };
    union
    {
      struct
      {
        PCL_ADD_UNION_RGB;
        float curvature;
      };
      float data_c[4];
    };
    PCL_ADD_EIGEN_MAPS_RGB;
    PCL_MAKE_ALIGNED_OPERATOR_NEW


    inline PointT (float _curvature = 0.f):
        PointT (0.f, 0.f, 0.f, 0, 0, 0, 0.f, 0.f, 0.f, _curvature) {}

    inline PointT (float _x, float _y, float _z):
      PointT (_x, _y, _z, 0, 0, 0) {}

    inline PointT (std::uint8_t _r, std::uint8_t _g, std::uint8_t _b):
      PointT (0.f, 0.f, 0.f, _r, _g, _b) {}

    inline PointT (float _x, float _y, float _z, std::uint8_t _r, std::uint8_t _g, std::uint8_t _b):
      PointT (_x, _y, _z, _r, _g, _b, 0.f, 0.f, 0.f) {}

    inline PointT (float _x, float _y, float _z, std::uint8_t _r, std::uint8_t _g, std::uint8_t _b,
                              float n_x, float n_y, float n_z, float _curvature = 0.f){
      x = _x; y = _y; z = _z;
      data[3] = 1.0f;
      r = _r; g = _g; b = _b;
      a = 255;
      normal_x = n_x; normal_y = n_y; normal_z = n_z;
      data_n[3] = 0.f;
      curvature = _curvature;
    }

    inline tg::pos3 getPos () const { return (tg::pos3(x, y, x)); }
    inline tg::color3 getColor () const { return (tg::color3(r, g, b)); }
    inline tg::vec3 getNormal () const { return (tg::vec3(normal_x, normal_y, normal_z)); }
    friend std::ostream& operator<< (std::ostream& os, const PointT& p){
        os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
        os << " - ";
        os << " (" << p.r << ", " << p.g << ", " << p.b << ")";
        os << " - ";
        os << " (" << p.normal_x << ", " << p.normal_y << ", " << p.normal_z << ")";
        os << " - ";
        os << " (" << p.confidence << ", " << p.semantic << ", " << p.instance << ", " << p.label << ")";
        return os;
    }
    
};




POINT_CLOUD_REGISTER_POINT_STRUCT(PointT,  
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
    (int, confidence, confidence)
    (int, semantic, semantic)
    (int, instance, instance)
    (int, label, label)
)




namespace linkml{


    // using PointT = pcl::PointXYZRGBNormal;


    using PointCloud = pcl::PointCloud<PointT>;

    static tg::aabb3 get_bbox(const PointCloud & cloud){
        float x_min = std::numeric_limits<float>::max();
        float y_min = std::numeric_limits<float>::max();
        float z_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::min();
        float y_max = std::numeric_limits<float>::min();
        float z_max = std::numeric_limits<float>::min();

        #pragma omp parallel for reduction(min:x_min, y_min, z_min) reduction(max:x_max, y_max, z_max)
        for (size_t i = 0; i < cloud.size(); i++){
            auto p = cloud.at(i);
            if (p.x < x_min) x_min = p.x;
            if (p.y < y_min) y_min = p.y;
            if (p.z < z_min) z_min = p.z;
            if (p.x > x_max) x_max = p.x;
            if (p.y > y_max) y_max = p.y;
            if (p.z > z_max) z_max = p.z;
        }

        return tg::aabb3(tg::pos3(x_min, y_min, z_min), tg::pos3(x_max, y_max, z_max));
    }

    struct point_cloud
    {
        typedef nanoflann::L2_Simple_Adaptor<float, point_cloud >                           DistanceMatrix;
        typedef nanoflann::KDTreeSingleIndexAdaptor<DistanceMatrix, point_cloud,3, int>     KDTree;

        using coord_t = float;  //!< The type of each coordinate

        std::vector<tg::pos3> pts = std::vector<tg::pos3>();
        std::vector<tg::vec3>  norm = std::vector<tg::vec3> ();
        std::vector<tg::color3> colors = std::vector<tg::color3>();


        std::vector<int> radiusSearch(const tg::pos3  &pt, const float radius) const {

            assert(tree_index != NULL && "You need to call buildIndex() on the point cloud before you can the radius searches");

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
        std::vector<int> radiusSearch(const int index,     const float radius) const {

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
        point_cloud(const std::vector<tg::pos3> & points, const std::vector<tg::vec3> & normals, std::vector<tg::color3> const & colors): 
            pts(points), 
            norm(normals),
            colors(colors){};


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