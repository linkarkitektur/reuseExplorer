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

}