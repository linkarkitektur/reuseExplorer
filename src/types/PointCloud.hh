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

#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include "functions/color.hh"

#include <vector>
#include <filesystem>
#include <opencv4/opencv2/core.hpp>

#include "types/Brep.hh"

struct EIGEN_ALIGN16 PointT
{
    using LableT = std::int32_t;
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union EIGEN_ALIGN16 {
        LableT data_i[4];
        struct{
            LableT confidence;
            LableT semantic;
            LableT instance;
            LableT label;
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

    inline tg::pos3 getPos () const { return (tg::pos3(x, y, z)); }
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
    
    PCL_MAKE_ALIGNED_OPERATOR_NEW
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
    (PointT::LableT, confidence, confidence)
    (PointT::LableT, semantic, semantic)
    (PointT::LableT, instance, instance)
    (PointT::LableT, label, label)
)




namespace linkml{


    /// @brief A point cloud.
    class PointCloud : public pcl::PointCloud<PointT>::Ptr {

      public:
        using Cloud = pcl::PointCloud<PointT>;

        using Cloud::Ptr::operator*;
        using Cloud::Ptr::operator->;
        using Cloud::Ptr::get;

      private:
        PointCloud::Cloud::Ptr cloud = *this;
      
      public:

        // Constructors
        // @brief Create an empty point cloud.
        PointCloud() : Cloud::Ptr(new Cloud()) {}


        /// @brief Load the point cloud from a file.
        PointCloud(std::string const & filename );

        // TODO: Implement Buffer protocol constructor
        // https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html


        /// @brief Load the point cloud from a file.
        static PointCloud load(std::string const& filename);

        static pcl::PCLHeader load_header(std::string const& filename);


        // TODO: Add set header function 
        // TODO: Add set confidence function


        /// @brief Save the point cloud to a file.
        void save(std::string const& filename, bool binary=true) const;

        /// @brief Get the image from the point cloud.
        cv::Mat image(std::string field_name = "rgb") const;

        /// @brief Get the point at the given coordinates.
        Cloud::PointType at(size_t x, size_t y);

        /// @brief Get the bounding box of the point cloud.
        tg::aabb3 get_bbox() const; 

        /// @brief Transform the point cloud.
        void filter( PointCloud::Cloud::PointType::LableT value = 2); 

        /// @brief Register the point cloud.
        void downsample(double leaf_size = 0.02f);

        /// @brief Annotate the point cloud.
        void annotate();

        /// @brief Region growing, plane fitting
        void region_growing(
          float angle_threshold = 0.96592583, // cos(25°)
          float plane_dist_threshold = 0.1,
          int minClusterSize = 2*(1/0.02)*(1/0.02), // ca 2sqm in 2cm resolution of point cloud 
          float early_stop = 0.3,
          float radius = 0.1,
          float interval_0 = 16, 
          float interval_factor = 1.5
        );

        void clustering(
          double cluster_tolerance = 0.02, // 2cm
          pcl::uindex_t min_cluster_size = 100,
          pcl::uindex_t max_cluster_size =  std::numeric_limits<pcl::uindex_t>::max()
        );


        /// @brief Solidify the point cloud.
        std::vector<Brep> solidify(
          unsigned int downsample_size = 5000000,
          double sx = 0.4,
          double sy = 0.4,
          double expand_factor = 2, 
          double inflate_factor = 2, 
          double max_loop = 10.0, 
          double mult_factor = 1.0,
          double fitting =  0.20,
          double coverage = 0.10,
          double complexity = 0.70
        );


        /// @brief Register the point cloud.
        void display(std::string name = "Cloud") const;

        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };
}
