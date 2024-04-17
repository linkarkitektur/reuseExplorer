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
#include <functions/color.hh>

#include <vector>
#include <filesystem>
#include <opencv4/opencv2/core.hpp>

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
    (int, confidence, confidence)
    (int, semantic, semantic)
    (int, instance, instance)
    (int, label, label)
)




namespace linkml{


    /// @brief A point cloud.
    class PointCloud : public pcl::PointCloud<PointT>{
      public:

        // Inherit types
        using pcl::PointCloud<PointT>::PointType;
        using pcl::PointCloud<PointT>::VectorType;
        using pcl::PointCloud<PointT>::CloudVectorType;

        // Inherit constructors
        using pcl::PointCloud<PointT>::PointCloud;

        // Inherit properties
        using pcl::PointCloud<PointT>::header;
        using pcl::PointCloud<PointT>::points;
        using pcl::PointCloud<PointT>::width;
        using pcl::PointCloud<PointT>::height;
        using pcl::PointCloud<PointT>::is_dense;
        using pcl::PointCloud<PointT>::sensor_origin_;
        using pcl::PointCloud<PointT>::sensor_orientation_;

        // Inherit operators
        using pcl::PointCloud<PointT>::operator +=;
        using pcl::PointCloud<PointT>::operator  +;
        using pcl::PointCloud<PointT>::operator ();
        using pcl::PointCloud<PointT>::operator [];

        using pcl::PointCloud<PointT>::value_type;
        using pcl::PointCloud<PointT>::reference;
        using pcl::PointCloud<PointT>::const_reference;
        using pcl::PointCloud<PointT>::difference_type;
        using pcl::PointCloud<PointT>::size_type;

        using pcl::PointCloud<PointT>::at;
        using pcl::PointCloud<PointT>::concatenate;
        using pcl::PointCloud<PointT>::isOrganized;
        using pcl::PointCloud<PointT>::getMatrixXfMap;

        using pcl::PointCloud<PointT>::iterator;
        using pcl::PointCloud<PointT>::const_iterator;
        using pcl::PointCloud<PointT>::reverse_iterator;
        using pcl::PointCloud<PointT>::const_reverse_iterator;
        using pcl::PointCloud<PointT>::begin;
        using pcl::PointCloud<PointT>::end;
        using pcl::PointCloud<PointT>::rbegin;
        using pcl::PointCloud<PointT>::rend;
        using pcl::PointCloud<PointT>::cbegin;
        using pcl::PointCloud<PointT>::cend;
        using pcl::PointCloud<PointT>::crbegin;
        using pcl::PointCloud<PointT>::crend;

        using pcl::PointCloud<PointT>::size;
        using pcl::PointCloud<PointT>::max_size;
        using pcl::PointCloud<PointT>::reserve;
        using pcl::PointCloud<PointT>::empty;
        using pcl::PointCloud<PointT>::data;

        using pcl::PointCloud<PointT>::resize;
        using pcl::PointCloud<PointT>::assign;

        using pcl::PointCloud<PointT>::front;
        using pcl::PointCloud<PointT>::back;

        using pcl::PointCloud<PointT>::push_back;
        using pcl::PointCloud<PointT>::transient_push_back;
        using pcl::PointCloud<PointT>::emplace_back;
        using pcl::PointCloud<PointT>::transient_emplace_back;

        using pcl::PointCloud<PointT>::insert;
        using pcl::PointCloud<PointT>::transient_insert;
      
        using pcl::PointCloud<PointT>::emplace;
        using pcl::PointCloud<PointT>::transient_emplace;

        using pcl::PointCloud<PointT>::erase;
        using pcl::PointCloud<PointT>::transient_erase;

        using pcl::PointCloud<PointT>::swap;
        using pcl::PointCloud<PointT>::clear;

        // Custom types
        using Ptr = pcl::shared_ptr<PointCloud>;
        using ConstPtr = pcl::shared_ptr<const PointCloud>;

        inline Ptr
        makeShared () const { return Ptr (new PointCloud (*this)); }


        // Constructors
        /// @brief Load the point cloud from a file.
        PointCloud(std::string const & filename ){
          pcl::io::loadPCDFile(filename, *this);
        }

        // TODO: Implement Buffer protocol
        // https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html

        /// @brief Load the point cloud from a file.
        static PointCloud::Ptr load(std::string const& filename) {

          std::filesystem::path pcd = filename;
          PointCloud::Ptr cloud(new PointCloud);
          
          pcl::io::loadPCDFile(pcd, *cloud);

          cloud->header = load_header(filename);

          return cloud;
        }

        static pcl::PCLHeader load_header(std::string const& filename) {
          pcl::PCLHeader header;

          std::filesystem::path head = filename;
          head.replace_extension(".head");

          if (std::filesystem::exists(head)){
              std::ifstream header_file(head);
              if (header_file.is_open()){
                  header_file >> header.frame_id;
                  header_file >> header.stamp;
                  header_file >> header.seq;
                  header_file.close();
              }
          }
          return header;
        }


        // Custom methods
        /// @brief Get the bounding box of the point cloud.
        tg::aabb3 get_bbox() const {
          float x_min = std::numeric_limits<float>::max();
          float y_min = std::numeric_limits<float>::max();
          float z_min = std::numeric_limits<float>::max();
          float x_max = std::numeric_limits<float>::min();
          float y_max = std::numeric_limits<float>::min();
          float z_max = std::numeric_limits<float>::min();

          #pragma omp parallel for reduction(min:x_min, y_min, z_min) reduction(max:x_max, y_max, z_max)
          for (size_t i = 0; i < this->size(); i++){
              auto p = this->at(i);
              if (p.x < x_min) x_min = p.x;
              if (p.y < y_min) y_min = p.y;
              if (p.z < z_min) z_min = p.z;
              if (p.x > x_max) x_max = p.x;
              if (p.y > y_max) y_max = p.y;
              if (p.z > z_max) z_max = p.z;
          }

          return tg::aabb3(tg::pos3(x_min, y_min, z_min), tg::pos3(x_max, y_max, z_max));

        }


        // TODO: Add set header function 
        // TODO: Add set confidence function

        /// @brief Save the point cloud to a file.
        PointCloud save(std::string const& filename, bool binary=true) const {
          pcl::io::savePCDFile(filename, *this, binary);

          // Save header information
          std::filesystem::path p(filename);
          p.replace_extension(".head");
          std::filesystem::exists(p) ? std::filesystem::remove(p) : std::filesystem::create_directories(p.parent_path());
          std::ofstream header_file(p);
          if (header_file.is_open()){
              header_file << this->header.frame_id << std::endl;
              header_file << this->header.stamp << std::endl;
              header_file << this->header.seq << std::endl;
              header_file.close();
          }
          return *this;
        }

        /// @brief Transform the point cloud.
        PointCloud filter(); 

        /// @brief Register the point cloud.
        PointCloud downsample(double leaf_size = 0.02f);

        /// @brief Annotate the point cloud.
        PointCloud annotate();

        /// @brief Region growing, plane fitting
        PointCloud region_growing(
          float angle_threshold = 0.96592583, // cos(25°)
          float plane_dist_threshold = 0.1,
          int minClusterSize = 2*(1/0.02)*(1/0.02), // ca 2sqm in 2cm resolution of point cloud 
          float early_stop = 0.3,
          float radius = 0.1,
          float interval_0 = 16, 
          float interval_factor = 1.5
        );

        PointCloud clustering(
          double cluster_tolerance = 0.02, // 2cm
          pcl::uindex_t min_cluster_size = 100,
          pcl::uindex_t max_cluster_size =  std::numeric_limits<pcl::uindex_t>::max()
        );



        cv::Mat image(std::string field_name = "rgb") const {

          if (!this->is_dense)
            throw std::runtime_error("Clouds must be dense");

          cv::Mat img = cv::Mat::zeros(this->height, this->width, CV_8UC3);

          if ( field_name == "rgb" ){
            #pragma omp parallel for shared(img)
            for (size_t i = 0; i < this->size(); i++){
                auto point = this->at(i);
                size_t y = i % this->width;
                size_t x = i / this->width;
                img.at<cv::Vec3b>(x, y) = cv::Vec3b(point.r, point.g, point.b);
            }
          }
          else if ( field_name == "semantic"){
            #pragma omp parallel for shared(img)
            for (size_t i = 0; i < this->size(); i++){
                auto point = this->at(i);
                size_t y = i % this->width;
                size_t x = i / this->width;
                auto c = linkml::get_color_forom_angle(linkml::sample_circle(point.semantic));
                img.at<cv::Vec3b>(x, y) = cv::Vec3b(c.r * 255, c.g * 255, c.b * 255);
            }
          }
          else {
            PCL_WARN ("Filed not implemented");
          }


          return img;
        }


        PointType at(size_t x, size_t y) {
          return this->points.at(y * this->width + x);
        }

        /// @brief Register the point cloud.
        PointCloud display(std::string name = "Cloud") const {

          polyscope::init();

          polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
          polyscope::view::setUpDir(polyscope::UpDir::YUp);


          auto  points = std::vector<std::array<float, 3>>();
          points.resize(this->points.size());

          auto colors = std::vector<std::array<float, 3>>();
          colors.resize(this->points.size());

          auto confideces = std::vector<int>();
          confideces.resize(this->points.size());

          auto confideces_colors = std::vector<tg::color3>();
          confideces_colors.resize(this->points.size());

          auto lables = std::vector<int>();
          lables.resize(this->points.size());

          auto lables_colors = std::vector<tg::color3>();
          lables_colors.resize(this->points.size());

          auto sematic_lables = std::vector<int>();
          sematic_lables.resize(this->points.size());

          auto sematic_colors = std::vector<tg::color3>();
          sematic_colors.resize(this->points.size());

          auto instance_lables = std::vector<int>();
          instance_lables.resize(this->points.size());

          auto instance_colors = std::vector<tg::color3>();
          instance_colors.resize(this->points.size());
        
          auto normals = std::vector<std::array<float, 3>>();
          normals.resize(this->points.size());

          auto normal_colors = std::vector<std::array<float, 3>>();
          normal_colors.resize(this->points.size());

          auto importance = std::vector<float>();
          importance.resize(this->points.size());


        

          #pragma omp parallel for
          for (size_t i = 0; i < this->points.size(); i++){

              points[i] = {this->points[i].x, this->points[i].y, this->points[i].z};

              colors[i] = {
                  static_cast<float>(this->points[i].r)/256,
                  static_cast<float>(this->points[i].g)/256,
                  static_cast<float>(this->points[i].b)/256};

              normals[i] = {
                  this->points[i].normal_x,
                  this->points[i].normal_y,
                  this->points[i].normal_z};

              // remap normals to 0-1
              normal_colors[i] = {
                  (this->points[i].normal_x + 1.0f)/2.0f,
                  (this->points[i].normal_y + 1.0f)/2.0f,
                  (this->points[i].normal_z + 1.0f)/2.0f};

              switch (this->points[i].confidence)
              {
              case 0:
                  confideces_colors[i] = tg::color3(1.0, 0.0, 0.0);
                  break;
              case 1:
                  confideces_colors[i] = tg::color3(1.0, 1.0, 0.0);
                  break;
              case 2:
                  confideces_colors[i] = tg::color3(0.0, 1.0, 0.0);
                  break;

              default:
                  break;
              }

              confideces[i] = this->points[i].confidence;
              lables[i] = this->points[i].label;
              lables_colors[i] = linkml::get_color_forom_angle(linkml::sample_circle(this->points[i].label));
              sematic_lables[i] = this->points[i].semantic;
              sematic_colors[i] = linkml::get_color_forom_angle(linkml::sample_circle(this->points[i].semantic));
              instance_lables[i] = this->points[i].instance;
              instance_colors[i] = linkml::get_color_forom_angle(linkml::sample_circle(this->points[i].instance));

          }


          // Set unused lables to 
          int selection[]{60, 56, 57, 58, 41, 62, 63, 64, 66, 41};
          #pragma omp parallel for
          for (size_t i = 0; i < this->points.size(); i++){
            auto semantic = sematic_lables[i];
            if (std::count(std::begin(selection), std::end(selection), semantic) == 0){
              sematic_lables[i] = 0;
              sematic_colors[i] = linkml::get_color_forom_angle(linkml::sample_circle(0));
            }
          }


          // Set importance
          #pragma omp parallel for
          for (size_t i = 0; i < this->points.size(); i++){
               importance[i] = (this->points[i].confidence+0.01f)
                                  *
                                  ( (this->points[i].label == 0 ? 0.1f : 1.0f)
                                  + (sematic_lables[i] == 0 ? 0.1f : 2.0f)
                                  + (this->points[i].instance == 0 ? 0.1f : 3.0f));         
          }


          auto pcd = polyscope::registerPointCloud(name, points);
          pcd->setPointRadius(0.001);

          auto normal_vectors = pcd->addVectorQuantity("Normals", normals);
          normal_vectors->setVectorRadius(0.00050);
          normal_vectors->setVectorLengthScale(0.0050);

          pcd->addColorQuantity("RGB", colors);
          pcd->addColorQuantity("Normal Colors", normal_colors);
          pcd->addScalarQuantity("Confidence", confideces);
          pcd->addScalarQuantity("Lables", lables);
          pcd->addScalarQuantity("Sematic Lables", sematic_lables);
          pcd->addScalarQuantity("Instance Lables", instance_lables);
          pcd->addScalarQuantity("Importance", importance );

          pcd->addColorQuantity("Confidence Colors", confideces_colors);
          pcd->addColorQuantity("Lables Colors", lables_colors);
          pcd->addColorQuantity("Sematic Colors", sematic_colors);
          pcd->addColorQuantity("Instance Colors", instance_colors);

          pcd->setPointRadiusQuantity("Importance", true);

          pcd->quantities["Lables Colors"]->setEnabled(true);

          polyscope::show();

          return *this;

        }

        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };
}
