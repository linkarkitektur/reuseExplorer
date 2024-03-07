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


    //using PointCloud = pcl::PointCloud<PointT>;

    class PointCloud : public pcl::PointCloud<PointT>{
      public:
        using pcl::PointCloud<PointT>::PointCloud;
        using pcl::PointCloud<PointT>::operator+=;
        using pcl::PointCloud<PointT>::operator +;
        using pcl::PointCloud<PointT>::concatenate;
        using pcl::PointCloud<PointT>::at;
        using pcl::PointCloud<PointT>::operator ();
        using pcl::PointCloud<PointT>::isOrganized;
        using pcl::PointCloud<PointT>::getMatrixXfMap;

        using pcl::PointCloud<PointT>::header;
        using pcl::PointCloud<PointT>::points;
        using pcl::PointCloud<PointT>::width;
        using pcl::PointCloud<PointT>::height;
        using pcl::PointCloud<PointT>::is_dense;
        using pcl::PointCloud<PointT>::sensor_origin_;
        using pcl::PointCloud<PointT>::sensor_orientation_;

        using pcl::PointCloud<PointT>::PointType;
        using pcl::PointCloud<PointT>::VectorType;
        using pcl::PointCloud<PointT>::CloudVectorType;

        using Ptr = std::shared_ptr<PointCloud>;
        using ConstPtr = std::shared_ptr<const PointCloud>;

        using pcl::PointCloud<PointT>::value_type;
        using pcl::PointCloud<PointT>::reference;
        using pcl::PointCloud<PointT>::const_reference;
        using pcl::PointCloud<PointT>::difference_type;
        using pcl::PointCloud<PointT>::size_type;

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

        using pcl::PointCloud<PointT>::operator[];
        using pcl::PointCloud<PointT>::front;
        using pcl::PointCloud<PointT>::back;

        using pcl::PointCloud<PointT>::assign;

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

        using pcl::PointCloud<PointT>::makeShared;



        // Custom methods
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

        PointCloud::Ptr save(std::string const& filename, bool binary=true) const {
          pcl::io::savePCDFile(filename, *this, binary);
          return pcl::make_shared<PointCloud>(*this);
        }

        PointCloud::Ptr load(std::string const& filename) {
          pcl::io::loadPCDFile(filename, *this);
          return pcl::make_shared<PointCloud>(*this);
        }

        PointCloud::Ptr display(std::string name = "Cloud") const {

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

              importance[i] = (this->points[i].confidence+0.01f)
                                  *
                                  ( (this->points[i].label == 0 ? 0.1f : 1.0f)
                                  + (this->points[i].semantic == 0 ? 0.1f : 2.0f)
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

          pcd->setPointRadiusQuantity("Importance", true);

          pcd->quantities["Lables Colors"]->setEnabled(true);

          polyscope::show();

          return pcl::make_shared<PointCloud>(*this);
        }

    };

    //template <typename PointCloud>
    //static tg::aabb3 get_bbox(const PointCloud & cloud){
    //    float x_min = std::numeric_limits<float>::max();
    //    float y_min = std::numeric_limits<float>::max();
    //    float z_min = std::numeric_limits<float>::max();
    //    float x_max = std::numeric_limits<float>::min();
    //    float y_max = std::numeric_limits<float>::min();
    //    float z_max = std::numeric_limits<float>::min();

    //    #pragma omp parallel for reduction(min:x_min, y_min, z_min) reduction(max:x_max, y_max, z_max)
    //    for (size_t i = 0; i < cloud.size(); i++){
    //        auto p = cloud.at(i);
    //        if (p.x < x_min) x_min = p.x;
    //        if (p.y < y_min) y_min = p.y;
    //        if (p.z < z_min) z_min = p.z;
    //        if (p.x > x_max) x_max = p.x;
    //        if (p.y > y_max) y_max = p.y;
    //        if (p.z > z_max) z_max = p.z;
    //    }

    //    return tg::aabb3(tg::pos3(x_min, y_min, z_min), tg::pos3(x_max, y_max, z_max));
    //}

}
