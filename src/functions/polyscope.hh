#pragma once
#include "types/Surface_Mesh.hh"
#include "types/PointCloud.hh"
#include "types/Surface_Mesh.hh"
#include "types/surface.hh"
#include "types/Dataset.hh"

#include "functions/polyscope_helpers.hh"
#include "functions/crop_plane_with_aabb.hh"
#include "functions/color.hh"

#include <typed-geometry/types/objects/aabb.hh>

#include <valarray>

#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>
#include <polyscope/surface_mesh.h>

#include <optional>

static bool polyscope_enabled = false;


namespace polyscope  {


    static void myinit(){

        if (polyscope_enabled) return;

        polyscope::init();
        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
        // polyscope::view::setUpDir(polyscope::UpDir::ZUp);
        polyscope::view::setUpDir(polyscope::UpDir::YUp);

        polyscope_enabled = true;


    }

    static void myshow(){
        polyscope::show();
    }


    //// Define a function to display a error message in case the display function is not defined
    template <typename T>
    void display(T, std::optional<const std::string> = std::nullopt);


    // template <>
    // void display(tg::aabb3 const& box, std::optional<const std::string> name);

    // template <>
    // void display( pcl::PointCloud<PointT> const& cloud, std::optional<const std::string> name);

    // template <>
    // void display(linkml::Surface_mesh const& mesh, std::optional<const std::string> name); 

    // static void display(const tg::plane3 & plane, const tg::aabb3 & bbox, std::optional<const std::string> name);

    // template <>
    // static void display(const linkml::surface& surface, std::optional<const std::string> name);


    enum class Field { 
        Points,
        RGB,
        Normal,
        Normal_color,
        Confidence,
        Confidence_color,
        Lables,
        Lables_color,
        Semantic,
        Semantic_color,
        Instance,
        Instance_color,
        Importance
    };

    
    template <Field F, typename Enable = void>
    struct FieldType;

    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Points ||
            F == Field::RGB ||
            F == Field::Normal ||
            F == Field::Normal_color
        >::type
    > {
        using type = std::array<float, 3>;
    };

    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Lables ||
            F == Field::Semantic ||
            F == Field::Instance
        >::type
    > {
        using type = int;
    };
    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Confidence ||
            F == Field::Importance
        >::type> {
        using type = float;
    };

    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Confidence_color ||
            F == Field::Lables_color ||
            F == Field::Semantic_color || 
            F == Field::Instance_color
        >::type> {
        using type = tg::color3;
    }; 

    template <Field F>
    class PolyscopeMap{
        private:
            pcl::PointCloud<PointT> const& cloud;
        public:
            PolyscopeMap(pcl::PointCloud<PointT> const& cloud) : cloud(cloud) {}

            size_t size() const { return cloud.points.size(); }
            typename FieldType<F>::type operator[](size_t idx) const {

                if constexpr (F == Field::Points) {
                    return std::array<float, 3>{cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z};
                } else if constexpr (F == Field::RGB){
                    return std::array<float, 3>{
                        static_cast<float>(cloud.points[idx].r)/256,
                        static_cast<float>(cloud.points[idx].g)/256,
                        static_cast<float>(cloud.points[idx].b)/256};
                } else if constexpr (F == Field::Normal){
                    return std::array<float, 3>{
                        cloud.points[idx].normal_x,
                        cloud.points[idx].normal_y,
                        cloud.points[idx].normal_z};
                } else if constexpr (F == Field::Normal_color){
                    return std::array<float, 3>{
                        static_cast<float>(cloud.points[idx].normal_x + 1.0f)/2.0f,
                        static_cast<float>(cloud.points[idx].normal_y + 1.0f)/2.0f,
                        static_cast<float>(cloud.points[idx].normal_z + 1.0f)/2.0f};
                } else if constexpr (F == Field::Confidence){
                    return cloud.points[idx].confidence;
                } else if constexpr (F == Field::Confidence_color){
                    switch (cloud.points[idx].confidence)
                    {
                    case 0:
                        return tg::color3(1.0, 0.0, 0.0);
                        break;
                    case 1:
                        return tg::color3(1.0, 1.0, 0.0);
                        break;
                    case 2:
                        return tg::color3(0.0, 1.0, 0.0);
                        break;
                    default:
                        break;
                    }
                } else if constexpr (F == Field::Lables){
                    return cloud.points[idx].label;
                } else if constexpr (F == Field::Lables_color){
                    return linkml::get_color_forom_angle(linkml::sample_circle(cloud.points[idx].label));
                } else if constexpr (F == Field::Semantic){
                    return cloud.points[idx].semantic;
                } else if constexpr (F == Field::Semantic_color){
                    return linkml::get_color_forom_angle(linkml::sample_circle(cloud.points[idx].semantic));
                } else if constexpr (F == Field::Instance){
                    return static_cast<int>(cloud.points[idx].instance);
                } else if constexpr (F == Field::Instance_color){
                    return linkml::get_color_forom_angle(linkml::sample_circle(cloud.points[idx].instance));
                } else if constexpr (F == Field::Importance){
                    return (cloud.points[idx].confidence+0.01f)
                                *
                                ( (cloud.points[idx].label == 0 ? 0.1f : 1.0f)
                                + (cloud.points[idx].semantic == 0 ? 0.1f : 2.0f)
                                + (cloud.points[idx].instance == 0 ? 0.1f : 3.0f));
            }
        }
    };

}
