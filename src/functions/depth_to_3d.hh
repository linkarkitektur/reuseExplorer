#pragma once
#include <optional>
#include <typed-geometry/tg-lean.hh>
#include "../types/point_cloud.hh"
#include "../types/data.hh"

namespace linkml {

    point_cloud depth_to_3d( 
        Eigen::MatrixXd const& depths,
        tg::dmat3 const& intrinsic_matrix,
        tg::dmat4 const& pose,
        std::optional<cv::Mat> const& image = std::nullopt
    ){

        auto point_cloud = linkml::point_cloud();
        cv::Mat colors;

        point_cloud.pts.resize(depths.rows() * depths.cols());
        point_cloud.norm.resize(depths.rows() * depths.cols());
        if (image.has_value()) {
            point_cloud.colors.resize(depths.rows() * depths.cols());
            cv::resize(image.value(), colors, cv::Size(depths.cols(), depths.rows())); // cv::INTER_LINEAR
            };

        #pragma omp parallel for collapse(2) shared(point_cloud, depths) firstprivate(intrinsic_matrix, pose)
        for (int row = 0; row < depths.rows(); row++){
            for (int col = 0; col < depths.cols(); col++){

                auto depth = depths(row,col);

                size_t index = row * depths.cols() + col;

                auto pos_h = tg::dvec3(col,row, 1);
                auto pos_n = tg::inverse(intrinsic_matrix) * pos_h;
                auto pos_3d = pos_n * depth;
                auto pos = tg::inverse(pose) * (tg::dpos3)pos_3d;


                auto normal =  (tg::dvec3)pos*-1;
                normal = tg::normalize(normal);
                normal = tg::inverse(pose) * normal;

                point_cloud.pts[index] = (tg::pos3)pos;
                point_cloud.norm[index]= (tg::vec3)normal;


                if (image.has_value()) {
                    cv::Vec3b vec = colors.at<cv::Vec3b>(row, col);
                    double r = static_cast<double>(vec[0])/256;
                    double g = static_cast<double>(vec[1])/256;
                    double b = static_cast<double>(vec[2])/256;
                    point_cloud.colors[index] = tg::color3(r,g,b);
                }

            }
        }

        return point_cloud;

    }

} // namespace linkml