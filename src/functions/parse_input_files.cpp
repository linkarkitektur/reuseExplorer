#include "parse_input_files.hh"
#include <types/dataset.hh>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <pcl/console/parse.h>
// #include <pcl/io/openni_grabber.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
// #include <pcl/people/ground_based_people_detection_app.h>
// #include <pcl/common/time.h>

#include <mutex>
#include <thread>


// #include <h5pp/h5pp.h>
// #include <omp.h>
// #include <string>
// #include <fstream>
// #include <filesystem>
// #include <fmt/printf.h>
// #include <optional>

// #include <vector>
// #include <any>

// #include <opencv4/opencv2/opencv.hpp>
// #include <functions/progress_bar.hh>

// #include <functions/lodepng.hh>

#include <typed-geometry/tg-std.hh>
#include <typed-geometry/tg.hh>


#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>

namespace linkml{

std::string print_matrix(Eigen::MatrixXd const& matrix, int n_rows = 10, int n_cols = 10, int precision = 3){

    // FIXME: Cell width is not correct
    // Example:
    // Matrix: 256x192
    //     |     0     1     2     3     4   ...  187   188   189   190   191 
    //     | ------------------------------------------------------------------
    //    0| 0.0234 0.0195 0.0195 0.0195 0.0195   ...0.0234 0.0234 0.0234 0.0234 0.0234 
    //    1| 0.0234 0.0234 0.0234 0.0234 0.0234   ...0.0234 0.0234 0.0234 0.0234 0.0234 
    //    2| 0.0234 0.0234 0.0234 0.0234 0.0234   ...0.0234 0.0234 0.0234 0.0234 0.0234 
    //    3| 0.0234 0.0234 0.0234 0.0234 0.0234   ...0.0156 0.0156 0.0156 0.0156 0.0156 
    //    4| 0.0234 0.0195 0.0195 0.0234 0.0234   ...0.0234 0.0234 0.0234 0.0234 0.0234 
                
    //  251| 0.0195 0.0195 0.0195 0.0195 0.0195   ...0.0195 0.0195 0.0195 0.0195 0.0195 
    //  252| 0.0195 0.0195 0.0195 0.0195 0.0195   ...0.0234 0.0234 0.0234 0.0234 0.0234 
    //  253| 0.0234 0.0234 0.0234 0.0273 0.0273   ...0.0195 0.0195 0.0195 0.0195 0.0195 
    //  254| 0.0195 0.0195 0.0195 0.0195 0.0195   ...0.0195 0.0195 0.0195 0.0195 0.0195 
    //  255| 0.0195 0.0195 0.0195 0.0195 0.0195   ...0.0195 0.0195 0.0195 0.0195 0.0195 


    int cols = matrix.cols();
    int rows = matrix.rows();

    auto columns_indexies = std::vector<int>();
    auto row_indexies = std::vector<int>();

    if (cols > n_cols){
        int n_cols_half = n_cols / 2;

        // First half
        for (int i = 0; i < n_cols_half; i++){
            columns_indexies.push_back(i);
        }

        // Sepparator
        columns_indexies.push_back(-1);

        // Second half
        for (int i = cols - n_cols_half; i < cols; i++){
            columns_indexies.push_back(i);
        }
    }
    else {
        for (int i = 0; i < cols; i++){
            columns_indexies.push_back(i);
        }
    }

    if (rows > n_rows){
        int n_rows_half = n_rows / 2;

        // First half
        for (int i = 0; i < n_rows_half; i++){
            row_indexies.push_back(i);
        }
        // Sepparator
        row_indexies.push_back(-1);

        // Second half
        for (int i = rows - n_rows_half; i < rows; i++){
            row_indexies.push_back(i);
        }
    }
    else {
        for (int i = 0; i < rows; i++){
            row_indexies.push_back(i);
        }
    }

    std::stringstream ss;

    ss << "Matrix: " << rows << "x" << cols << "\n";

    //Header
    ss << std::right << std::setw(7) << " " << "| ";
    for (auto const& col_index : columns_indexies){
        if (col_index == -1){
            ss << std::setw(5) << "...";
            continue;
        }
        ss << std::setw(precision+2) << std::setprecision(precision) << col_index << " ";
    }
    ss << "\n";

    // Sepparator
    ss << std::right << std::setw(7) << " " << "| ";
    for (auto const& col_index : columns_indexies){
        ss << std::setw(precision+2) << std::setprecision(precision) << "-----"  << "-";
    }
    ss << "\n";

    // Body
    for (auto const& row_index : row_indexies){


        // Row index
        if (row_index == -1) ss << " "; else  ss << std::right << std::setw(7) << row_index << "| ";

        // Cells
        for (auto const& col_index : columns_indexies){

            if (row_index == -1){
                ss << " ";
                continue;
            }

            if (col_index == -1){
                ss << std::setw(5) << "...";
                continue;
            }
            // FIXME: Cell width is not correct 

            ss << std::setw(precision+2) << std::setprecision(precision) << matrix(row_index, col_index) << " ";
        }
        
        // End of Row
        ss << "\n";
    }

    return ss.str();

}

    void parse_input_files(std::string const& path){

        auto dataset = Dataset(path, {Field::COLOR, Field::DEPTH, Field::CONFIDENCE, Field::ODOMETRY, Field::IMU, Field::POSES});

        // PCL viewer //
        // typedef pcl::PointXYZRGBA PointT;
        // typedef pcl::PointCloud<PointT> PointCloudT;
        typedef pcl::PointCloud<tg::dpos3> PointCloudT;

        polyscope::init();
        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
        // polyscope::view::setUpDir(polyscope::UpDir::ZUp);


        auto camera_intrisic_matrix = dataset.intrinsic_matrix();


        for (size_t i=0; i< 2; i+=5){
            auto data = dataset[i];

            auto depths = data.get<Field::DEPTH>();
            auto confidences = data.get<Field::CONFIDENCE>();
            auto image = data.get<Field::COLOR>();
            auto pose = data.get<Field::POSES>();

            
            cv::Mat color;
            cv::resize(image, color, cv::Size(depths.cols(), depths.rows())); // cv::INTER_LINEAR


            // auto point_cloud = PointCloudT::Ptr(new PointCloudT);
            std::vector<tg::dpos3> points;
            std::vector<tg::color3> colors;

            
            for (int row = 0; row < depths.rows(); row++){
                for (int col = 0; col < depths.cols(); col++){

                    auto depth = depths(row, col);
                    auto confidence = confidences(row, col);

                    // if (depth == 0 || confidence == 0) continue;

                    auto pos = tg::dvec3(col, row, 1);
                    pos = tg::inverse(camera_intrisic_matrix) * pos;
                    pos = pos * depth;

                    

                    auto point = tg::inverse(pose) * (tg::dpos3)pos;

                    // point_cloud->push_back(pos);


                    points.push_back(point);
                    auto color_value = color.at<cv::Vec3b>(row, col);
                    colors.push_back(tg::color3(static_cast<double>(color_value[0])/256, 
                                                static_cast<double>(color_value[1])/256, 
                                                static_cast<double>(color_value[2])/256));
                }
            }

            auto ps_cloud = polyscope::registerPointCloud("point cloud", points);
            auto ps_color = ps_cloud->addColorQuantity("color", colors);
            ps_color->setEnabled(true);
            polyscope::show();

        }


        // auto data = dataset[1];


        // for (auto&& [key, data] : data){

        //     switch (key)
        //     {
        //     case Field::COLOR:
        //         std::cout << key << "\n";
        //         // std::cout << std::any_cast<cv::Mat>(data) << "\n";
        //         break;
            
        //     default:
        //         std::cout << key << "\n";
        //         std::cout << print_matrix(std::any_cast<Eigen::MatrixXd>(data));
        //         break;
        //     }

        //     std::cout << "\n";
        // }
    }
}