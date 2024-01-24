#include "./dataset.hh"
#include <functions/progress_bar.hh>
#include <functions/lodepng.hh>

#include <filesystem>
#include <initializer_list>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <optional>
#include <cassert>

#include <fmt/printf.h>
#include <fmt/format.h>
#include <h5pp/h5pp.h>
#include <omp.h>

#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <opencv4/opencv2/opencv.hpp>


#define CHECK_EXSISTANCE(path, file) if (!std::filesystem::exists(path / file)){ fmt::printf("{} does not exist\n", file); break; }




template <typename T>
Eigen::MatrixXd image_to_matrix(std::vector<T> const& image, int width, int height, int channel = 0, int stride = 4){
    auto matrix = Eigen::MatrixXd(width, height);

    for (int i = 0; i < width; i++){
        for (int j = 0; j < height; j++){
            auto selection = i * height + j * stride + channel;
            matrix(i, j) = image[selection];
        }
    }

    return matrix;
}

Eigen::MatrixXd read_csv(std::ifstream & stream, char delimiter = ',', bool header = false){


    std::string line;
    std::vector<double> values;
    int rows = 0; int cols = 0;

    if (header) std::getline(stream, line);

    while (getline(stream, line))
    {
        std::stringstream line_stream(line);
        std::string cell;

        cols = 0;
        while(std::getline(line_stream, cell, delimiter)){
            values.push_back(std::stod(cell));
            cols++;
        }
        rows++;
    }


    Eigen::MatrixXd matrix(rows, cols);
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            matrix(i, j) = values[i * cols + j];
        }
    }


    return matrix;
}

Eigen::MatrixXd fram_to_matrix(cv::Mat const& frame){

    auto rows = frame.rows;
    auto cols = frame.cols;
    auto dims = frame.channels();

    auto matrix = Eigen::MatrixXd(rows, cols * dims);

    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols * dims; j++){
            matrix(i, j) = frame.at<uchar>(i, j);
        }
    }

    return matrix;
}

std::optional<Eigen::MatrixXd> read_frame_at_index(std::filesystem::path const& path, int idx){

    cv::VideoCapture cap(path);
    cap.set(cv::CAP_PROP_POS_FRAMES,idx);

    cv::Mat frame;
    bool sucsess = cap.read(frame);

    if (!sucsess){
        fmt::print("Failed to read frame at index: {}\n", idx);
        return {};
    }

    return fram_to_matrix(frame);

}

std::optional<Eigen::MatrixXd> read_camera_matrix(std::filesystem::path const& path){

    if (!std::filesystem::exists(path)){
        fmt::print("Camera matrix file does not exist\n");
        fmt::print("Path: {}\n", path.c_str());
        return {};
    }

    std::ifstream camera_matrix_stream;
    camera_matrix_stream.open(path);

    return read_csv(camera_matrix_stream, ',', false);
}

std::optional<Eigen::MatrixXd> read_odometry(std::filesystem::path const& path){

    if (!std::filesystem::exists(path)){
        fmt::print("Odometry file does not exist\n");
        fmt::print("Path: {}\n", path.c_str());

        return {};
    }

    std::ifstream odometry_matrix_stream;
    odometry_matrix_stream.open(path);

    return read_csv(odometry_matrix_stream, ',', true);

}

std::optional<Eigen::MatrixXd> read_imu(std::filesystem::path const& path){

    if (!std::filesystem::exists(path)){
        fmt::print("IMU file does not exist\n");
        fmt::print("Path: {}\n", path.c_str());

        return {};
    }

    std::ifstream imu_matrix_stream;
    imu_matrix_stream.open(path);

    return read_csv(imu_matrix_stream, ',', true);

}

std::optional<Eigen::MatrixXd> read_depth_image(std::filesystem::path const& path){


    std::vector<unsigned char> buffer1;
    unsigned width, height;

    //decode
    unsigned error = lodepng::decode(buffer1, width, height, path, LodePNGColorType::LCT_GREY, 16);

    //if there's an error, display it
    if(error) std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
    if(error) return {};

    std::vector<double> buffer2;

    for (size_t i = 0; i < buffer1.size(); i += 2) {
        uint16_t value = (buffer1[i] << 8) | buffer1[i + 1]; // Combine two bytes into a uint16_t
        buffer2.push_back(value);
    }

    return image_to_matrix(buffer2, height, width, 0, 1)  / 1000; // Convert to meters
}

std::optional<Eigen::MatrixXd> read_confidence_image(std::filesystem::path const& path){

    std::vector<unsigned char> buffer;
    unsigned width, height;

    //decode
    unsigned error = lodepng::decode(buffer, width, height, path, LodePNGColorType::LCT_GREY, 8);

    //if there's an error, display it
    if(error) std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
    if(error) return {};

    return image_to_matrix(buffer, height, width, 0, 1);
}





namespace linkml {



Dataset::Dataset(std::filesystem::path path, std::initializer_list<Field> fields) {

    // Check if directories and files exists
    assert(std::filesystem::exists(path) && fmt::format("Directory does not exist: {}", path.string()).c_str());
    _path = path;


    // Load data
    for (auto field : fields) {
        switch (field) {
            case Field::COLOR:
                CHECK_EXSISTANCE(path , "rgb.mp4");
                _fields.insert(field);
                break;
            case Field::DEPTH:
                CHECK_EXSISTANCE(path , "depth")

                // Get depth image paths
                std::transform(
                    std::filesystem::directory_iterator(path / "depth"), 
                    std::filesystem::directory_iterator(), std::back_inserter(_depth_paths), 
                    [](const auto& entry){return entry.path();});
                _fields.insert(field);
                break;

            case Field::CONFIDENCE:
                CHECK_EXSISTANCE(path , "confidence" );


                // Get depth image paths
                std::transform(
                    std::filesystem::directory_iterator(path / "confidence"), 
                    std::filesystem::directory_iterator(), std::back_inserter(_confidence_paths), 
                    [](const auto& entry){return entry.path();});
                _fields.insert(field);
                break;
            
            case Field::ODOMETRY:
                CHECK_EXSISTANCE(path , "odometry.csv" );
                _fields.insert(field);
                break;
            case Field::IMU:
                CHECK_EXSISTANCE(path , "imu.csv" );
                _fields.insert(field);
                break;

            default:
                throw std::runtime_error("Unknown field");
        }
    }
};

Eigen::MatrixXd Dataset::intrinsic_matrix() const {
    return read_camera_matrix( _path / "camera_matrix.csv").value();
}

Data Dataset::operator[] (int idx){

    // TODO: Check if idx is in range

    Data data;
    for (auto field : _fields) {
        switch (field) {
            case Field::COLOR:
                data[Field::COLOR] = read_frame_at_index(_path / "rgb.mp4", idx).value();
                break;
            case Field::DEPTH:
                data[Field::DEPTH] = read_depth_image(_depth_paths[idx]).value();
                break;
            case Field::CONFIDENCE:
                data[Field::CONFIDENCE] = read_confidence_image(_confidence_paths[idx]).value();
                break;
            case Field::ODOMETRY:
                data[Field::ODOMETRY] = read_odometry("odometry.csv").value();
                break;
            case Field::IMU:
                data[Field::IMU] = read_imu("imu.csv").value();
                break;
            default:
                throw std::runtime_error("Unknown field");
        }
    }
    return data;
}

} // namespace linkml

