#include "./Dataset.hh"
#include <functions/lodepng.hh>
#include <functions/polyscope.hh>
#include <fstream>
#include <iostream>
#include <string>
#include <optional>
#include <fmt/printf.h>
#include <typed-geometry/tg-std.hh>
#include <typed-geometry/tg.hh>

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

std::optional<cv::Mat> read_frame_at_index(std::filesystem::path const& path, int idx){

    cv::VideoCapture cap(path);
    cap.set(cv::CAP_PROP_POS_FRAMES,idx);

    cv::Mat frame;
    bool sucsess = cap.read(frame);

    if (!sucsess){
        fmt::print("Failed to read frame at index: {}\n", idx);
        return {};
    }

    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

    return frame;

}

size_t get_number_of_frames(std::filesystem::path const& path){
    cv::VideoCapture cap(path);
    return cap.get(cv::CAP_PROP_FRAME_COUNT);
}

std::optional<tg::dmat3> read_camera_matrix(std::filesystem::path const& path){

    if (!std::filesystem::exists(path)){
        fmt::print("Camera matrix file does not exist\n");
        fmt::print("Path: {}\n", path.c_str());
        return {};
    }

    std::ifstream camera_matrix_stream;
    camera_matrix_stream.open(path);


    auto matrix = read_csv(camera_matrix_stream, ',', false);

    auto  mat = tg::dmat3::diag(1);

    // (int col, int row)  <-- Tyepd geometry
    // (int row, int col)  <-- Eigen

    mat(0,0) = matrix(0,0);

    mat(0,1) = matrix(1,0);
    mat(1,0) = matrix(0,1);
    mat(1,1) = matrix(1,1);

    mat(0,2) = matrix(2,0);
    mat(1,2) = matrix(2,1);
    mat(2,0) = matrix(0,2);
    mat(2,1) = matrix(1,2);
    mat(2,2) = matrix(2,2);

    // mat(0,3) = matrix(3,0);
    // mat(1,3) = matrix(3,1);
    // mat(2,3) = matrix(3,2);
    // mat(3,0) = matrix(0,3);
    // mat(3,1) = matrix(1,3);
    // mat(3,2) = matrix(2,3);
    // mat(3,3) = matrix(3,3);

    return mat;
}

std::optional<Eigen::Matrix<double,     Eigen::Dynamic, Eigen::Dynamic>> read_odometry(std::filesystem::path const& path){

    if (!std::filesystem::exists(path)){
        fmt::print("Odometry file does not exist\n");
        fmt::print("Path: {}\n", path.c_str());

        return {};
    }

    std::ifstream odometry_matrix_stream;
    odometry_matrix_stream.open(path);

    return read_csv(odometry_matrix_stream, ',', true);

}

std::optional<Eigen::Matrix<double,     Eigen::Dynamic, Eigen::Dynamic>> read_imu(std::filesystem::path const& path){

    if (!std::filesystem::exists(path)){
        fmt::print("IMU file does not exist\n");
        fmt::print("Path: {}\n", path.c_str());

        return {};
    }

    std::ifstream imu_matrix_stream;
    imu_matrix_stream.open(path);

    return read_csv(imu_matrix_stream, ',', true);

}

std::optional<Eigen::Matrix<double,     Eigen::Dynamic, Eigen::Dynamic>> read_depth_image(std::filesystem::path const& path){


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

std::optional<Eigen::Matrix<double,     Eigen::Dynamic, Eigen::Dynamic>> read_confidence_image(std::filesystem::path const& path){

    std::vector<unsigned char> buffer;
    unsigned width, height;

    //decode
    unsigned error = lodepng::decode(buffer, width, height, path, LodePNGColorType::LCT_GREY, 8);

    //if there's an error, display it
    if(error) std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
    if(error) return {};

    return image_to_matrix(buffer, height, width, 0, 1);
}


auto create_pose(Eigen::Vector3d const& p, Eigen::Vector4d const& q){
    tg::pos3 pos(p(0), p(1), p(2));
    tg::dquat quat(q(0), q(1), q(2), q(3));

    auto mat = static_cast<tg::dmat4x4>(quat);
    mat.set_col(3, tg::dvec4(pos, 1));
    return mat;
}


namespace linkml {

Dataset::Dataset(const std::filesystem::path & path, const std::initializer_list<Field> & fields) {

    // Check if directories and files exists
    assert(std::filesystem::exists(path) && fmt::format("Directory does not exist: {}", path.string()).c_str());
    _path = path;

    _n_frames = get_number_of_frames(_path / "rgb.mp4");

    // TODO: Rather implement this as lazy loading.

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
                std::sort(_depth_paths.begin(), _depth_paths.end());
                _fields.insert(field);
                break;

            case Field::CONFIDENCE:
                CHECK_EXSISTANCE(path , "confidence" );

                // Get depth image paths
                std::transform(
                    std::filesystem::directory_iterator(path / "confidence"), 
                    std::filesystem::directory_iterator(), std::back_inserter(_confidence_paths), 
                    [](const auto& entry){return entry.path();});
                std::sort(_confidence_paths.begin(), _confidence_paths.end());
                _fields.insert(field);
                break;

            case Field::POSES:
                _fields.insert(field);
                // Chontious fall thorugh, since poses are dependent on Odemetry

            case Field::ODOMETRY:
                CHECK_EXSISTANCE(path , "odometry.csv" );

                _odometry_data = read_odometry(_path /"odometry.csv").value();
                _fields.insert(field);
                break;
                


            case Field::IMU:
                CHECK_EXSISTANCE(path , "imu.csv" );

                _imu_data = read_imu(_path /"imu.csv").value();
                _fields.insert(field);
                break;
            default:
                throw std::runtime_error("Unknown field");
        }
    }
};

tg::dmat3 Dataset::intrinsic_matrix() const {
    auto matrix = read_camera_matrix( _path / "camera_matrix.csv").value();

    // Scale intrinsic matrix to match depth image size
    auto scale_x = (double)_depth_width / (double)_rgb_width;
    auto scale_y = (double)_depth_hight / (double)_rgb_hight;

    matrix(0,0) = matrix(0,0) * scale_x;
    matrix(1,1) = matrix(1,1) * scale_y;
    matrix(2,0) = matrix(2,0) * scale_x;
    matrix(2,1) = matrix(2,1) * scale_y;

    return matrix;
}

Data Dataset::operator[] (int idx) const {

    // TODO: Check if idx is in range


    Data data;
    for (auto field : _fields) {
        switch (field) {
            case Field::COLOR:
                data.set<Field::COLOR>(read_frame_at_index(_path / "rgb.mp4", idx).value());
                break;
            case Field::DEPTH:
                data.set<Field::DEPTH>(read_depth_image(_depth_paths[idx]).value());
                break;
            case Field::CONFIDENCE:
                data.set<Field::CONFIDENCE>(read_confidence_image(_confidence_paths[idx]).value());
                break;
            case Field::ODOMETRY:
                data.set<Field::ODOMETRY>(_odometry_data.row(idx));
                break;
            case Field::POSES:
                data.set<Field::POSES>(create_pose(_odometry_data.block<1,3>(idx,2),_odometry_data.block<1,4>(idx,5) ));
                break;
            case Field::IMU:
                data.set<Field::IMU>(_imu_data.row(idx));
                break;
            default:
                throw std::runtime_error("Unknown field");
        }
    }
    return data;
}


void Dataset::display(std::string name, bool show) const {
    polyscope::myinit();
    polyscope::display<const Dataset &>(*this, name);
    if (show) polyscope::myshow();
}
    

} // namespace linkml

