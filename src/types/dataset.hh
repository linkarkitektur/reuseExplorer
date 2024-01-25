#pragma once
#include "./data.hh"

#include <filesystem>
#include <initializer_list>
#include <vector>
#include <set>
#include <map>
#include <any>
#include <Eigen/Dense>
#include <typed-geometry/tg-std.hh>


namespace linkml {

    class Dataset
    {
    private:
        std::filesystem::path               _path              = {};
        std::set<Field>                     _fields            = {};
        std::vector<std::filesystem::path>  _depth_paths       = {};
        std::vector<std::filesystem::path>  _confidence_paths  = {};
        Eigen::MatrixXd                     _odometry_data     = {};
        Eigen::MatrixXd                     _imu_data          = {};
        int                                 _n_frames          = 0;
        int                                 _rgb_width         = 1920;
        int                                 _rgb_hight         = 1440;


    



    public:
        Dataset(std::filesystem::path path, std::initializer_list<Field> fields);

        // Getters
        std::set<Field>                     fields()            const {return _fields;};
        tg::dmat3                           intrinsic_matrix()  const;
    


        // Operators
        operator bool() const {return !_fields.empty();};
        Data operator[] (int idx);
    };

} // namespace linkml