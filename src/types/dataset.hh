#pragma once
#include <filesystem>
#include <initializer_list>
#include <vector>
#include <set>
#include <map>
#include <any>
#include <Eigen/Dense>


namespace linkml {

    enum Field {
        COLOR,
        DEPTH,
        CONFIDENCE,
        ODOMETRY,
        IMU
    };


    class Data : public std::map<Field, Eigen::MatrixXd>
    {};


    class Dataset
    {
    private:
        std::filesystem::path               _path              = {};
        std::set<Field>                     _fields            = {};
        std::vector<std::filesystem::path>  _depth_paths       = {};
        std::vector<std::filesystem::path>  _confidence_paths  = {};

    public:
        Dataset(std::filesystem::path path, std::initializer_list<Field> fields);

        // Getters
        std::set<Field>                    fields()            const {return _fields;};
        Eigen::MatrixXd                    intrinsic_matrix()  const;
    


        // Operators
        operator bool() const {return !_fields.empty();};
        Data operator[] (int idx);
    };

} // namespace linkml