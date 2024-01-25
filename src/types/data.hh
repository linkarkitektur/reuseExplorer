#pragma once
#include <any>
#include <map>
#include <Eigen/Dense>
#include <typed-geometry/tg.hh>
#include <typed-geometry/tg-std.hh>
#include <opencv4/opencv2/opencv.hpp>


namespace linkml{

    enum Field {
        COLOR,
        DEPTH,
        CONFIDENCE,
        ODOMETRY,
        IMU,
        POSES,
    };

//     std::ostream& operator<<(std::ostream& lhs, Field e) {
//     switch(e) {
//         case COLOR: lhs << "COLOR"; break;
//         case DEPTH: lhs << "DEPTH"; break;
//         case CONFIDENCE: lhs << "CONFIDENCE"; break;
//         case ODOMETRY: lhs << "ODOMETRY"; break;
//         case IMU: lhs << "IMU"; break;
//     }
//     return lhs;
// }


    template <int F>
    struct FieldType;

    template <>
    struct FieldType<Field::COLOR> {
        using type = cv::Mat;
    };

    template <>
    struct FieldType<Field::DEPTH> {
        using type = Eigen::MatrixXd;
    };

    template <>
    struct FieldType<Field::CONFIDENCE> {
        using type = Eigen::MatrixXd;
    };

    template <>
    struct FieldType<Field::ODOMETRY> {
        using type = Eigen::MatrixXd;
    };

    template <>
    struct FieldType<Field::IMU> {
        using type = Eigen::MatrixXd;
    };

    template <>
    struct FieldType<Field::POSES> {
        using type = tg::dmat4x4;
    };




    class Data
    {
    private:
        std::map<Field, std::any> _data;

    public:
        Data() = default;

        template <Field F>
        typename FieldType<F>::type get() const {
            using ValueType = typename FieldType<F>::type;
            return std::any_cast<ValueType>(_data.at(F));
        }

        template <Field F>
        void set(typename FieldType<F>::type value) {
            _data[F] = value;
        }

        auto begin() const { return _data.begin(); }
        auto end() const { return _data.end(); }

    };

}