#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <linkml.h>
#include <eigen3/Eigen/Core>


namespace py = pybind11;

typedef Eigen::MatrixXf Matrix;
typedef Matrix::Scalar Scalar;
constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;


PYBIND11_MODULE(linkml_py, m) {
    m.doc() = R"pbdoc(
        LINK Python Plugin
        -----------------------

        This is allows for the segmentation of point clouds in python.
    )pbdoc";

    PYBIND11_NUMPY_DTYPE(tg::pos3, x, y, z);
    PYBIND11_NUMPY_DTYPE(tg::vec3, x, y, z);


    py::class_<Matrix>(m, "Matrix", py::buffer_protocol())
        .def(py::init([](py::buffer b) {
            typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> Strides;

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some basic validation checks ... */
            if (info.format != py::format_descriptor<Scalar>::format())
                throw std::runtime_error("Incompatible format: expected a double array!");

            if (info.ndim != 2)
                throw std::runtime_error("Incompatible buffer dimension!");

            auto strides = Strides(
                info.strides[rowMajor ? 0 : 1] / (py::ssize_t)sizeof(Scalar),
                info.strides[rowMajor ? 1 : 0] / (py::ssize_t)sizeof(Scalar));

            auto map = Eigen::Map<Matrix, 0, Strides>(
                static_cast<Scalar *>(info.ptr), info.shape[0], info.shape[1], strides);

            return Matrix(map);
        }));


    py::class_<std::vector<tg::pos3>>(m, "Points", py::buffer_protocol())
        .def(py::init([](py::buffer b) {

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some basic validation checks ... */
            if (info.format != py::format_descriptor<float>::format())
                throw std::runtime_error("Incompatible format: expected a float array!");

            if (info.ndim != 2)
                throw std::runtime_error("Incompatible buffer dimension!");

            std::vector<tg::pos3> output = std::vector<tg::pos3>();

            return output;

        }));

    py::class_<std::vector<tg::vec3>>(m, "Normals", py::buffer_protocol())
        .def(py::init([](py::buffer b) {

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some basic validation checks ... */
            if (info.format != py::format_descriptor<float>::format())
                throw std::runtime_error("Incompatible format: expected a float array!");

            if (info.ndim != 2)
                throw std::runtime_error("Incompatible buffer dimension!");


            std::vector<tg::vec3> output = std::vector<tg::vec3>();

            return output;

        }));

    py::class_<linkml::point_cloud>(m, "PointCloud");



    m.def("create_point_cloud", [](Matrix points, Matrix normals ){
//            const std::vector<tg::pos3> *points_vec = reinterpret_cast<std::vector<tg::pos3* >>(points.data());
//            const std::vector<tg::vec3> *normals_vec = reinterpret_cast<std::vector<tg::vec3* >>(normals.data()).;
//        return linkml::point_cloud( &points_vec, &normals_vec);
    }, R"pbdoc(
        Create a point cloud from two numpy arrays.
    )pbdoc");

    m.def("create_point_cloud2", [](const py::array_t<tg::pos3>& points, const py::array_t<tg::vec3>& normals ){

//            auto *points_ = points.unchecked<2>().data();
//            auto *normals_ = normals.unchecked<2>().data();




//            std::vector<tg::pos3> pos = std::vector<tg::pos3>(points_);

//            std::vector<tg::pos3> vec;


            //            return linkml::point_cloud((std::vector<tg::pos3>)points, (std::vector<tg::vec3>)normals);
                return NULL;
        }, R"pbdoc(
        Create a point cloud from two numpy arrays.
    )pbdoc");



}
