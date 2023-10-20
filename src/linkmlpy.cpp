#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <linkml.h>
#include <eigen3/Eigen/Core>


namespace py = pybind11;

typedef Eigen::MatrixXf Matrix;
typedef Matrix::Scalar Scalar;
//constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;


//typedef Eigen::Map<Eigen::VectorXf, 0, Eigen::InnerStride<sizeof(tg::pos3)/sizeof(float)> > PosMap;
//typedef Eigen::Map<Eigen::VectorXf, 0, Eigen::InnerStride<sizeof(tg::vec3)/sizeof(float)> > VecMap;



PYBIND11_MODULE(linkml_py, m) {

    // Headder
    m.doc() = R"pbdoc(
        LINK Python Plugin
        -----------------------

        This is allows for the segmentation of point clouds in python.
    )pbdoc";



    // Classes
    py::class_<linkml::point_cloud>(m, "PointCloud", py::return_value_policy::reference_internal);
    py::class_<linkml::reg>(m, "Register", py::return_value_policy::reference_internal);
    py::class_<linkml::plane_fitting_parameters>(m, "PlaneFittingParams", py::return_value_policy::reference_internal);
    py::class_<linkml::plane_fit_resutl>(m, "PlaneFittingResults", py::return_value_policy::reference_internal);


    // Functions
    m.def("create_point_cloud", [](const py::array_t<float> & points, const py::array_t<float>& normals ){

            auto points_ = points.unchecked<2>();
            auto normals_ = normals.unchecked<2>();

            if (points_.shape(1) != 3)
                throw std::length_error("The point row size needs to be 3");
            if (normals_.shape(1) != 3)
                throw std::length_error("The normal row size needs to be 3");
            if (points_.shape(0) != normals_.shape(0))
                throw std::length_error("Points and Normals need to have the same length");


            auto pos_vec = std::vector<tg::pos3>(points_.shape(0));
            auto norm_vec = std::vector<tg::vec3>(normals_.shape(0));

            for (long i = 0; i < points_.shape(0); i++)
            {
                pos_vec.push_back(
                    tg::pos3(
                        *points_.data(i,0),
                        *points_.data(i,1),
                        *points_.data(i,2)
                ));
                norm_vec.push_back(
                    tg::vec3(
                        *normals_.data(i,0),
                        *normals_.data(i,1),
                        *normals_.data(i,2)
                ));

            };

            return linkml::point_cloud(pos_vec, norm_vec);
        }, R"pbdoc(
        Create a point cloud from two numpy arrays.
    )pbdoc", py::return_value_policy::reference_internal);


    m.def("fit_plane", [](const py::array_t<float> & points, const py::array_t<float>& normals ){

        auto points_ = points.unchecked<2>();
        auto normals_ = normals.unchecked<2>();

        if (points_.shape(1) != 3)
            throw std::length_error("The point row size needs to be 3");
        if (normals_.shape(1) != 3)
            throw std::length_error("The normal row size needs to be 3");
        if (points_.shape(0) != normals_.shape(0))
            throw std::length_error("Points and Normals need to have the same length");


        auto pos_vec = std::vector<tg::pos3>(points_.shape(0));
        auto norm_vec = std::vector<tg::vec3>(normals_.shape(0));

        for (long i = 0; i < points_.shape(0); i++)
        {
            pos_vec.push_back(
                tg::pos3(
                    *points_.data(i,0),
                    *points_.data(i,1),
                    *points_.data(i,2)
                    ));
            norm_vec.push_back(
                tg::vec3(
                    *normals_.data(i,0),
                    *normals_.data(i,1),
                    *normals_.data(i,2)
                    ));

        };



        linkml::point_cloud cloud = linkml::point_cloud(pos_vec, norm_vec);

        linkml::reg processed_reg = linkml::reg(cloud.pts.size());
        const linkml::plane_fitting_parameters params = linkml::plane_fitting_parameters();
        py::print(cloud);
        return 0;

//        return linkml::fit_plane(
//                    cloud,
//                    processed_reg,
//                    params
//                );
          },  py::return_value_policy::reference_internal);

}



//PYBIND11_NUMPY_DTYPE(tg::pos3, x, y, z);
//PYBIND11_NUMPY_DTYPE(tg::vec3, x, y, z);

//py::class_<std::vector<tg::pos3>>(m, "Matrix", py::buffer_protocol())
//    .def(py::init([](py::buffer b) {
//        typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> Strides;

//        /* Request a buffer descriptor from Python */
//        py::buffer_info info = b.request();

//        /* Some basic validation checks ... */
//        if (info.format != py::format_descriptor<Scalar>::format())
//            throw std::runtime_error("Incompatible format: expected a double array!");

//        if (info.ndim != 2)
//            throw std::runtime_error("Incompatible buffer dimension!");

//        auto strides = Strides(
//            info.strides[rowMajor ? 0 : 1] / (py::ssize_t)sizeof(Scalar),
//            info.strides[rowMajor ? 1 : 0] / (py::ssize_t)sizeof(Scalar));


//        //            auto map = Eigen::Map<PosMap, 0, Strides>(
//        //                static_cast<Scalar *>(info.ptr), info.shape[0], info.shape[1], strides);

//        auto map = Eigen::Map<Matrix, 0, Strides>(
//            static_cast<Scalar *>(info.ptr), info.shape[0], info.shape[1], strides); //std::vector<tg::pos3>

//        return Matrix(map);
//    }));
