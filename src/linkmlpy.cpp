#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h>

#include <linkml.h>

#include <eigen3/Eigen/Core>
#include <sstream>
#include <string>

#define PYBIND11_DETAILED_ERROR_MESSAGES

namespace py = pybind11;
using namespace pybind11::literals;

typedef Eigen::MatrixXf Matrix;
typedef Matrix::Scalar Scalar;




typedef tg::pos3 PosTest;


PYBIND11_MODULE(linkml_py, m) {

    // Headder
    m.doc() = R"pbdoc(
        LINK Python Plugin
        -----------------------

        This is allows for the segmentation of point clouds in python.
    )pbdoc";


    // Attributes
    //    m.attr("default_params") = linkml::plane_fitting_parameters();

    PYBIND11_NUMPY_DTYPE(tg::pos3, x, y, z);
    PYBIND11_NUMPY_DTYPE(tg::vec3, x, y, z);

    // Classes
    py::class_<linkml::point_cloud>(m, "PointCloud")
        .def(py::init<const std::vector<tg::pos3> &, const std::vector<tg::vec3> &>(), py::return_value_policy::automatic_reference)
        .def("from_numpy", [](py::array_t<float> const & points, py::array_t<float> const & normals ){

            auto points_ = points.unchecked<2>();
            auto normals_ = normals.unchecked<2>();

            if (points_.shape(1) != 3)
                throw std::length_error("The point row size needs to be 3");
            if (normals_.shape(1) != 3)
                throw std::length_error("The normal row size needs to be 3");
            if (points_.shape(0) != normals_.shape(0))
                throw std::length_error("Points and Normals need to have the same length");


            py::buffer_info points_info = points.request();
            tg::pos3 *points_ptr = static_cast<tg::pos3 *>(points_info.ptr);

            py::buffer_info normals_info = normals.request();
            tg::pos3 *normals_ptr = static_cast<tg::pos3 *>(normals_info.ptr);


            auto pos_vec = std::vector<tg::pos3>();
            auto norm_vec = std::vector<tg::vec3>();

            pos_vec.reserve(points_.shape(0));
            norm_vec.reserve(normals_.shape(0));

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
        })
        .def("from_numpy", [](py::array_t<float> const & points, py::array_t<float> const & normals, py::array_t<float> const & colors ){

            auto points_ = points.unchecked<2>();
            auto normals_ = normals.unchecked<2>();
            auto colors_ = colors.unchecked<2>();


            if (points_.shape(1) != 3)
                throw std::length_error("The point row size needs to be 3");
            if (normals_.shape(1) != 3)
                throw std::length_error("The normal row size needs to be 3");
            if (points_.shape(0) != normals_.shape(0))
                throw std::length_error("Points and Normals need to have the same length");
            if (points_.shape(0) != colors_.shape(0))
                throw std::length_error("Points and Colors need to have the same length");


            py::buffer_info points_info = points.request();
            tg::pos3 *points_ptr = static_cast<tg::pos3 *>(points_info.ptr);

            py::buffer_info normals_info = normals.request();
            tg::pos3 *normals_ptr = static_cast<tg::pos3 *>(normals_info.ptr);

            py::buffer_info colors_info = normals.request();
            tg::pos3 *colors_ptr = static_cast<tg::pos3 *>(colors_info.ptr);


            auto pos_vec = std::vector<tg::pos3>();
            auto norm_vec = std::vector<tg::vec3>();
            auto color_vec = std::vector<tg::color3>();


            pos_vec.reserve(points_.shape(0));
            norm_vec.reserve(normals_.shape(0));
            color_vec.reserve(normals_.shape(0));


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
                 color_vec.push_back(
                     tg::color3(
                         *colors_.data(i,0),
                         *colors_.data(i,1),
                         *colors_.data(i,2)
                         ));

             };

            return linkml::point_cloud(pos_vec, norm_vec, color_vec);
        })
        .def("__repr__", [](const linkml::point_cloud &a){
            std::stringstream ss;
            ss << "Point cloud (" << a.pts.size() << "," << a.norm.size() << ")";
            return ss.str();
            //std::format("Point cloud {}, {}",a.pts.size() ,a.norm.size());
        })
        .def("buildIndex", &linkml::point_cloud::buildIndex)
        .def("radiusSearch", py::overload_cast<const int, const float>(&linkml::point_cloud::radiusSearch, py::const_), "Radius search")
        .def("radiusSearch", py::overload_cast<const tg::pos3  &, const float>(&linkml::point_cloud::radiusSearch, py::const_), "Radius search")
        ;
    py::class_<tg::pos3>(m, "pos")
        .def(py::init<const float, const float, const float>())
        .def(("__repr__"), [](const tg::pos3 &p){
            std::stringstream ss;;
            ss << "Pos X=" << p.x << " y="<< p.y << " z=" << p.z;
            return ss.str();
        })
        .def_readwrite("x", &tg::pos3::x)
        .def_readwrite("y", &tg::pos3::y)
        .def_readwrite("z", &tg::pos3::z)
        ;
    py::class_<tg::triangle3>(m, "triangle")
        .def(py::init<const tg::pos3, const tg::pos3, const tg::pos3>())
        .def(("__repr__"), [](const tg::triangle3 &t){
            std::stringstream ss;;
            ss << "Triangle "
                <<"P1("<< t.pos0.x <<","<< t.pos0.y<<","<<t.pos0.z<<") "
                <<"P2("<< t.pos1.x <<","<< t.pos1.y<<","<<t.pos1.z<<") "
                <<"P2("<< t.pos2.x <<","<< t.pos2.y<<","<<t.pos2.z<<")";
            return ss.str();
        })
        .def_readwrite("pos0", &tg::triangle3::pos0)
        .def_readwrite("pos1", &tg::triangle3::pos1)
        .def_readwrite("pos2", &tg::triangle3::pos2)

        ;
    py::class_<tg::vec3>(m, "vec")
        .def(py::init<const float, const float, const float>())
        .def(("__repr__"), [](const tg::vec3 &v){
            std::stringstream ss;;
            ss << "Vec X=" << v.x << " y="<< v.y << " z=" << v.z;
            return ss.str();
        })
        ;
    py::class_<tg::dir3>(m, "dir")
        .def(py::init<const float, const float, const float>())
        .def(("__repr__"), [](const tg::dir3 &v){
            std::stringstream ss;;
            ss << "Dir X=" << v.x << " y="<< v.y << " z=" << v.z;
            return ss.str();
        })
        .def_property_readonly("valid", [](const tg::dir3 &v){ return tg::normalize_safe((tg::vec3)v) !=  tg::vec3::zero;
         })
        ;
    py::class_<linkml::Plane>(m, "Plane")
        .def(py::init<>())
        .def(py::init<const float ,const float ,const float ,const float>())
        .def(py::init<const float ,const float ,const float ,const float, const float, const float, const float>())
        .def("__repr__", [](const linkml::Plane &p){
            std::stringstream ss;
            ss << "Plane A("<< p.normal.x<<") B(" <<p.normal.y << ") C(" <<p.normal.z << ") D(" << p.dis << ")";
            return ss.str();
        })
        .def_readwrite("origin", &linkml::Plane::origin)
        .def_readonly("normal", &linkml::Plane::normal)

        ;
    py::class_<linkml::reg>(m, "Register")
        .def(py::init<const int>())
        .def("__repr__", [](const linkml::reg &a){
            std::stringstream ss;
            ss << "Register of size => " << a.mask.size();
            return ss.str();
        })
        ;
    py::class_<linkml::plane_fitting_parameters>(m, "PlaneFittingParams")
        .def(py::init<const float &,const float &,const float &,const int & >(),
             "cosalpha"_a =0.96592583,
             "normal_distance_threshhold"_a=0.05,
             "distance_threshhold"_a=0.15,
             "plane_size_threshhold"_a=500)
        .def("__repr__", [](const linkml::plane_fitting_parameters &p){
            std::stringstream ss;
            ss << "Plane fitting Parameters:" << std::endl
            << "cosalpha=" << p.cosalpha << std::endl
            << "normal_distance_threshhold=" << p.normal_distance_threshhold << std::endl
            << "distance_threshhold=" << p.distance_threshhold << std::endl
            << "plane_size_threshhold=" << p.plane_size_threshhold;
            return ss.str();
        })
        ;
    py::class_<linkml::result_fit_plane>(m, "PlaneFittingResult")
        .def(py::init<const int &>())
        .def(py::init<const linkml::Plane &, const std::vector<int> &>())
        .def_property_readonly("valid", [](linkml::result_fit_plane &r){return r.valid;})
        .def_property_readonly("plane", [](linkml::result_fit_plane &r){return r.plane;})
        .def_property_readonly("indecies", [](linkml::result_fit_plane &r){return r.indecies;})
        .def_property_readonly("index", [](linkml::result_fit_plane &r){return r.index;})
        .def("__repr__", [](const linkml::result_fit_plane &a){
            std::stringstream ss;
            if (a.valid){
                ss << "Plane Result A("<< a.plane.normal.x<<") B(" <<a.plane.normal.y << ") C(" <<a.plane.normal.z << ") D(" << a.plane.dis << ")";
            }
            else{
                ss << "Invalid Result, the starting index was :" << a.index;
            }
            return ss.str();
        })
        ;
    py::class_<linkml::speckle>(m, "Speckle")
        .def(py::init<std::string, std::string>())
        .def("__repr__", [](linkml::speckle &a){
            return a.get_status();
        })
        ;


    py::class_<linkml::result_fit_planes>(m, "PlaneFittingResults")
        .def("from_numpy", [](std::vector<linkml::Plane> const & planes, std::vector<std::vector<int>> const & indecies ){

            auto res = linkml::result_fit_planes();

            res.planes = planes;
            res.indecies = indecies;

            return  res;
        })
        .def_property_readonly("planes", [](linkml::result_fit_planes &r){return r.planes;})
        .def_property_readonly("indecies", [](linkml::result_fit_planes &r){return r.indecies;})
        .def("__repr__", [](const linkml::result_fit_planes &a){
            std::stringstream ss;
            ss << "Planes :" << a.planes.size();
            return ss.str();
        })
        ;
    py::class_<linkml::PlaneFit_Solver >(m, "PlaneFit_Solver")
        ;
    py::class_<std::vector<std::atomic_bool>>(m, "a_bool")
        ;
    py::class_<linkml::CellComplex>(m, "CellComplex")
        .def_property_readonly("box", [](linkml::CellComplex &c){return c.pos.aabb();})
        .def_property_readonly("vertecies", [](linkml::CellComplex &c){return c.ps_vertecies();})
        .def_property_readonly("faces", [](linkml::CellComplex &c){return c.faces();})
        .def("__repr__", [](linkml::CellComplex &cw){
            std::stringstream ss;
            ss << "CellComplex :" << cw.ps_faces().size();
            return ss.str();
        })
        ;
    py::class_<linkml::refinement_parameters>(m, "Refinement_Parameters")
        .def(py::init<>())
        .def_readwrite("angle_threashhold", &linkml::refinement_parameters::angle_threashhold)
        .def_readwrite("distance_threshhold", &linkml::refinement_parameters::distance_threshhold)
        ;




    // Functions
    m.def("fit_plane",
            static_cast<
              linkml::result_fit_plane(*)(
                  linkml::point_cloud const  &,
                  linkml::plane_fitting_parameters const &)> (&linkml::fit_plane),
          "point_cloud"_a,
          "params"_a);

    m.def("fit_planes", [](linkml::point_cloud const &cloud, linkml::plane_fitting_parameters const &param) {
        py::scoped_ostream_redirect stream(
            std::cout,                               // std::ostream&
            py::module_::import("sys").attr("stdout") // Python output
            );

        auto s = linkml::PlaneFit_Solver();
        return s.run(cloud, param);
    },
    "point_cloud"_a,
    "params"_a
        );

    m.def("create_cell_complex", &linkml::create_cell_complex, "point_cloud"_a, "plane_fit_results"_a );
    m.def("refine_planes", &linkml::refine, "cloud_a"_a, "fit_plane_results_a"_a, "params_a"_a);
    m.def("fit_plane_thorugh_points", 
        static_cast<
            linkml::Plane(*)(
                linkml::point_cloud const&, 
                std::vector<int> const&)> (&linkml::fit_plane_thorugh_points),
        "cloud"_a, 
        "indecies"_a);
    m.def("fit_plane_thorugh_points", 
        static_cast<
            linkml::Plane(*)(
                std::vector<tg::pos3> const&)> (&linkml::fit_plane_thorugh_points),
        "points"_a);
    m.def("clustering", &linkml::clustering, "point_cloud"_a, "fit_plane_results"_a);

    // Alternative
    // m.def("fit_plane_thorugh_points", [](const linkml::point_cloud& cloud, const std::vector<int>& indices) {
    //     return linkml::fit_plane_thorugh_points(cloud, indices);
    // }, "cloud"_a, "indices"_a);
    // m.def("fit_plane_thorugh_points", [](const std::vector<tg::pos3>& points) {
    //     return linkml::fit_plane_thorugh_points(points);
    // }, "points"_a);


//    m.def("fit_planes", &linkml::fit_planes,
//          "point_cloud"_a,
//          "params"_a
//          );

//    m.def("main", &linkml::main);

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
