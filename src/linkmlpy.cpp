#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/iostream.h>

#include <linkml.hh>

#include <eigen3/Eigen/Core>
#include <sstream>
#include <string>

#define PYBIND11_DETAILED_ERROR_MESSAGES

namespace py = pybind11;
using namespace pybind11::literals;




PYBIND11_MODULE(linkml_py, m) {

    // Header
    m.doc() = R"pbdoc(
        LINK Python Plugin
        -----------------------

        This is allows for the segmentation of point clouds in python.
    )pbdoc";


    // Attributes
    //    m.attr("default_params") = linkml::plane_fitting_parameters();

    PYBIND11_NUMPY_DTYPE(tg::pos3, x, y, z);
    PYBIND11_NUMPY_DTYPE(tg::vec3, x, y, z);

    /// Classes

    // Dastatset
    /// @brief A handle for accessing raw data, like the strayscanner export.
    py::class_<linkml::Dataset>(m, "Dataset")
        .def(py::init<const std::string &>())
        .def("fields", &linkml::Dataset::fields)
        .def("intrinsic_matrix", &linkml::Dataset::intrinsic_matrix)
        .def("__bool__", &linkml::Dataset::operator bool)
        .def("__getitem__", &linkml::Dataset::operator[])
        .def("__getitem__", [](const linkml::Dataset &d, py::slice slice) {
            py::list list;
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(d.size(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();
            
            for (size_t i = start; i < stop; i += step){
                list.append(d[i]);
            }
            return list;}, "Get a data package", "slice"_a)
        .def("__len__", &linkml::Dataset::size)
        .def_property_readonly("size", &linkml::Dataset::size)
        .def_property_readonly("color_size", &linkml::Dataset::color_size)
        .def_property_readonly("depth_size", &linkml::Dataset::depth_size)
        ;

    /// @brief Data is the indevidual frames that the dataset provieds.
    /// Think of the data-set as a clollection of data packages.
    py::class_<linkml::Data>(m, "Data")
        .def_property_readonly("color", [](const linkml::Data & d){return d.get<linkml::Field::COLOR>();})
        .def_property_readonly("depth", [](const linkml::Data & d){return d.get<linkml::Field::DEPTH>();})
        .def_property_readonly("confidence", [](const linkml::Data & d){return d.get<linkml::Field::CONFIDENCE>();})
        .def_property_readonly("odometry", [](const linkml::Data & d){return d.get<linkml::Field::ODOMETRY>();})
        .def_property_readonly("imu", [](const linkml::Data & d){return d.get<linkml::Field::IMU>();})
        .def_property_readonly("pose", [](const linkml::Data & d){return d.get<linkml::Field::POSES>();})
        ;

    /// @brief The collection of data types a data package can provide.
    /// They are passed to the Dataset on construction to limmit the amount of data that will be loaded.
    py::enum_<linkml::Field>(m, "Field")
        .value("COLOR", linkml::Field::COLOR)
        // .value("Color", linkml::Field::COLOR)
        .value("DEPTH", linkml::Field::DEPTH)
        // .value("Depth", linkml::Field::DEPTH)
        .value("CONFIDENCE", linkml::Field::CONFIDENCE)
        // .value("Confidence", linkml::Field::CONFIDENCE)
        .value("ODOMETRY", linkml::Field::ODOMETRY)
        // .value("Odometry", linkml::Field::ODOMETRY)
        .value("IMU", linkml::Field::IMU)
        // .value("Imu", linkml::Field::IMU)
        .value("POSES", linkml::Field::POSES)
        // .value("Poses", linkml::Field::POSES)
        .export_values();

    /// @brief CV Mat, used to hold image and Depth data
    py::class_<cv::Mat>(m, "Mat", py::buffer_protocol())
        .def_buffer([](cv::Mat &m) -> py::buffer_info {
            auto buffer = py::buffer_info();
            buffer.ptr = m.data;           /* Pointer to buffer */
            buffer.itemsize = sizeof(unsigned char); /* Size of one scalar */
            buffer.format = pybind11::format_descriptor<unsigned char>::format();  /* Python struct-style format descriptor */
            buffer.ndim = 3;          /* Number of dimensions */
            buffer.shape = {  m.rows, m.cols, m.channels() }; /* Buffer dimensions */
            buffer.strides = {
                    (long)sizeof(unsigned char) * m.channels() * m.cols,
                    (long)sizeof(unsigned char) * m.channels(),
                    (long)sizeof(unsigned char)
                }; /* Strides (in bytes) for each index */
            buffer.readonly = true;           /* Buffer is read-write */
            return buffer; 
        })
        ;

    /// @brief Matrix classe used for the other rawd data that the dataset provides.
    typedef Eigen::MatrixXd Matrix;
    typedef Matrix::Scalar Scalar;
    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;
    py::class_<Matrix>(m, "Matrix", py::buffer_protocol())
        .def_buffer([](Matrix &m) -> py::buffer_info {
            auto buffer = py::buffer_info();
            buffer.ptr = m.data();           /* Pointer to buffer */
            buffer.itemsize = sizeof(Scalar);/* Size of one scalar */
            buffer.format = py::format_descriptor<Scalar>::format(); /* Python struct-style format descriptor */
            buffer.ndim = 2;          /* Number of dimensions */
            buffer.shape = { m.rows(), m.cols() }; /* Buffer dimensions */
            buffer.strides = {
                    sizeof(Scalar) * (rowMajor ? m.cols() : 1),
                    (long)sizeof(Scalar) * (rowMajor ? 1 : m.rows())
                }; /* Strides (in bytes) for each index */
            buffer.readonly = true;           /* Buffer is read-write */
            return buffer; 
        })
        ;
 

    /// @brief PointCloud class
    py::class_<linkml::PointCloud, std::shared_ptr<linkml::PointCloud>>(m, "PointCloud")
        .def(py::init<const std::string &>(), "Load a point cloud from disk"
            "path"_a)
        .def_static("load", &linkml::PointCloud::load, "Load a point cloud from disk"
            "path"_a)
        .def("save", &linkml::PointCloud::save, "Save a point cloud to disk"
            "output_file"_a,
            "binary"_a = true)
        .def("filter", &linkml::PointCloud::filter, "Filter the point cloud")
        .def("downsample", &linkml::PointCloud::downsample, "Downsample the point cloud" "leaf_size"_a=0.02)
        .def("bbox", &linkml::PointCloud::get_bbox, "Get the bounding box of the point cloud")
        .def("display",&linkml::PointCloud::display, "Display the point cloud"
            "name"_a = "Cloud")
        .def("__len__", &linkml::PointCloud::size)
        ;

    /// @brief Collection of point clouds in memory class
    py::class_<linkml::PointCloudsInMemory>(m, "PointCloudsInMemory")
        .def(py::init<const std::string &>(), "Load a point cloud from disk"
            "path"_a)
        .def_static("load", &linkml::PointCloudsInMemory::load, "Load a point cloud from disk"
            "path"_a)
        .def("filter", &linkml::PointCloudsInMemory::filter, "Filter the point cloud")
        .def("register", &linkml::PointCloudsInMemory::register_clouds, "Register the point clouds")
        .def("merge", &linkml::PointCloudsInMemory::merge, "Merge the point clouds")
        .def("__getitem__", [](const linkml::PointCloudsInMemory &obj, std::size_t index) {
            return obj[index]; }, "Get a point cloud", "index"_a)
        .def("__getitem__", [](const linkml::PointCloudsInMemory &obj, py::slice slice) {
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(obj.size(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();
            return obj[std::slice(start, stop, step)];}, "Get a point cloud", "slice"_a)
        .def("__len__", &linkml::PointCloudsInMemory::size)
        ; 

    /// @brief Collection of point clouds on disk class
    py::class_<linkml::PointCloudsOnDisk>(m, "PointCloudsOnDisk")
        .def(py::init<const std::string &>(), "Load a point cloud from disk"
            "path"_a)
        .def_static("load", &linkml::PointCloudsOnDisk::load, "Load a point cloud from disk"
            "path"_a)
        .def("filter", &linkml::PointCloudsOnDisk::filter, "Filter the point cloud")
        .def("register", &linkml::PointCloudsOnDisk::register_clouds, "Register the point clouds")
        .def("merge", &linkml::PointCloudsOnDisk::merge, "Merge the point clouds")
        .def("__getitem__", [](const linkml::PointCloudsOnDisk &obj, std::size_t index) {
            return obj[index]; }, "Get a point cloud", "index"_a)
        .def("__getitem__", [](const linkml::PointCloudsOnDisk &obj, py::slice slice) {
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(obj.size(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();
            return obj[std::slice(start, stop, step)];}, "Get a point cloud", "slice"_a)
        .def("__len__", &linkml::PointCloudsOnDisk::size)
        ;

    /// @brief Plane class
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
 
    /// @brief Position
    py::class_<tg::pos3>(m, "Pos")
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

    /// @brief Vector
    py::class_<tg::vec3>(m, "Vec")
        .def(py::init<const float, const float, const float>())
        .def(("__repr__"), [](const tg::vec3 &v){
            std::stringstream ss;;
            ss << "Vec X=" << v.x << " y="<< v.y << " z=" << v.z;
            return ss.str();
        })
        ;

    /// @brief Direction, Normalised vector
    py::class_<tg::dir3>(m, "Dir")
        .def(py::init<const float, const float, const float>())
        .def(("__repr__"), [](const tg::dir3 &v){
            std::stringstream ss;;
            ss << "Dir X=" << v.x << " y="<< v.y << " z=" << v.z;
            return ss.str();
        })
        .def_property_readonly("valid", [](const tg::dir3 &v){ return tg::normalize_safe((tg::vec3)v) !=  tg::vec3::zero;
         })
        ;


    /// @brief Speckle, class to access interopability with the Speckle API
    py::class_<linkml::speckle>(m, "Speckle")
        .def(py::init<std::string, std::string>())
        .def("__repr__", [](linkml::speckle &a){
            return a.get_status();
        })
        ;


   //py::class_<linkml::result_fit_plane>(m, "PlaneFittingResult")
    //    .def(py::init<const int &>())
    //    .def(py::init<const linkml::Plane &, const std::vector<int> &>())
    //    .def_property_readonly("valid", [](linkml::result_fit_plane &r){return r.valid;})
    //    .def_property_readonly("plane", [](linkml::result_fit_plane &r){return r.plane;})
    //    .def_property_readonly("indecies", [](linkml::result_fit_plane &r){return r.indecies;})
    //    .def_property_readonly("index", [](linkml::result_fit_plane &r){return r.index;})
    //    .def("__repr__", [](const linkml::result_fit_plane &a){
    //        std::stringstream ss;
    //        if (a.valid){
    //            ss << "Plane Result A("<< a.plane.normal.x<<") B(" <<a.plane.normal.y << ") C(" <<a.plane.normal.z << ") D(" << a.plane.dis << ")";
    //        }
    //        else{
    //            ss << "Invalid Result, the starting index was :" << a.index;
    //        }
    //        return ss.str();
    //    })
    //    ;


    //py::class_<linkml::result_fit_planes>(m, "PlaneFittingResults")
    //    .def("from_numpy", [](std::vector<linkml::Plane> const & planes, std::vector<std::vector<int>> const & indecies ){

    //        auto res = linkml::result_fit_planes();

    //        res.planes = planes;
    //        res.indecies = indecies;

    //        return  res;
    //    })
    //    .def_property_readonly("planes", [](linkml::result_fit_planes &r){return r.planes;})
    //    .def_property_readonly("indecies", [](linkml::result_fit_planes &r){return r.indecies;})
    //    .def("__repr__", [](const linkml::result_fit_planes &a){
    //        std::stringstream ss;
    //        ss << "Planes :" << a.planes.size();
    //        return ss.str();
    //    })
    //    ;


   //py::class_<linkml::CellComplex>(m, "CellComplex")
   //     .def_property_readonly("box", [](linkml::CellComplex &c){return c.pos.aabb();})
   //     .def_property_readonly("vertecies", [](linkml::CellComplex &c){return c.ps_vertecies();})
   //     .def_property_readonly("faces", [](linkml::CellComplex &c){return c.faces();})
   //     .def("__repr__", [](linkml::CellComplex &cw){
   //         std::stringstream ss;
   //         ss << "CellComplex :" << cw.ps_faces().size();
   //         return ss.str();
   //     })
   //     ;

   //py::class_<tg::mat<4,4,double>>(m, "Pos")
   //     .def("__str__", [](const tg::mat<4,4,double> &m){
   //         std::stringstream ss;
   //         ss << m;
   //         return ss.str();
   //     })
   //     .def("__repr__", [](const tg::mat<4,4,double> &m){
   //         std::stringstream ss;
   //         ss << m;
   //         return ss.str();
   //     })
   //     ;





    // Functions
    m.def("parse_dataset", &linkml::parse_Dataset, "Parse a StrayScanner scan in to a point cloud"
        "dataset"_a,
        "output_path"_a,
        "start"_a = 0,
        "stop"_a = nullptr,
        "step"_a = 5
    );



    m.def("create_cell_complex", &linkml::create_cell_complex, "point_cloud"_a, "plane_fit_results"_a );
    m.def("refine", [](
        linkml::PointCloud::Ptr const cloud, 
        std::vector<pcl::PointIndices> const & clusters, 
        float angle, 
        float dist ){return linkml::refine(cloud, clusters, tg::degree(angle), dist);}, 
        "cloud"_a, "clusters"_a, "angle_threashhold_degree"_a =25, "distance_threshhold"_a = 0.5);
    m.def("clustering", &linkml::clustering, "point_cloud"_a, "fit_plane_results"_a);
    //m.def("read", &linkml::parse_input_files, "Parse a StrayScanner scan in to a point cloud"
    //    "path"_a,
    //    "start"_a = size_t(0),
    //    "step"_a = size_t(5),
    //    "n_frames"_a = size_t(0),
    //    "inference"_a = true
    //);
    m.def("region_growing", &linkml::region_growing, "Region growing"
        "Point cloud"_a,
        "minClusterSize"_a = 2*(1/0.02)*(1/0.02),
        "numberOfNeighbours"_a = 30,
        "smoothnessThreshold"_a =  3.0 / 180.0 * M_PI,
        "curvatureThreshold"_a = 0.1
    );
} // PYBIND11_MODULE(linkml_py, m) 