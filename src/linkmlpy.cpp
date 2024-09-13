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

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

#define PYBIND11_DETAILED_ERROR_MESSAGES

namespace py = pybind11;
using namespace pybind11::literals;




PYBIND11_MODULE(_core, m) {

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
    PYBIND11_NUMPY_DTYPE(linkml::PointCloud::Cloud::PointType, x, y, z, rgb, normal_x, normal_y, normal_z, curvature, confidence, semantic, instance, label);


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
        .def_property_readonly("name", &linkml::Dataset::name)
        .def("display", &linkml::Dataset::display, "Display the dataset",
            "name"_a,
            "show"_a = true )
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

    py::enum_<linkml::Brep::BrepTrim::BrepTrimType>(m, "BrepTrimType")
        .value("Unknown", linkml::Brep::BrepTrim::BrepTrimType::Unknown)
        .value("Boundary", linkml::Brep::BrepTrim::BrepTrimType::Boundary)
        .value("Mated", linkml::Brep::BrepTrim::BrepTrimType::Mated)
        .value("Seam", linkml::Brep::BrepTrim::BrepTrimType::Seam)
        .value("Singular", linkml::Brep::BrepTrim::BrepTrimType::Singular)
        .value("CurveOnSurface", linkml::Brep::BrepTrim::BrepTrimType::CurveOnSurface)
        .value("PointOnSurface", linkml::Brep::BrepTrim::BrepTrimType::PointOnSurface)
        .value("Slit", linkml::Brep::BrepTrim::BrepTrimType::Slit)
        .export_values();

    py::enum_<linkml::Brep::BrepLoop::BrepLoopType>(m, "BrepLoopType")
        .value("Unknown", linkml::Brep::BrepLoop::BrepLoopType::Unknown)
        .value("Outer", linkml::Brep::BrepLoop::BrepLoopType::Outer)
        .value("Inner", linkml::Brep::BrepLoop::BrepLoopType::Inner)
        .value("Slit", linkml::Brep::BrepLoop::BrepLoopType::Slit)
        .value("CurveOnSurface", linkml::Brep::BrepLoop::BrepLoopType::CurveOnSurface)
        .value("PointOnSurface", linkml::Brep::BrepLoop::BrepLoopType::PointOnSurface)
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
 
    // TODO: Capture standard out on all functions that use the progress bar
    // https://pybind11.readthedocs.io/en/stable/advanced/pycpp/utilities.html#capturing-standard-output-from-ostream

    /// @brief PointCloud class
    py::class_<linkml::PointCloud>(m, "PointCloud", py::buffer_protocol())
        .def(py::init<const std::string &>(), 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def(py::init([](
            const py::array_t<float> xyz, 
            std::optional<const py::array_t<uint8_t>> rgb, 
            std::optional<const py::array_t<float>> normal, 
            std::optional<const py::array_t<int32_t>> semantic, 
            std::optional<const py::array_t<int32_t>> instance, 
            std::optional<const py::array_t<int32_t>> label) {

                /* Request a buffer descriptor from Python */
                py::buffer_info xyz_info = xyz.request();

                /* Some basic validation checks ... */
                if ( xyz_info.format != py::format_descriptor<float>::format()
                     && xyz_info.format != py::format_descriptor<double>::format())
                    throw std::runtime_error("Incompatible format: expected a float or double array!");

                if (xyz_info.ndim != 2)
                    throw std::runtime_error("Incompatible buffer dimension!");

                auto xyz_ = xyz.unchecked<2>();


                linkml::PointCloud cloud;
                (*cloud).points.resize(xyz.shape(0));
                (*cloud).height = xyz.shape(0);
                (*cloud).width = 1;
                
                for (size_t i = 0; i < (size_t)xyz_.shape(0); i++){
                    (*cloud).points[i].x = *xyz_.data(i, 0);
                    (*cloud).points[i].y = *xyz_.data(i, 1);
                    (*cloud).points[i].z = *xyz_.data(i, 2);
                }

                if (rgb.has_value()){
                    auto rgb_ = rgb.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++){
                        (*cloud).points[i].r = *rgb_.data(i, 2);
                        (*cloud).points[i].g = *rgb_.data(i, 1);
                        (*cloud).points[i].b = *rgb_.data(i, 0);
                    }
                }
                    
                

                if (normal.has_value()){
                    auto normal_ = normal.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++){
                        (*cloud).points[i].normal_x = *normal_.data(i, 0);
                        (*cloud).points[i].normal_y = *normal_.data(i, 1);
                        (*cloud).points[i].normal_z = *normal_.data(i, 2);
                    }
                }
                

                if (semantic.has_value()){
                    auto semantic_ = semantic.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++)
                        (*cloud).points[i].semantic = *semantic_.data(i, 0);
                }
                    
                

                if (instance.has_value()){
                    auto instance_ = instance.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++)
                        (*cloud).points[i].instance = *instance_.data(i, 0);
                }
                    
                

                if (label.has_value()){
                    auto label_ = label.value().unchecked<2>();
                    for (size_t i = 0; i < (size_t)xyz_.shape(0); i++)
                        (*cloud).points[i].label = *label_.data(i, 0);
                }
                    
                
                return cloud;
            }),
            "Create Point Cloud from Numpy arrays",
            "xyz"_a, 
            "rgb"_a = py::none(), 
            "normal"_a = py::none(), 
            "semantic"_a = py::none(), 
            "instance"_a = py::none(), 
            "label"_a = py::none(),
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def_static("load", &linkml::PointCloud::load,
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("save", &linkml::PointCloud::save, 
            "Save a point cloud to disk",
            "output_file"_a,
            "binary"_a = true, py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("filter", &linkml::PointCloud::filter, 
            "Filter the point cloud", 
            "value"_a = 2,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("downsample", &linkml::PointCloud::downsample, 
            "Downsample the point cloud", 
            "leaf_size"_a=0.02, 
            py::return_value_policy::reference,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("region_growing", &linkml::PointCloud::region_growing, 
            "Region growing",
            "angle_threshold"_a = float(0.96592583), // cos(25°)
            "plane_dist_threshold"_a = float(0.1),
            "minClusterSize"_a = int(2*(1/0.02)*(1/0.02)),
            "early_stop"_a = float(0.3),
            "radius"_a = float(0.1),
            "interval_0"_a = float(16),
            "interval_factor"_a = float(1.5),
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("clustering", &linkml::PointCloud::clustering, "Cluster the sematic labels in to instances",
            "cluster_tolerance"_a = 0.02,
            "min_cluster_size"_a = 100,
            "max_cluster_size"_a = std::numeric_limits<pcl::uindex_t>::max(),
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("solidify", &linkml::PointCloud::solidify, 
            "Solidify the point cloud",
            "downsample_size"_a = 5000000,
            "sx"_a = 0.4,
            "sy"_a = 0.4,
            "expand_factor"_a = 2,
            "inflate_factor"_a = 1.2,
            "max_loop"_a = 10.0,
            "mult_factor"_a = 1.0,
            "fitting"_a = 0.20,
            "coverage"_a = 0.10,
            "complexity"_a = 0.70,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("display",&linkml::PointCloud::display, 
            "Display the point cloud",
            "name"_a = "Cloud", 
            py::return_value_policy::reference_internal
        )
        .def("bbox", &linkml::PointCloud::get_bbox, 
            "Get the bounding box of the point cloud"
        )
        .def("__len__", [](linkml::PointCloud &cloud){ return cloud->size();})
        .def_buffer([](linkml::PointCloud &cloud) -> py::buffer_info {
            return py::buffer_info(
                cloud->points.data(),
                sizeof(linkml::PointCloud::Cloud::PointType),
                py::format_descriptor<linkml::PointCloud::Cloud::PointType>::format(),
                1,
                { cloud->points.size() },
                { sizeof(linkml::PointCloud::Cloud::PointType)}
            );
        })
        ;

    /// @brief Collection of point clouds in memory class
    py::class_<linkml::PointCloudsInMemory>(m, "PointCloudsInMemory")
        .def(py::init<const std::string &>(), 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def_static("load", &linkml::PointCloudsInMemory::load, 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("filter", &linkml::PointCloudsInMemory::filter, 
            "Filter the point cloud",
            "value"_a = 2,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("register", &linkml::PointCloudsInMemory::register_clouds, 
            "Register the point clouds", 
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("merge", &linkml::PointCloudsInMemory::merge, 
            "Merge the point clouds",
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("annotate", &linkml::PointCloudsInMemory::annotate, 
            "Annotate the point clouds",
            "yolo_path"_a,
            "dataset"_a = py::none(),
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("display", &linkml::PointCloudsInMemory::display, 
            "Display the point clouds",
            "show_clouds"_a = false, 
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("__getitem__", [](const linkml::PointCloudsInMemory &obj, std::size_t index) {
            return obj[index]; }, 
            "Get a point cloud", 
            "index"_a
        )
        .def("__getitem__", [](const linkml::PointCloudsInMemory &obj, py::slice slice) {
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(obj.size(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();
            return obj[std::slice(start, stop, step)];}, 
            "Get a point cloud", 
            "slice"_a
        )
        .def("__len__", &linkml::PointCloudsInMemory::size)
        ; 

    /// @brief Collection of point clouds on disk class
    py::class_<linkml::PointCloudsOnDisk>(m, "PointCloudsOnDisk")
        .def(py::init<const std::string &>(), 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def_static("load", &linkml::PointCloudsOnDisk::load, 
            "Load a point cloud from disk",
            "path"_a,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("filter", &linkml::PointCloudsOnDisk::filter, 
            "Filter the point cloud",
            "value"_a = 2,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("register", &linkml::PointCloudsOnDisk::register_clouds, 
            "Register the point clouds", 
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("merge", &linkml::PointCloudsOnDisk::merge, 
            "Merge the point clouds",
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("annotate", &linkml::PointCloudsOnDisk::annotate, 
            "Annotate the point clouds",
            "yolo_path"_a, 
            "dataset"_a = py::none(), 
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("display", &linkml::PointCloudsOnDisk::display, 
            "Display the point clouds",
            "show_clouds"_a = false,
            py::return_value_policy::reference_internal,
            py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
        )
        .def("__getitem__", [](const linkml::PointCloudsOnDisk &obj, std::size_t index) {
            return obj[index]; }, 
            "Get a point cloud", 
            "index"_a
        )
        .def("__getitem__", [](const linkml::PointCloudsOnDisk &obj, py::slice slice) {
            py::size_t start, stop, step, slicelength;
            if (!slice.compute(obj.size(), &start, &stop, &step, &slicelength))
                throw py::error_already_set();
            return obj[std::slice(start, stop, step)];}, 
            "Get a point cloud", 
            "slice"_a
        )
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

    /// @brief Position
    py::class_<tg::pos2>(m, "Pos2D")
        .def(py::init<const float, const float>())
        .def(("__repr__"), [](const tg::pos2 &p){
            std::stringstream ss;;
            ss << "Pos2D X=" << p.x << " y="<< p.y;
            return ss.str();
        })
        .def_readwrite("x", &tg::pos2::x)
        .def_readwrite("y", &tg::pos2::y)
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
    py::class_<tg::aabb3>(m, "AABB")
        .def(py::init<const tg::pos3, const tg::pos3>())
        .def("__repr__", [](const tg::aabb3 &a){
            std::stringstream ss;
            ss << "AABB min(" << a.min.x << ", " << a.min.y << ", " << a.min.z << ") max(" << a.max.x << ", " << a.max.y << ", " << a.max.z << ")";
            return ss.str();
        })
        .def("volume", [](const tg::aabb3 &a){
            return (a.max.x - a.min.x) * (a.max.y - a.min.y) * (a.max.z - a.min.z);
        })
        .def("center", [](const tg::aabb3 &a){
            return tg::pos3((a.max.x + a.min.x) / 2, (a.max.y + a.min.y) / 2, (a.max.z + a.min.z) / 2);
        })
        .def("xInterval", [](const tg::aabb3 &a){
            return std::make_tuple(a.min.x, a.max.x);
        })
        .def("yInterval", [](const tg::aabb3 &a){
            return std::make_tuple(a.min.y, a.max.y);
        })
        .def("zInterval", [](const tg::aabb3 &a){
            return std::make_tuple(a.min.z, a.max.z);
        })
        ;
    
    py::class_<linkml::Brep>(m, "Brep")
        .def("save", &linkml::Brep::save)
        .def_static("load", &linkml::Brep::load, py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>())
        .def("volume", &linkml::Brep::volume)
        .def("area", &linkml::Brep::area)
        .def("is_closed", &linkml::Brep::is_closed)
        .def("bbox", &linkml::Brep::get_bbox)

        .def("Curves2D", &linkml::Brep::get_Curves2D)
        .def("Curves3D", &linkml::Brep::get_Curves3D)

        .def("Edges", &linkml::Brep::get_Edges)
        .def("Faces", &linkml::Brep::get_Faces)
        .def("Vertices", &linkml::Brep::get_Vertices)
        .def("Surfaces", &linkml::Brep::get_Surfaces)

        .def("Loops", &linkml::Brep::get_Loops)
        .def("Trims", &linkml::Brep::get_Trims)
        .def("Orientation", &linkml::Brep::get_Orientation)
        .def("get_mesh", &linkml::Brep::get_Mesh)
        .def("display", &linkml::Brep::display, 
            "Display the Brep", 
            "name"_a = "Brep", 
            "show_mesh"_a = true)
        ;

    py::class_<linkml::Brep::Face>(m, "Face")
        .def(py::init<>())
        .def_readwrite("SurfaceIndex", &linkml::Brep::Face::SurfaceIndex)
        .def_readwrite("OuterLoopIndex", &linkml::Brep::Face::OuterLoopIndex)
        .def_readwrite("OrientationReversed", &linkml::Brep::Face::OrientationReversed)
        .def_readwrite("LoopIndices", &linkml::Brep::Face::LoopIndices)
        ;
    
    py::class_<linkml::Brep::Edge>(m, "Edge")
        .def(py::init<>())
        .def_readwrite("Curve3dIndex", &linkml::Brep::Edge::Curve3dIndex)
        .def_readwrite("TrimIndices", &linkml::Brep::Edge::TrimIndices)
        .def_readwrite("StartIndex", &linkml::Brep::Edge::StartIndex)
        .def_readwrite("EndIndex", &linkml::Brep::Edge::EndIndex)
        .def_readwrite("ProxyCurveIsReversed", &linkml::Brep::Edge::ProxyCurveIsReversed)
        .def_readwrite("Domain", &linkml::Brep::Edge::Domain)
        ;
    py::class_<linkml::Brep::Surface>(m, "Surface")
        .def(py::init<>())
        .def_readwrite("degreeU", &linkml::Brep::Surface::degreeU)
        .def_readwrite("degreeV", &linkml::Brep::Surface::degreeV)
        .def_readwrite("rational", &linkml::Brep::Surface::rational)
        .def_readwrite("area", &linkml::Brep::Surface::area)
        .def_readwrite("pointData", &linkml::Brep::Surface::pointData)
        .def_readwrite("countU", &linkml::Brep::Surface::countU)
        .def_readwrite("countV", &linkml::Brep::Surface::countV)
        .def_readwrite("bbox", &linkml::Brep::Surface::bbox)
        .def_readwrite("closedU", &linkml::Brep::Surface::closedU)
        .def_readwrite("closedV", &linkml::Brep::Surface::closedV)
        .def_readwrite("domainU", &linkml::Brep::Surface::domainU)
        .def_readwrite("domainV", &linkml::Brep::Surface::domainV)
        .def_readwrite("knotsU", &linkml::Brep::Surface::knotsU)
        .def_readwrite("knotsV", &linkml::Brep::Surface::knotsV)
        ;
    py::class_<linkml::Brep::BrepTrim>(m, "BrepTrim")
        .def(py::init<>())
        .def_readwrite("EdgeIndex", &linkml::Brep::BrepTrim::EdgeIndex)
        .def_readwrite("StartIndex", &linkml::Brep::BrepTrim::StartIndex)
        .def_readwrite("EndIndex", &linkml::Brep::BrepTrim::EndIndex)
        .def_readwrite("FaceIndex", &linkml::Brep::BrepTrim::FaceIndex)
        .def_readwrite("LoopIndex", &linkml::Brep::BrepTrim::LoopIndex)
        .def_readwrite("CurveIndex", &linkml::Brep::BrepTrim::CurveIndex)
        .def_readwrite("IsoStatus", &linkml::Brep::BrepTrim::IsoStatus)
        .def_readwrite("TrimType", &linkml::Brep::BrepTrim::TrimType)
        .def_readwrite("IsReversed", &linkml::Brep::BrepTrim::IsReversed)
        .def_readwrite("Domain", &linkml::Brep::BrepTrim::Domain)
        ;   
    py::class_<linkml::Brep::BrepLoop>(m, "BrepLoop")
        .def(py::init<>())
        .def_readwrite("FaceIndex", &linkml::Brep::BrepLoop::FaceIndex)
        .def_readwrite("TrimIndices", &linkml::Brep::BrepLoop::TrimIndices)
        .def_readwrite("Type", &linkml::Brep::BrepLoop::Type)
        ;


    py::class_<linkml::LinkMesh>(m, "Mesh")
        .def(py::init<>())
        .def("volume", &linkml::LinkMesh::volume)
        .def("area", &linkml::LinkMesh::area)
        .def("bbox", &linkml::LinkMesh::get_bbox)
        .def("vertices", &linkml::LinkMesh::get_vertices)
        .def("faces", &linkml::LinkMesh::get_faces)
        .def("colors", &linkml::LinkMesh::get_colors)
        .def("textrueCoords", &linkml::LinkMesh::get_textrueCoords)
        ;




    // TODO: Those should be moved to their respecive classes
    // Functions
    m.def("parse_dataset", &linkml::parse_Dataset, "Parse a StrayScanner scan in to a point cloud"
        "dataset"_a,
        "output_path"_a,
        "start"_a = 0,
        "stop"_a = nullptr,
        "step"_a = 5
    );

    m.def("extract_instances", &linkml::extract_instances, "Extract the instance for a point cloud"
        "point_cloud"_a);




#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif    
    
} // PYBIND11_MODULE(linkml_py, m) 