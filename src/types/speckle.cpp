#include <types/speckle.hh>
#include <pybind11/embed.h>
#include <iostream>

namespace py = pybind11;


// **base_kwargs 
// base_kwargs = {
//     "applicationId":"linkml",
//     "units":Units.m
// }



linkml::speckle::speckle(std::string const & stream_url, std::string const & token){

    if ( Py_IsInitialized() == 0 ) {
        //interpreter is not running, do you work
        // py::scoped_interpreter python;
        py::scoped_interpreter guard{};
    }

    auto StreamWrapper = py::module_::import("specklepy.api").attr("wrapper").attr("StreamWrapper");

    auto wrapper = StreamWrapper(stream_url);
    auto client = wrapper.attr("get_client")(token);
    status = client.attr("__repr__")().cast<std::string>();

    client.release();

}

linkml::speckle::~speckle()
{
}




py::object PlaneXY(tg::pos3 point){

    if ( Py_IsInitialized() == 0 ) {
        py::scoped_interpreter guard{};
    }

    auto specklepy_geo = py::module_::import("specklepy.objects.geometry");

    auto Plane = specklepy_geo.attr("Plane");
    auto Point = specklepy_geo.attr("Point");
    auto Vector = specklepy_geo.attr("Vector");

    auto plane = Plane();
    plane.attr("origin") = Point( point.x, point.y, point.z );
    plane.attr("normal") = Vector(0.0, 0.0, 1.0);
    plane.attr("xdir") = Vector(1.0,0.0, 0.0);
    plane.attr("ydir") = Vector(0.0,1.0, 0.0);

    return plane;

} 

py::object ToSpeckle(tg::aabb3 const & box){

    if ( Py_IsInitialized() == 0 ) {
        py::scoped_interpreter guard{};
    }


    auto specklepy_geo = py::module_::import("specklepy.objects.geometry");
    auto Base = specklepy_geo.attr("Base");
    auto Plane = specklepy_geo.attr("Plane");
    auto Point = specklepy_geo.attr("Point");
    auto Vector = specklepy_geo.attr("Vector");
    auto Box = specklepy_geo.attr("Box");


    auto specklepy_primitive = py::module_::import("specklepy.objects.primitive");
    auto Interval = specklepy_primitive.attr("Interval");

    auto specklepy_units = py::module_::import("specklepy.objects.units");
    auto Units = specklepy_units.attr("Units");


    auto basePlane =  PlaneXY(tg::pos3(0,0,0));
    auto xSize = Interval(box.min.x,box.max.x );
    auto ySize = Interval(box.min.y,box.max.y );
    auto zSize = Interval(box.min.z,box.max.z );
    auto volume = (box.max.x - box.min.x) * (box.max.y - box.min.y) * (box.max.z - box.min.z);

    return Box(
            basePlane = basePlane,
            xSize = xSize,
            ySize = ySize,
            zSize = zSize,
            volume = volume
        );
}

py::object ToSpeckle(linkml::PointCloud const & cloud){

    if ( Py_IsInitialized() == 0 ) {
        py::scoped_interpreter guard{};
    }

    auto specklepy_geo = py::module_::import("specklepy.objects.geometry");
    auto Pointcloud = specklepy_geo.attr("Pointcloud");

    auto specklepy_units = py::module_::import("specklepy.objects.units");
    auto Units = specklepy_units.attr("Units");

    // Rotate point cloud

    // points = np.asarray(point_cloud.points).flatten().tolist() #11.025.661
    // c = np.asarray(np.asarray(pc.colors)*256, dtype=int)
    // colors = ((c[:,0] << 16) + (c[:,1] << 8) + (c[:,2])).tolist()
    // #colors = np.asarray(np.asarray(point_cloud.colors).flatten()*256, dtype=int).tolist()
        

    auto points = std::vector<float>(); // Flattened
    points.resize(cloud.size() *3);

    #pragma omp parallel for
    for (size_t i = 0; i < cloud.size(); i++){
        points[i*3] = cloud.points[i].x;
        points[i*3+1] = cloud.points[i].y;
        points[i*3+2] = cloud.points[i].z;
    }

    auto colors = std::vector<int>(); // Flattened
    colors.resize(cloud.size());

    #pragma omp parallel for
    for (size_t i = 0; i < cloud.size(); i++){
        // It might be possible to access the value with shifing based on how it is stored in the point cloud
        colors[i] = (int(cloud.points[i].r) << 16) + (int(cloud.points[i].g) << 8) + int(cloud.points[i].b);
    }
    
    auto sizes = std::vector<int>(cloud.size(), 3); // Nuber of values per point
    auto bbox = ToSpeckle(cloud.get_bbox());

    auto sp_cloud = Pointcloud();
    sp_cloud.attr("points") = points;
    sp_cloud.attr("colors") = colors;
    sp_cloud.attr("sizes") = sizes;
    sp_cloud.attr("bbox") = bbox;
    sp_cloud.attr("units") = Units.attr("m");

    sp_cloud.attr("add_chunkable_attrs").attr("points") = 31250;
    sp_cloud.attr("add_chunkable_attrs").attr("colors") = 62500;
    sp_cloud.attr("add_chunkable_attrs").attr("sizes") = 62500;

    std::string attributs[] = {"points", "colors", "sizes"};
    sp_cloud.attr("add_detachable_attrs") = attributs;

    return sp_cloud;

}
