#pragma once

#include <pybind11/embed.h>
#include <types/point_cloud.h>
#include <types/result_fit_planes.h>
#include <functions/alpha_shape.h>
#include <functions/color_facetes.h>
#include <functions/project_2d.h>
#include <functions/fit_plane_thorugh_points.h>

#include <typed-geometry/types/objects/polygon.hh>
#include <typed-geometry/functions/objects/contains.hh>
#include <typed-geometry/types/objects/ray.hh>
#include <typed-geometry/types/comp.hh>

#include <embree3/rtcore.h>


namespace py = pybind11;


namespace linkml{

    // https://www.youtube.com/watch?v=FDyenWWlPdU
    tg::vec3 static toCartesian(tg::comp3 spherical){

            auto rho = spherical.comp0;
            auto theta = spherical.comp1;
            auto phi = spherical.comp2;

            // Spherical to Cartesian
            // x = rho * sin(phi) * cos(theta)
            // y = rho * sin(phi) * sin(theta)
            // Z = rho * cos(phi)


            auto x = rho * std::sin(theta) * std::cos(phi);
            auto y = rho * std::sin(theta) * std::sin(phi);
            auto z = rho * std::cos(theta);

            return tg::vec3(x,y,z);

    }
    tg::comp3 static toSpherical(tg::vec3 cartesian){

            auto x = cartesian.x;
            auto y = cartesian.y;
            auto z = cartesian.z;


            // Cartesian > Sperical

            // r = sqrt(x² + y² + z²)
            // cos(theta) = z/r
            // x² + y² = rho² * sin(theta) => sin(theta) = (sqrt(x²+Y²)/r)

            // cos(phi) = x/(r*sin(theta))
            // sin(phi) = y/(r*sin(theta)) => y / sqrt(x² + y²)

            // rho = sqrt(x² + y² + z²)
            // theta = arc_tan(y/x)
            // phi = arc_cos(z/rho)


            // auto r = CGAL::sqrt(x*x + y*y + z*z );
            // auto cos_theta = x/r;
            // auto sin_theta = CGAL::sqrt((x*x+y*y)/r);
            // auto cos_phi = x/CGAL::sqrt(x*x + y*y);
            // auto sin_phi = y/CGAL:sqrt(x*x + y*y);


            auto rho = CGAL::sqrt(x*x + y*y + z*z );
            // auto theta = std::atan(y/x);
            auto theta = (rho > 0) ? std::acos(z / rho) : FT(0.0);
            // auto phi = std::acos(z/rho);
            auto phi = std::atan2(y, x);

            // x => rho, y => theta, z => phi
            return tg::comp3(rho, theta, phi);
    }
    std::vector<tg::ray3> filter_vectors(std::vector<tg::dir3> vectors, tg::pos3 pt, tg::dir3 dir) {


        auto sub_list = std::vector<tg::ray3>();

        for (int i = 0; i < vectors.size() ; i++){
            auto v = vectors[i];
            if (tg::dot(v, dir) < 0) continue;
            sub_list.push_back(tg::ray3(pt, v));
        }

        return sub_list;

    }
    std::vector<tg::dir3> get_sphere_vectors(int n_slices = 10, int n_stacks = 10 ) {

        // Create sphere with all possible vectors
        auto vectors = std::vector<tg::dir3>();

        // add north vetcor
        vectors.push_back(tg::dir3(0, 1, 0));

        // generate vertices per stack / slice
        for (int i = 0; i < n_stacks - 1; i++)
        {
            auto phi = M_PI * double(i + 1) / double(n_stacks);
            for (int j = 0; j < n_slices; j++)
            {
            auto theta = 2.0 * M_PI * double(j) / double(n_slices);
            auto x = std::sin(phi) * std::cos(theta);
            auto y = std::cos(phi);
            auto z = std::sin(phi) * std::sin(theta);
            vectors.push_back(tg::dir3(x, y, z));
            }
        }

        // add south vetcor
        vectors.push_back(tg::dir3(0, -1, 0));

        return vectors;

    }



    pybind11::list clustering(linkml::point_cloud const& cloud, result_fit_planes const& results ) {

        if ( Py_IsInitialized() == 0 ) {
            py::scoped_interpreter guard{};
        }

        auto sparse = py::module_::import("scipy.sparse");
        auto np = py::module_::import("numpy");



        // TEST SPARSE MATRIX
        py::kwargs kwargs;
        kwargs["shape"] = py::make_tuple(10000, 10000);
        kwargs["dtype"] = np.attr("int8");
        py::function lil_matrix = sparse.attr("lil_matrix");
        pybind11::object matrix = lil_matrix(kwargs);

        auto selector = py::make_tuple(0, 0);
        matrix[selector] = matrix[selector].cast<int>() + 1;

        selector = py::make_tuple(10, 10);
        matrix[selector] = matrix[selector].cast<int>() + 1;
        // END TEST SPARSE MATRIX


        // created cell complex by evaluating each point for wich side it lies on for each plane
        auto cell_map = make_cw(cloud, results);
        auto cell_map_int_to_id = std::map<int, size_t>();
        auto cell_map_id_to_int = std::map<int, size_t>();


        int counter = 0;
        for (auto const& [id, _ ]: cell_map){
            cell_map_int_to_id[counter] = id;
            cell_map_id_to_int[id] = counter;
            counter++;
        }

        auto cell_map_plane = std::map<size_t, linkml::Plane>();
        auto cell_map_boundary_point_indecies = std::map<size_t, std::vector<size_t>>();

        #pragma omp parallel for shared(cell_map_plane, cell_map_boundary_point_indecies, cloud, cell_map)
        for (size_t i = 0; i< cell_map.size(); i++){

            auto id = cell_map_int_to_id[i];

            auto points = std::vector<tg::pos3>();
            std::transform(cell_map[id].begin(), cell_map[id].end(), std::back_insert_iterator(points), [&](int j){
                return cloud.pts[j];
            });

            if (points.size() < 3) continue;

            // get the plane that the cell is on
            auto plane = fit_plane_thorugh_points(points);

            // Ensure normal is allighned with point normals
            auto normals = std::vector<tg::vec3>();
            std::transform(cell_map[id].begin(), cell_map[id].end(), std::back_insert_iterator(normals), [&](int j){
                return cloud.norm[j];
            });
            auto average_normal = tg::average(normals);

            // flip the normal if it is not alligned with the average normal
            if (tg::dot(average_normal, plane.normal) < 0)
                plane.normal = -plane.normal;

            cell_map_plane[id] = plane;

            // project point in to 2d space
            auto points2d = plane.to2d(points);
            auto indecies = alpha_shape(points2d);
            cell_map_boundary_point_indecies[id] = indecies;

        }


 
        // create a sparce matrix from python
        auto n_cells = cell_map.size();
        // py::kwargs kwargs;
        // kwargs["dtype"] = np.attr("int8");
        // py::tuple shape = py::make_tuple(n_cells, n_cells);
        // pybind11::object matrix = sparse.attr("lil_matrix")(shape, kwargs);


        // Create Embree context and Scene
        RTCDevice device = rtcNewDevice(NULL);
        RTCScene scene   = rtcNewScene(device);

        auto genomID_map = std::map<unsigned int, size_t>();

        for (int i = 0; i < cell_map_int_to_id.size(); i++){

            auto id = cell_map_int_to_id[i];
            auto indecies = cell_map_boundary_point_indecies[id];

            if (indecies.size() < 3) continue;
            auto n_trinages = indecies.size() - 2;


            RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

            float* vb = (float*) rtcSetNewGeometryBuffer(geom,
                RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), indecies.size());
            unsigned* ib = (unsigned*) rtcSetNewGeometryBuffer(geom,
                RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(unsigned), n_trinages);


            // Add the points to the vertex buffer
            for (int j = 0; j < indecies.size(); j++){
                auto point = cloud.pts[indecies[j]];
                vb[3*j+0] = point.x;
                vb[3*j+1] = point.y;
                vb[3*j+2] = point.z;
            }

            // Add the faces to the face buffer
            for (int j = 0; j < n_trinages; j++){
                ib[3*j+0] = 0;
                ib[3*j+1] = j+1;
                ib[3*j+2] = j+2;
            }

            rtcCommitGeometry(geom);
            auto genomID = rtcAttachGeometry(scene, geom);
            rtcReleaseGeometry(geom);

            // Save the mapping between the genomID and the cell id
            genomID_map[genomID] = id;
        }

        // Commit the scene
        rtcCommitScene(scene);

        // Create all rays

        auto all_vectors = get_sphere_vectors();

        auto all_rays = std::vector<std::tuple<size_t, RTCRayHit>>(); 


        for (int i = 0; i < cell_map_int_to_id.size(); i++){
            auto id = cell_map_int_to_id[i];
            auto plane = cell_map_plane[id];

            auto rays = filter_vectors(all_vectors, plane.origin, plane.normal);

            for (auto ray : rays){
                RTCRayHit rayhit; 
                rayhit.ray.org_x = ray.origin.x; 
                rayhit.ray.org_y = ray.origin.y;
                rayhit.ray.org_z = ray.origin.z;

                rayhit.ray.dir_x = ray.dir.x;
                rayhit.ray.dir_y = ray.dir.y;
                rayhit.ray.dir_z = ray.dir.z;

                rayhit.ray.tnear  = 0.f;
                rayhit.ray.tfar   = std::numeric_limits<float>::infinity();
                rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;


                all_rays.push_back(std::make_tuple(id, rayhit));
            }
        }

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);


        // Intersect all rays
        // rtcIntersect1M(scene, &context, &all_rays[0], all_rays.size(), sizeof(std::tuple<sitz_t, RTCRayHit>) );
        for (auto [id, rayhit] : all_rays){
            rtcIntersect1(scene, &context, &rayhit);
            if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

                auto source_id = id;
                auto target_id = genomID_map[rayhit.hit.geomID];


                auto source_int = cell_map_id_to_int[source_id];
                auto target_int = cell_map_id_to_int[target_id];

                // Increment the matrix
                auto selector = py::make_tuple(source_int, target_int);
                matrix.operator[](selector) = matrix.operator[](selector).cast<int>() + 1;
                // auto value = matrix.attr("__getitem__")(selector);
                // matrix.attr("__setitem__")(selector,  value.cast<int>() + 1);

            }
        }

        // Release the scene and device
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);

        pybind11::object mc = py::module_::import("markov_clustering");
        pybind11::object result = mc.attr("run_mcl")(matrix);           // run MCL with default parameters
        pybind11::object clusters = mc.attr("get_clusters")(result);    // get clusters


        return clusters;

    }
}