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



namespace py = pybind11;


namespace linkml{

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
    std::function<tg::ray3(int, tg::pos3, tg::vec3)> get_ray_caster(int max) {

            float a = 1.0f;         // Constant term in radial distance
            float b = 0.1f;         // Linear increase term in radial distance
            // float theta0 = 0.5f;    // Constant polar angle
            // float phi0 = 0.0f;      // Constant azimuthal angle

            auto spiral_rho = std::vector<float>();

            for (int t = 0; t < max; ++t)
                    spiral_rho.push_back(float(a+b*t));

            auto func = [&](int idx, tg::pos3 pt, tg::vec3 dir){ 

                    auto spherical = toSpherical(dir);
                    spherical.comp0 = spiral_rho[idx];
                    auto cartesian = toCartesian(spherical);

                    auto ray = tg::ray3(pt, tg::normalize(cartesian) );

                    return ray;
            
            };

            return func;

    }




    pybind11::module_ clustering(linkml::point_cloud const& cloud, result_fit_planes const& results ) {

        if ( Py_IsInitialized() == 0 ) {
            py::scoped_interpreter guard{};
        }

        // created cell complex by evaluating each point for wich side it lies on for each plane

        // for each cell create an alligned bounding box

        // create a sparce matrix from python
        auto sparse = py::module_::import("scipy.sparse");
        auto np = py::module_::import("numpy");



        auto cell_map = make_cw(cloud, results);
        auto cell_map_id = std::map<int, size_t>();

        int i = 0;
        for (auto const& [id, _ ]: cell_map){
            cell_map_id[i] = id;
            i++;
        }

        auto cell_map_plane = std::map<size_t, linkml::Plane>();
        auto cell_map_polyline = std::map<size_t, tg::pgon2>();


        #pragma omp parallel for
        for (int i = 0; i< cell_map.size(); i++){

            auto id = cell_map_id[i];

            auto points = std::vector<tg::pos3>();
            std::transform(cell_map[id].begin(), cell_map[id].end(), std::back_insert_iterator(points), [&](int j){
                return cloud.pts[j];
            });

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

            // create a polyline from the indecies
            std::vector<tg::pos2> polyline;
            std::transform(indecies.begin(), indecies.end(), std::back_insert_iterator(polyline), [&](int j){
                return points2d[j];
            });
            cell_map_polyline[id] = tg::pgon2(polyline);

        }


        auto n_cells = cell_map.size();
        pybind11::object matrix = sparse.attr("lil_matrix")((n_cells,n_cells), np.attr("int8"));

        auto n_rays = 5;
        auto ray_caster = get_ray_caster(n_rays);

        #pragma omp parallel for collapse(2)
        for (int i = 0; i < cell_map.size(); i++){
            auto s_id = cell_map_id[i]; // soruce id
            auto s_plane = cell_map_plane[s_id];

            for ( int j =0; j< n_rays; j++){

                auto plane = cell_map_plane[s_id];
  
                tg::ray3 ray = ray_caster(j, plane.origin, plane.normal);

                float closest_intersection_dist = std::numeric_limits<float>::max();
                size_t closest_intersection_index = -1;


                #pragma omp parallel for reduction(min:closest_intersection_dist) reduction(min:closest_intersection_index)
                for (int k = 0; k < cell_map.size(); k++){
                    if (i == j) continue;

                    auto t_id = cell_map_id[k]; // target id

                    auto t_plane = cell_map_plane[t_id];
                    auto param = tg::intersection_parameter(ray, t_plane);
                    auto hit_pt = ray.origin + ray.dir * param[0];
                    float dist = tg::distance(s_plane.origin, hit_pt);

                    // check if hit point is inside the polygon
                    auto pgon = cell_map_polyline[t_id];
                    auto point2d = t_plane.to2d(hit_pt);
                    if (!tg::contains(pgon, point2d)) continue;
                    
                    if (dist < closest_intersection_dist){
                        closest_intersection_dist = dist;
                        closest_intersection_index = k;
                    }
                }


                if (closest_intersection_index == -1) continue;

                // Increment the matrix
                auto selector = py::make_tuple(i, closest_intersection_index);
                auto value = matrix.attr("__getitem__")(selector);
                matrix.attr("__setitem__")(selector,  value.cast<int>() + 1);
            }
        }


        pybind11::object mc = py::module_::import("markov_clustering");
        pybind11::object result = mc.attr("run_mcl")(matrix);           // run MCL with default parameters
        pybind11::object clusters = mc.attr("get_clusters")(result);    // get clusters


        return clusters;

    }
}