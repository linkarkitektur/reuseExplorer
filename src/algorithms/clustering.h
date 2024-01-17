#pragma once

#include <pybind11/embed.h>
#include <types/point_cloud.h>
#include <types/result_fit_planes.h>
#include <functions/alpha_shape.h>
#include <functions/color_facetes.h>
#include <functions/project_2d.h>
#include <functions/fit_plane_thorugh_points.h>
#include <functions/convex_hull_2d.h>
#include <algorithms/markov_clustering.h>

#include <typed-geometry/types/objects/polygon.hh>
#include <typed-geometry/functions/objects/contains.hh>
#include <typed-geometry/types/objects/ray.hh>
#include <typed-geometry/types/comp.hh>

#include <embree3/rtcore.h>

#include <Eigen/Sparse>

typedef Eigen::SparseMatrix<float> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<float> Triplet;

# define Visualize 0

#if 1
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"

#include <functions/color.h>
#endif


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
            if (tg::dot(v, dir) > 0.1) continue;
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



    pybind11::object clustering(linkml::point_cloud const& cloud, result_fit_planes const& results ) {

        if ( Py_IsInitialized() == 0 ) {
            py::scoped_interpreter guard{};
        }

#if Visualize
        polyscope::init();
        polyscope::view::setUpDir(polyscope::view::UpDir::ZUp);
        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
#endif

        auto sparse = py::module_::import("scipy.sparse");
        auto np = py::module_::import("numpy");


#if Visualize
        // Visualize the point cloud
        auto ps_cloud = polyscope::registerPointCloud("point cloud", cloud.pts);

        // Visualize the planes
        auto plane_colors = std::vector<tg::color3>(cloud.pts.size(), tg::color3(0,0,0));
        for (int i = 0; i < results.indecies.size(); i++){
            auto indcies = results.indecies[i];
            auto color = linkml::get_color_forom_angle(linkml::sample_circle(i));
            for (auto const& index : indcies){
                plane_colors[index] = color;
            }
        }
        ps_cloud->addColorQuantity("plames", plane_colors);
#endif




        // created cell complex by evaluating each point for wich side it lies on for each plane
        auto cell_map = make_cw(cloud, results);
        auto cell_map_int_to_id = std::map<int, size_t>();
        auto cell_map_id_to_int = std::map<int, size_t>();


// Visualize the cells
#if 0
        auto cell_colors = std::vector<tg::color3>(cloud.pts.size(), tg::color3(0,0,0));
        for (auto const& [i, indcies ]: cell_map){
            auto color = linkml::get_color_forom_angle(linkml::sample_circle(i));
            for (auto const& idx : indcies){
                cell_colors[idx] = color;
            }
        }
        ps_cloud->addColorQuantity("cells", cell_colors);
#endif

        int counter = 0;
        for (auto const& [id, _ ]: cell_map){
            cell_map_int_to_id[counter] = id;
            cell_map_id_to_int[id] = counter;
            counter++;
        }

        auto cell_map_plane = std::map<size_t, linkml::Plane>();
        auto cell_map_boundary_point_indecies = std::map<size_t, std::vector<size_t>>();

        // Find alpha shape for each cell
        #pragma omp parallel for shared(cell_map_plane, cell_map_boundary_point_indecies, cloud)
        for (size_t i = 0; i< cell_map.size(); i++){

            auto id = cell_map_int_to_id[i];

            auto points = std::vector<tg::pos3>();
            std::transform(cell_map[id].begin(), cell_map[id].end(), std::back_insert_iterator(points), [&](int j){
                #pragma omp critical
                return cloud.pts[j];
            });

            if (points.size() < 3) continue;

            // get the plane that the cell is on
            auto plane = fit_plane_thorugh_points(points);
            // plane.origin = tg::project(tg::average(points), plane);
            plane.origin =  plane.to3d(plane.to2d(tg::average(points)));

            // Ensure normal is allighned with point normals
            auto normals = std::vector<tg::vec3>();
            std::transform(cell_map[id].begin(), cell_map[id].end(), std::back_insert_iterator(normals), [&](int j){
                return cloud.norm[j];
            });
            auto average_normal = tg::average(normals);

            // flip the normal if it is not alligned with the average normal
            if (tg::dot(average_normal, plane.normal) < 0)
                plane.normal = -plane.normal;

            #pragma omp critical
            cell_map_plane[id] = plane;

            // project point in to 2d space
            auto points2d = plane.to2d(points);
            auto local_indecies = alpha_shape(points2d);

            auto endge_points = std::vector<tg::pos2>();
            std::transform(local_indecies.begin(), local_indecies.end(), std::back_insert_iterator(endge_points), [&](int j){
                return points2d[j];
            });
            auto sorted_endge_indecies  = convex_hull(endge_points);



            auto indecies = std::vector<size_t>();
            std::transform(sorted_endge_indecies.begin(), sorted_endge_indecies.end(), std::back_insert_iterator(indecies), [&](int j){
                auto index = local_indecies[j];
                return cell_map[id][index];
            });

            #pragma omp critical
            cell_map_boundary_point_indecies[id] = indecies;

        }

#if 0
        auto alpha_shape_color = std::vector<tg::color3>(cloud.pts.size(), tg::color3(0,0,0));
        for (auto const& [i, indcies ]: cell_map_boundary_point_indecies){
            auto color = linkml::get_color_forom_angle(linkml::sample_circle(i));
            for (auto const& index : indcies){
                alpha_shape_color[index] = color;
            }
        }
        auto ps_alpha_shape_color = ps_cloud->addColorQuantity("alpha shape", alpha_shape_color);
        ps_alpha_shape_color->setEnabled(true);
#endif


 
        // create a sparce matrix from python
        auto n_cells = cell_map.size();
        py::function lil_matrix = sparse.attr("lil_matrix");
        pybind11::object matrix = lil_matrix(py::make_tuple(n_cells, n_cells)).attr("astype")(np.attr("float32"));


        // set the diagonal to 1
        for (int i = 0; i < n_cells; i++){
            auto selector = py::make_tuple(i, i);
            matrix[selector] = 1;
        }


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


#if 0
            // Visualize the mesh
            auto meshVerts = std::vector<tg::pos3>();

            for (int j = 0; j < indecies.size() ; j++){
                auto point = cloud.pts[indecies[j]];
                meshVerts.push_back(point);
            }

            auto meshFaces = std::vector<std::array<size_t, 3>>();

            // Add the faces to the face buffer
            for (int j = 0; j < n_trinages; j++){
                size_t p1 = j+1;
                size_t p2 = j+2;
                meshFaces.push_back({0, p1, p2});
            }
            auto name = "Mesh " + std::to_string(id);

            polyscope::registerSurfaceMesh(name, meshVerts, meshFaces);
#endif

        }

        

        // Commit the scene
        rtcCommitScene(scene);

        // Create all rays

        auto all_vectors = get_sphere_vectors(50,50);

        auto all_rays = std::vector<std::tuple<size_t, RTCRayHit>>(); 


        for (int i = 0; i < cell_map_int_to_id.size(); i++){
            auto id = cell_map_int_to_id[i];
            auto plane = cell_map_plane[id];

            auto new_origin = plane.origin + plane.normal * 0.01f;

            auto rays = filter_vectors(all_vectors, new_origin, plane.normal);

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


        auto tripletList = std::vector<Triplet>();

#if 1
        rtcIntersect1M(scene, &context, (RTCRayHit*)&all_rays[0], (unsigned int)all_rays.size(), sizeof(std::tuple<size_t, RTCRayHit>));
        bool any_hit = false;

        // #pragma omp parallel 
        // #pragma omp single nowait
        for (auto [id, rayhit] : all_rays){
            if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

                auto source_id = id;
                auto ray = rayhit;

                // #pragma omp task shared(source_id, ray) //shared(tripletList, matrix, any_hit )
                // {

                    auto target_id = genomID_map[ray.hit.geomID];

                    auto source_int = cell_map_id_to_int[source_id];
                    auto target_int = cell_map_id_to_int[target_id];
                    auto selector = py::make_tuple(source_int, target_int);


                    auto inverse = 5 / ray.ray.tfar;
                    auto vaule = (matrix[selector].cast<float>() + inverse ) / 2;

                    // #pragma omp critical
                    // {
                        // Increment the matrix
                        matrix[selector] = vaule;
                        tripletList.push_back(Triplet(source_int, target_int, vaule));

                        selector = py::make_tuple(target_int,source_int);
                        matrix[selector] = vaule;
                        tripletList.push_back(Triplet(target_int, source_int, vaule));
                        
                        any_hit = true;
                    // }

                // }

            }
        }

#endif

#if 0
        // Intersect all rays
        bool any_hit = false;
        for (auto [id, rayhit] : all_rays){
            rtcIntersect1(scene, &context, &rayhit);
            if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

                auto source_id = id;
                auto target_id = genomID_map[rayhit.hit.geomID];

                auto source_int = cell_map_id_to_int[source_id];
                auto target_int = cell_map_id_to_int[target_id];
                auto selector = py::make_tuple(source_int, target_int);


                auto inverse = 5 / rayhit.ray.tfar;
                auto vaule = (matrix[selector].cast<float>() + inverse ) / 2;

                // Increment the matrix
                matrix[selector] = vaule;
                tripletList.push_back(Triplet(source_int, target_int, vaule));

                selector = py::make_tuple(target_int,source_int);
                matrix[selector] = vaule;
                tripletList.push_back(Triplet(target_int, source_int, vaule));


                std::cout << "hit: " << source_int << ", " << target_int << std::endl;
                any_hit = true;
            }
        }
#endif
        std::cout << std::endl;
        std::printf("any_hit: %d\n", any_hit);

        // Convert the triplet list to a sparse matrix
        SpMat sp_matrix(n_cells, n_cells);
        sp_matrix.setFromTriplets(tripletList.begin(), tripletList.end());


        // Release the scene and device
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);

        // py::object normalize = py::module_::import("sklearn.preprocessing").attr("normalize");
        // matrix = normalize(matrix, "l2");

        py::object mc = py::module_::import("markov_clustering");
        // py::kwargs mc_kwargs;
        // mc_kwargs["inflation"] = py::cast(2.5);
        // 2.5 < infaltion < 2.8  => 2.6
        py::object result = mc.attr("run_mcl")(matrix, 2, 3);           // run MCL with default parameters
        py::object clusters = mc.attr("get_clusters")(result);    // get clusters


        auto c_clusters = clusters.cast<std::vector<py::object>>();
        std::printf("n_clusters: %d\n", c_clusters.size());


        // C++ Markov Clustering
        sp_matrix = markov_clustering::run_mcl(sp_matrix, 2, 3);
        auto c_clusters2 = markov_clustering::get_clusters(sp_matrix);
        std::printf("n_clusters sp_matrix: %d\n", c_clusters2.size());



#if 1
        polyscope::init();
        polyscope::view::setUpDir(polyscope::view::UpDir::ZUp);
        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
#endif
#if 1

        auto ps_cloud = polyscope::registerPointCloud("point cloud", cloud.pts);
        auto cluster_colors = std::vector<tg::color3>(cloud.pts.size(), tg::color3(0,0,0));
        auto cluster2_colors = std::vector<tg::color3>(cloud.pts.size(), tg::color3(0,0,0));

        // Loop over all c_clusters2
        int couter = 0;
        for (auto cluster : c_clusters2){
            auto color = linkml::get_color_forom_angle(linkml::sample_circle(couter));
            couter++;

            // Loop over all cells in a cluster
            for (auto const& cell_index : cluster){

                auto cell_id = cell_map_int_to_id[cell_index];
                auto cell_indecies = cell_map[cell_id];

                // Loop over all points in a cell
                for (auto const& index : cell_indecies){
                    cluster_colors[index] = color;
                }
            }
        }

        

        // Loop over all clusters
        for (int i = 0; i < c_clusters.size(); i++){
            auto color = linkml::get_color_forom_angle(linkml::sample_circle(i));
            auto cell_indecies = c_clusters[i].cast<std::vector<int>>();

            // Loop over all cells in a cluster
            for (auto const& cell_index : cell_indecies){

                auto cell_id = cell_map_int_to_id[cell_index];
                auto cell_indecies = cell_map[cell_id];

                // Loop over all points in a cell
                for (auto const& index : cell_indecies){
                    cluster_colors[index] = color;
                }
            }
        }


        auto ps_cluster_color = ps_cloud->addColorQuantity("clusters", cluster_colors);
        ps_cloud->addColorQuantity("clusters2", cluster2_colors);

        ps_cluster_color->setEnabled(true);
#endif


#if 1
        polyscope::show();
#endif

        return matrix.attr("tocsr")();

    }
}