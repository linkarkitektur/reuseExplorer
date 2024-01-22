#pragma once


#include <iostream>
#include <fstream>
#include <vector>
#include <set>

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



# define Visualize 1

#if Visualize
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"

#include <functions/color.h>
#endif


typedef Eigen::SparseMatrix<float>  SpMat;      // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<float>       Triplet;



// Write a sparse matrix to a file
void writeSparseMatrixToFile(const SpMat& matrix, const std::string& filename) {
    std::ofstream file(filename);

    if (file.is_open()) {
        file << matrix.rows() << " " << matrix.cols() << "\n";
        for (int k = 0; k < matrix.outerSize(); ++k) {
            for (SpMat::InnerIterator it(matrix, k); it; ++it) {
                file << it.row() << " " << it.col() << " " << it.value() << "\n";
            }
        }

        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

// // Read a sparse matrix from a file
// SparseMatrix<double> readSparseMatrixFromFile(const std::string& filename) {
//     std::ifstream file(filename);
//     int rows, cols;

//     if (file.is_open()) {
//         // Read the number of rows and columns
//         file >> rows >> cols;

//         // Initialize the sparse matrix
//         SparseMatrix<double> matrix(rows, cols);

//         // Read non-zero entries
//         int row, col;
//         double value;
//         while (file >> row >> col >> value) {
//             matrix.coeffRef(row, col) = value;
//         }

//         file.close();
//         return matrix;
//     } else {
//         std::cerr << "Unable to open file: " << filename << std::endl;
//         return SparseMatrix<double>(0, 0);
//     }
// }





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
            if (tg::dot(v, dir) > -0.3) continue;
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



    std::vector<std::vector<size_t>> clustering(linkml::point_cloud const& cloud, result_fit_planes const& results ) {

        auto clampted_points = std::vector<tg::pos3>(cloud.pts);
        for (size_t i = 0; i < results.indecies.size(); i++){
            auto indecies = results.indecies[i];
            auto plane = results.planes[i];

            for (auto const& index : indecies){
                auto point = clampted_points[index];
                auto new_point = tg::project(point, plane);
                clampted_points[index] = new_point;
            }
        }


// Initialize and visualize the point cloud
#if Visualize
        // Initialize polyscope
        polyscope::init();
        polyscope::view::setUpDir(polyscope::view::UpDir::ZUp);
        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;

        // Visualize the point cloud
        // auto ps_cloud = polyscope::registerPointCloud("point cloud", cloud.pts);
        auto ps_cloud = polyscope::registerPointCloud("point cloud", clampted_points);


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
#if Visualize
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
            plane.origin = tg::project(tg::average(points), plane);
            // plane.origin =  plane.to3d(tg::average(plane.to2d(points)));

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


        // Convert the triplet list to a sparse matrix
        auto n_cells = cell_map.size();
        SpMat matrix(n_cells, n_cells);
        matrix += Eigen::VectorXd::Ones(matrix.cols()).template cast<float>().asDiagonal();


        // Create Embree context and Scene
        RTCDevice device = rtcNewDevice(NULL);
        RTCScene scene   = rtcNewScene(device);

        auto genomID_map = std::map<unsigned int, size_t>();

#if Visualize
        // Visualize the mesh
        auto meshVerts = std::vector<tg::pos3>();
        auto meshFaces = std::vector<std::array<size_t, 3>>();
        auto meshColors = std::vector<tg::color3>();
#endif

        size_t offset = 0;
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
                // auto point = cloud.pts[indecies[j]];
                auto point = clampted_points[indecies[j]];
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

#if Visualize
            for (int j = 0; j < indecies.size() ; j++){
                // auto point = cloud.pts[indecies[j]];
                auto point = clampted_points[indecies[j]];
                meshVerts.push_back(point);
            }


            // Add the faces to the face buffer
            for (int j = 0; j < n_trinages; j++){
                size_t p1 = j+1+offset;
                size_t p2 = j+2+offset;
                meshFaces.push_back({offset, p1, p2});
            }

            auto color = linkml::get_color_forom_angle(linkml::sample_circle(i));
            for (int j = 0; j < n_trinages; j++)
                meshColors.push_back(color);

            offset += indecies.size();
#endif

        }

#if Visualize
        auto ps_mesh = polyscope::registerSurfaceMesh("Mesh Faces", meshVerts, meshFaces);
        auto ps_mesh_colors = ps_mesh->addFaceColorQuantity("mesh", meshColors);
        ps_mesh_colors->setEnabled(true);
#endif


        // Commit the scene
        rtcCommitScene(scene);

        // Create all rays

        auto all_vectors = get_sphere_vectors(50,50);
        auto all_rays = std::vector<std::tuple<size_t, RTCRayHit>>();
        all_rays.reserve(int(all_vectors.size()/2) * cell_map_int_to_id.size());

#if Visualize
        auto ray_nodes = std::vector<tg::pos3>();
        auto ray_edges = std::vector<std::array<size_t, 2>>();
        auto ray_node_colors = std::vector<tg::color3>();
        size_t offset_rays = 0;
#endif

        for (int i = 0; i < cell_map_int_to_id.size(); i++){
            auto id = cell_map_int_to_id[i];
            auto plane = cell_map_plane[id];

            auto new_origin = plane.origin + (plane.normal * -0.1f);

            auto rays = filter_vectors(all_vectors, new_origin, plane.normal);
            auto color = linkml::get_color_forom_angle(linkml::sample_circle(i));


#if Visualize
            size_t couter = 0;
#endif
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

#if Visualize
                // Visualize the rays
                ray_nodes.push_back(ray.origin);
                ray_nodes.push_back(ray.origin + ray.dir * 0.5f);
                ray_edges.push_back({ offset_rays + couter*2, offset_rays + couter*2 + 1});

                ray_node_colors.push_back(color);
                ray_node_colors.push_back(color);
                couter++;
#endif
            }
#if Visualize
            offset_rays += rays.size() * 2;
#endif
        }

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

#if Visualize
        auto ps_rays = polyscope::registerCurveNetwork("rays", ray_nodes, ray_edges);
        ps_rays->setRadius(0.00010);
        auto ps_ray_color = ps_rays->addNodeColorQuantity("rays", ray_node_colors);
        ps_ray_color->setEnabled(true);
#endif



        rtcIntersect1M(scene, &context, (RTCRayHit*)&all_rays[0], (unsigned int)all_rays.size(), sizeof(std::tuple<size_t, RTCRayHit>));
        bool any_hit = false;

        #pragma omp parallel 
        #pragma omp single nowait
        for (auto [id, rayhit] : all_rays){
            if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

                auto source_id = id;
                auto ray = rayhit;
                
                #pragma omp task shared( matrix, any_hit) firstprivate(source_id, ray)
                {
                    auto target_id = genomID_map[ray.hit.geomID];

                    auto source_int = cell_map_id_to_int[source_id];
                    auto target_int = cell_map_id_to_int[target_id];

   
                    auto inverse = 5 / ray.ray.tfar;

                    #pragma omp critical
                    {
                        // auto vaule = (matrix.coeff(source_int, target_int) + inverse ) / 2;
                        // auto vaule = 1;
                        auto vaule = matrix.coeff(source_int, target_int) + 1;

                        // Increment the matrix
                        matrix.coeffRef(source_int, target_int) = vaule;
                        matrix.coeffRef(target_int, source_int) = vaule;
                        any_hit = true;
                    }

                }

            }
        }


        // Release the scene and device
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);
        

        std::cout << std::endl;
        std::printf("any_hit: %d\n", any_hit);


        // writeSparseMatrixToFile(matrix, "sparse_matrix_pre.txt");


        // C++ Markov Clustering
        // 2.5 < infaltion < 2.8  => 3.5
        matrix = markov_clustering::run_mcl(matrix, 2, 1.3);
        std::set<std::vector<size_t>> clusters = markov_clustering::get_clusters(matrix);
        std::printf("n_clusters sp_matrix: %d\n", clusters.size());


        // Write the matrix to a file
        // writeSparseMatrixToFile(matrix, "sparse_matrix_after.txt");



#if Visualize

        auto cluster_colors = std::vector<tg::color3>(cloud.pts.size(), tg::color3(0,0,0));

        // Loop over all clusters
        int couter = 0;
        for (auto cluster : clusters){
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

        
        auto ps_cluster_color = ps_cloud->addColorQuantity("clusters", cluster_colors);
        ps_cluster_color->setEnabled(true);
#endif


#if Visualize
        polyscope::show();
#endif

        auto clusters_list = std::vector<std::vector<size_t>>();
        for (auto cluster : clusters)
            clusters_list.push_back(cluster);


        // make list of list of indecies
        return clusters_list;

    }
}