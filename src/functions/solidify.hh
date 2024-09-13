#pragma once

#include "types/PointCloud.hh"
#include "types/Surface_Mesh.hh"

#include "functions/progress_bar.hh"

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>


#include <CGAL/Polygonal_surface_reconstruction/internal/hypothesis.h>
#include <CGAL/Polygonal_surface_reconstruction/internal/compute_confidences.h>
#include <CGAL/Polygonal_surface_reconstruction.h>

#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup_extension.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/internal/Snapping/snap.h>

#include <omp.h>

namespace PMP = CGAL::Polygon_mesh_processing;


//#if defined(GUROBI_FOUND)
#include "types/GUROBI_mixed_integer_program_traits.hh"
using MIP_Solver            = CGAL::GUROBI_mixed_integer_program_traits<double>;
//#else
//#include <CGAL/SCIP_mixed_integer_program_traits.h>
//using MIP_Solver            = CGAL::SCIP_mixed_integer_program_traits<double>;
//#endif




// Custom reduction function for std::unordered_set<int>
inline void merge_sets(std::unordered_set<int> &lhs, const std::unordered_set<int> &rhs) {
    for (auto it = rhs.begin(); it != rhs.end(); it++)
        lhs.insert(*it);
}


#pragma omp declare reduction(merge : std::unordered_set<int> : merge_sets(omp_out, omp_in)) \
    initializer(omp_priv = std::unordered_set<int>())



namespace linkml
{
    template<typename PointT>
    struct PointMap
    {

        const PointT& operator[](const PointCloud::Cloud::PointType& k) const { 
            return PointT(k.x, k.y, k.z);
        }

        friend PointT get(const PointMap&, const PointCloud::Cloud::PointType& k) { 
            return PointT(k.x, k.y, k.z);
    }
        friend void put(const PointMap&, PointCloud::Cloud::PointType& k, const PointT& v) { 
                    k.x = CGAL::to_double(v[0]);
                    k.y = CGAL::to_double(v[1]);
                    k.z = CGAL::to_double(v[2]);
            }
    };

    template<typename VectorT>
    struct NormalMap
    {

        const VectorT& operator[](const PointCloud::Cloud::PointType& k) const { 
            return VectorT(k.normal_x, k.normal_y, k.normal_z);
        }

        friend VectorT get(const NormalMap&, const PointCloud::Cloud::PointType& k) { 
            return VectorT(k.normal_x, k.normal_y, k.normal_z);
        }
        friend void put(const NormalMap&, PointCloud::Cloud::PointType& k, const VectorT& v) { 
                k.normal_x = CGAL::to_double(v[0]);
                k.normal_y = CGAL::to_double(v[1]);
                k.normal_z = CGAL::to_double(v[2]);
            }
    };

    struct PlaneIndexMap
    {

        const int & operator[](const PointCloud::Cloud::PointType& point) const { return point.label;}

        friend int get(const PlaneIndexMap&, const PointCloud::Cloud::PointType& k) { return k.label; }
        friend void put(const PlaneIndexMap&, PointCloud::Cloud::PointType& k, const int & v) { k.label = (std::int32_t)v;}


    };

    static std::vector<pcl::PointIndices::Ptr> make_single_cluster(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        std::vector<pcl::PointIndices::Ptr> clusters;
        pcl::PointIndices::Ptr indices( new pcl::PointIndices());
        indices->indices.resize(cloud->size());
        std::iota(indices->indices.begin(), indices->indices.end(), 0);
        clusters.push_back(indices);
        return clusters;
    }

    template<typename Mesh>
    void repair(Mesh & mesh){

        using Point = typename Mesh::Point;
        // TODO: Check what the correct index type is
        using Index = size_t /*typename Mesh::Vertex_index*/;


        // Extract the points and polygons
        std::vector<Point> points = std::vector<Point>();
        std::vector<std::vector<Index>> polygons = std::vector<std::vector<Index>>();
        PMP::polygon_mesh_to_polygon_soup(mesh, points, polygons);


        PMP::merge_duplicate_points_in_polygon_soup(points, polygons);
        PMP::orient_polygon_soup(points, polygons);

        mesh.clear();
        PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
        PMP::triangulate_faces(mesh);

        bool is_outward_oriented = PMP::is_outward_oriented(mesh);

        mesh.clear();
        PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
        if (!is_outward_oriented)
            PMP::reverse_face_orientations(mesh);
    }

    template<typename Mesh>
    bool is_vaild(Mesh mesh /*Don't pass a reference so the triangulation does't affect the final*/){

        using Point = typename Mesh::Point;
        using Kernel = typename CGAL::Kernel_traits<Point>::Kernel;
        using FT = typename Kernel::FT;


        if (!mesh.is_valid())
            return false;

        if (mesh.is_empty())
            return false;

        if (mesh.number_of_vertices() == 0)
            return false;

        // Filter out meshes with no faces
        if (mesh.number_of_faces() == 0)
            return false;

        // Filter out meshes with open boundaries
        if (!CGAL::is_closed(mesh))
            return false;

        auto face_map = mesh.template add_property_map<typename Mesh::Face_index, int>("f:component").first;

        if (PMP::connected_components(mesh, face_map) > 1)
            return false;


        PMP::triangulate_faces(mesh); // Volume requiers the mesh to be triangulated

        // Filter out meshes with small volumes
        if (CGAL::abs(PMP::volume(mesh)) < FT(3))
            return false;

        return true;
    }

    template<typename PointT, typename Mesh>
    void create_polysurface(
        typename pcl::PointCloud<PointT>::ConstPtr cloud, 
        pcl::PointIndices::ConstPtr sellection, 
        Mesh & mesh,
        size_t i = 0,
        double fitting =  0.20,
        double coverage = 0.10,
        double complexity = 0.70
    )
    {
        // using Kernel_inexact = CGAL::Exact_predicates_inexact_constructions_kernel;

        using Point = typename Mesh::Point;
        using Kernel = typename CGAL::Kernel_traits<Point>::Kernel;
        using Vector = typename Kernel::Vector_3;

        // #pragma omp critical(print)
        // printf("Task %d\n", i);

        // Create the extraction object
        pcl::ExtractIndices<PointT> extract;

        // Set the input cloud and the indices
        extract.setInputCloud (cloud);
        extract.setIndices (sellection);
        extract.setNegative (false);

        // Extract the points
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
        extract.filter(*cluster_cloud);


        // Since CGAL explects planes to be number contiguously from 0, we need to renumber the planes
        // The plane indecies need to be rempated.

        // Get the unique plane indices
        std::unordered_set<int> plane_indices_set = std::unordered_set<int>();
        // #pragma omp parallel for reduction(merge:plane_indices_set)
        for (size_t i = 0; i < cluster_cloud->points.size(); i++)
            plane_indices_set.insert(cluster_cloud->points[i].label);


        // If there are more than 3 planes attempt finding a volume
        if (plane_indices_set.size() >=4){


            #pragma omp critical(print)


            int i = 0;
            std::unordered_map<int, int> plane_index_map = std::unordered_map<int, int>();
            for (auto it = plane_indices_set.begin(); it != plane_indices_set.end(); it++)
                plane_index_map[*it] = i++;

            // Renumber the plane indices
            #pragma omp parallel for
            for (size_t i = 0; i < cluster_cloud->points.size(); i++)
                cluster_cloud->points[i].label = plane_index_map[cluster_cloud->points[i].label];

            printf("Computing poly surface for Task %d with %d planes.\n", i, plane_indices_set.size());
            CGAL::Polygonal_surface_reconstruction<Kernel> psr(cluster_cloud->points, PointMap<Point>(), NormalMap<Vector>(), PlaneIndexMap());
            psr.template reconstruct<MIP_Solver>(mesh, 0.20 /*fitting*/, 0.10/*coverage*/, 0.70/*complexity*/);

            // repair(mesh);
        }

        // #pragma omp critical(print)
        // printf("Task %d done!\n", i);
    }
    

    template<typename PointT>
    static std::vector<Surface_mesh> solidify(
        typename pcl::PointCloud<PointT>::ConstPtr cloud, 
        std::optional<std::vector<pcl::PointIndices::Ptr>> clusters_in = std::nullopt,
        double fitting =  0.20,
        double coverage = 0.10,
        double complexity = 0.70
    )
    {
        std::cout << "Solidifying..." << std::endl;
        // If clusters are not provided treat the whole cloud as a single cluster
        std::vector<pcl::PointIndices::Ptr> clusters = (clusters_in.has_value()) ? clusters_in.value() : make_single_cluster(cloud);

        std::cout << "Number of clusters: " << clusters.size() << std::endl;


        std::vector<Surface_mesh> meshes = std::vector<Surface_mesh>(clusters.size());
        auto solidification_bar = util::progress_bar(clusters.size(), "Solidification");

#if 0 //Parallelize the solidification
        #pragma omp parallel
        {
            #pragma omp single
            {
                for (size_t i = 0; i < clusters.size(); i++){
                
                    #pragma omp task private(i, fitting, coverage, complexity)
                        create_polysurface<PointT, Surface_mesh>(cloud, clusters[i], meshes[i], i, fitting, coverage, complexity);
                }
            }
            #pragma omp taskwait
        }
#else
        for (size_t i = 0; i < clusters.size(); i++){
            create_polysurface<PointT, Surface_mesh>(cloud, clusters[i], meshes[i], i, fitting, coverage, complexity);
            solidification_bar.update();
        }
#endif
        solidification_bar.stop();

        

        // Filter meshes with no faces and too small volumes
        std::vector<Surface_mesh> meshes_temp = std::vector<Surface_mesh>();
        std::copy_if(meshes.begin(), meshes.end(), std::back_inserter(meshes_temp) , is_vaild<Surface_mesh>);
        std::swap(meshes, meshes_temp);

        for (size_t i = 0; i < meshes.size(); i++)
            PMP::reverse_face_orientations(meshes[i]);
    
        printf("Number of meshes: %d\n", (int)meshes.size());


        return meshes;
    }
} // namespace linkml
