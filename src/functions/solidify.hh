#pragma once

#include "types/PointCloud.hh"
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

//using Kernel                = CGAL::Exact_predicates_inexact_constructions_kernel;
//using FT                    = Kernel::FT;
using Point_3               = Kernel::Point_3;
using Vector_3              = Kernel::Vector_3;
//using PointSet              = CGAL::internal::Point_set_with_planes<linkml::Kernel>;
//using Candidate_confidences = CGAL::internal::Candidate_confidences_custom<Kernel>;
//using Candidate_visibility  = CGAL::internal::Candidate_visibility<Kernel>;
//using Surface_mesh          = CGAL::Surface_mesh<Kernel::Point_3>;
//using face_descriptor       = Surface_mesh::Face_index;
//using MapBetweenFaces       = std::map<face_descriptor, face_descriptor>;

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

    struct PointMap2
    {
        const Point_3& operator[](const PointCloud::Cloud::PointType& k) const { 
            auto p = k.getVector3fMap();
            return Point_3(p[0], p[1], p[2]);
        }

        friend Point_3 get(const PointMap2&, const PointCloud::Cloud::PointType& k) { 
            auto p = k.getVector3fMap();
            return Point_3(p[0], p[1], p[2]);
    }
        friend void put(const PointMap2&, PointCloud::Cloud::PointType& k, const Point_3& v) { 
                    k.x = v[0];
                    k.y = v[1];
                    k.z = v[2];
            }
    };
    struct NormalMap2
    {
      const Vector_3& operator[](const PointCloud::Cloud::PointType& k) const { 
        auto v = k.getNormalVector3fMap(); 
        return Vector_3(v[0], v[1], v[2]);
      }

      friend Vector_3 get(const NormalMap2&, const PointCloud::Cloud::PointType& k) { 
        auto v =  k.getNormalVector3fMap();
        return Vector_3(v[0], v[1], v[2]);
      }
      friend void put(const NormalMap2&, PointCloud::Cloud::PointType& k, const Vector_3& v) { 
            k.normal[0] = v[0];
            k.normal[1] = v[1];
            k.normal[2] = v[2];
        }
    };
    struct PlaneIndexMap2
    {

        const int & operator[](const PointCloud::Cloud::PointType& point) const { return (int)point.label;}

        friend int get(const PlaneIndexMap2&, const PointCloud::Cloud::PointType& k) { return (int)k.label; }
        friend void put(const PlaneIndexMap2&, PointCloud::Cloud::PointType& k, const int & v) { k.label = (std::uint8_t)v;}


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
    static void repair(Mesh & mesh){

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

    template<typename PointT, typename Mesh>
    static void create_polysurface(typename pcl::PointCloud<PointT>::ConstPtr cloud, pcl::PointIndices::ConstPtr sellection, Mesh &mesh, int i = 0)
    {

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
        #pragma omp parallel for reduction(merge:plane_indices_set)
        for (size_t i = 0; i < cluster_cloud->points.size(); i++)
            plane_indices_set.insert(cluster_cloud->points[i].label);


        int index = 0;
        std::unordered_map<int, int> plane_index_map = std::unordered_map<int, int>();
        for (auto it = plane_indices_set.begin(); it != plane_indices_set.end(); it++)
            plane_index_map[*it] = index++;


        // If there are more than 3 planes attempt finding a volume
        if (plane_indices_set.size() >=4){


            #pragma omp critical(print)
            std::cout << "Computing poly surface for cluster " << i << " with " << plane_indices_set.size() << " planes." << std::endl;

            // Renumber the plane indices
            #pragma omp parallel for
            for (size_t i = 0; i < cluster_cloud->points.size(); i++)
                cluster_cloud->points[i].label = plane_index_map[cluster_cloud->points[i].label];

            
            // Create the surface reconstruction object
            CGAL::Polygonal_surface_reconstruction<Kernel> psr(cluster_cloud->points, PointMap2(), NormalMap2(), PlaneIndexMap2());
            psr.reconstruct<MIP_Solver>(mesh, 0.20 /*fitting*/, 0.10/*coverage*/, 0.70/*complexity*/);

            repair(mesh);
        }
    }


    template<typename PointT>
    static std::vector<Surface_mesh> solidify(typename pcl::PointCloud<PointT>::ConstPtr cloud, std::optional<std::vector<pcl::PointIndices::Ptr>> clusters_in = std::nullopt)
    {
        std::cout << "Solidifying..." << std::endl;
        // If clusters are not provided treat the whole cloud as a single cluster
        std::vector<pcl::PointIndices::Ptr> clusters = (clusters_in.has_value()) ? clusters_in.value() : make_single_cluster(cloud);

        std::cout << "Number of clusters: " << clusters.size() << std::endl;


        std::vector<Surface_mesh> meshes = std::vector<Surface_mesh>(clusters.size());
        
        // #pragma omp parallel
        // #pragma omp single nowait
        for (size_t i = 0; i < clusters.size(); i++)
            // #pragma omp task shared(cloud, clusters, meshes)
                create_polysurface<PointT>(cloud, clusters[i], meshes[i], i);
        
        // #pragma omp taskwait
        

        std::cout << "Solidification complete." << std::endl;
        
    
       

        // Filter meshes with no faces and too small volumes
        std::vector<Surface_mesh> meshes_temp = std::vector<Surface_mesh>();
        std::copy_if(meshes.begin(), meshes.end(), std::back_inserter(meshes_temp) , [](Surface_mesh m /*Don't pass a reference so the triangulation does't affect the final*/){

                // Filter out meshes with no faces
                if (m.number_of_faces() == 0)
                    return false;

                // Filter out meshes with open boundaries
                if (!CGAL::is_closed(m))
                    return false;

                using face_descriptor = Surface_mesh::Face_index;
                auto face_map = m.add_property_map<face_descriptor, int>("f:component").first;

                if (PMP::connected_components(m, face_map) > 1)
                    return false;

                PMP::triangulate_faces(m); // Volume requiers the mesh to be triangulated

                // Filter out meshes with small volumes
                if (std::abs(PMP::volume(m)) < 3)
                    return false;

                return true;
            });

        std::swap(meshes, meshes_temp);


        return meshes;
    }
} // namespace linkml
