#pragma once


#include <iostream>
#include <fstream>
#include <vector>
#include <set>

#include <types/PointCloud.hh>
#include <functions/alpha_shape.hh>
#include <functions/color_facetes.hh>
#include <functions/project_2d.hh>
#include <functions/fit_plane_thorugh_points.hh>
#include <functions/convex_hull_2d.hh>
#include "functions/polyscope.hh"
#include <algorithms/markov_clustering.hh>

#include <typed-geometry/types/objects/polygon.hh>
#include <typed-geometry/functions/objects/contains.hh>
#include <typed-geometry/types/objects/ray.hh>
#include <typed-geometry/types/comp.hh>

#include <embree3/rtcore.h>

#include <Eigen/Sparse>

#include <CGAL/Polygonal_surface_reconstruction/internal/hypothesis.h>
#include <CGAL/Polygonal_surface_reconstruction/internal/compute_confidences.h>
#include <CGAL/Polygonal_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

#include "types/compute_visibility.hh"
#include "types/compute_conficences.hh"
#include "functions/color.hh"



# define Visualize 1

#if Visualize
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"

#include <functions/color.hh>
#endif




namespace PMP = CGAL::Polygon_mesh_processing;

using Kernel                = CGAL::Exact_predicates_inexact_constructions_kernel;
using FT                    = Kernel::FT;
using Point_3               = Kernel::Point_3;
using Vector_3              = Kernel::Vector_3;
using PointSet              = CGAL::internal::Point_set_with_planes<linkml::Kernel>;
using Candidate_confidences = CGAL::internal::Candidate_confidences_custom<Kernel>;
using Candidate_visibility  = CGAL::internal::Candidate_visibility<Kernel>;
using Surface_mesh          = CGAL::Surface_mesh<Kernel::Point_3>;
using face_descriptor       = Surface_mesh::Face_index;
using MapBetweenFaces       = std::map<face_descriptor, face_descriptor>;


#include "types/GUROBI_mixed_integer_program_traits.hh"
using MIP_Solver            = CGAL::GUROBI_mixed_integer_program_traits<double>;


using SpMat                 = Eigen::SparseMatrix<FT>;      // declares a column-major sparse matrix type of double
using Triplet               = Eigen::Triplet<FT>;


// NOTE: How to triangulate a mesh while preserving custom data in CGAL
// https://stackoverflow.com/questions/76831855/triangulate-faces-while-preserving-custom-data-in-cgal
template <class Mesh, typename Map>
void copy_property_map( 
    Mesh& mesh, 
    const std::map<typename Mesh::Face_index, typename Mesh::Face_index> fmap, 
    const std::string name, 
    const Map& property_map) {

    using KT = typename Map::key_type;
    using VT = typename Map::value_type;

    Map new_property_map = mesh.template
        add_property_map<KT, VT>(name, VT() ).first;

    for (typename Mesh::Face_index fi: mesh.faces()) {
        if (fmap.find(fi) == fmap.end()){
            continue;
        }
        new_property_map[fi] = property_map[fmap.at(fi)];
    }
}

namespace linkml{

    struct PointMap
    {
      const Point_3& operator[](const PointCloud::Cloud::PointType& k) const { 
        auto p = k.getVector3fMap();
        return Point_3(p[0], p[1], p[2]);
      }

      friend Point_3 get(const PointMap&, const PointCloud::Cloud::PointType& k) { 
        auto p = k.getVector3fMap();
        return Point_3(p[0], p[1], p[2]);
    }
      friend void put(const PointMap&, PointCloud::Cloud::PointType& k, const Point_3& v) { 
            k.x = v[0];
            k.y = v[1];
            k.z = v[2];
        }
    };
    struct NormalMap
    {
      const Vector_3& operator[](const PointCloud::Cloud::PointType& k) const { 
        auto v = k.getNormalVector3fMap(); 
        return Vector_3(v[0], v[1], v[2]);
      }

      friend Vector_3 get(const NormalMap&, const PointCloud::Cloud::PointType& k) { 
        auto v =  k.getNormalVector3fMap();
        return Vector_3(v[0], v[1], v[2]);
      }
      friend void put(const NormalMap&, PointCloud::Cloud::PointType& k, const Vector_3& v) { 
            k.normal[0] = v[0];
            k.normal[1] = v[1];
            k.normal[2] = v[2];
        }
    };
    struct PlaneIndexMap
    {
      const int & operator[](const PointCloud::Cloud::PointType& point) const { return (int)point.label; }

      friend int get(const PlaneIndexMap&, const PointCloud::Cloud::PointType& k) { return k.label; }
      friend void put(const PlaneIndexMap&, PointCloud::Cloud::PointType& k, const int & v) { k.label = (std::uint8_t)v;}
    };

    struct TriangulateVisitor : 
      public PMP::Triangulate_faces::Default_visitor<Surface_mesh>
    {
      void before_subface_creations(face_descriptor fsplit) {
        *ofaceindex = fsplit;
      }

      void after_subface_created(face_descriptor fnew) {
        (*fmap).insert(std::make_pair(fnew, *ofaceindex));
      }
    
      TriangulateVisitor()
        : fmap(new MapBetweenFaces()),
          ofaceindex(new face_descriptor())
      {}
    
      std::shared_ptr<MapBetweenFaces> fmap;
      std::shared_ptr<face_descriptor> ofaceindex;
    };
    
    static std::vector<std::vector<size_t>> clustering(linkml::PointCloud::Cloud::ConstPtr cloud){

        // Surface mesh
        Surface_mesh mesh;
        PointSet point_set(cloud->points, PointMap(), NormalMap(), PlaneIndexMap() );

        // Create a hypothesis/cell complex
        CGAL::internal::Hypothesis<Kernel> hypothesis;
        hypothesis.generate( point_set, mesh); std::cout << "Hyposis generated\n";

        // Compute confidences
        Candidate_confidences conf;
        conf.compute(point_set, mesh); std::cout << "Confidences computed\n";

        // Compute visibilities
        Candidate_visibility vis;
        vis.compute(mesh); std::cout << "Visibility computed\n";

        SpMat matrix = vis.get_matrix();
        auto int_to_face = vis.get_int_to_face();


        // Markov Clustering
        //// 2.5 < infaltion < 2.8  => 3.5
        matrix = markov_clustering::run_mcl(matrix, 2, 1.3);
        auto clusters = markov_clustering::get_clusters(matrix);
        std::printf("n_clusters sp_matrix: %d\n", (int)clusters.size());


        // The number of supporting points of each face
        typename Surface_mesh::template Property_map<Surface_mesh::Face_index, std::size_t> cluster_map =
                mesh.template add_property_map<Surface_mesh::Face_index, std::size_t>("f:cluster_index").first;

        int cluster_count = 0; 
        for (auto & cluster: clusters) {
            for (auto & f_idx : cluster) {
                cluster_map[int_to_face.at(f_idx)] = cluster_count;
            }
            cluster_count++;
        }


        auto face_areas = mesh.template property_map<Surface_mesh::Face_index, FT>("f:face_area").first;
        auto face_covered_areas  = mesh.template property_map<Surface_mesh::Face_index, FT>("f:covered_area").first;
        auto num_supporting_points  = mesh.template property_map<Surface_mesh::Face_index, size_t>("f:num_supporting_points").first;
        cluster_map  = mesh.template property_map<Surface_mesh::Face_index, size_t>("f:cluster_index").first;


        // Triangulate the faces for visualization
        TriangulateVisitor visor;
        CGAL::Polygon_mesh_processing::triangulate_faces(mesh, CGAL::parameters::visitor(visor));
        MapBetweenFaces fmap = *(visor.fmap);



        copy_property_map(mesh, fmap, "f:face_area", face_areas);
        copy_property_map(mesh, fmap, "f:covered_area", face_covered_areas);
        copy_property_map(mesh, fmap, "f:num_supporting_points", num_supporting_points);
        copy_property_map(mesh, fmap, "f:cluster_index", cluster_map);

        for (auto & f : mesh.faces()) {
            if (cluster_map[f] == 0) {
                CGAL::Euler::remove_face(mesh.halfedge(f), mesh);
            }
        }


        face_areas = mesh.template property_map<Surface_mesh::Face_index, FT>("f:face_area").first;
        face_covered_areas  = mesh.template property_map<Surface_mesh::Face_index, FT>("f:covered_area").first;
        num_supporting_points  = mesh.template property_map<Surface_mesh::Face_index, size_t>("f:num_supporting_points").first;
        cluster_map  = mesh.template property_map<Surface_mesh::Face_index, size_t>("f:cluster_index").first;


        int index;
        std::vector<FT> face_coverages(mesh.number_of_faces());
        std::vector<size_t> num_supporting_points_vec(mesh.number_of_faces());
        std::vector<size_t> cluster_vec(mesh.number_of_faces());
        std::vector<tg::color3> cluster_colors(mesh.number_of_faces());

        index = 0;
        for (Surface_mesh::Face_index f : mesh.faces()) {
            if (!f.is_valid()) continue;

            face_coverages[index] = face_covered_areas[f] / face_areas[f];
            num_supporting_points_vec[index] = num_supporting_points[f];
            cluster_vec[index] = cluster_map[f];
            cluster_colors[index] = get_color_forom_angle(sample_circle(cluster_map[f]));
            index++;
        }


        Surface_mesh mesh2;
        CGAL::Polygonal_surface_reconstruction<Kernel> psr(cloud->points, PointMap(), NormalMap(), PlaneIndexMap());
        psr.reconstruct<MIP_Solver>(mesh2);
        CGAL::Polygon_mesh_processing::triangulate_faces(mesh2);


        // Initialize polyscope
        polyscope::myinit();

        polyscope::display(cloud);

        // Visualize the mesh
        polyscope::display(mesh, "mesh");
        auto ps_mesh = polyscope::getSurfaceMesh("mesh");
        ps_mesh->addFaceScalarQuantity("face_coverage", face_coverages);
        ps_mesh->addFaceScalarQuantity("num_supporting_points", num_supporting_points_vec);
        ps_mesh->addFaceScalarQuantity("clusters", cluster_vec);
        ps_mesh->addFaceColorQuantity("cluster_colors", cluster_colors);

        polyscope::display(mesh2, "mesh2");

        polyscope::show();

        return std::vector<std::vector<size_t>>();




        //auto clusters_list = std::vector<std::vector<size_t>>();
        //for (auto mk_cluster : mk_clusters)
        //    clusters_list.push_back(mk_cluster);


        //// make list of list of indecies
        //return clusters_list;

    }
}