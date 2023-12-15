// Copyright (c) 2023  Povl Filip Sonne-Frederiksen. All rights reserved.
//
// Author(s) : Povl Filip Sonne-Frederiksen

#pragma once

#include <CGAL/license/Polygonal_surface_reconstruction.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/intersections.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/assertions.h>
#include <CGAL/Polygonal_surface_reconstruction/internal/point_set_with_planes.h>
#include <CGAL/Polygonal_surface_reconstruction/internal/alpha_shape_mesh.h>
#include <CGAL/Polygonal_surface_reconstruction/internal/hypothesis.h>
#include <CGAL/Polygonal_surface_reconstruction/internal/parameters.h>


// Concurrency
typedef CGAL::Parallel_if_available_tag Concurrency_tag;

namespace CGAL {
namespace internal {

                /**
                *        Computes the confidences of the candidate faces.
                */

                template <typename Kernel>
                class Candidate_visibility
                {
                private:
                        typedef typename Kernel::FT                             FT;
                        typedef typename Kernel::Point_3                        Point;
                        typedef typename Kernel::Point_2                        Point2;
                        typedef typename Kernel::Vector_3                       Vector;
                        typedef typename Kernel::Ray_3                          Ray;
                        typedef typename Kernel::Direction_3                    Dir;
                        typedef typename Kernel::Line_3                         Line;
                        typedef typename Kernel::Segment_3                      Segment;
                        typedef typename Kernel::Plane_3                        Plane;
                        typedef CGAL::Polygon_2<Kernel>                         Polygon;
                        typedef internal::Planar_segment<Kernel>                Planar_segment;
                        typedef internal::Point_set_with_planes<Kernel>         Point_set;
                        typedef CGAL::Surface_mesh<Point>                       Polygon_mesh;
                        typedef typename Polygon_mesh::Face_index               Face_descriptor;
                        typedef typename Polygon_mesh::Edge_index               Edge_descriptor;
                        typedef typename Polygon_mesh::Vertex_index             Vertex_descriptor;
                        typedef typename Polygon_mesh::Halfedge_index           Halfedge_descriptor;



                public:
                        Candidate_visibility() {}
                        ~Candidate_visibility() {}

                        // TODO: Update description
                        /// Computes the confidence values for each face
                        /// - supporting point number:        stored as property 'f:num_supporting_points'
                        /// - face area:                                stored as property 'f:face_area'
                        /// - covered area:                                stored as property 'f:covered_area'
                        void compute(const Point_set& point_set, Polygon_mesh& mesh);

                private:

                        Point center(Face_descriptor f, const Polygon_mesh& mesh) const {

                                const typename Polygon_mesh::template Property_map<Vertex_descriptor, Point>& coords = mesh.points();

                                Halfedge_around_face_circulator<Polygon_mesh> cir(mesh.halfedge(f), mesh), done(cir);

                                Halfedge_descriptor p_hd = *cir;
                                Vertex_descriptor p_vd = mesh.target(p_hd);
                                Point& center_pt = coords[p_vd];
                                ++cir;

                                FT x = center_pt.x();
                                FT y = center_pt.y();
                                FT z = center_pt.z();


                                int count = 1;
                                do {
                                        Halfedge_descriptor p_hd = *cir;
                                        Vertex_descriptor p_vd = mesh.target(p_hd);
                                        const Point& p = coords[p_vd];

                                        x += p.x();
                                        y += p.y();
                                        z += p.z();
                                        ++count;
                                        ++cir;
                                } while (cir != done);

                                x = x / count;
                                y = y / count;
                                z = z / count;

                                return Point(x,y,z);
                        }

                        std::pair<Point2, FT> planXray(Plane const & plane, Ray const & ray ) const {

                                auto result = CGAL::intersection(plane, ray);
                                const Point* p =  (result) ? boost::get<Point>(&*result)     : nullptr;

                                //FIXME: This results in a segmentation fault at runtim.

                                const Point2& p2 = plane.to_2d(&(*p));
    
                                const FT dist = CGAL::squared_distance(plane, ray.source());

                                return std::pair<Point2, FT>(p2, dist);

                        }

                        Polygon face_polygon(Face_descriptor face, const Polygon_mesh& mesh) const {

                                Polygon plg; // The projection of the face onto it supporting plane

                                if (face == Polygon_mesh::null_face())
                                        return plg;

                                // The supporting planar segment of each face
                                typename Polygon_mesh::template Property_map<Face_descriptor, Planar_segment*> face_supporting_segments =
                                        mesh.template property_map<Face_descriptor, Planar_segment*>("f:supp_segment").first;

                                Planar_segment* segment = face_supporting_segments[face];
                                if (segment == nullptr)
                                        return plg;

                                // The supporting plane of each face
                                typename Polygon_mesh::template Property_map<Face_descriptor, const Plane*> face_supporting_planes =
                                        mesh.template property_map<Face_descriptor, const Plane*>("f:supp_plane").first;

                                // We do everything by projecting the point onto the face's supporting plane
                                const Plane* supporting_plane = face_supporting_planes[face];
                                CGAL_assertion(supporting_plane == segment->supporting_plane());

                                const typename Polygon_mesh::template Property_map<Vertex_descriptor, Point>& coords = mesh.points();
                                Halfedge_around_face_circulator<Polygon_mesh> cir(mesh.halfedge(face), mesh), done(cir);
                                do {
                                        Halfedge_descriptor hd = *cir;
                                        Vertex_descriptor vd = mesh.target(hd);
                                        const Point& p = coords[vd];
                                        const Point2& q = supporting_plane->to_2d(p);

                                        // Removes duplicated vertices
                                        // The last point in the polygon
                                        if (!plg.is_empty()) {
                                                const Point2& r = plg[plg.size() - 1];
                                                if (CGAL::squared_distance(q, r) < CGAL::snap_squared_distance_threshold<FT>()) {
                                                        ++cir;
                                                        continue;
                                                }
                                        }
                                        plg.push_back(q);

                                        ++cir;
                                } while (cir != done);

                                return plg;

                        }

                        CGAL::Bounded_side bounded_side_2(Polygon plg, Point2 pt){
                                return CGAL::bounded_side_2(plg.begin(), plg.end(), pt, Kernel());
                        }

                        std::function<Ray(int, Plane, Point, Dir)> get_ray_caster(int max) const {

                                return [&](int idx, Plane plane, Point pt, Dir dir){
                                        return Ray(pt, dir);
                                };

                        }
                };


                //////////////////////////////////////////////////////////////////////////

                // implementation

                template <typename Kernel>
                void Candidate_visibility<Kernel>::compute(const Point_set& point_set, Polygon_mesh& mesh) {
                        const unsigned int K = 6;

                        const typename Point_set::Point_map& points = point_set.point_map();
                        FT avg_spacing = compute_average_spacing<Concurrency_tag>(points, K);

                        // The number of supporting points of each face
                        typename Polygon_mesh::template Property_map<Face_descriptor, std::size_t> face_num_supporting_points =
                                mesh.template add_property_map<Face_descriptor, std::size_t>("f:num_supporting_points").first;

                        // The area of each face
                        typename Polygon_mesh::template Property_map<Face_descriptor, FT> face_areas =
                                mesh.template add_property_map<Face_descriptor, FT>("f:face_area").first;

                        // The point covered area of each face
                        typename Polygon_mesh::template Property_map<Face_descriptor, FT> face_covered_areas =
                                mesh.template add_property_map<Face_descriptor, FT>("f:covered_area").first;

                        // The supporting plane of each face
                        typename Polygon_mesh::template Property_map<Face_descriptor, const Plane*> face_supporting_planes =
                                mesh.template property_map<Face_descriptor, const Plane*>("f:supp_plane").first;

                        FT degenerate_face_area_threshold = CGAL::snap_squared_distance_threshold<FT>() * CGAL::snap_squared_distance_threshold<FT>();


                        std::cout << "\nHere we are supposed to do computation to find a matrix of faces that can view eachother" << std::endl;


                        // Static list of Face_descriptors used to optimized loop with OpenMP
                        std::vector<Face_descriptor> const faces_descriptors = std::vector<Face_descriptor>(mesh.faces().begin(), mesh.faces().end());
                        
                        int n_faces = faces_descriptors.size();
                        int n_rays = 10; // Number of rays to cast per face.

                        // centers and plane normals.
                        std::vector<Point>      centers = std::vector<Point>(n_faces);
                        std::vector<Dir>        normals = std::vector<Dir>(n_faces);
                        std::vector<Plane>      planes  = std::vector<Plane>(n_faces);
                        std::vector<Polygon>    polygons= std::vector<Polygon>(n_faces);


                        std::vector<std::vector<FT>> distance = std::vector<std::vector<FT>>(
                                n_faces, std::vector<FT>(n_rays, FT(std::numeric_limits<FT>::infinity())));
                        std::vector<std::vector<FT>> hit      = std::vector<std::vector<FT>>(n_faces,std::vector<FT>(n_rays, FT(-1)));


                        // Precompute values form ray caster

                        std::cout << "Centers" << std::endl;
                        #pragma omp parallel for
                        for (int i = 0; i< n_faces; i++){
                                centers[i] = center(faces_descriptors[i], mesh);
                        }

                        std::cout << "Normals" << std::endl;
                        #pragma omp parallel for
                        for (int i = 0; i< n_faces; i++){
                                auto plan = face_supporting_planes[faces_descriptors[i]];
                                normals[i] = Dir(plan->a(), plan->b(), plan->c());
                        }

                        std::cout << "Planes" << std::endl;
                        #pragma omp parallel for
                        for (int i = 0; i< n_faces; i++){
                                planes[i] = *face_supporting_planes[faces_descriptors[i]];
                        }

                        std::cout << "Polygons" << std::endl;
                        #pragma omp parallel for
                        for (int i = 0; i< n_faces; i++){
                                polygons[i] = face_polygon(faces_descriptors[i], mesh);
                        }

                        auto const ray_caster = get_ray_caster(n_rays);
                        


                        std::cout << "GPU" << std::endl;
                        // Ray caster
                        #pragma omp target map(to: centers, normals, planes, polygons) map(tofrom: distance, hit)
                        {
                                #pragma omp parallel for collapse(3) // private(i) private(j) 
                                for (int i = 0; i < n_faces; i++){
                                        for (int j = 0; j < n_faces; j++){
                                                for (int k = 0; k < n_rays; k++){

                                                        // Ray 
                                                        Ray ray =  ray_caster(j, planes[i], centers[i], normals[i]);

                                                        // Ray plane intersection
                                                        auto [pt, dist ] = planXray(planes[j], ray);

                                                        // Point in Polygon
                                                        bool inside = bounded_side_2(polygons[i], pt) == CGAL::ON_BOUNDED_SIDE;

                                                        inside = (i == j )? false : inside; // Mask out values for identical planes
                                                        distance[i][k]  = (dist < distance[i][k] && inside)? dist : distance[i][k];
                                                        hit[i][k]       = (dist < distance[i][k] && inside)? j    : hit[i][k];
                                                }
                                        }
                                }
                        }

                        std::cout << "Done" << std::endl;


                }

        } //namespace internal

} //namespace CGAL


