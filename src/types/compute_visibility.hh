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


#include <embree3/rtcore.h>
#include <Eigen/Sparse>
#include "functions/progress_bar.hh"

#include <omp.h>


// Concurrency
typedef CGAL::Parallel_if_available_tag Concurrency_tag;


// Formular for calculating the reflection of a vector
//reflection_direction=incident_direction−2×(incident_direction⋅normal)×normal

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
        typedef typename std::vector<Vector>                    Vectors;
        typedef typename Kernel::Ray_3                          Ray;
        typedef typename std::vector<Ray>                       Rays;
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

        typedef Eigen::SparseMatrix<FT>                         SpMat;




public:
        Candidate_visibility() {}
        ~Candidate_visibility() {}

        // TODO: Update description
        /// Computes the vesibility values for each face
        /// - selection:                        stored as property 'f:selection' 
        void compute(Polygon_mesh& mesh);

        SpMat get_matrix() const { return matrix; }
        std::unordered_map<unsigned int, Face_descriptor> get_int_to_face() const { return int_to_face; }
        std::unordered_map<Face_descriptor, unsigned int> get_face_to_int() const { return face_to_int; }

private:

        Vectors VectorSphere(int n_slices = 10, int n_stacks = 10 ) {

                // Create sphere with all possible vectors
                Vectors vectors((n_slices-1)*n_stacks+1);

                // add north vetcor
                vectors[0] = Vector(0, 1, 0);


                // generate vertices per stack / slice
                for (int i = 0; i < n_stacks - 1; i++)
                {
                    auto phi = M_PI * FT(i + 1) / FT(n_stacks);
                    for (int j = 0; j < n_slices; j++)
                    {
                        auto theta = 2.0 * M_PI * FT(j) / FT(n_slices);

                        auto x = std::sin(phi) * std::cos(theta);
                        auto y = std::cos(phi);
                        auto z = std::sin(phi) * std::sin(theta);

                        vectors[i * n_slices + j + 1] = Vector(x, y, z);
                    }
                }

                // add south vetcor
                vectors.push_back(Vector(0, -1, 0));

                return vectors;

        }       
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
        Rays filter_vectors(Vectors &vectors, Point pt, Vector dir) {

                auto selection = Vectors();
                std::copy_if(vectors.begin(), vectors.end(), std::back_inserter(selection), [&](Vector v){
                        return CGAL::scalar_product(v, dir) < FT(-0.3);
                });


                auto rays = Rays(selection.size());
                for (size_t i = 0; i < selection.size(); i++)
                        rays[i] = Ray(pt, selection[i]);
                
                return rays;
        }

        std::unordered_map<Face_descriptor, unsigned int> face_to_int;
        std::unordered_map<unsigned int, Face_descriptor> int_to_face;
        
        SpMat matrix;
};


//////////////////////////////////////////////////////////////////////////

// implementation

template <typename Kernel>
void Candidate_visibility<Kernel>::compute(Polygon_mesh& mesh) {

        // The number of supporting points of each face
        // auto [face_num_supporting_points, success_num_supporting_points] = mesh.template property_map<Face_descriptor, std::size_t>("f:num_supporting_points");

        // The area of each face
        auto face_areas = mesh.template property_map<Face_descriptor, FT>("f:face_area").first;
        // The point covered area of each face
        auto face_covered_areas = mesh.template property_map<Face_descriptor, FT>("f:covered_area").first;
        // The supporting plane of each face
        auto face_supporting_planes = mesh.template property_map<Face_descriptor,const Plane*>("f:supp_plane").first;


        //// The coverage of each face
        //typename Polygon_mesh::template Property_map<Face_descriptor, bool> face_selection =
        //        mesh.template add_property_map<Face_descriptor, bool>("f:selection").first;


        // Select faces 
        std::vector<Face_descriptor> selected_faces;
        std::copy_if(mesh.faces_begin(), mesh.faces_end(), std::back_inserter(selected_faces), [face_areas, face_covered_areas](Face_descriptor f){
                return (face_areas[f]/face_covered_areas[f]) > 0.5;
        });

        face_to_int.clear();
        int_to_face.clear();
        for (size_t i = 0; i < selected_faces.size(); i++){
                face_to_int[selected_faces[i]] = i;
                int_to_face[i] = selected_faces[i];
        }


        // Create Embree context and Scene
        RTCDevice device = rtcNewDevice(NULL);
        RTCScene scene   = rtcNewScene(device);


        // Create the scene by adding the selected faces as Embree geometries
        auto genomID_map = std::unordered_map<unsigned int, Face_descriptor>();
        auto sceen_bar = util::progress_bar(selected_faces.size(), "Creating scene");
        #pragma omp parallel for
        for (size_t i = 0; i < selected_faces.size(); i++){
                Face_descriptor f = selected_faces[i];

                // Get the number of vertecies per face
                CGAL::Vertex_around_face_circulator<Polygon_mesh> vcirc(mesh.halfedge(f), mesh), done(vcirc);
                std::vector<Vertex_descriptor> indices;
                do {
                    indices.push_back(*vcirc++);
                } while (vcirc != done);

                // Number of triangles
                auto n_trinages = indices.size() - 2;

                // Make a new Embree geometry
                RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

                // Get the vertex buffer
                float* vb = (float*) rtcSetNewGeometryBuffer(geom,
                    RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), indices.size());

                // Get the index buffer
                unsigned* ib = (unsigned*) rtcSetNewGeometryBuffer(geom,
                    RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(unsigned), n_trinages);


                // Add the points to the vertex buffer
                for (int j = 0; j < indices.size(); j++){
                        Point point = mesh.point(indices[j]);
                        vb[3*j+0] = point.x();
                        vb[3*j+1] = point.y();
                        vb[3*j+2] = point.z();
                }

                // Add the faces to the index buffer
                for (int j = 0; j < n_trinages; j++){
                    ib[3*j+0] = 0;
                    ib[3*j+1] = j+1;
                    ib[3*j+2] = j+2;
                }

                #pragma omp critical
                {

                rtcCommitGeometry(geom);
                auto genomID = rtcAttachGeometry(scene, geom);
                rtcReleaseGeometry(geom);

                // Save the mapping between the genomID and face
                genomID_map[genomID] = f;
                }
                
                sceen_bar.update();
        }
        sceen_bar.stop();
        rtcCommitScene(scene);

        // Precomputed sphere vectors
        Vectors vector_sphere = VectorSphere(50,50);

        // Create vectors for all the faces
        // TODO: Write custom reduction for openMP
        auto rays = std::vector<std::tuple<size_t, RTCRayHit>>();
        rays.reserve(int(vector_sphere.size()/2) * selected_faces.size());
        auto ray_bar = util::progress_bar(genomID_map.size(), "Creating rays");
        for (std::pair<unsigned int, Face_descriptor> pair : genomID_map){

                unsigned int id = pair.first;
                Face_descriptor face = pair.second;
                Plane plane = *face_supporting_planes[face];
                Vector normal = Vector(plane.a(), plane.b(), plane.c());

                Point origin = center(face, mesh);
                origin = origin + (normal * 0.1f); // Move the origin a bit away from the face

                for (auto ray: filter_vectors(vector_sphere, origin, normal)){
                        RTCRayHit rayhit; 
                        rayhit.ray.org_x = ray.source().x(); 
                        rayhit.ray.org_y = ray.source().y();
                        rayhit.ray.org_z = ray.source().z();
        
                        rayhit.ray.dir_x = ray.direction().dx();
                        rayhit.ray.dir_y = ray.direction().dy();
                        rayhit.ray.dir_z = ray.direction().dz();

                        rayhit.ray.id = id;
        
                        rayhit.ray.tnear  = 0.f;
                        rayhit.ray.tfar   = std::numeric_limits<float>::infinity();
                        rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                        
                        rays.push_back(std::make_tuple(id, rayhit));
                }
                ray_bar.update();
        }
        ray_bar.stop();

        // Instatiate the context
        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        // Intersect all rays with the scene
        rtcIntersect1M(scene, &context, (RTCRayHit*)&rays[0], (unsigned int)rays.size(), sizeof(std::tuple<size_t, RTCRayHit>));

        // Initialize the matrix
        matrix = SpMat(selected_faces.size(), selected_faces.size());
        matrix += Eigen::VectorXd::Ones(matrix.cols()).template cast<FT>().asDiagonal();


        auto result_bar = util::progress_bar(rays.size(), "Processing rays");
        for (auto [source_id, ray] : rays){
            if (ray.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

                //auto source_id = ray.ray.id;
                auto target_id = ray.hit.geomID;

                Face_descriptor source_face = genomID_map[source_id];
                Face_descriptor target_face = genomID_map[target_id];

                int source_int = face_to_int[source_face];
                int target_int = face_to_int[target_face];

                //auto inverse = 5 / ray.ray.tfar;

                // auto vaule = (matrix.coeff(source_int, target_int) + inverse ) / 2;
                // auto vaule = 1;
                auto vaule = matrix.coeff(source_int, target_int) + 1;

                // Increment the matrix
                matrix.coeffRef(source_int, target_int) = vaule;
                matrix.coeffRef(target_int, source_int) = vaule;

            }
            result_bar.update();
        }
        result_bar.stop();


        // Release the scene and device
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);

}

} //namespace internal

} //namespace CGAL


