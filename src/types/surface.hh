#pragma once

#include "types/PointCloud.hh"
#include <pcl/point_types.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Alpha_shape_2.h>
//#include <CGAL/Polygonal_surface_reconstruction/internal/alpha_shape_mesh.h>
#include <embree3/rtcore.h>
#include <Eigen/Core>
#include <pcl/common/centroid.h>



namespace PMP = CGAL::Polygon_mesh_processing;

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Mesh = CGAL::Surface_mesh<Kernel>;
using Ray = RTCRayHit;
using Rays = std::vector<RTCRayHit>;

using Plane_3 = Kernel::Plane_3;
using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Vector_2 = Kernel::Vector_2;

using Dt = CGAL::Delaunay_triangulation_2<Kernel>;
using Ht = CGAL::Triangulation_hierarchy_2<Dt>;
using Polygon_mesh = CGAL::Surface_mesh<Kernel::Point_3>;


//using Alpha_shape_mesh = CGAL::internal::Alpha_shape_mesh;
//using Polygon_mesh = CGAL::Polygon_mesh_processing::Polygon_mesh<Kernel>;
//using Polygon = Kernel::Polygon_2;

namespace linkml
{
    class Surface
    {
    private:
        Plane_3 plane;
        Mesh mesh;
        Rays embree_rays;
        RTCGeometry embree_geometry;
    public:
        Surface(PointCloud::ConstPtr cloud, pcl::Indices indices);

        RTCGeometry get_embree_geometry(){
            return embree_geometry;
        };
        Rays get_embree_rays(unsigned int source = RTC_INVALID_GEOMETRY_ID ){
            for (auto& ray : embree_rays)
                ray.ray.id = source;
            return embree_rays;
        };
    private:

        pcl::Indices supporting_points(typename Mesh::Face_index face, const Mesh& mesh, const PointCloud::ConstPtr cloud, const pcl::Indices indices);


        FT face_area(typename Mesh::Face_index f, const Mesh& mesh) const {
                FT result(0);

                const typename Mesh::template Property_map<typename Mesh::Vertex_index, Point_3>& coords = 
                    mesh.property_map<typename Mesh::Vertex_index, Point_3>("v:points").first;

                typename Mesh::Vertex_around_face_circulator cir(mesh.halfedge(f), mesh), done_v(cir);

                Point_3 points[4];
                points[0] = coords[*cir]; ++cir;
                points[1] = coords[*cir]; ++cir;
                points[2] = coords[*cir]; ++cir;
                points[3] = coords[*cir];
                
                std::array<Point_2,4> points_2d;
                for (int i = 0; i < 4; i++)
                    points_2d[i] = plane.to_2d(points[i]);                

                auto bbox = CGAL::bounding_box(points_2d.begin(), points_2d.end());
                FT dx = bbox.xmax() - bbox.xmin();
                FT dy = bbox.ymax() - bbox.ymin();

                result = dx * dy;

                return result;
        }
 

    };



// Implementation

    pcl::Indices Surface::supporting_points(typename Mesh::Face_index face, const Mesh& mesh, const PointCloud::ConstPtr cloud, const pcl::Indices indices_in) {
        pcl::Indices indices;

        if (face == Mesh::null_face())
                return indices;

        const typename Mesh::template Property_map<typename Mesh::Vertex_index, Point_3>& coords = 
            mesh.property_map<typename Mesh::Vertex_index, Point_3>("v:points").first;

        typename Mesh::Vertex_around_face_circulator cir(mesh.halfedge(face), mesh), done(cir);
        Point_3 points_3d[4];
        points_3d[0] = coords[*cir]; ++cir;
        points_3d[1] = coords[*cir]; ++cir;
        points_3d[2] = coords[*cir]; ++cir;
        points_3d[3] = coords[*cir];

        
        std::array<Point_2, 4> points_2d;
        for (int i = 0; i < 4; i++)
            points_2d[i] = plane.to_2d(points_3d[i]); 

        auto bbox = CGAL::bounding_box(points_2d.begin(), points_2d.end());


        for (size_t i = 0; i < indices_in.size(); i++){

            auto point = cloud->points[indices_in[i]].getVector3fMap();
            Point_2 p = plane.to_2d(Point_3(point[0], point[1], point[2]));
        
            if ( bbox.xmin() <= p.x() && bbox.xmax() >= p.x() && bbox.ymin() <= p.y() && bbox.ymax() >= p.y() )
                indices.push_back(indices_in[i]);
        }

        return indices;
    }
    Surface::Surface(PointCloud::ConstPtr cloud, pcl::Indices indices){

        // Fit Plane through points
        Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
        Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
        Eigen::Matrix3f clust_cov;

        pcl::computeMeanAndCovarianceMatrix (*cloud, indices, clust_cov, clust_centroid);
        
        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
        pcl::eigen33 (clust_cov, eigen_value, eigen_vector);

        Eigen::Vector4f plane_params;
        plane_params[0] = eigen_vector[0];
        plane_params[1] = eigen_vector[1];
        plane_params[2] = eigen_vector[2];
        plane_params[3] = 0;
        plane_params[3] = -1 * plane_params.dot (clust_centroid);

        vp -= clust_centroid;
        float cos_theta = vp.dot (plane_params);
        if (cos_theta < 0)
        {
            plane_params *= -1;
            plane_params[3] = 0;
            plane_params[3] = -1 * plane_params.dot (clust_centroid);
        }

        // Flip normal if pointing outwards
        Eigen::Vector3f average_normal = Eigen::Vector3f::Zero();
        for (size_t i = 0; i < indices.size(); i++)
            average_normal += cloud->points[indices[i]].getVector3fMap();
        average_normal /= indices.size();
        if (average_normal.dot(plane_params.head<3>()) < 0)
            plane_params *= -1;

        // Construct Plane
        plane = Plane_3(plane_params[0], plane_params[1], plane_params[2], plane_params[3]);


        // Project points to 2D
        std::vector<Point_2> points_2d(indices.size());
        for (size_t i = 0; i < indices.size(); i++){
            auto point = cloud->points[indices[i]].getVector3fMap();
            Point_3 p = Point_3(point[0], point[1], point[2]);
            points_2d[i] = plane.to_2d(p);
        }

        // Find Bounding Box in 2D
        auto bbox = CGAL::bounding_box(points_2d.begin(), points_2d.end());

        // Subdivide Bounding Box
        auto dx  = bbox.xmax() - bbox.xmin();
        auto dy  = bbox.ymax() - bbox.ymin();
        int nx = dx / 0.2;
        int ny = dy / 0.2;
        
        // Create Surface Mesh
        size_t num_verticies = (nx+1) * (ny+1);
        std::vector<Mesh::Vertex_index> vertices(num_verticies);
        for (size_t i = 0; i < num_verticies; i++)
            vertices[i] = mesh.add_vertex();

        auto points = mesh.template add_property_map<Mesh::Vertex_index, Point_3>("v:points").first;
        for (int i = 0; i < nx; i++)
            for (int j = 0; j < ny; j++)
                points[vertices[i + j * (nx+1)]] = plane.to_3d(Point_2(bbox.xmin() + i * dx / nx, bbox.ymin() + j * dy / ny));
        
        for (int i = 0; i < nx; i++)
            for (int j = 0; j < ny; j++){
                Mesh::Vertex_index vertices[4];
                vertices[0] = vertices[i + j * (nx+1)];
                vertices[1] = vertices[i+1 + j * (nx+1)];
                vertices[2] = vertices[i+1 + (j+1) * (nx+1)];
                vertices[3] = vertices[i + (j+1) * (nx+1)];
                mesh.add_face(vertices[0], vertices[1], vertices[2], vertices[3]);
            }
                

        // Compute occupancy per face 
        auto face_areas = mesh.template add_property_map<Mesh::Face_index, FT>("f:face_areas").first;
        auto face_num_supporting_points = mesh.template add_property_map<Mesh::Face_index, std::size_t>("f:face_num_supporting_points").first;
        auto face_covered_areas = mesh.template add_property_map<Mesh::Face_index, FT>("f:covered_area").first;
        
        //auto occupancy = mesh.template add_property_map<Mesh::Vertex_index, Point_3>("f:occupancy").first;

        FT degenerate_face_area_threshold = CGAL::snap_squared_distance_threshold<FT>() * CGAL::snap_squared_distance_threshold<FT>();

        std::vector<typename Mesh::Face_index> faces;
        for (auto f : mesh.faces())
            faces.push_back(f);

        #pragma omp parallel for
        for (size_t i = 0; i < faces.size(); i++){

                // Face area
                FT area = face_area(faces[i], mesh);
                face_areas[faces[i]] = area;

                if (area > degenerate_face_area_threshold) {
                        const pcl::Indices indices = supporting_points(faces[i], mesh, cloud, indices);
                        face_num_supporting_points[faces[i]] = indices.size();

                        std::vector<Point_2> pts;
                        for (std::size_t i = 0; i < indices.size(); ++i) {
                                std::size_t idx = indices[i];
                                auto point = cloud->points[idx].getVector3fMap();
                                pts.push_back(plane.to_2d(Point_3(point.x(), point.y(), point.z())));
                        }

                        FT covered_area(0);


                        using Triangle = std::vector<std::size_t>;
                        using Alpha_shape = CGAL::Alpha_shape_2<Ht>;

                        auto alpha_shape = Alpha_shape(pts.begin(), pts.end());
                        std::vector<Triangle>        faces;


                        typename Alpha_shape::Finite_faces_iterator fit = alpha_shape.finite_faces_begin();
                        for (; fit != alpha_shape.finite_faces_end(); ++fit) {
                                if (alpha_shape.classify(fit) == Alpha_shape::INTERIOR) {
                                        Triangle tri;
                                        for (int i = 0; i < 3; ++i) {
                                                typename Alpha_shape::Vertex_handle vh = fit->vertex(i);
                                                int idx = vh->index();
                                                tri.push_back(idx);
                                        }
                                        faces.push_back(tri);
                                }
                        }

                        //if (faces.empty())
                        //        return false;

                        Polygon_mesh covering_mesh;
                        covering_mesh.clear();

                        std::vector<Vertex_descriptor> descriptors(original_points_.size());
                        for (std::size_t i = 0; i < original_points_.size(); ++i) {
                                const Point3* p = original_points_[i];
                                descriptors[i] = covering_mesh.add_vertex(*p);
                        }

                        for (std::size_t i = 0; i < faces.size(); ++i) {
                                std::vector<Vertex_descriptor> face;
                                const Triangle& tri = faces[i];
                                for (std::size_t j = 0; j < tri.size(); ++j) {
                                        std::size_t idx = tri[j];
                                        face.push_back(descriptors[idx]);
                                }
                                covering_mesh.add_face(face);;
                        }

                        FT radius = avg_spacing * FT(5.0);
                        if (alpha_mesh.extract_mesh(radius * radius, covering_mesh)) {
                                // We cannot use the area of the 3D faces, because the alpha shape mesh is
                                // not perfectly planar
                                const typename Polygon_mesh::template Property_map<Vertex_descriptor, Point>& coords = covering_mesh.points();
                                for(auto face : covering_mesh.faces()) {
                                        // We have to use the projected version
                                        Polygon plg; // the projection of the face onto it supporting plane
                                        Halfedge_around_face_circulator<Polygon_mesh> cir(covering_mesh.halfedge(face), covering_mesh), done(cir);
                                        do {
                                                Halfedge_descriptor hd = *cir;
                                                Vertex_descriptor vd = covering_mesh.target(hd);
                                                const Point& p = coords[vd];
                                                const Point2& q = supporting_plane->to_2d(p);
                                                plg.push_back(q);
                                                ++cir;
                                        } while (cir != done);
                                        covered_area += std::abs(plg.area());
                                }
                        }

                        face_covered_areas[f] = covered_area;
                        if (covered_area > area)
                                face_covered_areas[f] = area;
                }
                else { // For tiny faces, we can simple assign zero supporting points
                        face_num_supporting_points[f] = 0;
                        face_covered_areas[f] = FT(0.0);
                }
        }
        // - Create property map with indecies

        // Create Embree Geometry

        // For faces with occupancy > 0.5
        //  -> Create Embree Rays

    }    
} // namespace linkml



