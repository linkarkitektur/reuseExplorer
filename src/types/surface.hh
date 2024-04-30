#pragma once

#include "types/PointCloud.hh"

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include <CGAL/Simple_cartesian.h>

#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_hierarchy_2.h>

#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/measure.h>

#include <embree3/rtcore.h>

#include <Eigen/Core>



namespace PMP = CGAL::Polygon_mesh_processing;

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Plane_3 = Kernel::Plane_3;
using Vector_2 = Kernel::Vector_2;
using Vector_3 = Kernel::Vector_3;

using Dt = CGAL::Delaunay_triangulation_2<Kernel>;
using Ht = CGAL::Triangulation_hierarchy_2<Dt>;


using Vb = CGAL::Alpha_shape_vertex_base_2<Kernel>;
using Fb = CGAL::Alpha_shape_face_base_2<Kernel>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using Triangulation = CGAL::Delaunay_triangulation_2<Kernel, Tds>;
using Alpha_shape = CGAL::Alpha_shape_2<Triangulation>;

//using Alpha_shape = CGAL::Alpha_shape_2<Ht>;

using Mesh = CGAL::Surface_mesh<Kernel>;
using Mesh_2 = CGAL::Surface_mesh<Kernel::Point_2>;
using Mesh_3 = CGAL::Surface_mesh<Kernel::Point_3>;

using Ray = RTCRayHit;
using Rays = std::vector<RTCRayHit>;
using Geometries = std::vector<RTCGeometry>;


namespace linkml
{
    class Surface
    {
    private:
        Plane_3 plane;
        Mesh mesh;
        Rays default_rays;

    public:
        Surface() = default;
        Surface(PointCloud::ConstPtr cloud, pcl::Indices indices);
        Rays Create_Embree_Geometry(RTCDevice device, RTCScene scene, std::unordered_map<unsigned int, pcl::Indices>& point_map);

    private:

        pcl::Indices supporting_points(typename Mesh::Face_index face, const Mesh& mesh, const PointCloud::ConstPtr cloud, const pcl::Indices indices);
        FT face_area(typename Mesh::Face_index f, const Mesh& mesh) const;
 

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
    Rays Surface::Create_Embree_Geometry(RTCDevice device, RTCScene scene, std::unordered_map<unsigned int, pcl::Indices>& point_map){

       //Geometries embree_geometry;
       Rays embree_rays;

       auto covered_area = mesh.property_map<Mesh::Face_index, FT>("f:covered_area").first;
       auto face_area = mesh.property_map<Mesh::Face_index, FT>("f:face_areas").first;
       auto face_point_indecies = mesh.property_map<Mesh::Face_index, pcl::Indices>("f:point_indecies").first;


       for (auto f : mesh.faces()){

           // Skip faces with too low coverage
           if (covered_area[f] < 0.5 * face_area[f])
               continue;

           // Create the geometry
           RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

           // Get the vertex buffer
           float* vb = (float*) rtcSetNewGeometryBuffer(geom,
               RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), 4);

           // Get the index buffer
           unsigned* ib = (unsigned*) rtcSetNewGeometryBuffer(geom,
               RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(unsigned), 2);


           const typename Mesh::template Property_map<typename Mesh::Vertex_index, Point_3>& coords = 
               mesh.property_map<typename Mesh::Vertex_index, Point_3>("v:points").first;

           typename Mesh::Vertex_around_face_circulator cir(mesh.halfedge(f), mesh), done_v(cir);

           Point_3 points[4];
           points[0] = coords[*cir]; ++cir;
           points[1] = coords[*cir]; ++cir;
           points[2] = coords[*cir]; ++cir;
           points[3] = coords[*cir];

           
           // Add the points to the vertex buffer
           for (int i = 0; i < 4; i++){
               vb[3*i+0] = points[i].x();
               vb[3*i+1] = points[i].y();
               vb[3*i+2] = points[i].z();
           }

           // Add the faces to the index buffer
           for (int j = 0; j < 2; j++){
               ib[3*j+0] = 0;
               ib[3*j+1] = j+1;
               ib[3*j+2] = j+2;
           }

           // Attach the geometry to the scene
           auto genomID = rtcAttachGeometry(scene, geom);

           // Store the face point indecies
           point_map[genomID] = face_point_indecies[f];

           //embree_geometry.push_back(geom);


           // Finde the avereage point of the face (Center of mass)
           FT X(0.0), Y(0.0), Z(0.0);
           for (int i = 0; i < 4; i++){
               X += points[i].x();
               Y += points[i].y();
               Z += points[i].z();
           }
           X /= 4; Y /= 4; Z /= 4;

           // Move the origin a bit away from the face
           X += plane.orthogonal_direction().dx() * FT(0.01);
           Y += plane.orthogonal_direction().dy() * FT(0.01);
           Z += plane.orthogonal_direction().dz() * FT(0.01);

           Point_3 origin = Point_3(X, Y, Z);

           // Create rays
           for (auto ray: default_rays){

                   RTCRayHit rayhit = RTCRayHit(ray);
                   rayhit.ray.org_x = origin.x();
                   rayhit.ray.org_y = origin.y();
                   rayhit.ray.org_z = origin.z(); 

                   rayhit.ray.id = genomID;
    
                   rayhit.ray.tnear  = 0.f;
                   rayhit.ray.tfar   = std::numeric_limits<float>::infinity();
                   rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

                   embree_rays.push_back(rayhit);
           }
       }

       return embree_rays;

    };  
    FT Surface::face_area(typename Mesh::Face_index f, const Mesh& mesh) const {
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
        #pragma omp parallel for
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
                Mesh::Vertex_index local_vertices[4];
                local_vertices[0] = vertices[i + j * (nx+1)];
                local_vertices[1] = vertices[i+1 + j * (nx+1)];
                local_vertices[2] = vertices[i+1 + (j+1) * (nx+1)];
                local_vertices[3] = vertices[i + (j+1) * (nx+1)];
                mesh.add_face(local_vertices[0], local_vertices[1], local_vertices[2], local_vertices[3]);
            }


        // Compute occupancy per face 
        auto face_areas = mesh.template add_property_map<Mesh::Face_index, FT>("f:face_areas").first;
        auto face_num_supporting_points = mesh.template add_property_map<Mesh::Face_index, std::size_t>("f:face_num_supporting_points").first;
        auto face_covered_areas = mesh.template add_property_map<Mesh::Face_index, FT>("f:covered_area").first;
        auto face_point_indecies = mesh.template add_property_map<Mesh::Face_index, pcl::Indices>("f:point_indecies").first;
        
        FT degenerate_face_area_threshold = CGAL::snap_squared_distance_threshold<FT>() * CGAL::snap_squared_distance_threshold<FT>();


        std::vector<typename Mesh::Face_index> faces;
        for (auto f : mesh.faces())
            faces.push_back(f);


        // Compute face areas and covered areas
        //auto bar = util::progress_bar(faces.size(), "Computing face areas");
        //#pragma omp parallel for
        #pragma omp parallel
        #pragma omp single
        for (size_t i = 0; i < faces.size(); i++){

            #pragma omp task
            {
            // Face area
            FT area = face_area(faces[i], mesh);
            face_areas[faces[i]] = area;

            if (area > degenerate_face_area_threshold) {
                const pcl::Indices idxs = supporting_points(faces[i], mesh, cloud, indices);

                face_point_indecies[faces[i]] = idxs;
                face_num_supporting_points[faces[i]] = idxs.size();

                std::vector<Point_2> pts;
                pts.reserve(idxs.size());
                for (std::size_t i = 0; i < idxs.size(); ++i) {
                        std::size_t idx = idxs[i];
                        auto point = cloud->points[idx].getVector3fMap();
                        pts.emplace_back(plane.to_2d(Point_3(point.x(), point.y(), point.z())));
                }

                FT covered_area(0);

                //face_covered_areas[faces[i]] = covered_area;
                //if (pts.size() < 3)
                //    continue;

                //std::cout << "Face " << i << " has " << idxs.size() << " supporting points" << std::endl;
                Alpha_shape alpha_shape(pts.begin(), pts.end());

                auto it = alpha_shape.finite_faces_begin();
                for(; it!=alpha_shape.finite_faces_end(); ++it){

                    if (alpha_shape.classify(it) != Alpha_shape::INTERIOR)
                        continue;

                    auto p0 = alpha_shape.point(it, 0);
                    auto p1 = alpha_shape.point(it, 1);
                    auto p2 = alpha_shape.point(it, 2);

                    covered_area += CGAL::area(p0, p1, p2);

                }


                face_covered_areas[faces[i]] = covered_area;
                if (covered_area > area)
                        face_covered_areas[faces[i]] = area;
            }
            else { // For tiny faces, we can simple assign zero supporting points
                face_num_supporting_points[faces[i]] = 0;
                face_covered_areas[faces[i]] = FT(0.0);
                //std::cout << "Skipping face " << i << " with area " << area << std::endl;
            }
            }
            //bar.update();
        }
        //bar.stop();

        // Create sphere with all possible vectors
        int n_slices = 10;
        int n_stacks = 10;

        Rays ray_sphere((n_slices-1)*n_stacks+1);

        // add north vetcor
        ray_sphere[0] = Ray();
        ray_sphere[0].ray.dir_x = 0;
        ray_sphere[0].ray.dir_y = 1;
        ray_sphere[0].ray.dir_z = 0;

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

                ray_sphere[i * n_slices + j + 1] = Ray();
                ray_sphere[i * n_slices + j + 1].ray.dir_x = x;
                ray_sphere[i * n_slices + j + 1].ray.dir_y = y;
                ray_sphere[i * n_slices + j + 1].ray.dir_z = z;

            }
        }

        // add south vetcor
        Ray last_ray = Ray();
        last_ray.ray.dir_x = 0;
        last_ray.ray.dir_y = -1;
        last_ray.ray.dir_z = 0;
        ray_sphere[(n_stacks - 1) * n_slices] = last_ray;
        //ray_sphere.push_back(Vector_3(0, -1, 0));

        // Filter rays that are pointing forward form the surface
        std::copy_if(ray_sphere.begin(), ray_sphere.end(), std::back_inserter(default_rays), [&](const Ray& ray){
            Vector_3 v = Vector_3(ray.ray.dir_x, ray.ray.dir_y, ray.ray.dir_z);
            Vector_3 dir = plane.orthogonal_vector();
            return CGAL::scalar_product(v, dir) < FT(-0.3);
        });

    }

} // namespace linkml



