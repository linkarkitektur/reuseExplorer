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

    constexpr double const_sin(double x)
    {
        double sin{0}, pow{x};
        for (auto fac{1LLU}, n{1ULL}; n != 20; fac *= ++n, pow *= x)
            if (n & 1)
                sin += (n & 2 ? -pow : pow) / fac;
        return sin;
    }

    constexpr double const_cos(double x)
    {
        double cos{1}, pow{x};
        for (auto fac{1ull}, n{1ull}; n != 19; fac *= ++n, pow *= x)
            if ((n & 1) == 0)
                cos += (n & 2 ? -pow : pow) / fac;
        return cos;
    }

    static constexpr auto CreateSphere(){

        constexpr int n_stacks = 10; // number of stacks
        constexpr int n_slices = 10; // number of slices

        std::array<Ray, (n_stacks-1)*n_slices+1> rays{};


        // add north vetcor
        rays[0].ray.dir_x = 0;
        rays[0].ray.dir_y = 1;
        rays[0].ray.dir_z = 0;

        // generate vertices per stack / slice
        for (int i = 0; i < n_stacks - 1; i++)
        {
            auto phi = M_PI * FT(i + 1) / FT(n_stacks);
            for (int j = 0; j < n_slices; j++)
            {
                auto theta = 2.0 * M_PI * FT(j) / FT(n_slices);

                auto x = const_sin(phi) * const_cos(theta);
                auto y = const_cos(phi);
                auto z = const_sin(phi) * const_sin(theta);

                rays[i * n_slices + j + 1].ray.dir_x = x;
                rays[i * n_slices + j + 1].ray.dir_y = y;
                rays[i * n_slices + j + 1].ray.dir_z = z;
            }
        }


        // add south vetcor
        int last_ray_index = (n_stacks - 1) * n_slices;
        rays[last_ray_index].ray.dir_x = 0;
        rays[last_ray_index].ray.dir_y = -1;
        rays[last_ray_index].ray.dir_z = 0;

        return rays;
    }
    
    class Surface
    {
    private:

        Plane_3 plane;
        Mesh mesh;
        Rays default_rays;

        // The size of the tiles
        float tile_x_size = 0.4;
        float tile_y_size = 0.4;
            
        static const constexpr auto ray_sphere = CreateSphere();

    public:
        Surface() = default;
        Surface(PointCloud::Ptr cloud, pcl::Indices indices);
        void Create_Embree_Geometry(RTCDevice & device, RTCScene & scene, std::unordered_map<unsigned int, pcl::Indices>& point_map, std::back_insert_iterator<Rays> rays);

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
    void Surface::Create_Embree_Geometry(RTCDevice & device, RTCScene & scene, std::unordered_map<unsigned int, pcl::Indices>& point_map, std::back_insert_iterator<Rays> rays){


        //auto covered_area = mesh.property_map<Mesh::Face_index, FT>("f:covered_area").first;
        //auto face_area = mesh.property_map<Mesh::Face_index, FT>("f:face_areas").first;
        auto face_point_indecies = mesh.property_map<Mesh::Face_index, pcl::Indices>("f:point_indecies").first;
        auto face_num_supporting_points = mesh.property_map<Mesh::Face_index, std::size_t>("f:face_num_supporting_points").first;

       

        for (auto f : mesh.faces()){
            

            // FIXME: The area is not correct
            // So the check if skipped. 
            // Skip faces with too low coverage
            //if (covered_area[f] < 0.5 * face_area[f])
            //    continue;

            if (face_num_supporting_points[f] == 0)
                continue;

            // Create the geometry
            RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_QUAD);

            // Get the vertex buffer
            float* vb = (float*) rtcSetNewGeometryBuffer(geom,
                RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), 4);

            // Get the index buffer
            unsigned* ib = (unsigned*) rtcSetNewGeometryBuffer(geom,
                RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, 4*sizeof(unsigned), 1);

            const typename Mesh::template Property_map<typename Mesh::Vertex_index, Point_3>& coords = 
                mesh.property_map<typename Mesh::Vertex_index, Point_3>("v:points").first;

            typename Mesh::Vertex_around_face_circulator cir(mesh.halfedge(f), mesh), done_v(cir);

            Point_3 points[4];
            points[0] = coords[*cir]; ++cir;
            points[1] = coords[*cir]; ++cir;
            points[2] = coords[*cir]; ++cir;
            points[3] = coords[*cir];

           
            // Add the points to the vertex buffer
            vb[0] = points[0].x(); vb[1]  = points[0].y(); vb[2]  = points[0].z();
            vb[3] = points[1].x(); vb[4]  = points[1].y(); vb[5]  = points[1].z();
            vb[6] = points[2].x(); vb[7]  = points[2].y(); vb[8]  = points[2].z();
            vb[9] = points[3].x(); vb[10] = points[3].y(); vb[11] = points[3].z();
            

            // Add the faces to the index buffer
            ib[0] = 0;
            ib[1] = 1;
            ib[2] = 2;
            ib[3] = 3;





            // Attach the geometry to the scene
            rtcCommitGeometry(geom);
            auto genomID = rtcAttachGeometry(scene, geom);
            rtcReleaseGeometry(geom);

            // Store the face point indecies
            point_map[genomID] = face_point_indecies[f];


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

            std::transform(default_rays.begin(), default_rays.end(), rays, [&](const Ray& ray){


                RTCRayHit rayhit;

                rayhit.ray.org_x = origin.x();
                rayhit.ray.org_y = origin.y();
                rayhit.ray.org_z = origin.z();
                rayhit.ray.id = genomID;
                
                rayhit.ray.dir_x = ray.ray.dir_x;
                rayhit.ray.dir_y = ray.ray.dir_y;
                rayhit.ray.dir_z = ray.ray.dir_z;

                rayhit.ray.tnear  = 0.f;
                rayhit.ray.tfar   = std::numeric_limits<float>::infinity();
                rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

                return rayhit;
            });

            // Set the intersect filter function
            // https://spec.oneapi.io/oneart/0.5-rev-1/embree-spec.html#rtcsetgeometryintersectfilterfunction


            RTCGeometry sphere = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SPHERE_POINT);

             // Get the vertex buffer (x,y,z,r)
            float* s_vb = (float*) rtcSetNewGeometryBuffer(sphere,
                RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4*sizeof(float), 1);
            
            s_vb[0] = origin.x();
            s_vb[1] = origin.y();
            s_vb[2] = origin.z();
            s_vb[3] = 0.4;

          

       }


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
    Surface::Surface(PointCloud::Ptr cloud, pcl::Indices indices){

        // Construct Plane
        double x, y, z, n_x, n_y, n_z = 0;


        // TODO: Avoid impresission due to large intermediate values
        // Theoretically this can cause the sum of the points is to be very 
        // large resulting in inpersision due to the floating point arithmetics.
        // Ideally the sum should be done in buckets
        #pragma omp parallel for reduction(+:x,y,z, n_x, n_y, n_z)
        for (size_t i = 0; i < indices.size(); i++){

            x += cloud->points[indices[i]].x;
            y += cloud->points[indices[i]].y;
            z += cloud->points[indices[i]].z;

            auto norm = cloud->points[indices[i]].normal;
            n_x += norm[0];
            n_y += norm[1];
            n_z += norm[2];
        }

        x /= indices.size();
        y /= indices.size();
        z /= indices.size();
        n_x /= indices.size();
        n_y /= indices.size();
        n_z /= indices.size();


        Eigen::Vector3f centroid(x, y, z);
        Eigen::Vector3f normal(n_x, n_y, n_z);

        normal.normalize();

        plane = Plane_3(normal[0], normal[1], normal[2], -1 * normal.dot(centroid));


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
        int nx = dx / tile_x_size;
        int ny = dy / tile_y_size;
        

        // Create Surface Mesh
        size_t num_verticies = (nx+1) * (ny+1);
        std::vector<Mesh::Vertex_index> vertices(num_verticies);
        for (size_t i = 0; i < num_verticies; i++)
            vertices[i] = mesh.add_vertex();

        auto points = mesh.template add_property_map<Mesh::Vertex_index, Point_3>("v:points").first;

        for (int i = 0; i < nx+1; i++)
            for (int j = 0; j < ny+1; j++)
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

            }
            else { // For tiny faces, we can simple assign zero supporting points
                face_num_supporting_points[faces[i]] = 0;
                face_covered_areas[faces[i]] = FT(0.0);
            }
            }
        }


        // Filter rays that are pointing forward form the surface
        std::copy_if(ray_sphere.begin(), ray_sphere.end(), std::back_inserter(default_rays), [&](const Ray& ray){
            Vector_3 v = Vector_3(ray.ray.dir_x, ray.ray.dir_y, ray.ray.dir_z);
            Vector_3 dir = plane.orthogonal_vector();
            return CGAL::scalar_product(v, dir) > FT(0.3);
        });


    }

} // namespace linkml



