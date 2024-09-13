#include "types/surface.hh"

#include <CGAL/bounding_box.h>


namespace linkml
{
    
    // Implementation
    std::pair<pcl::Indices, size_t> Surface::supporting_points(typename Mesh::Face_index face, const Mesh& mesh, const PointCloud::Cloud::ConstPtr cloud, const pcl::Indices indices_in) {

        pcl::Indices indices;
        if (face == Mesh::null_face())
                return std::make_pair(indices,0);

        
        typename Mesh::Vertex_around_face_circulator cir(mesh.halfedge(face), mesh);
        Point_3 points_3d[4];
        points_3d[0] = mesh.point(*cir); ++cir;
        points_3d[1] = mesh.point(*cir); ++cir;
        points_3d[2] = mesh.point(*cir); ++cir;
        points_3d[3] = mesh.point(*cir);

        
        std::array<Point_2, 4> points_2d;
        for (int i = 0; i < 4; i++)
            points_2d[i] = plane.to_2d(points_3d[i]); 

        auto bbox = CGAL::bounding_box(points_2d.begin(), points_2d.end());

        size_t count = 0;
        for (size_t i = 0; i < indices_in.size(); i++){

            auto point = cloud->points[indices_in[i]].getVector3fMap();
            Point_2 p = plane.to_2d(Point_3(point[0], point[1], point[2]));
        
            if ( bbox.xmin() <= p.x() && bbox.xmax() >= p.x() && bbox.ymin() <= p.y() && bbox.ymax() >= p.y() ){
                indices.push_back(indices_in[i]);
                count++;
            }

        }

        return std::make_pair(indices, count);
    }
    void Surface::Create_Embree_Geometry(RTCDevice & device, RTCScene & scene){

        auto num_supporting_points = mesh.property_map<Mesh::Face_index, size_t>(num_supporting_points_name).value();

        for (auto f : mesh.faces()){
            
            if (num_supporting_points[f] == 0 )
                continue;

            // Create the geometry
            RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_QUAD);

            // Get the vertex buffer
            float* vb = (float*) rtcSetNewGeometryBuffer(geom,
                RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), 4);

            // Get the index buffer
            unsigned* ib = (unsigned*) rtcSetNewGeometryBuffer(geom,
                RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, 4*sizeof(unsigned), 1);

            typename Mesh::Vertex_around_face_circulator cir(mesh.halfedge(f), mesh); 

            Point_3 points[4];
            points[0] = mesh.point(*cir); ++cir;
            points[1] = mesh.point(*cir); ++cir;
            points[2] = mesh.point(*cir); ++cir;
            points[3] = mesh.point(*cir);

           
            // Add the points to the vertex buffer
            vb[0] = CGAL::to_double(points[0].x()); vb[1]  = CGAL::to_double(points[0].y()); vb[2]  = CGAL::to_double(points[0].z());
            vb[3] = CGAL::to_double(points[1].x()); vb[4]  = CGAL::to_double(points[1].y()); vb[5]  = CGAL::to_double(points[1].z());
            vb[6] = CGAL::to_double(points[2].x()); vb[7]  = CGAL::to_double(points[2].y()); vb[8]  = CGAL::to_double(points[2].z());
            vb[9] = CGAL::to_double(points[3].x()); vb[10] = CGAL::to_double(points[3].y()); vb[11] = CGAL::to_double(points[3].z());
            

            // Add the faces to the index buffer
            ib[0] = 0;
            ib[1] = 1;
            ib[2] = 2;
            ib[3] = 3;


            // Attach the geometry to the scene
            rtcCommitGeometry(geom);
            auto genomID = rtcAttachGeometry(scene, geom);
            rtcReleaseGeometry(geom);


            valid[genomID] = f;


       }


    
    };  
    Point_3 Surface::compute_centroid(typename Mesh::Face_index f, const Mesh& mesh) const {

            // Set the default values
            Point_3 centroid(0,0,0);

            // Get the vertex buffer
            typename Mesh::Vertex_around_face_circulator cir(mesh.halfedge(f), mesh), done_v(cir);

            // Get the vertex coordinates
            Point_3 points[4];
            points[0] = mesh.point(*cir); ++cir;
            points[1] = mesh.point(*cir); ++cir;
            points[2] = mesh.point(*cir); ++cir;
            points[3] = mesh.point(*cir);

           
            // Compute the centroid
            FT X(0.0), Y(0.0), Z(0.0);
            for (int i = 0; i < 4; i++){
                X += points[i].x();
                Y += points[i].y();
                Z += points[i].z();
            }
            X /= 4; Y /= 4; Z /= 4;
            centroid = Point_3(X, Y, Z);

            return centroid;

    }
    Surface::Surface(
        PointCloud::Cloud::Ptr cloud,
        pcl::Indices indices,
        double tile_x_size,
        double tile_y_size
    ) : mesh() {



        // Initialize the Mesh properties
        // auto coords = mesh.template add_property_map<Mesh::Vertex_index, Point_3>(coords_name).first;
        auto num_supporting_points = mesh.template add_property_map<Mesh::Face_index, std::size_t>(num_supporting_points_name, 0).first;
        auto indecies = mesh.template add_property_map<Mesh::Face_index, pcl::Indices>(indecies_name).first;
        auto centeroids = mesh.template add_property_map<Mesh::Face_index, Point_3>(centeroids_name).first;


        // Construct Plane
        double x=0, y=0, z=0, n_x=0, n_y=0, n_z=0;


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

        // Offset the bounding box
        // Ofthen there is a gap between surfaces, allowing stray rays to escape
        // muddeling the results
        FT offset = 0.15;
        bbox = CGAL::Bbox_2(
            CGAL::to_double(bbox.xmin() - offset),
            CGAL::to_double(bbox.ymin() - offset),
            CGAL::to_double(bbox.xmax() + offset),
            CGAL::to_double(bbox.ymax() + offset));

        // Subdivide Bounding Box
        double dx  = CGAL::to_double(bbox.xmax() - bbox.xmin());
        double dy  = CGAL::to_double(bbox.ymax() - bbox.ymin());
        int nx = dx / tile_x_size;
        int ny = dy / tile_y_size;
        

        // Create Surface Mesh
        size_t num_verticies = (nx+1) * (ny+1);
        std::vector<Mesh::Vertex_index> vertices(num_verticies);
        for (size_t i = 0; i < num_verticies; i++)
            vertices[i] = mesh.add_vertex();


        for (int i = 0; i < nx+1; i++)
            for (int j = 0; j < ny+1; j++){
                // coords[vertices[i + j * (nx+1)]] = plane.to_3d(Point_2(bbox.xmin() + i * dx / nx, bbox.ymin() + j * dy / ny));
                mesh.point(vertices[i + j * (nx+1)]) = plane.to_3d(Point_2(bbox.xmin() + i * dx / nx, bbox.ymin() + j * dy / ny));
            }
            
        
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
        std::vector<typename Mesh::Face_index> faces;
        for (auto f : mesh.faces())
            faces.push_back(f);


        // Compute face areas and covered areas
        #pragma omp parallel for
        for (size_t i = 0; i < faces.size(); i++){

            auto f = faces[i];

            // Face centroid
            const auto & [idx, count] = supporting_points(f, mesh, cloud, indices);

            indecies[f] = idx;
            num_supporting_points[f] = count;

            centeroids[f] = compute_centroid(f, mesh);
        }

    }
}