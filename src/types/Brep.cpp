#include "types/Brep.hh"

#include <typed-geometry/tg.hh>

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/aabb.hh>
#include <typed-geometry/types/objects/quad.hh>

#include <typed-geometry/functions/objects/area.hh>
#include <typed-geometry/functions/objects/aabb.hh>

#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh/IO/OFF.h>
#include <CGAL/Optimal_bounding_box/oriented_bounding_box.h>

#include "types/Plane.hh"
#include "functions/fit_plane_thorugh_points.hh"

#include <vector>
#include <algorithm>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace linkml
{
    static void compute_planes(Surface_mesh const& mesh, Surface_mesh::Property_map<Surface_mesh::Face_index, linkml::Plane> const& face_planes)
    {
        for (auto f : mesh.faces())
        {
            auto vertices = mesh.vertices_around_face(mesh.halfedge(f));
            auto points = std::vector<tg::pos3>();
            std::transform(vertices.begin(), vertices.end(), std::back_inserter(points), [&mesh](auto v) {
                auto p = mesh.point(v);
                return tg::pos3(p.x(), p.y(), p.z());
            });

            auto plane = fit_plane_thorugh_points(points);

            // auto plane = tg::plane_from_points(points[0], points[1], points[2]);

            face_planes[f] = plane;
        }
    }
    
    template <typename Mesh, typename Face_Handel>
    static auto compute_plane(Mesh mesh, Face_Handel f ){
            auto normal = PMP::compute_face_normal(f, mesh);
            typename Mesh::Vertex_index v = *mesh.vertices_around_face(mesh.halfedge(f)).begin();
            Kernel::Point_3 p = mesh.point(v);
            return Kernel::Plane_3(p, normal);
    }

    template <typename Mesh, typename Face_Handel>
    double compute_angle_between_faces(Mesh& mesh, Face_Handel f1, Face_Handel f2) {
        Vector_3 normal1 = PMP::compute_face_normal(f1, mesh);
        Vector_3 normal2 = PMP::compute_face_normal(f2, mesh);
        double cosine_angle = normal1 * normal2 / (std::sqrt(normal1.squared_length()) * std::sqrt(normal2.squared_length()));
        return std::acos(cosine_angle);
    }

    template  <typename Mesh, typename Vertex>
    bool is_planar(Mesh& mesh, Vertex v1, Vertex v2, Vertex v3, Vertex v4) {
        using PointT = typename Mesh::PointT;

        PointT p1 = mesh.point(v1);
        PointT p2 = mesh.point(v2);
        PointT p3 = mesh.point(v3);
        PointT p4 = mesh.point(v4);

        Vector_3 v12 = p2 - p1;
        Vector_3 v13 = p3 - p1;
        Vector_3 v14 = p4 - p1;

        Vector_3 normal = CGAL::cross_product(v12, v13);
        double d = -CGAL::scalar_product(normal, p1);

        double dist = CGAL::abs(CGAL::scalar_product(normal, p4) + d) / std::sqrt(normal.squared_length());
        return dist < 1e-6;
    }
        


    template <typename Mesh>
    static void unweld(Mesh& mesh){


        using Halfedge_index = typename Mesh::Halfedge_index;
        using Edge_index = typename Mesh::Edge_index;
        using Vertex_index = typename Mesh::Vertex_index;
        using Face_index = typename Mesh::Face_index;

        double angle_threshold = CGAL_PI / 4; // Example threshold of 45 degrees

        std::unordered_set<Edge_index> edges_to_unweld;
        for (Edge_index e : mesh.edges()) {
            Halfedge_index h = mesh.halfedge(e);
            Halfedge_index opposite_h = mesh.opposite(h);
            if (!mesh.is_border(h) && !mesh.is_border(opposite_h)) {
                Face_index f1 = mesh.face(h);
                Face_index f2 = mesh.face(opposite_h);
                double angle = compute_angle_between_faces(mesh, f1, f2);
                if (angle > angle_threshold) {
                    edges_to_unweld.insert(e);
                }
            }
        }

        for (Edge_index e : edges_to_unweld) {
            Halfedge_index h = mesh.halfedge(e);
            Halfedge_index opposite_h = mesh.opposite(h);

            // Unweld vertices of the first face
            Face_index f1 = mesh.face(h);
            std::vector<Vertex_index> new_vertices;
            for (Halfedge_index v_h : halfedges_around_face(mesh.halfedge(f1), mesh)) {
                Vertex_index v = mesh.target(v_h);
                Point_3 p = mesh.point(v);
                Vertex_index new_v = mesh.add_vertex(p);
                new_vertices.push_back(new_v);
            }
            Halfedge_index start = mesh.halfedge(f1);
            Halfedge_index curr = start;
            int i = 0;
            do {
                mesh.set_target(curr, new_vertices[i]);
                curr = mesh.next(curr);
                ++i;
            } while (curr != start);

            // Unweld vertices of the second face
            Face_index f2 = mesh.face(opposite_h);
            new_vertices.clear();
            for (Halfedge_index v_h : halfedges_around_face(mesh.halfedge(f2), mesh)) {
                Vertex_index v = mesh.target(v_h);
                Point_3 p = mesh.point(v);
                Vertex_index new_v = mesh.add_vertex(p);
                new_vertices.push_back(new_v);
            }
            start = mesh.halfedge(f2);
            curr = start;
            i = 0;
            do {
                mesh.set_target(curr, new_vertices[i]);
                curr = mesh.next(curr);
                ++i;
            } while (curr != start);
        }
    }

    Brep::Brep(Surface_mesh const& mesh) : mesh(mesh){
        this->mesh.add_property_map<Surface_mesh::Face_index, Kernel::Point_2>("f:origin", Kernel::Point_2(0,0));
    }

    void Brep::save(std::string const& filename) const { 
        std::ofstream file(filename);
        CGAL::IO::write_OFF(file, mesh);
    }

    Brep Brep::load(std::string const& filename) {


        std::ifstream file(filename);
        Surface_mesh mesh;
        CGAL::IO::read_OFF(file, mesh);

        return Brep(mesh);
    }

    float Brep::volume() const { return PMP::volume(mesh);}
    
    float Brep::area() const { return PMP::area(mesh);}
    
    bool Brep::is_closed() const { return CGAL::is_closed(mesh);}
    
    tg::aabb3 Brep::get_bbox() const { 
        auto box = CGAL::bounding_box(mesh.points().begin(), mesh.points().end());
        return tg::aabb3(tg::pos3(box.xmin(), box.ymin(), box.zmin()), tg::pos3(box.xmax(), box.ymax(), box.zmax()));
    }
    
    int Brep::get_Orientation() const { return 1;}
    
    LinkMesh Brep::get_Mesh() const {
        auto mesh_copy = Surface_mesh(mesh);
        PMP::triangulate_faces(mesh_copy);
        PMP::orient(mesh_copy);
        PMP::merge_reversible_connected_components(mesh_copy);
        PMP::reverse_face_orientations(mesh_copy);
        unweld(mesh_copy);
        PMP::orient(mesh_copy);
        return LinkMesh(mesh_copy);
    }

    Brep::Curves2D Brep::get_Curves2D() const {

        auto face_origin = this->mesh.property_map<Surface_mesh::Face_index, Kernel::Point_2>("f:origin").first;

        Curves2D curves = Curves2D();
        curves.resize(mesh.number_of_halfedges());

        for (auto h : mesh.halfedges()){

            Brep::Curve2D curve = std::vector<tg::pos2>(2);


            auto v1 = mesh.source(h);
            auto v2 = mesh.target(h);

            auto plane = compute_plane(mesh, mesh.face(h));

            // Alight geometry to surace
            auto o = face_origin[mesh.face(h)];
            Kernel::Vector_2 Ov = Kernel::Vector_2(o.x(), o.y());
   
            auto pt1 = plane.to_2d(mesh.point(v1)) - Ov;
            auto pt2 = plane.to_2d(mesh.point(v2)) - Ov;

            curve[1] = tg::pos2(pt1.x(), pt1.y());
            curve[0] = tg::pos2(pt2.x(), pt2.y());

            curves[h.idx()] = curve;
            
        }
        return curves;
    }

    Brep::Curves3D Brep::get_Curves3D() const {
        Curves3D curves = Curves3D();

        curves.resize(mesh.number_of_edges());
        for (auto e : mesh.edges())
        {
            auto curve = std::vector<tg::pos3>(2);
            auto h = mesh.halfedge(e, 0);

            auto p1 = mesh.point(mesh.source(h));
            auto p2 = mesh.point(mesh.target(h));
            
            curve[0] = tg::pos3(p1.x(), p1.y(), p1.z());
            curve[1] = tg::pos3(p2.x(), p2.y(), p2.z());

            curves[e.idx()] = curve;
        }
        return curves;
    }

    Brep::Edges Brep::get_Edges() const { 
        Edges edges = Edges();
        edges.resize(mesh.num_edges());
        for (auto e : mesh.edges())
        {
            auto edge = Brep::Edge();
            edge.Curve3dIndex = e.idx();

            auto h = mesh.halfedge(e);
            edge.TrimIndices.resize(2);
            edge.TrimIndices[0] = h.idx();
            edge.TrimIndices[1] = mesh.opposite(h).idx(); 

            auto v1 = mesh.vertex(e, 1);
            auto v2 = mesh.vertex(e, 0);
            auto p1 = mesh.point(v1);
            auto p2 = mesh.point(v2);

            edge.StartIndex = v1.idx();
            edge.EndIndex = v2.idx();
            edge.ProxyCurveIsReversed = false;

            edge.Domain = Interval(0, tg::distance(tg::pos3(p1.x(), p1.y(), p1.z()),tg::pos3(p2.x(), p2.y(), p2.z())));

            edge.Curve3dIndex = e.idx();
            edges[e.idx()] = edge;
        }
        return edges;
    }
    
    Brep::Faces Brep::get_Faces() const { 
        Faces faces = Faces();
        faces.resize(mesh.number_of_faces());
        for (auto f : mesh.faces())
        {
            auto face = Brep::Face();

            face.SurfaceIndex = f.idx();
            face.OuterLoopIndex =  f.idx();
            face.LoopIndices = f.idx();
            faces[f.idx()] = face;
        }

        return faces;
    }

    Brep::Vertices Brep::get_Vertices() const {
        Vertices vertices = Vertices();
        vertices.resize(mesh.number_of_vertices());
        for (auto v : mesh.vertices()){   
            auto p = mesh.point(v);
            vertices[v.idx()] = tg::pos3(p.x(), p.y(), p.z());
        }
        return vertices;
    }

    Brep::Surfaces Brep::get_Surfaces() const { 

        auto surface_origin = this->mesh.property_map<Surface_mesh::Face_index, Kernel::Point_2>("f:origin").first;

        Surfaces surfaces = Surfaces();
        surfaces.resize(mesh.number_of_faces());
        for (auto f : mesh.faces())
        {
            auto surface = Brep::Surface();
            surface.degreeU = 1;
            surface.degreeV = 1;
            surface.rational = false;
            surface.countU = 2;
            surface.countV = 2;
            surface.closedU = false;
            surface.closedV = false;
            surface.pointData.reserve(4/*Points*/ * 4/*Fields*/);


            // Get all the points of the face
            std::vector<Kernel::Point_3> points;
            for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
                points.push_back(mesh.point(v));

            auto normal = PMP::compute_face_normal(f, mesh);
            CGAL::Plane_3 plane = Kernel::Plane_3(points[0], normal);

            std::vector<Kernel::Point_2> points_2d;
            for (auto p: points)
                points_2d.push_back(plane.to_2d(p));


            auto bbox = CGAL::bounding_box(points_2d.begin(), points_2d.end());


            // Offset bbox
            // TODO: Find a way to calculate the correct offset.
            // This is need for surfaces that are twisted in regards to the trum curves
            static const double offset = 50;
            bbox = CGAL::Bbox_2(bbox.xmin()-offset, bbox.ymin()-offset, bbox.xmax()+offset, bbox.ymax()+offset);

            points.clear();
            points.resize(4);
            points[0] = plane.to_3d(Kernel::Point_2(bbox.xmin(), bbox.ymin()));
            points[2] = plane.to_3d(Kernel::Point_2(bbox.xmax(), bbox.ymin()));
            points[1] = plane.to_3d(Kernel::Point_2(bbox.xmin(), bbox.ymax()));
            points[3] = plane.to_3d(Kernel::Point_2(bbox.xmax(), bbox.ymax()));

            // 0 1 2 3 <= Open planar surface normal wrong way
            // 0 1 3 2 <= Invalid, twisted
            // 0 2 1 3 <= Invalid, propper size !
            // 0 2 3 1 <= Invalid, twisted
            // 0 3 1 2 <= Invalid, twisted 
            // 0 3 2 1 <= Invalid, twisted

            // 1 0 2 3 <= Closed, but small
            // 1 0 3 2 <= Closed, but small
            // 1 2 0 3 <= Invalid, twisted
            // 1 2 3 0 <= Invalid, twisted
            // 1 3 0 2 <= Invalid, flat disk
            // 1 3 2 0 <= Invalid, twisted

            // 2 0 1 3 <= Invalid, twisted
            // 2 0 3 1 <= Invalid, twisted
            // 2 1 0 3 <= Closed but twisted
            // 2 1 3 0 <= Invalid, twisted
            // 2 3 0 1 <= Closed, but small
            // 2 3 1 0 <= Clused, but twisted

            // 3 0 1 2 <= Invalid, twisted
            // 3 0 2 1 <= Invalid, twisted
            // 3 1 0 2 <= Invalid, twisted
            // 3 1 2 0 <= Ivalid and but small
            // 3 2 0 1 <= Invalid, twisted 
            // 3 2 1 0 <= Invalid, prisim




            for (auto & p: points)
            {
                surface.pointData.push_back(p.x());
                surface.pointData.push_back(p.y());
                surface.pointData.push_back(p.z());
                surface.pointData.push_back(1.0);
            }

            surface_origin[f] = Kernel::Point_2(bbox.xmin(), bbox.ymin());
     
            float distU = CGAL::sqrt(CGAL::squared_distance(points[0], points[1]));
            float distV = CGAL::sqrt(CGAL::squared_distance(points[0], points[2]));
            surface.domainU = Interval(0, distU);
            surface.domainV = Interval(0, distV);
            surface.knotsU = std::vector<float>{0, distU}; 
            surface.knotsV = std::vector<float>{0, distV};

            surfaces[f.idx()] = surface;
        }
        return surfaces;
    }

    Brep::Loops Brep::get_Loops() const { 
        Loops loops = Loops();
        loops.resize(mesh.number_of_faces());
        for (auto f : mesh.faces()){
            auto loop = Brep::BrepLoop();
            loop.FaceIndex = f.idx();
            loop.Type = Brep::BrepLoop::BrepLoopType::Outer;
            loop.TrimIndices = std::vector<int>();


            for (auto h: mesh.halfedges_around_face(mesh.halfedge(f)))
                loop.TrimIndices.push_back(h.idx());
            std::reverse(loop.TrimIndices.begin(), loop.TrimIndices.end());

            
            loops[f.idx()] = loop;
        }
        return loops;
    }

    Brep::Trims Brep::get_Trims() const { 
        Trims trims = Trims();
        trims.resize(mesh.num_halfedges());
        for (auto h : mesh.halfedges())
        {
            auto trim = Brep::BrepTrim();

            // 3D curve index
            trim.EdgeIndex = mesh.edge(h).idx();


            trim.StartIndex = mesh.source(h).idx();
            trim.EndIndex = mesh.target(h).idx();


            trim.FaceIndex = mesh.face(h).idx();
            trim.LoopIndex = mesh.face(h).idx();

            // 2D curve index
            trim.CurveIndex = h.idx();// mesh.edge(h).idx();
 
            trim.IsoStatus = 0; //What is this? --> https://developer.rhino3d.com/api/rhinocommon/rhino.geometry.isostatus
            trim.TrimType = Brep::BrepTrim::BrepTrimType::Mated;
            
            // This values needs to be set
            auto e = mesh.edge(h);
            trim.IsReversed =  ( mesh.vertex(e,0) == mesh.source(h) )? true : false;

            auto p1 = mesh.point(mesh.source(h));
            auto p2 = mesh.point(mesh.target(h));
            auto dist = tg::distance(tg::pos3(p1.x(), p1.y(), p1.z()),tg::pos3(p2.x(), p2.y(), p2.z()));
            trim.Domain = Interval(0, dist);

            trims[h.idx()] = trim;
        }   

        return trims;
    }

} // namespace linkml