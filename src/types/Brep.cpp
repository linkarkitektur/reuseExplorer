#include "types/Brep.hh"
#include "functions/polyscope.hh"

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

// defining OPENNURBS_PUBLIC_INSTALL_DIR enables automatic linking using pragmas
#define OPENNURBS_PUBLIC_INSTALL_DIR "../extern/opennurbs"
// uncomment the next line if you want to use opennurbs as a DLL
//#define OPENNURBS_IMPORTS
#include "../extern/opennurbs/opennurbs_public.h"

namespace PMP = CGAL::Polygon_mesh_processing;

namespace linkml
{
 
    template <typename Mesh, typename Face_Handel>
    auto compute_plane(Mesh mesh, Face_Handel f ){
            auto normal = PMP::compute_face_normal(f, mesh);
            typename Mesh::Vertex_index v = *mesh.vertices_around_face(mesh.halfedge(f)).begin();
            Kernel::Point_3 p = mesh.point(v);
            return Kernel::Plane_3(p, normal);
    }

    template <typename Mesh>
    void unweld(Mesh& mesh){

        using Vertex_index = typename Mesh::Vertex_index;
        using Face_index = typename Mesh::Face_index;

        for (auto f: mesh.faces()){

            std::vector<Vertex_index> vertices;
            for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
                vertices.push_back(v);
            
            for (size_t i = 0; i < vertices.size(); i++)
                vertices[i] =  mesh.add_vertex(mesh.point(vertices[i]));
            
            mesh.add_face(vertices);
            mesh.remove_face(f);
        }
        mesh.collect_garbage();
    }


    Brep::Brep(Surface_mesh const& mesh) : mesh(mesh){
        this->mesh.add_property_map<Surface_mesh::Face_index, Kernel::Point_2>("f:origin", Kernel::Point_2(0,0));

        // ON_Mesh ONmesh = ON_Mesh();
        // for (auto v: mesh.vertices())
        //     ONmesh.SetVertex(v.idx(), ON_3dPoint(mesh.point(v).x(), mesh.point(v).y(), mesh.point(v).z()));


        // for (auto f: mesh.faces()){
        //     ON_SimpleArray<unsigned int> face;
        //     for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
        //         face.Append(v.idx());
        //     ONmesh.AddNgon(face);
        // }
        
        // ON_Brep* pBrep = ON_BrepFromMeshWithNgons(ONmesh.Topology(), false, true, 0.000001);

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

    double Brep::volume() const { return CGAL::to_double(PMP::volume(mesh));}
    
    double Brep::area() const {
        //TODO: Consider computing the footprint area
        return CGAL::to_double(PMP::area(mesh));
    }
    
    bool Brep::is_closed() const { return CGAL::is_closed(mesh);}
    
    tg::aabb3 Brep::get_bbox() const { 
        auto box = CGAL::bounding_box(mesh.points().begin(), mesh.points().end());
        return tg::aabb3(
            tg::pos3(
                CGAL::to_double(box.xmin()), 
                CGAL::to_double(box.ymin()), 
                CGAL::to_double(box.zmin())
            ), 
            tg::pos3(
                CGAL::to_double(box.xmax()), 
                CGAL::to_double(box.ymax()), 
                CGAL::to_double(box.zmax()))
            );
    }
    
    int Brep::get_Orientation() const { return 1;}
    
    LinkMesh Brep::get_Mesh() const {
        auto mesh_copy = Surface_mesh(mesh);
        unweld(mesh_copy);
        PMP::triangulate_faces(mesh_copy);
        return LinkMesh(mesh_copy);
    }

    Brep::Curves2D Brep::get_Curves2D() const {

        auto face_origin = this->mesh.property_map<Surface_mesh::Face_index, Kernel::Point_2>("f:origin").value();

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

            curve[1] = tg::pos2(CGAL::to_double(pt1.x()), CGAL::to_double(pt1.y()));
            curve[0] = tg::pos2(CGAL::to_double(pt2.x()), CGAL::to_double(pt2.y()));

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
            
            curve[0] = tg::pos3(CGAL::to_double(p1.x()), CGAL::to_double(p1.y()), CGAL::to_double(p1.z()));
            curve[1] = tg::pos3(CGAL::to_double(p2.x()), CGAL::to_double(p2.y()), CGAL::to_double(p2.z()));

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

            edge.Domain = Interval(0, tg::distance(tg::pos3(CGAL::to_double(p1.x()), CGAL::to_double(p1.y()), CGAL::to_double(p1.z())),tg::pos3(CGAL::to_double(p2.x()), CGAL::to_double(p2.y()), CGAL::to_double(p2.z()))));

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
            vertices[v.idx()] = tg::pos3(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
        }
        return vertices;
    }

    Brep::Surfaces Brep::get_Surfaces() const { 

        auto surface_origin = this->mesh.property_map<Surface_mesh::Face_index, Kernel::Point_2>("f:origin").value();

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
            // Some surfaces seem to require an offset
            static const double offset = 0.3;
            bbox = CGAL::Bbox_2(
                CGAL::to_double(bbox.xmin()-offset), 
                CGAL::to_double(bbox.ymin()-offset), 
                CGAL::to_double(bbox.xmax()+offset), 
                CGAL::to_double(bbox.ymax()+offset));

            points.clear();
            points.resize(4);
            points[0] = plane.to_3d(Kernel::Point_2(bbox.xmin(), bbox.ymin()));
            points[1] = plane.to_3d(Kernel::Point_2(bbox.xmin(), bbox.ymax()));
            points[2] = plane.to_3d(Kernel::Point_2(bbox.xmax(), bbox.ymin()));
            points[3] = plane.to_3d(Kernel::Point_2(bbox.xmax(), bbox.ymax()));


            for (auto & p: points)
            {
                surface.pointData.push_back(CGAL::to_double(p.x()));
                surface.pointData.push_back(CGAL::to_double(p.y()));
                surface.pointData.push_back(CGAL::to_double(p.z()));
                surface.pointData.push_back(1.0);
            }

            surface_origin[f] = Kernel::Point_2(bbox.xmin(), bbox.ymin());
     
            float distU = bbox.xmax()-bbox.xmin();
            float distV = bbox.ymax()-bbox.ymin();
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
            auto dist = tg::distance(
                tg::pos3(
                    CGAL::to_double(p1.x()), 
                    CGAL::to_double(p1.y()), 
                    CGAL::to_double(p1.z())),
                tg::pos3(
                    CGAL::to_double(p2.x()), 
                    CGAL::to_double(p2.y()), 
                    CGAL::to_double(p2.z())));
            trim.Domain = Interval(0, dist);

            trims[h.idx()] = trim;
        }   

        return trims;
    }

    void Brep::display(std::string name, bool show ) const{
        polyscope::myinit();
        polyscope::display<linkml::Surface_mesh const&>(this->get_Mesh(), name);
        if (show) polyscope::show();
    }

} // namespace linkml