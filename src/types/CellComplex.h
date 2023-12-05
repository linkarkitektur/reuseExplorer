#pragma once
#include <vector>
#include <array>

#include <clean-core/vector.hh>

#include <types/plane.h>

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/aabb.hh>

#include <polymesh/Mesh.hh>
#include <polymesh/attributes.hh>

typedef pm::vertex_attribute<tg::pos3>              Pos;

typedef pm::face_attribute<linkml::Plane>           Planes;
typedef pm::face_attribute<tg::color3>              Colors;
typedef pm::face_attribute<float>                   Scalars;
typedef pm::face_attribute<std::vector<int>>        Facets;

typedef std::vector<std::array<int, 3>>             Faces;
typedef std::vector<std::array<float, 3>>           Verts;

namespace linkml{

    // struct Facet {
    //     // List of face handles that make up the facet.
    //     FaceHs faces = std::vector<pm::face_handle>();

    //     // Vertecies that make up the convec hull of the facet.
    //     std::vector<tg::pos3> vertecies = std::vector<tg::pos3>();

    //     // Indecies of point on facet.
    //     cc::vector<int> points = cc::vector<int>();
    // };

    
    struct CellComplex : public pm::Mesh
    {

        // ID idetifying each facet
        Facets facets{*this};// = Facets(m);

        // pos3 that make up the mesh
        Pos pos{*this};// = Pos(m);

        // The supporting plane
        Planes supporting_plans{*this};// = Planes(m);

        // Colors
        Colors plane_colors{*this};// = Colors(m);
        Colors facet_colors{*this};// = Colors(m);

        // The point converage in % of the facet, values 0-1
        Scalars coverage{*this};// = Scalars(m);


        Faces ps_faces();
        Verts ps_vertecies();
        tg::aabb3 box();

        void copy_vertex_attributes(pm::vertex_handle const h1, CellComplex const & cw, pm::vertex_handle const h2){
            pos[h1] = cw.pos[h2];
        }

        void copy_face_attributes(pm::face_handle const h1, CellComplex const & cw, pm::face_handle const h2){
            supporting_plans[h1] = cw.supporting_plans[h2];
            plane_colors[h1] = cw.plane_colors[h2];
            facet_colors[h1] = cw.facet_colors[h2];
            coverage[h1] = cw.coverage[h2];
            facets[h1] = cw.facets[h2];
        }
    };
}
