#pragma once
#include <vector>
#include <array>

#include <clean-core/vector.hh>

#include <types/plane.hh>

#include <typed-geometry/types/pos.hh>
#include <typed-geometry/types/objects/aabb.hh>

#include <polymesh/Mesh.hh>
#include <polymesh/attributes.hh>

typedef pm::vertex_attribute<tg::pos3>              Pos;

typedef pm::face_attribute<linkml::Plane>           Planes;
typedef pm::face_attribute<tg::color3>              Colors;
typedef pm::face_attribute<int>                     Integer;
typedef pm::face_attribute<float>                   Scalars;
typedef pm::face_attribute<std::size_t>             Facets;

typedef std::vector<std::array<int, 3>>             Faces;
typedef std::vector<std::array<float, 3>>           Verts;


namespace linkml{


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
        Scalars facet_area{*this};// = Scalars(m);
        Scalars covered_area{*this};// = Scalars(m);
        Scalars coverage{*this};// = Scalars(m);
        Integer supporting_point_num{*this};



        Faces ps_faces();
        Verts ps_vertecies();

        void copy_vertex_attributes(pm::vertex_handle const h1, CellComplex const & cw, pm::vertex_handle const h2){
            pos[h1] = cw.pos[h2];
        }

        void copy_face_attributes(pm::face_handle const h1, CellComplex const & cw, pm::face_handle const h2){
            facets[h1] = cw.facets[h2];
            
            supporting_plans[h1] = cw.supporting_plans[h2];
            plane_colors[h1] = cw.plane_colors[h2];
            facet_colors[h1] = cw.facet_colors[h2];

            facet_area[h1] = cw.facet_area[h2];
            covered_area[h1] = cw.covered_area[h2];
            coverage[h1] = cw.coverage[h2];
            supporting_point_num[h1] = cw.supporting_point_num[h2];
        }
    };
}
