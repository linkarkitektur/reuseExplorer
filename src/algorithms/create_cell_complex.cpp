#include <algorithms/create_cell_complex.h>
#include <types/result_fit_planes.h>
#include <types/CellComplex.h>

#include <vector>
#include <execution>

#include <functions/progress_bar.h>
#include <functions/get_aabb.h>
#include <functions/color.h>
#include <functions/polyscope_helpers.h>
#include <functions/crop_plane_with_aabb.h>
#include <functions/convex_hull_2d.h>
#include <functions/project_2d.h>

#include <typed-geometry/tg.hh>
#include <typed-geometry/feature/std-interop.hh>
#include <typed-geometry/detail/optional.hh>

#include <clean-core/map.hh>

#include <polymesh/pm.hh>
#include <polymesh/copy.hh>
#include <polymesh/Mesh.hh>
#include <polymesh/algorithms.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <polymesh/algorithms/fill_hole.hh>
#include <polymesh/algorithms/edge_split.hh>

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

#define  EPSILON 0.001

typedef std::vector<polymesh::face_handle> FacHs;
typedef polymesh::face_handle FacH;



auto do_intersect = [](pm::face_handle h, linkml::CellComplex& cw, linkml::Plane plane){
    if (cw.supporting_plans[h] == plane) return false;

    // Construct trangle form face
    auto vh = h.vertices().to_vector();
    auto p0 = cw.pos[vh[0]];
    auto p1 = cw.pos[vh[1]];
    auto p2 = cw.pos[vh[2]];
    auto triangle = tg::triangle3(p0, p1, p2);

    // classify vertices
    auto sign_v1 = tg::signed_distance(triangle.pos0, plane) < 0 ? false : true;
    auto sign_v2 = tg::signed_distance(triangle.pos1, plane) < 0 ? false : true;
    auto sign_v3 = tg::signed_distance(triangle.pos2, plane) < 0 ? false : true;

    if (sign_v1 == sign_v2 && sign_v2 == sign_v3) // no intersection (early out)
        return false;

    bool iv = (sign_v1 == sign_v2) ? sign_v3 : (sign_v1 == sign_v3) ? sign_v2 : sign_v1;

    float d0, d1, d2;
    // intersection exists (exactly 2 vertices on one side of the plane and exactly 1 vertex on the other side)
    if (iv == sign_v1)
    {
        d0 = tg::distance(triangle.pos0, plane);
        d1 = tg::distance(triangle.pos1, plane);
        d2 = tg::distance(triangle.pos2, plane);
    }
    else if (iv == sign_v2)
    {
        d0 = tg::distance(triangle.pos1, plane);
        d1 = tg::distance(triangle.pos0, plane);
        d2 = tg::distance(triangle.pos2, plane);
    }
    else if (iv == sign_v3)
    {
        d0 = tg::distance(triangle.pos2, plane);
        d1 = tg::distance(triangle.pos0, plane);
        d2 = tg::distance(triangle.pos1, plane);
    }


    if (d0 < EPSILON) return false;
    if (d1 < EPSILON and d2 < EPSILON) return false;

    return true;
};

#define not_inplane  [&](pm::face_handle h){return cw.supporting_plans[h] != plane;}
#define do_intersect [&](pm::face_handle h){ return do_intersect(h, cw, plane);}

void linkml::create_cell_complex(linkml::point_cloud& cloud, linkml::result_fit_planes& results){


    linkml::CellComplex cw;

    auto box = get_aabb(cloud.pts);

    // Intersect CW with bounding box to generate bounded planes.
    linkml::crop_plane_with_aabb(cw, box, results);


    // Intersect all planes in CW with each other
    auto bar = util::progress_bar(results.planes.size(), "Split planes");
    for (auto& plane : results.planes){
        pm::split_edges_trimesh(cw.m,
            // Check if an edge intersects the plane
            [&](pm::edge_handle e) -> tg::optional<float> {
                auto seg = tg::segment3(cw.pos[e.vertexA()], cw.pos[e.vertexB()]);
                if (!tg::intersects(seg, plane)) {
                    return {};
                }
                if (tg::distance(seg.pos0, plane) < EPSILON )
                    return {};
                if (tg::distance(seg.pos1, plane) < EPSILON )
                    return {};
                return 1;
            },
            // Split edge along plane
            [&](pm::vertex_handle v, pm::halfedge_handle he, pm::vertex_handle v_from, pm::vertex_handle v_to) {
                auto seg = tg::segment3(cw.pos[v_to], cw.pos[v_from]);
                cw.pos[v] = tg::intersection(seg, plane).value();

                auto faces = v.faces().where([](pm::face_handle h){ return h.is_valid() and !h.is_removed(); }).to_vector();
                auto n = faces.size();

                FacH A, B, C, D;


                if (n == 2){
                    A = int(faces[0]) < int(faces[1]) ? faces[0] : faces[1];
                    B = int(faces[0]) < int(faces[1]) ? faces[1] : faces[0];
                    cw.colors[B] = cw.colors[A];
                    cw.supporting_plans[B] = cw.supporting_plans[A];
                }
                else if (n == 4) {
                    A = he.face();
                    B = he.next().opposite().face();
                    C = he.next().opposite().next().opposite().face();
                    D = he.opposite().face();

                    cw.colors[B] = cw.colors[A];
                    cw.colors[C] = cw.colors[D];
                    cw.supporting_plans[B] = cw.supporting_plans[A];
                    cw.supporting_plans[C] = cw.supporting_plans[D];
                }
                else {
                    std::printf("Unresolved case with %d faces", n);
                    for (auto & face :faces){
                        std::printf("Face IDX: %d  ", int(face) );
                        auto color = cw.colors[face];
                        std::printf("Color  R=%.2f, G=%.2f, B=%.2f\n", color.r, color.g, color.b );
                    }
                }

            });
        bar.update();
    }


    //Color Facetes
    const std::vector<int> default_id(results.planes.size()+1, 0);
    auto cell_id = pm::face_attribute<std::vector<int>>(cw.m, default_id);
    auto bar_color_facets = util::progress_bar(results.planes.size(), "Color Facets");
    for (int i = 0; i < (int)results.planes.size(); i++){

        for (auto face : cw.m.faces()){

            if (cw.supporting_plans[face] == results.planes[i] ){
                cell_id[face][results.planes.size()] = i;
                continue;
            }

            auto center = face.vertices().avg(cw.pos);
            auto distance = tg::signed_distance(center, results.planes[i]);
            // if (tg::abs(distance)< EPSILON) continue;
            cell_id[face][i] = (distance > 0)? 1 :0;
        }
        bar_color_facets.update();
    }

    int i = 0;
    auto cell_color_look_up = cell_id.to_map([&](std::vector<int>){auto c = get_color_forom_angle(sample_circle(i)); i++; return c;});

    for (auto face : cw.m.faces()){
        cw.facets_colors[face] = cell_color_look_up[cell_id[face]];
    }





    // Decimated
    auto cw2 = CellComplex();

    auto ids = std::vector<std::vector<int>>();
    for (auto& pair : cell_color_look_up)
        ids.push_back(pair.first );


    auto face_indecies = std::vector<std::vector<size_t>>(ids.size());
    auto face_vertecies = std::vector<cc::vector<tg::pos3>>(ids.size());
    auto refference_face_handle = std::vector<pm::face_handle>(ids.size());


#pragma omp parallel
#pragma omp for

    for (int i = 0; i < ids.size(); i++){

        auto id = ids[i];

        auto facet = cw.m.faces().where([&](pm::face_handle h){ return cell_id[h] == id;});
        auto plane = cw.supporting_plans[facet.first()];

        auto verts = cc::vector<tg::pos3>();
        for (auto f : facet)
            for ( auto v: f.vertices())
                verts.push_back(cw.pos[v]);

        auto indecies = convex_hull(project_2d(verts, plane));


        auto indecies_simplified = std::vector<size_t>();
        // Simplify convex hull by comparing entrance and exit vector if they are close to a streight line.
        if (indecies.size() > 3){
        
            auto v_in = tg::normalize_safe(verts[indecies[0]] - verts[indecies[indecies.size()-1]]);
            int i = 0;
            auto reduce = [&](int n, int m){
                auto p_o = verts[indecies[n]];
                auto p_n = verts[indecies[m]];

                auto v1_out = tg::normalize_safe(p_n-p_o);

                //TODO: Check value
                if (tg::dot(v_in,v1_out) < 0.97 ) {
                    indecies_simplified.push_back(indecies[i]);
                    v_in = v1_out;

                };

            };
            while (i < indecies.size() -1){
                reduce(i, i+1);
                i++;
            }

            reduce(indecies.size() -1, 0);

        }else{
            indecies_simplified = indecies;
        }


        face_indecies[i] = indecies_simplified;
        face_vertecies[i] = verts;
        refference_face_handle[i] = facet.first();
    }

    for (int i = 0; i < face_indecies.size(); i++){

        auto indecies = face_indecies[i];
        auto facet_h = refference_face_handle[i];
        auto verts = face_vertecies[i];

        auto vh0 = cw2.m.vertices().add();
        cw2.pos[vh0] = verts[indecies[0]];

        for (int j = 1; j < ((int)indecies.size()-1); j++){
            
            auto vhj = cw2.m.vertices().add();
            auto vhk = cw2.m.vertices().add();

            cw2.pos[vhj] = verts[indecies[j]];
            cw2.pos[vhk] = verts[indecies[j+1]];

            auto fh = cw2.m.faces().add(vh0,vhj, vhk);


            cw2.colors[fh] = cw.colors[facet_h];
            cw2.facets_colors[fh] = cw.facets_colors[facet_h];
        }
    }

    polyscope::init();

    auto ps_mesh_old = polyscope::registerSurfaceMesh("Mesh Old", ps_helpers::vertecies(cw.m, cw.pos), ps_helpers::faces(cw.m));
    auto face_color_old = ps_mesh_old->addFaceColorQuantity("Face Color", cw.m.faces().map([&](pm::face_handle h){ 
        auto c = cw.colors[h]; 
        return cc::array<float,3>{c.r,c.g,c.b};
    }).to_vector());

    auto facet_color_old = ps_mesh_old->addFaceColorQuantity("Facets Color", cw.m.faces().map([&](pm::face_handle h){ 
        auto c = cw.facets_colors[h]; 
        return cc::array<float,3>{c.r,c.g,c.b};
    }).to_vector());


    cw.m.clear();
    cw.pos.clear();
    cw.supporting_plans.clear();
    cw.colors.clear();

    cw2.m.compactify();
    // pm::deduplicate(cw2.m, cw2.pos);

    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);


    auto pcd = polyscope::registerPointCloud("Cloud", cloud.pts);
    pcd->setPointRadius(0.001);
    // auto pcd_color =  pcd->addColorQuantity("RGB", cloud.colors);
    // pcd_color->setEnabled(true)

    
    auto ps_mesh2 = polyscope::registerSurfaceMesh("Mesh", ps_helpers::vertecies(cw2.m, cw2.pos), ps_helpers::faces(cw2.m));
    auto face_color2 = ps_mesh2->addFaceColorQuantity("Face Color", cw2.m.faces().map([&](pm::face_handle h){ 
        auto c = cw2.colors[h]; 
        return cc::array<float,3>{c.r,c.g,c.b};
    }).to_vector());
    auto facet_color2 = ps_mesh2->addFaceColorQuantity("Facets Color", cw2.m.faces().map([&](pm::face_handle h){ 
        auto c = cw2.facets_colors[h]; 
        return cc::array<float,3>{c.r,c.g,c.b};
    }).to_vector());
    facet_color2->setEnabled(true);
    // ps_mesh->face("ID", cw.m.faces().map([&](pm::face_handle h){return cell_id[h];}), );

    
    polyscope::show();



}
