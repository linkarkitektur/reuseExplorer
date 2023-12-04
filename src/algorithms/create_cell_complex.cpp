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
#include <functions/split_cellcomplex_with_planes.h>
#include <functions/color_facetes.h>
#include <functions/decimate_cell_complex.h>

#include <typed-geometry/tg.hh>
#include <typed-geometry/feature/std-interop.hh>


#include <clean-core/map.hh>

#include <polymesh/pm.hh>
#include <polymesh/copy.hh>
#include <polymesh/Mesh.hh>
#include <polymesh/algorithms.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <polymesh/algorithms/fill_hole.hh>


#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

#define  EPSILON 0.001





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

    // Intersect CW with planes
    linkml::split_cellcomplex_with_planes(cw, results);


    // Group facets
    const std::vector<int> default_id(results.planes.size()+1, 0);
    auto cell_id = pm::face_attribute<std::vector<int>>(cw.m, default_id);
    linkml::color_facets(cw, results, cell_id);


    // Decimated
    auto cw2 = CellComplex();
    int i = 0;
    auto cell_color_look_up = cell_id.to_map([&](std::vector<int>){auto c = get_color_forom_angle(sample_circle(i)); i++; return c;});
    for (auto face : cw.m.faces()){
        cw.facets_colors[face] = cell_color_look_up[cell_id[face]];
    }
    linkml::decimate_cell_complex(cw, cw2, cell_id, cell_color_look_up);




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
