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
#include <polymesh/attributes/partitioning.hh>
#include <polymesh/ranges.hh>


#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"


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
    cw.facets =  pm::face_attribute<std::vector<int>>(cw, default_id);
    linkml::color_facets(cw, results);
    int i = 0;
    auto cell_color_look_up = cw.facets.to_map([&](std::vector<int>){auto c = get_color_forom_angle(sample_circle(i)); i++; return c;});
    for (auto face : cw.faces()){
        cw.facet_colors[face] = cell_color_look_up[cw.facets[face]];
    }


    // Decimated
    linkml::decimate_cell_complex(cw);


    // Compute Coverage

    // Make a set vector of all facet ids.
    auto set = cw.faces().to_set([&](pm::face_handle h){return cw.facets[h]; });
    std::vector<std::vector<int>> ids(set.begin(), set.end());
#pragma omp parallel
#pragma omp for


    for (int i = 0; i < ids.size(); i++){

        auto facet = cw.faces().where([&](pm::face_handle h){return  cw.facets[h] == ids[i];});
        auto facet_vec = facet.to_vector();
        auto plane = cw.supporting_plans[facet.first()];

        
        // Find point inside facet.
        int idx = 0;

        std::vector<tg::pos3> point_in_facet;
        auto point_in_facet_index = std::vector<int>();

        std::copy_if(cloud.pts.begin(), cloud.pts.end(), std::back_inserter(point_in_facet), [&](tg::pos3 p){
            idx++;

            // Find point on plane.
            if (tg::distance(p, plane) > 0.2) return false; // TODO: Make value configurable

            for (auto & vh : facet_vec){
                auto vrts = vh.vertices().to_vector();

                auto trig = tg::triangle3(cw.pos[vrts[0]], cw.pos[vrts[1]], cw.pos[vrts[2]]);
                p = tg::project(p, plane);

                if (tg::contains(trig, p)) {
                    point_in_facet_index.push_back(idx-1); // -1 since we incremented first as the final path ins not clear.
                    return true;
                } // TODO: Make value configurable


            }
            return false;
        });

        // Check if there are any point in the facet
        if (point_in_facet.size() == 0) {
            for (auto & h : facet_vec) cw.coverage[h] = 0;
            continue;
        }


        float area_points = 0;
        auto trigs = alpha_shape(project_2d(point_in_facet, plane));
        for (auto t : trigs) area_points += tg::area(t);
            

        float area_facet = facet.sum([&](auto f) { return face_area(f, cw.pos); });
        for (auto & h : facet_vec) cw.coverage[h] = area_points / area_facet;

    }


    cw.compactify();
    // pm::deduplicate(cw, cw.pos);

    polyscope::init();

    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);

    auto pcd = polyscope::registerPointCloud("Cloud", cloud.pts);
    pcd->setPointRadius(0.001);
    // auto pcd_color =  pcd->addColorQuantity("RGB", cloud.colors);
    // pcd_color->setEnabled(true)
    
    auto ps_mesh = polyscope::registerSurfaceMesh("Mesh", ps_helpers::vertecies(cw, cw.pos), ps_helpers::faces(cw));
    auto face_color = ps_mesh->addFaceColorQuantity("Face Color", cw.faces().map([&](pm::face_handle h){ 
        auto c = cw.plane_colors[h]; 
        return cc::array<float,3>{c.r,c.g,c.b};
    }).to_vector());
    auto facet_color = ps_mesh->addFaceColorQuantity("Facets Color", cw.faces().map([&](pm::face_handle h){ 
        auto c = cw.facet_colors[h]; 
        return cc::array<float,3>{c.r,c.g,c.b};
    }).to_vector());
    facet_color->setEnabled(true);
    ps_mesh->addFaceScalarQuantity("Coverage", cw.faces().map([&](pm::face_handle h){
        return cw.coverage[h];
    }).to_vector());

    
    polyscope::show();



}
