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


static pm::vertex_attribute<tg::quadric3> compute_quadrics(linkml::CellComplex const& cw, pm::face_attribute<std::vector<int>> const& cell_id){

    pm::vertex_attribute<tg::quadric3> qs = pm::vertex_attribute<tg::quadric3>(cw.m);

    for (auto v: cw.m.vertices()){

        auto q = tg::quadric3();

        auto id = cell_id[v.faces().first()];

        bool all = true;

        for (auto f : v.faces()){

            auto id_ = cell_id[f];

            if (id != cell_id[f]) { all= false; break;}
        }

        if (all) {
            auto pt = cw.pos[v];
            auto pl = cw.supporting_plans[v.faces().first()];
            q.add_plane(pt, pl.normal, 0);
        }

        
        qs[v] = q;
    }

    return qs;
}
// static void add_cube_triangualted(linkml::Mesh * cell, tg::aabb3 box){


//     pm::objects::add_cube(cell->m, [&](pm::vertex_handle v, float x, float y, float z) {

//         x = (x < 0.5) ? box.min.x :box.max.x;
//         y = (y < 0.5) ? box.min.y :box.max.y;
//         z = (z < 0.5) ? box.min.z :box.max.z;

//         cell->pos[v] = tg::pos3(x, y, z);
//     });
//     pm::triangulate_naive(cell->m);
//     cell->m.compactify();
//     };
// static bool intersects_by_face(linkml::Cell * cell, tg::plane3 const & plane){

//     bool intersects = false;

//     // Loop over all faces
//     auto faces = cell->m.faces();
//     for (int i = 0; i < faces.size(); ++i){
//         auto vertecies = std::vector<Pos>();

//         for (auto vh: faces[i].vertices())
//             if (faces[i].is_valid())
//                 vertecies.push_back(cell->pos[vh]);

//         auto trig = tg::triangle3(vertecies);

//         auto dpos0 = tg::distance(trig.pos0, plane) < EPSILON;
//         auto dpos1 = tg::distance(trig.pos1, plane) < EPSILON;
//         auto dpos2 = tg::distance(trig.pos2, plane) < EPSILON;

//         // Contine of and point is just touching the plane
//         bool too_close = dpos0 | dpos1 | dpos2;

//         if (too_close) continue;
//         if (tg::intersects(trig, plane)) intersects = true;
//     }

//     return intersects;

// }

// static auto vertex_filter = [](pm::vertex_handle h){ return h.is_valid();};
// static auto face_filter = [](pm::face_handle h){ return h.is_valid();};
// static auto edges_filter = [](pm::edge_handle h){ return h.is_valid();};

// static bool is_valid(linkml::Cell *cell){
//     int n_faces = cell->m.faces().where(face_filter).count();
//     int n_verts = cell->m.vertices().where(vertex_filter).count();
//     int n_endges = cell->m.edges().where(edges_filter).count();



//     if (n_verts - n_endges + n_faces != 2) {
//         std::printf("V %d, E %d F %d\n", n_verts, n_endges, n_faces);
//         return false;
//     };

//     if (n_verts == 3) return false;

//     for (auto h : cell->m.vertices()){
//         auto pt = cell->pos[h];
//         if (tg::is_inf(pt.x)) return false;
//         if (tg::is_inf(pt.y)) return false;
//         if (tg::is_inf(pt.z)) return false;

//         if (tg::is_nan(pt.x)) return false;
//         if (tg::is_nan(pt.y)) return false;
//         if (tg::is_nan(pt.z)) return false;
//     }


//     return true;


// }

// template <class T, class F>
// decltype(auto) map(const std::vector<T> a, const F fn) {
//     std::vector<decltype( fn(a[0]) )> result = {};
//     std::transform(a.cbegin(), a.cend(), std::back_inserter(result), fn);
//     return result;
// }



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
        // auto plane = results.planes[i];
        // auto faces = cw.m.faces().where(not_inplane);

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
    // auto cfg = pm::decimate_config<Pos, tg::quadric3>::up_to_error(0.1);
    // cfg.max_normal_dev = 0.1;
    // auto errors = compute_quadrics(cw, cell_id);
    // pm::decimate(cw.m,cw.pos, errors, cfg);


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

        auto vert_hs = cc::vector<pm::vertex_handle>();
        auto verts = cc::vector<tg::pos3>();
        for (auto f : facet)
            for ( auto v: f.vertices())
                verts.push_back(cw.pos[v]);

        face_indecies[i] = convex_hull(project_2d(verts, plane));
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
            
            auto vh1 = cw2.m.vertices().add();
            auto vh2 = cw2.m.vertices().add();

            cw2.pos[vh1] = verts[indecies[j]];
            cw2.pos[vh2] = verts[indecies[j+1]];

            auto fh = cw2.m.faces().add(vh0,vh1, vh2);


            cw2.colors[fh] = cw.colors[facet_h];
            cw2.facets_colors[fh] = cw.facets_colors[facet_h];
        }
    }


    cw.m.clear();
    cw.pos.clear();
    cw.supporting_plans.clear();
    cw.colors.clear();

    cw2.m.compactify();
    pm::deduplicate(cw2.m, cw2.pos);

    polyscope::init();
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



    // polyscope::init();
    // polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;

    // polyscope::PointCloud* psCloud = polyscope::registerPointCloud("Cloud", cloud.pts);
    // psCloud->setPointRadius(0.001);
    // psCloud->setPointRenderMode(polyscope::PointRenderMode::Sphere);
    // psCloud->setEnabled(false);
    

    // auto cells = std::vector<Cell *>();
    // auto cells_new = std::vector<Cell *>();

    // cells.push_back(std::make_unique<Cell>().release());
    // add_cube_triangualted(cells[0], tg::aabb_of(cloud.pts));

    // auto bar = util::progress_bar(planes.size(), "Number of planes");
    
    // // Loop over all planes
    // for (int i = 0; i < (int)planes.size(); i++){

    //     bar.update();

    //     // add_plane_to_polyscope(plane, i);

    //     // Loop over all cells
    //     for (int j = 0; j < (int)cells.size(); j++){

    //         std::printf("Plane %d, Cell %d \n", i, j);


    //         // // Check if all indecies are valid
    //         // bool stop = false;
    //         // for (auto v :cells[j]->m.vertices()){
    //         //     auto p = cells[j]->pos[v];
    //         //     if (tg::any(tg::is_inf(p)) or tg::any(tg::is_nan(p))) stop = true;
    //         // }
    //         // if (stop) continue;


    //         int SEL_PLNAE = 185;
    //         int SEL_CELL = 6580;



    //         if (i == SEL_PLNAE and j== SEL_CELL){
    //             for (auto h : cells[j]->m.vertices()){
    //                 auto pt = cells[j]->pos[h];
    //                 std::printf("X=%d Y=%d Z=%d\n", pt.x, pt.y, pt.z);
    //             }
    //             polyscope::registerSurfaceMesh("Input", cells[j]->vertecies(), cells[j]->faces());
    //             polyscope::show();
    //         }

            
    //         // Check if the whole volume is being intersected.
    //         // Otherwise there is no need to check each face.
    //         if (!tg::intersects(cells[j]->box(), (tg::plane3)planes[i])) continue;

    //         // For all faces check if they have intersection
    //         if (!intersects_by_face(cells[j], planes[i])) continue;

    //         pm::split_edges_trimesh(cells[j]->m,
    //             // Check if an edge intersects the plane
    //             [&](pm::edge_handle e) -> tg::optional<float> {
    //                 auto pos = cells[j]->pos;
    //                 auto seg = tg::segment3(pos[e.vertexA()], pos[e.vertexB()]);
    //                 if (!tg::intersects(seg, planes[i])) {
    //                     return {};
    //                 }
    //                 if (tg::distance(seg.pos0, planes[i]) < EPSILON )
    //                     return {};
    //                 if (tg::distance(seg.pos1, planes[i]) < EPSILON )
    //                     return {};
    //                 return 1;
    //             },
    //             // Split edge along plane
    //             [&](pm::vertex_handle v, pm::halfedge_handle, pm::vertex_handle v_from, pm::vertex_handle v_to) {
    //                 auto pos = cells[j]->pos;
    //                 auto seg = tg::segment3(pos[v_to], pos[v_from]);
    //                 cells[j]->pos[v] = tg::intersection(seg, planes[i]).value();
    //             });

    //         // Create a copy of the mesh
    //         auto cell2 =  new Cell();
    //         cell2->m.copy_from(cells[j]->m);
    //         cell2->pos.copy_from(cells[j]->pos);

    //         // Delete vertecies infront of plane
    //         cells[j]->m.vertices().map([&](polymesh::vertex_handle h){
    //             auto pt = cells[j]->pos[h];
    //             if (tg::signed_distance(pt, planes[i]) > EPSILON) cells[j]->m.vertices().remove(h);
    //             return true;
    //         });

    //         // Delete vertecies behind the plane
    //         cell2->m.vertices().map([&](pm::vertex_handle h){
    //             auto pt = cell2->pos[h];
    //             if (tg::signed_distance(pt, planes[i]) < EPSILON*-1) cell2->m.vertices().remove(h);
    //             return true;
    //         });

            

    //         if (i == SEL_PLNAE and j== SEL_CELL){
    //             polyscope::registerSurfaceMesh("Split A", cells[j]->vertecies(), cells[j]->faces());
    //             polyscope::registerSurfaceMesh("Split B", cell2->vertecies(), cells[j]->faces());
    //             polyscope::show();
    //         }
            

    //         // Fill hole
    //         auto bs1 = cells[j]->m.halfedges().where([](pm::halfedge_handle h){return h.is_boundary(); } ).first();
    //         pm::fill_hole(cells[j]->m, cells[j]->pos, bs1);
    //         auto bs2 = cell2->m.halfedges().where([](pm::halfedge_handle h){return h.is_boundary(); } ).first();
    //         pm::fill_hole(cell2->m, cell2->pos, bs2);


    //         if (i == SEL_PLNAE and j== SEL_CELL){
    //             polyscope::registerSurfaceMesh("Patched A", cells[j]->vertecies(), cells[j]->faces());
    //             polyscope::registerSurfaceMesh("Patched B", cell2->vertecies(), cells[j]->faces());
    //             polyscope::show();
    //         }


    //         // Display Geometry
    //         // polyscope::registerSurfaceMesh("Cell j filled", cells[j]->vertecies(), cells[j]->faces());
    //         // polyscope::registerSurfaceMesh("Cell 2 filled", cell2->vertecies(), cell2->faces());
    

    //         cells[j]->m.compactify();
    //         cell2->m.compactify();

            
    //         // Decimated
    //         auto cfg1 = pm::decimate_config<Pos, tg::quadric3>::up_to_error(0.1);
    //         cfg1.max_normal_dev = 0.1;

    //         auto cfg2 = pm::decimate_config<Pos, tg::quadric3>::up_to_error(0.1);
    //         cfg2.max_normal_dev = 0.1;

    //         auto errors1 = compute_quadrics(cells[j]->m,cells[j]->pos);
    //         auto errors2 = compute_quadrics(cell2->m,cell2->pos);

    //         pm::decimate(cells[j]->m, cells[j]->pos, errors1, cfg1);
    //         pm::decimate(cell2->m, cell2->pos, errors2, cfg2);


    //         if (i == SEL_PLNAE and j== SEL_CELL){
    //             polyscope::registerSurfaceMesh("Deceminated A", cells[j]->vertecies(), cells[j]->faces());
    //             polyscope::registerSurfaceMesh("Deceminated B", cell2->vertecies(), cells[j]->faces());
    //             polyscope::show();
    //         }

    //         cells[j]->m.compactify();
    //         cell2->m.compactify();



    //         if (!is_valid(cells[j])) cells.erase(cells.begin()+j);

    //         // check if the mesh is good
    //         // auto cj_non_mainfold = cells[j]->m.faces().where(manifold_check).count() > 1;
    //         // auto c2_non_mainfold =    cell2->m.faces().where(manifold_check).count() > 1;

    //         // std::printf("CJ %d\n", cj_non_mainfold);
    //         // std::printf("C2 %d\n", c2_non_mainfold);

    //         if (is_valid(cell2))
    //             cells_new.push_back(cell2);


    //     }

    //     for (auto & c : cells_new)
    //         cells.push_back(c);

    //     cells_new.clear();

        
    //     // // Show intermediate Steps
    //     // auto names = std::vector<std::string>();
    //     // for (int j = 0; j < cells.size(); j++){
    //     //     std::stringstream name;
    //     //     name <<"Plane : " << i << "Cell : " << j;

    //     //     names.push_back(name.str());
    //     //     polyscope::SurfaceMesh * psPMesh = polyscope::registerSurfaceMesh(name.str(), cells[j]->vertecies(), cells[j]->faces());
    //     //     psPMesh->setTransparency(0.5);
    //     // }

    //     // polyscope::show();
    //     // for (auto & n : names)
    //     //     polyscope::removeSurfaceMesh(n);

    // };




    // for (auto& cell : cells){
    //     pm::deduplicate(cell->m, cell->pos);
    //     cell->m.compactify();
    // }



    // // Visualise all cells
    // for (int i = 0; i < (int)cells.size(); i++){
    //     std::stringstream name;
    //     name << "Cell : " << i;
    //     polyscope::SurfaceMesh * psPMesh = polyscope::registerSurfaceMesh(name.str(), cells[i]->vertecies(), cells[i]->faces());
    //     psPMesh->setTransparency(1);
    // }



    // polyscope::show();

    // return cells;

}
