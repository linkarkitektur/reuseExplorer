#include <algorithms/create_cell_complex.h>

#include <vector>
#include <functions/progress_bar.h>

#include <typed-geometry/tg.hh>
#include <typed-geometry/feature/std-interop.hh>
#include <typed-geometry/detail/optional.hh>

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

float THREASHHOLD = 0.001;

typedef std::vector<polymesh::face_handle> FacHs;


static pm::vertex_attribute<tg::quadric3> compute_quadrics(pm::Mesh & m, PosH  &pos){
    pm::vertex_attribute<tg::quadric3> qs = pm::vertex_attribute<tg::quadric3>(m);

    for (auto v: m.vertices()){

        auto q = tg::quadric3();
        for (auto f : v.faces()){

            auto p1 = pos[f.halfedges().first().vertex_from()];
            auto p2 = pos[f.halfedges().first().next().vertex_from()];
            auto p3 = pos[f.halfedges().last().vertex_from()];

            auto dir = tg::cross((p2 -p1), (p3-p1));

            q.add_plane(p1, dir, 0);
        }
        qs[v] = q;
    }

    return qs;
}
static void add_cube_triangualted(linkml::Cell * cell, tg::aabb3 box){


    pm::objects::add_cube(cell->m, [&](pm::vertex_handle v, float x, float y, float z) {

        x = (x < 0.5) ? box.min.x :box.max.x;
        y = (y < 0.5) ? box.min.y :box.max.y;
        z = (z < 0.5) ? box.min.z :box.max.z;

        cell->pos[v] = tg::pos3(x, y, z);
    });
    pm::triangulate_naive(cell->m);
    cell->m.compactify();
    };
static bool intersects_by_face(linkml::Cell * cell, tg::plane3 const & plane){

    bool intersects = false;

    // Loop over all faces
    auto faces = cell->m.faces();
    for (int i = 0; i < faces.size(); ++i){
        auto vertecies = std::vector<Pos>();

        for (auto vh: faces[i].vertices())
            if (faces[i].is_valid())
                vertecies.push_back(cell->pos[vh]);

        auto trig = tg::triangle3(vertecies);

        auto dpos0 = tg::distance(trig.pos0, plane) < THREASHHOLD;
        auto dpos1 = tg::distance(trig.pos1, plane) < THREASHHOLD;
        auto dpos2 = tg::distance(trig.pos2, plane) < THREASHHOLD;

        // Contine of and point is just touching the plane
        bool too_close = dpos0 | dpos1 | dpos2;

        if (too_close) continue;
        if (tg::intersects(trig, plane)) intersects = true;
    }

    return intersects;

}


template <class T, class F>
decltype(auto) map(const std::vector<T> a, const F fn) {
    std::vector<decltype( fn(a[0]) )> result = {};
    std::transform(a.cbegin(), a.cend(), std::back_inserter(result), fn);
    return result;
}


std::vector<linkml::Cell *> linkml::create_cell_complex(linkml::point_cloud const &cloud, std::vector<linkml::Plane> const &planes){

    polyscope::init();
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;

    polyscope::PointCloud* psCloud = polyscope::registerPointCloud("Cloud", cloud.pts);
    psCloud->setPointRadius(0.001);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Sphere);
    psCloud->setEnabled(false);
    

    auto cells = std::vector<Cell *>();
    auto cells_new = std::vector<Cell *>();

    cells.push_back(std::make_unique<Cell>().release());
    add_cube_triangualted(cells[0], tg::aabb_of(cloud.pts));

    auto bar = util::progress_bar(planes.size(), "Number of planes");
    
    // Loop over all planes
    for (int i = 0; i < (int)planes.size(); i++){

        bar.update();

        // add_plane_to_polyscope(plane, i);

        // Loop over all cells
        for (int j = 0; j < (int)cells.size(); j++){


            // Check if all indecies are valid
            bool stop = false;
            for (auto v :cells[j]->m.vertices()){
                auto p = cells[j]->pos[v];
                if (tg::any(tg::is_inf(p)) or tg::any(tg::is_nan(p))) stop = true;
            }
            if (stop) continue;


            
            // Check if the whole volume is being intersected.
            // Otherwise there is no need to check each face.
            if (!tg::intersects(cells[j]->box(), (tg::plane3)planes[i])) continue;

            // For all faces check if they have intersection
            if (!intersects_by_face(cells[j], planes[i])) continue;

            pm::split_edges_trimesh(cells[j]->m,
                // Check if an edge intersects the plane
                [&](pm::edge_handle e) -> tg::optional<float> {
                    auto pos = cells[j]->pos;
                    auto seg = tg::segment3(pos[e.vertexA()], pos[e.vertexB()]);
                    if (!tg::intersects(seg, planes[i])) {
                        return {};
                    }
                    if (tg::distance(seg.pos0, planes[i]) < THREASHHOLD )
                        return {};
                    if (tg::distance(seg.pos1, planes[i]) < THREASHHOLD )
                        return {};
                    return 1;
                },
                // Split edge along plane
                [&](pm::vertex_handle v, pm::halfedge_handle, pm::vertex_handle v_from, pm::vertex_handle v_to) {
                    auto pos = cells[j]->pos;
                    auto seg = tg::segment3(pos[v_to], pos[v_from]);
                    cells[j]->pos[v] = tg::intersection(seg, planes[i]).value();
                });

            // Create a copy of the mesh
            auto cell2 =  new Cell();
            cell2->m.copy_from(cells[j]->m);
            cell2->pos.copy_from(cells[j]->pos);

            // Delete vertecies infront of plane
            cells[j]->m.vertices().map([&](polymesh::vertex_handle h){
                auto pt = cells[j]->pos[h];
                if (tg::signed_distance(pt, planes[i]) > THREASHHOLD) cells[j]->m.vertices().remove(h);
                return true;
            });

            // Delete vertecies behind the plane
            cell2->m.vertices().map([&](pm::vertex_handle h){
                auto pt = cell2->pos[h];
                if (tg::signed_distance(pt, planes[i]) < THREASHHOLD*-1) cell2->m.vertices().remove(h);
                return true;
            });

            

            // Fill hole
            auto bs1 = cells[j]->m.halfedges().where([](pm::halfedge_handle h){return h.is_boundary(); } ).first();
            pm::fill_hole(cells[j]->m, cells[j]->pos, bs1);
            auto bs2 = cell2->m.halfedges().where([](pm::halfedge_handle h){return h.is_boundary(); } ).first();
            pm::fill_hole(cell2->m, cell2->pos, bs2);



            // Display Geometry
            // polyscope::registerSurfaceMesh("Cell j filled", cells[j]->vertecies(), cells[j]->faces());
            // polyscope::registerSurfaceMesh("Cell 2 filled", cell2->vertecies(), cell2->faces());
    

            cells[j]->m.compactify();
            cell2->m.compactify();

            
            // Decimated
            auto cfg1 = pm::decimate_config<Pos, tg::quadric3>::up_to_error(0);
            cfg1.max_normal_dev = 0;

            auto cfg2 = pm::decimate_config<Pos, tg::quadric3>::up_to_error(0);
            cfg2.max_normal_dev = 0;

            auto errors1 = compute_quadrics(cells[j]->m,cells[j]->pos);
            auto errors2 = compute_quadrics(cell2->m,cell2->pos);

            pm::decimate(cells[j]->m, cells[j]->pos, errors1, cfg1);
            pm::decimate(cell2->m, cell2->pos, errors2, cfg2);

            cells[j]->m.compactify();
            cell2->m.compactify();



            // check if the mesh is good
            // auto cj_non_mainfold = cells[j]->m.faces().where(manifold_check).count() > 1;
            // auto c2_non_mainfold =    cell2->m.faces().where(manifold_check).count() > 1;

            // std::printf("CJ %d\n", cj_non_mainfold);
            // std::printf("C2 %d\n", c2_non_mainfold);


            cells_new.push_back(cell2);


        }

        for (auto & c : cells_new)
            cells.push_back(c);

        cells_new.clear();

        
        // // Show intermediate Steps
        // auto names = std::vector<std::string>();
        // for (int j = 0; j < cells.size(); j++){
        //     std::stringstream name;
        //     name <<"Plane : " << i << "Cell : " << j;

        //     names.push_back(name.str());
        //     polyscope::SurfaceMesh * psPMesh = polyscope::registerSurfaceMesh(name.str(), cells[j]->vertecies(), cells[j]->faces());
        //     psPMesh->setTransparency(0.5);
        // }

        // polyscope::show();
        // for (auto & n : names)
        //     polyscope::removeSurfaceMesh(n);

    };




    for (auto& cell : cells){
        pm::deduplicate(cell->m, cell->pos);
        cell->m.compactify();
    }



    // Visualise all cells
    for (int i = 0; i < (int)cells.size(); i++){
        std::stringstream name;
        name << "Cell : " << i;
        polyscope::SurfaceMesh * psPMesh = polyscope::registerSurfaceMesh(name.str(), cells[i]->vertecies(), cells[i]->faces());
        psPMesh->setTransparency(1);
    }



    polyscope::show();

    return cells;

};
