#pragma once

#include <types/CellComplex.h>
#include <types/result_fit_planes.h>
#include <functions/progress_bar.h>

#include <typed-geometry/tg.hh>
#include <typed-geometry/detail/optional.hh>

#include <polymesh/pm.hh>
#include <polymesh/algorithms/edge_split.hh>

#define  EPSILON 0.001
typedef std::vector<polymesh::face_handle> FacHs;
typedef polymesh::face_handle FacH;

namespace linkml {


    static void split_cellcomplex_with_planes(linkml::CellComplex & cw, linkml::result_fit_planes const & results){

        // Intersect all planes in CW with each other
        auto bar = util::progress_bar(results.planes.size(), "Split planes");
        for (auto& plane : results.planes){
            pm::split_edges_trimesh(cw,
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

                        cw.copy_face_attributes(B, cw, A);

                    }
                    else if (n == 4) {
                        A = he.face();
                        B = he.next().opposite().face();
                        C = he.next().opposite().next().opposite().face();
                        D = he.opposite().face();

                        cw.copy_face_attributes(B, cw, A);
                        cw.copy_face_attributes(C, cw, D);

                    }
                    else {
                        std::printf("Unresolved case with %d faces", n);
                        for (auto & face :faces){
                            std::printf("Face IDX: %d  ", int(face) );
                            auto color = cw.plane_colors[face];
                            std::printf("Color  R=%.2f, G=%.2f, B=%.2f\n", color.r, color.g, color.b );
                        }
                    }

                });
            bar.update();
        }
    }


}