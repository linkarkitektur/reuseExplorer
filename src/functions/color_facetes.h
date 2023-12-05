#pragma once

// #include <types/CellComplex.h>
// #include <types/result_fit_planes.h>
// #include <functions/progress_bar.h>

// #include <typed-geometry/tg.hh>
// #include <typed-geometry/detail/optional.hh>

// #include <polymesh/pm.hh>
// #include <polymesh/algorithms/edge_split.hh>


namespace linkml{

    void color_facets(linkml::CellComplex & cw, linkml::result_fit_planes const & results ){

        //Color Facetes
        auto bar_color_facets = util::progress_bar(results.planes.size(), "Color Facets");
        for (int i = 0; i < (int)results.planes.size(); i++){

            for (auto face : cw.faces()){

                if (cw.supporting_plans[face] == results.planes[i] ){
                    cw.facets[face][results.planes.size()] = i;
                    continue;
                }

                auto center = face.vertices().avg(cw.pos);
                auto distance = tg::signed_distance(center, results.planes[i]);
                // if (tg::abs(distance)< EPSILON) continue;
                cw.facets[face][i] = (distance > 0)? 1 :0;
            }
            bar_color_facets.update();
        }

    }
}