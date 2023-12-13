#pragma once

// #include <types/CellComplex.h>
// #include <types/result_fit_planes.h>
// #include <functions/progress_bar.h>

// #include <typed-geometry/tg.hh>
// #include <typed-geometry/detail/optional.hh>

// #include <polymesh/pm.hh>
// #include <polymesh/algorithms/edge_split.hh>

template <typename T>
std::size_t hashVector(const std::vector<T>& vec) {
  std::size_t seed = vec.size();
  for(auto x : vec) {
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = (x >> 16) ^ x;
    seed ^= x + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;

}

namespace linkml{

    void color_facets(linkml::CellComplex & cw, linkml::result_fit_planes const & results ){

        const std::vector<int> default_id(results.planes.size()+1, 0);
        auto facets_vec =  pm::face_attribute<std::vector<int>>(cw, default_id);

        //Color Facetes
        auto bar_color_facets = util::progress_bar(results.planes.size(), "Color Facets");
        for (int i = 0; i < (int)results.planes.size(); i++){

            for (auto face : cw.faces()){

                if (cw.supporting_plans[face] == results.planes[i] ){
                    facets_vec[face][results.planes.size()] = i;
                    continue;
                }

                auto center = face.vertices().avg(cw.pos);
                auto distance = tg::signed_distance(center, results.planes[i]);
                // if (tg::abs(distance)< EPSILON) continue;
                facets_vec[face][i] = (distance > 0)? 1 :0;
            }
            bar_color_facets.update();
        }


        for (auto h : cw.faces()){
            cw.facets[h] = hashVector(facets_vec[h]); 
        }
    }
}