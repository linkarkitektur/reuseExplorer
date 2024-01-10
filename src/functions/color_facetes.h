#pragma once
#include <omp.h>

#include <types/point_cloud.h>
#include <types/CellComplex.h>
#include <types/result_fit_planes.h>
#include <functions/progress_bar.h>

#include <typed-geometry/tg.hh>
#include <typed-geometry/detail/optional.hh>

#include <polymesh/pm.hh>
#include <polymesh/algorithms/edge_split.hh>

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

    /**
     * @brief A map that associates a size_t key with a vector of integers.
     * 
     * This map is used to store cell complex information, where each key represents a cell ID
     * and the associated vector contains the indices of points belonging to that cell.
     */
    std::map<size_t, std::vector<int>> make_cw(point_cloud const& cloud, result_fit_planes const& results ) {

        auto cell_map = std::map<size_t, std::vector<int>>();
        const std::vector<int> default_id(results.planes.size()+1, 0);
        auto bar_create_cw = util::progress_bar(cloud.pts.size(), "Create Cell Complex");

        #pragma omp parallel for
        for (int i = 0; i < cloud.pts.size(); i++){
            auto point = cloud.pts[i];

            auto point_location_map = std::vector<int>(default_id);


            for (int j = 0; j < results.planes.size(); j++){

                auto point_indecies = results.indecies[j];
                if (std::find(point_indecies.begin(), point_indecies.end(), i) != point_indecies.end()) {
                    point_location_map[results.planes.size()] = j;
                    continue;
                }

                auto plane = results.planes[j];
                auto distance = tg::signed_distance(point, plane);

                point_location_map[j] = (distance > 0)? 1 :0;

            }

            auto id = hashVector(point_location_map);
            cell_map[id].push_back(i);
            bar_create_cw.update();
        }


        return cell_map;

    }

}