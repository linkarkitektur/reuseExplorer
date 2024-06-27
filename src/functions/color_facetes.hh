#pragma once
#include <omp.h>

#include <types/PointCloud.hh>
#include <types/CellComplex.hh>
#include <types/PlanarPointSet.hh>
#include <functions/progress_bar.hh>

#include <typed-geometry/tg.hh>
#include <clean-core/optional.hh>

#include <polymesh/pm.hh>
#include <polymesh/algorithms/edge_split.hh>

template <typename T>
static std::size_t hashVector(const std::vector<T>& vec) {
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

    // static void color_facets(linkml::CellComplex & cw, std::vector<PlanarPointSet> const & clusters){

    //     const std::vector<int> default_id(clusters.size()+1, 0);
    //     auto facets_vec =  pm::face_attribute<std::vector<int>>(cw, default_id);

    //     //Color Facetes
    //     auto bar_color_facets = util::progress_bar(clusters.size(), "Color Facets");
    //     for (int i = 0; i < (int)clusters.size(); i++){

    //         for (auto face : cw.faces()){

    //             if (cw.supporting_plans[face] == clusters[i].get_Plane() ){
    //                 facets_vec[face][clusters.size()] = i;
    //                 continue;
    //             }

    //             auto center = face.vertices().avg(cw.pos);
    //             auto distance = tg::signed_distance(center, clusters[i].get_Plane());
                
    //             // if (tg::abs(distance)< EPSILON) continue;
    //             facets_vec[face][i] = (distance > 0)? 1 :0;
    //         }
    //         bar_color_facets.update(i);
    //     }


    //     for (auto h : cw.faces()){
    //         cw.facets[h] = hashVector(facets_vec[h]); 
    //     }
    // }

    // /**
    //  * @brief A map that associates a size_t key with a vector of integers.
    //  * 
    //  * This map is used to store cell complex information, where each key represents a cell ID
    //  * and the associated vector contains the indices of points belonging to that cell.
    //  */
    // static std::map<size_t, std::vector<int>> make_cw(PointCloud::Cloud::ConstPtr cloud,  std::vector<PlanarPointSet> const & clusters ) {

    //     auto cell_map = std::map<size_t, std::vector<int>>();
    //     const std::vector<int> default_id(clusters.size()+1, 0);
    //     auto bar_create_cw = util::progress_bar(cloud->size(), "Create Cell Complex");

    //     #pragma omp parallel for shared(cell_map)
    //     for (size_t i = 0; i < cloud->size(); i++){
    //         auto point = cloud->points[i].getPos();

    //         auto point_location_map = std::vector<int>(default_id);


    //         for (int j = 0; j < clusters.size(); j++){

    //             auto point_indecies = clusters[j].indices;
    //             if (std::find(point_indecies.begin(), point_indecies.end(), i) != point_indecies.end()) {
    //                 point_location_map[clusters.size()] = j;
    //                 continue;
    //             }

    //             auto plane = clusters[j].get_Plane();
    //             auto distance = tg::signed_distance(point, plane);

    //             point_location_map[j] = (distance > 0)? 1 :0;

    //         }

    //         auto id = hashVector(point_location_map);

    //         if (cell_map.find(id) == cell_map.end()){
    //             #pragma omp critical
    //             cell_map[id] = std::vector<int>();
    //         }

    //         #pragma omp critical
    //         cell_map[id].push_back(i);
    //         bar_create_cw.update();
    //     }

    //     bar_create_cw.stop();


    //     return cell_map;

    // }

}