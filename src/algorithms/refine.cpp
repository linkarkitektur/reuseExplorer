#include <algorithms/refine.hh>
#include <typed-geometry/tg.hh>
#include <types/PointCloud.hh>
#include <clean-core/vector.hh>

#include <functions/fit_plane_thorugh_points.hh>
#include <functions/progress_bar.hh>

// #include <polyscope/polyscope.h>
// #include <polyscope/surface_mesh.h>
// #include <polyscope/point_cloud.h>

#include <polymesh/pm.hh>
#include <polymesh/algorithms/triangulate.hh>
#include <functions/crop_plane_with_aabb.hh>
#include <clean-core/format.hh>


// static void ShowBox(tg::aabb3 &box){
//         pm::Mesh bm;
//         auto bm_pos = pm::vertex_attribute<tg::pos3>(bm);
//         pm::objects::add_cube(bm, [&](pm::vertex_handle v, float x, float y, float z) {

//             x = (x < 0.5) ? box.min.x :box.max.x;
//             y = (y < 0.5) ? box.min.y :box.max.y;
//             z = (z < 0.5) ? box.min.z :box.max.z;

//             bm_pos[v] = tg::pos3(x, y, z);
//         });
//         pm::triangulate_naive(bm);

//         auto vertecies_filter_1 = [&](pm::vertex_handle h){return bm_pos[h];};
//         auto faces_filter = [&](pm::face_handle h){
//             auto vx = h.vertices().to_vector();
//             std::array<int, 3> indexis{int(vx[0]),int(vx[1]),int(vx[2])};
//             return indexis;
//             };

//         auto vertecies = bm.vertices().map(vertecies_filter_1).to_vector();
//         auto faces = bm.faces().map(faces_filter).to_vector();

//         auto ps_box = polyscope::registerSurfaceMesh("Box", vertecies, faces);
//         ps_box->setTransparency(0.5);
// }
// static cc::vector<std::string> ShowPlanes(std::vector<linkml::Plane> planes, tg::aabb3 box){
//     auto names = cc::vector<std::string>();

//     auto faces_filter = [&](pm::face_handle h){
//         auto vx = h.vertices().to_vector();
//         std::array<int, 3> indexis{int(vx[0]),int(vx[1]),int(vx[2])};
//         return indexis;
//         };

//     for (int i = 0; i<(int)planes.size(); i++){

//         pm::Mesh m;
//         auto pos = pm::vertex_attribute<tg::pos3>(m);

//         auto vertecies_filter = [&](pm::vertex_handle h){return pos[h];};

//         auto face_handels = crop_plane_with_aabb(m, pos, box, planes[i]);
//         if (!face_handels.has_value()) continue;
//         auto name = cc::format("Plane {}", i);
//         auto vertecies = m.vertices().map(vertecies_filter).to_vector();
//         auto faces = m.faces().map(faces_filter).to_vector();

//         polyscope::registerSurfaceMesh(name.c_str(), vertecies, faces);

//         names.push_back(name.c_str());
//     }
    
//     return names;
// }

namespace linkml{


    std::vector<pcl::PointIndices> refine(
            PointCloud::Cloud::Ptr const cloud, 
            std::vector<pcl::PointIndices> const & clusters,
            tg::angle angle_threashhold,
            float distance_threshhold
            ){

        std::vector<linkml::Plane> planes;
        planes.resize(clusters.size());

        #pragma omp parallel for
        for (size_t i = 0; i < clusters.size(); ++i)
            planes[i] = fit_plane_thorugh_points(cloud, clusters[i].indices);

        
        std::vector<pcl::PointIndices> indecies;
        std::copy(clusters.begin(), clusters.end(), std::back_inserter(indecies));



        auto pbar = util::progress_bar(planes.size(), "Plane Refinement");
        for (int i = 0; i < (int)planes.size(); i++){

            std::vector<int> sel;

            // Check angle
            for (auto j = i+1; j < (int)planes.size(); j++){

                auto dot = tg::dot(planes[i].normal, planes[j].normal);
                if (dot < tg::cos(angle_threashhold)){
                    sel.push_back(j);
                }
            }


            auto to_be_deleted = std::vector<int>();
            // Check overlap and merge
            for (auto & j: sel){

                auto A = planes[i];
                auto B = planes[j];


                auto A_idx =indecies[i];
                auto B_idx =indecies[j];
                
                auto n_point_of_B_in_A = (long)tg::sum(B_idx.indices, [&](int idx){ return tg::distance(A,cloud->points.at(idx).getPos()) < distance_threshhold; });
                auto n_point_of_A_in_B = (long)tg::sum(A_idx.indices, [&](int idx){ return tg::distance(B,cloud->points.at(idx).getPos()) < distance_threshhold; });


                auto Nt = (long)tg::min( A_idx.indices.size(),B_idx.indices.size())/5;

                if (n_point_of_A_in_B > Nt and n_point_of_B_in_A > Nt){
                    to_be_deleted.push_back(j);
                    // Or mark them as empty
                    pcl::PointIndices merged;
                    std::copy(A_idx.indices.begin(), A_idx.indices.end(), std::back_inserter(merged.indices));
                    std::copy(B_idx.indices.begin(), B_idx.indices.end(), std::back_inserter(merged.indices));
                    planes[i] = fit_plane_thorugh_points(cloud, merged.indices);
                    indecies[i] = merged;
                }

            }

            // Erase merged itmes
            for (auto it = to_be_deleted.rbegin(); it != to_be_deleted.rend(); ++it){
                size_t idx = *it;
                auto p_it = std::next(planes.begin(), idx );
                auto i_it = std::next(indecies.begin(), idx ); 
                planes.erase(p_it);
                indecies.erase(i_it);
            }

            pbar.update(1+to_be_deleted.size());

        }
        pbar.stop();

        return indecies;
            

    }
}

