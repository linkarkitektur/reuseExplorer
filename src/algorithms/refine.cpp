#include <algorithms/refine.h>
#include <typed-geometry/tg.hh>
#include <types/result_fit_planes.h>
#include <types/point_cloud.h>
#include <clean-core/vector.hh>

#include <functions/fit_plane_thorugh_points.h>
#include <functions/progress_bar.h>




namespace linkml{
    std::vector<Plane> refine(point_cloud cloud, result_fit_planes & rs,  refinement_parameters const & param){

        std::vector<Plane> planes;
        std::copy(rs.planes.begin(), rs.planes.end(), std::back_inserter(planes));

        std::vector<std::vector<int>> indecies;
        std::copy(rs.indecies.begin(), rs.indecies.end(), std::back_inserter(indecies));

        auto pbar = util::progress_bar(planes.size(), "Plane Refinement");


        for (int i = 0; i < (int)planes.size(); i++){

            std::vector<int> sel;

            // Check angle
            for (auto j = i+1; j < (int)planes.size(); j++){

                auto dot = tg::dot(planes[i].normal, planes[j].normal);
                if (dot < tg::cos(param.angle_threashhold)){
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

                auto n_point_of_B_in_A = (long)tg::sum(B_idx, [&](int idx){ return tg::distance(A,cloud.pts[idx]) < param.distance_threshhold; });
                auto n_point_of_A_in_B = (long)tg::sum(A_idx, [&](int idx){ return tg::distance(B,cloud.pts[idx]) < param.distance_threshhold; });


                auto Nt = (long)tg::min( A_idx.size(),B_idx.size())/5;

                if (n_point_of_A_in_B > Nt and n_point_of_B_in_A > Nt){
                    to_be_deleted.push_back(j);
                    // Or mark them as empty
                    std::vector<int> merged;
                    std::copy(A_idx.begin(), A_idx.end(), std::back_inserter(merged));
                    std::copy(B_idx.begin(), B_idx.end(), std::back_inserter(merged));
                    planes[i] = fit_plane_thorugh_points(cloud, merged);
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

        return planes;

    }

}

