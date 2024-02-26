#pragma once
#include <types/CellComplex.hh>
#include <types/point_cloud.hh>


namespace linkml {


    void compute_coverage(linkml::CellComplex & cw, linkml::point_cloud const & cloud){

        // Make a set vector of all facet ids.
        auto set = cw.faces().to_set([&](pm::face_handle h){return cw.facets[h]; });
        std::vector<std::size_t> ids(set.begin(), set.end());

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

            for (auto & h : facet_vec) cw.facet_area[h] = area_facet;
            for (auto & h : facet_vec) cw.covered_area[h] = area_points;
            for (auto & h : facet_vec) cw.supporting_point_num[h] = point_in_facet_index.size();

        }


    }






}