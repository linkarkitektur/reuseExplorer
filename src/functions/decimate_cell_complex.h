#pragma once

namespace linkml {

    void decimate_cell_complex(
        linkml::CellComplex const & cw, 
        linkml::CellComplex & cw2, 
        polymesh::face_attribute<std::vector<int>> const &cell_id,
        std::map<std::vector<int>, tg::color3> const & cell_color_look_up ){


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

            auto verts = cc::vector<tg::pos3>();
            for (auto f : facet)
                for ( auto v: f.vertices())
                    verts.push_back(cw.pos[v]);

            auto indecies = convex_hull(project_2d(verts, plane));


            auto indecies_simplified = std::vector<size_t>();
            // Simplify convex hull by comparing entrance and exit vector if they are close to a streight line.
            if (indecies.size() > 3){
            
                auto v_in = tg::normalize_safe(verts[indecies[0]] - verts[indecies[indecies.size()-1]]);
                int i = 0;
                auto reduce = [&](int n, int m){
                    auto p_o = verts[indecies[n]];
                    auto p_n = verts[indecies[m]];

                    auto v1_out = tg::normalize_safe(p_n-p_o);

                    //TODO: Check value
                    if (tg::dot(v_in,v1_out) < 0.97 ) {
                        indecies_simplified.push_back(indecies[i]);
                        v_in = v1_out;

                    };

                };
                while (i < indecies.size() -1){
                    reduce(i, i+1);
                    i++;
                }

                reduce(indecies.size() -1, 0);

            }else{
                indecies_simplified = indecies;
            }


            face_indecies[i] = indecies_simplified;
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
                
                auto vhj = cw2.m.vertices().add();
                auto vhk = cw2.m.vertices().add();

                cw2.pos[vhj] = verts[indecies[j]];
                cw2.pos[vhk] = verts[indecies[j+1]];

                auto fh = cw2.m.faces().add(vh0,vhj, vhk);


                cw2.colors[fh] = cw.colors[facet_h];
                cw2.facets_colors[fh] = cw.facets_colors[facet_h];
            }
        }




    }
}