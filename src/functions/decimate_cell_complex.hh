#pragma once
#include <functions/alpha_shape.h>
#include <types/CellComplex.h>

namespace linkml {

    void decimate_cell_complex(linkml::CellComplex & cw){

        // Make a set vector of all facet ids.
        auto set = cw.faces().to_set([&](pm::face_handle h){return cw.facets[h]; });
        std::vector<std::size_t> ids(set.begin(), set.end());


        auto face_indecies = std::vector<std::vector<size_t>>(ids.size()); // Indecies of selected veticies
        auto face_vertecies = std::vector<cc::vector<tg::pos3>>(ids.size()); // All vertecies
        auto old_faces = std::vector<std::vector<pm::face_handle>>(ids.size()); //List of all faces that have been simplified



#pragma omp parallel
#pragma omp for
        for (int i = 0; i < ids.size(); i++){

            auto id = ids[i];

            auto facet = cw.faces().where([&](pm::face_handle h){ return cw.facets[h] == id;});
            old_faces[i] = facet.to_vector();

            auto plane = cw.supporting_plans[facet.first()];

            auto verts = cc::vector<tg::pos3>();
            for (auto f : facet)
                for ( auto v: f.vertices())
                    verts.push_back(cw.pos[v]);
            face_vertecies[i] = verts;



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
                    if (tg::dot(v_in,v1_out) < 0.99 ) {
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
        }


// End parallel loop


        for (int i = 0; i < face_indecies.size(); i++){

            auto indecies = face_indecies[i];
            auto faces = old_faces[i];
            auto verts = face_vertecies[i];


            // Construct face
            auto vh0 = cw.vertices().add();
            cw.pos[vh0] = verts[indecies[0]];

            for (int j = 1; j < ((int)indecies.size()-1); j++){
                
                auto vhj = cw.vertices().add();
                auto vhk = cw.vertices().add();

                cw.pos[vhj] = verts[indecies[j]];
                cw.pos[vhk] = verts[indecies[j+1]];

                auto fh = cw.faces().add(vh0,vhj, vhk);
                cw.copy_face_attributes(fh, cw,faces[0] );
            }
            for (auto h : faces)
                cw.faces().remove(h);
        }

    }
}