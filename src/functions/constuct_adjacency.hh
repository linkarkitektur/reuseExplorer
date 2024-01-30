#pragma once
#include <types/CellComplex.hh>
#include <polymesh/pm-std.hh>
#include <assert.h>



// #define HASH_VAL(val) hasher(val) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2)
// #define ROUND_VAL(val)  (int)std::floor(val * 1000) 


// struct MyHash {

//     std::size_t operator()(const tg::pos3& value) const {
//         int x = ROUND_VAL(value.x);
//         int y = ROUND_VAL(value.y);
//         int z = ROUND_VAL(value.z);

//         // std::cout << "Values: " << x << " " << y << " " << z << std::endl;
//         // std::cout << "Actual: " << value.x << " " << value.y << " " << value.z << std::endl;


//         std::hash<float> hasher;
//         size_t hashValue = 0;

//         // Combine hash values of individual components
//         hashValue ^= HASH_VAL(x);
//         hashValue ^= HASH_VAL(y);
//         hashValue ^= HASH_VAL(z);

//         // std::cout << "Hash: " << hashValue << std::endl;


//         return hashValue;
//     }

//     std::size_t operator()(const tg::pos3* value) const {
//         int x = ROUND_VAL(value->x);
//         int y = ROUND_VAL(value->y);
//         int z = ROUND_VAL(value->z);

//         std::hash<float> hasher;
//         size_t hashValue = 0;

//         // Combine hash values of individual components
//         hashValue ^= HASH_VAL(x);
//         hashValue ^= HASH_VAL(y);
//         hashValue ^= HASH_VAL(z);


//         return hashValue;
//     }

//     bool operator()(const tg::pos3& a, const tg::pos3& b) const {
//         // Implement equality comparison here
//         return ROUND_VAL(a.x) == ROUND_VAL(b.x) &&
//                ROUND_VAL(a.y) == ROUND_VAL(b.y) &&
//                ROUND_VAL(a.z) == ROUND_VAL(b.z);
//     }

//     bool operator()(const tg::pos3* a, const tg::pos3* b) const {
//         // Implement equality comparison here
//         return ROUND_VAL(a->x) == ROUND_VAL(b->x) &&
//                ROUND_VAL(a->y) == ROUND_VAL(b->y) &&
//                ROUND_VAL(a->z) == ROUND_VAL(b->z);
//     }

// };



namespace linkml {

    struct SuperEdge : public std::vector<pm::face_handle> {
        SuperEdge(tg::pos3 s, tg::pos3 t): s(s), t(t){}
        const tg::pos3 s;
        const tg::pos3 t;
    };


    typedef typename std::unordered_set<tg::pos3>                                   PosSet;
    typedef typename std::unordered_map<const tg::pos3*, std::unordered_set<int>>   Edge_map;
    typedef typename std::unordered_map<const tg::pos3*, Edge_map>                  Face_pool;
    typedef typename std::vector<SuperEdge>						                    Adjacency;


    static Adjacency constuct_adjacency(CellComplex & cw){

        assert(cw.is_compact());

        
        Face_pool face_pool;


        PosSet unique_vertecies;
        for (auto h: cw.vertices()) unique_vertecies.insert(cw.pos[h]);


        // polyscope::registerPointCloud("Unique points", unique_vertecies);
        std::vector<tg::pos3> nodes(unique_vertecies.begin(), unique_vertecies.end());
        std::vector<std::array<int,2>> edges;
        std::vector<double> lengt;



        for (auto h: cw.halfedges()){

            // Check if half edge has face
            if (h.face().is_invalid())
                continue;


            pm::vertex_handle sh = h.vertex_from();
            pm::vertex_handle th = h.vertex_to();

            // Obtain iterators to unique vertices
            auto s_iter = unique_vertecies.find(cw.pos[sh]);
            auto t_iter = unique_vertecies.find(cw.pos[th]);

            // Ensure that an edge is unique
            if (s_iter == unique_vertecies.end() or t_iter == unique_vertecies.end()) continue; 

            auto s = &(*s_iter);
            auto t = &(*t_iter);

            if (s > t)
                std::swap(s, t);

            auto id = int(h.face());
            face_pool[s][t].insert(id);
            
        }


        Adjacency fans;
        for (auto & it : face_pool){
            const auto s = it.first;

            for (auto & cur : it.second){

                const auto t = cur.first;
                const auto faces = cur.second;


                // Skip inner edges:
                if (faces.size() == 2){
                    auto jt = faces.begin();
                    auto fiA = *jt;
                    jt++;
                    auto fiB = *jt;

                    auto fA = cw.faces()[fiA];
                    auto fB = cw.faces()[fiB];

                    auto A = cw.facets[fA];
                    auto B = cw.facets[fB];

                    if (A == B) continue;

                }

                
                auto fan = SuperEdge(*s, *t);

                auto s_it = std::find(nodes.begin(), nodes.end(), *s);
                int sid = s_it - nodes.begin();

                auto t_it = std::find(nodes.begin(), nodes.end(), *t);
                int tid = t_it - nodes.begin();

                std::array<int,2> arr {sid, tid};
                edges.push_back(arr);
                lengt.emplace_back(tg::length(tg::segment3(*s, *t)));

                for (auto & id : faces) fan.push_back(cw.faces()[id]);
                    
                fans.push_back(fan);
            }
        }

        return fans;

    }

}