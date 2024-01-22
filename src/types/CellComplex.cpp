
#include <types/CellComplex.hh>
#include <typed-geometry/feature/vector.hh>
#include <typed-geometry/types/objects/aabb.hh>
#include <typed-geometry/feature/std-interop.hh>


namespace linkml{

    Faces CellComplex::ps_faces(){

        return this->faces().map([&](pm::face_handle f){
            auto [vh0, vh1, vh2] = f.vertices().to_array<3>();
            std::array<int, 3> indexis{int(vh0),int(vh1),int(vh2)};
            return indexis;
        }).to_vector();
    }

    Verts CellComplex::ps_vertecies(){

        auto array_view = pos.view([](tg::pos3& p) -> std::array<float, 3>& { 
            std::array<float, 3> pt{{ p.x, p.y, p.z }};
            return pt;
            });

        return this->vertices().map([&](pm::vertex_handle h){ return array_view[h]; }).to_vector();
    }

}
   