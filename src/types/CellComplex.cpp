
#include <types/CellComplex.h>
#include <typed-geometry/feature/vector.hh>
#include <typed-geometry/types/objects/aabb.hh>
#include <typed-geometry/feature/std-interop.hh>


namespace linkml{

    Faces CellComplex::faces(){
        return m.faces().map([&](pm::face_handle f){
            auto vx = f.vertices().to_vector();
            std::array<int, 3> indexis{int(vx[0]),int(vx[1]),int(vx[2])};
            return indexis;
        }).to_vector();
    }

    Verts CellComplex::vertecies(){
        return m.vertices().map([&](pm::vertex_handle h){
            auto p = pos[h];
            std::array<float,3> arr = std::array<float,3>();
            arr[0] = p.x;
            arr[1] = p.y;
            arr[2] = p.z;
            return arr;
        }).to_vector();
    }

    tg::aabb3 CellComplex::box(){
        auto box = this->pos.aabb();
        return tg::aabb3(box.min, box.max ); }
}
   