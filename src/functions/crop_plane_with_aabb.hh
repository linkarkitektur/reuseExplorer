
#pragma once
#include <types/Plane.hh>
#include <types/Surface_Mesh.hh>

#include <functions/color.hh>
#include <functions/get_csystem.hh>

#include <typed-geometry/tg.hh>
#include <typed-geometry/feature/std-interop.hh>

#include <CGAL/boost/graph/Euler_operations.h>

#include <polymesh/pm.hh>
#include <polymesh/Mesh.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <clean-core/set.hh>
#include <clean-core/array.hh>
#include <clean-core/vector.hh>
#include <clean-core/pair.hh>
#include <clean-core/tuple.hh>


static auto comparator = [](cc::pair<tg::angle, tg::pos3> p1, cc::pair<tg::angle, tg::pos3> p2) {
    return p1.first < p2.first;
};
static cc::array<tg::segment3, 12> get_segemets(tg::aabb3 const & box) {

        auto segemets = cc::array<tg::segment3, 12>();

        auto p0 = tg::pos3(box.min.x,box.min.y, box.min.z); // 0
        auto p1 = tg::pos3(box.max.x,box.min.y, box.min.z); // 1
        auto p2 = tg::pos3(box.min.x,box.max.y, box.min.z); // 2
        auto p3 = tg::pos3(box.max.x,box.max.y, box.min.z); // 3

        auto p4 = tg::pos3(box.min.x,box.min.y, box.max.z); // 4
        auto p5 = tg::pos3(box.max.x,box.min.y, box.max.z); // 5
        auto p6 = tg::pos3(box.min.x,box.max.y, box.max.z); // 6
        auto p7 = tg::pos3(box.max.x,box.max.y, box.max.z); // 7



        segemets[0] = tg::segment3(p0, p1); 
        segemets[1] = tg::segment3(p1, p3); 
        segemets[2] = tg::segment3(p3, p2); 
        segemets[3] = tg::segment3(p2, p0); 
        segemets[4] = tg::segment3(p4, p5); 
        segemets[5] = tg::segment3(p5, p7); 
        segemets[6] = tg::segment3(p7, p6);
        segemets[7] = tg::segment3(p6, p4);
        segemets[8] = tg::segment3(p0, p4);
        segemets[9] = tg::segment3(p1, p5);
        segemets[10]= tg::segment3(p3, p7);
        segemets[11]= tg::segment3(p2, p6);

        return segemets;
}
static std::vector<tg::pos3> get_points(cc::array<tg::segment3, 12> const & segments, tg::plane3 const & plane){

    auto points = std::vector<tg::pos3>();

    for (auto & seg : segments){

        auto pt = tg::intersection(seg, plane);

        if (!pt.has_value()) continue;
        points.push_back(pt.value());
    }


    return points;

}
static std::vector<tg::angle> get_angles_in_plane(tg::mat3 mat, std::vector<tg::pos3> points, tg::pos3 center ){
    auto angs = std::vector<tg::angle>();
    for (auto& p : points){
        auto v = (tg::vec3)tg::normalize(p-center);
        v = tg::inverse(mat) *v;
        auto a = tg::atan2(v.x, v.y);
        angs.push_back(a);
    }
    return angs;
}
static void make_unique(std::vector<tg::pos3> & collection){
    auto set = std::unordered_set<tg::pos3>();

    for (auto & item : collection)
        set.insert(item);

    collection.clear();

    for (auto & item : set)
        collection.push_back(item);
}


namespace linkml {

    //static cc::optional<std::vector<pm::face_handle>> crop_plane_with_aabb(pm::Mesh& m, pm::vertex_attribute<tg::pos3>& pos, tg::aabb3 const &box, Plane const &plane ){


    //    std::vector<pm::face_handle> faces;

    //    auto const segemets = get_segemets(box);
    //    auto points = get_points(segemets, plane);
    //    
    //    make_unique(points);

    //    if (points.size() < 3) return {};


    //    auto [mat, center] = get_csystem(points, plane);
    //    auto agles = get_angles_in_plane(mat, points, center);

    //    cc::vector<cc::pair<tg::angle, tg::pos3>> pairs;
    //    for (size_t i = 0; i < agles.size(); ++i) {
    //        pairs.emplace_back(agles[i], points[i]);
    //    }

    //    std::sort(pairs.begin(), pairs.end(), comparator);

    //    auto plist = cc::vector<tg::pos3>();
    //    for (auto & p : pairs)
    //        plist.push_back(p.second);



    //    // Add faces
    //    for (int i = 0; i< (int)plist.size()-1; ++i){

    //        const auto vh0 = m.vertices().add();
    //        const auto vh1 = m.vertices().add();
    //        const auto vh2 = m.vertices().add();

    //        pos[vh0] = plist[i];
    //        pos[vh1] = plist[i+1];
    //        pos[vh2] = center;

    //        faces.emplace_back(m.faces().add(vh0, vh1, vh2));
    //    }

    //    // Add last face
    //    const auto vh0 = m.vertices().add();
    //    const auto vh1 = m.vertices().add();
    //    const auto vh2 = m.vertices().add();

    //    pos[vh0] = plist[(int)plist.size()-1];
    //    pos[vh1] = plist[0];
    //    pos[vh2] = center;

    //    faces.emplace_back(m.faces().add(vh0, vh1, vh2));

    //    pm::deduplicate(m, pos);
    //    m.compactify();
    
    //    return faces;
    //}
    
    //static void crop_plane_with_aabb(linkml::CellComplex& cw, tg::aabb3& box, std::vector<pcl::PointIndices> const & clusters){

    //    // Intersect all planes wiht the bounding box to generate initila face candidates.
    //    for (int i = 0; i < (int)results.planes.size(); i++){
    //        auto faces = crop_plane_with_aabb(cw, cw.pos, box, results.planes[i]);
    //        if (!faces.has_value()) continue;
    //        for (auto h : faces.value()){
    //            if (!h.is_valid()) continue;
    //            cw.supporting_plans[h] = results.planes[i];
    //            cw.plane_colors[h] = get_color_forom_angle(sample_circle(i));
    //        }
    //    }
    //}

    static void crop_plane_with_aabb(Surface_mesh & mesh, const tg::aabb3& box, const tg::plane3 plane ){
    
            auto const segemets = get_segemets(box);
            auto points = get_points(segemets, plane);

            
            make_unique(points);
    
            if (points.size() < 3) return;
    
            auto [mat, center] = get_csystem(points, plane);
            auto agles = get_angles_in_plane(mat, points, center);
    
            cc::vector<cc::pair<tg::angle, tg::pos3>> pairs;
            for (size_t i = 0; i < agles.size(); ++i) {
                pairs.emplace_back(agles[i], points[i]);
            }
    
            std::sort(pairs.begin(), pairs.end(), comparator);
    
            std::vector<Surface_mesh::vertex_index> vertecies;
            vertecies.reserve(pairs.size());
            for (auto & p : pairs)
                vertecies.push_back(mesh.add_vertex(Point_3(p.second.x, p.second.y, p.second.z)));


            int t = vertecies.size()-1;
            for (int i = 0; i < t-1; ++i)
                mesh.add_face(vertecies[i], vertecies[i+1], vertecies[t]);
            
            

    }

}