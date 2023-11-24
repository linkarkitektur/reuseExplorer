
#pragma once
#include <typed-geometry/tg.hh>
#include <polymesh/pm.hh>
#include <polymesh/Mesh.hh>
#include <clean-core/set.hh>
#include <clean-core/array.hh>
#include <clean-core/vector.hh>
#include <clean-core/pair.hh>
#include <clean-core/tuple.hh>

#include <types/plane.h>

static auto comparator = [](cc::pair<tg::angle, tg::pos3> p1, cc::pair<tg::angle, tg::pos3> p2) {
    return p1.first < p2.first;
};
static cc::array<tg::line3, 6> get_lines(tg::aabb3 box, linkml::Plane plane){

        auto lines = cc::array<tg::line3, 6>();

        lines[0] = tg::intersection(tg::plane3(tg::dir3(0,0,1), box.max.z) , plane);
        lines[1] = tg::intersection(tg::plane3(tg::dir3(0,0,-1), box.min.z), plane);
        lines[2] = tg::intersection(tg::plane3(tg::dir3(0,1,0), box.max.y) , plane);
        lines[3] = tg::intersection(tg::plane3(tg::dir3(0,-1,0), box.min.y), plane);
        lines[4] = tg::intersection(tg::plane3(tg::dir3(1,0,0), box.max.x) , plane);
        lines[5] = tg::intersection(tg::plane3(tg::dir3(-1,0,0), box.min.x), plane);

        return lines;

}
static cc::vector<tg::angle> get_angles_in_plane(tg::mat3 mat, cc::set<tg::pos3> points, tg::pos3 center ){
    auto angs = cc::vector<tg::angle>();
    for (auto& p : points){
        auto v = (tg::vec3)tg::normalize(p-center);
        v = tg::inverse(mat) *v;
        auto a = tg::atan2(v.x, v.y);
        angs.push_back(a);
    }
    return angs;
}
static cc::tuple<tg::mat3, tg::pos3> get_csystem(cc::set<tg::pos3> points, linkml::Plane plane){
        auto center = tg::average(points);
        auto point =  *points.begin();
        auto mat = tg::mat3();
        auto aX = tg::normalize_safe(point-center);
        auto aY = tg::normalize_safe(tg::cross(plane.normal, aX));
        mat.set_col(0, aX);
        mat.set_col(1, aY);
        mat.set_col(2, plane.normal);
        return cc::tuple(mat, center); 
}

namespace linkml {

    static bool crop_plane_with_aabb(pm::Mesh & m, pm::vertex_attribute<tg::pos3> pos, tg::aabb3 box, Plane plane ){


        auto lines = get_lines(box, plane);

        auto points = cc::set<tg::pos3>();

        for (auto & line : lines){
            auto s = tg::intersection(line, box);
            if (!s.has_value())  continue;
            points.add(s.value().pos0);
            points.add(s.value().pos1);
        }

        if (points.size() < 3) return false;

        auto [mat, center] = get_csystem(points, plane);
        auto agles = get_angles_in_plane(mat, points, center);

        cc::vector<cc::pair<tg::angle, tg::pos3>> pairs;
        for (size_t i = 0; i < agles.size(); ++i) {
            pairs.emplace_back(agles[i], points[i]);
        }

        std::sort(pairs.begin(), pairs.end(), comparator);

        auto plist = cc::vector<tg::pos3>();
        for (auto & p : pairs)
            plist.push_back();



        // Add faces
        for (int i = 0; i< (int)plist.size()-1; ++i){

            const auto vh0 = m.vertices().add();
            const auto vh1 = m.vertices().add();
            const auto vh2 = m.vertices().add();

            pos[vh0] = plist[i];
            pos[vh1] = plist[i+1];
            pos[vh2] = center;

            m.faces().add(vh0, vh1, vh2);
        }

        // Add last face
        const auto vh0 = m.vertices().add();
        const auto vh1 = m.vertices().add();
        const auto vh2 = m.vertices().add();

        pos[vh0] = plist[(int)plist.size()-1];
        pos[vh1] = plist[0];
        pos[vh2] = center;

        m.faces().add(vh0, vh1, vh2);

        pm::deduplicate(m, pos);
        m.compactify();
    
        return true;
    }
    
}