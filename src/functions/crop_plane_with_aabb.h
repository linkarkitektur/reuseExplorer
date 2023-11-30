
#pragma once
#include <types/plane.h>
#include <types/CellComplex.h>
#include <types/result_fit_planes.h>

#include <functions/color.h>
#include <functions/get_csystem.h>

#include <typed-geometry/tg.hh>
#include <typed-geometry/feature/std-interop.hh>

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
static cc::array<cc::optional<tg::line3>, 6> get_lines(tg::aabb3 &box, linkml::Plane &plane){

        auto lines = cc::array<cc::optional<tg::line3>, 6>();

        auto p0 = tg::plane3(tg::dir3( 1,0,0), tg::abs(box.max.x));
        auto p1 = tg::plane3(tg::dir3(-1,0,0), tg::abs(box.min.x));
        auto p2 = tg::plane3(tg::dir3(0, 1,0), tg::abs(box.max.y));
        auto p3 = tg::plane3(tg::dir3(0,-1,0), tg::abs(box.min.y));
        auto p4 = tg::plane3(tg::dir3(0,0, 1), tg::abs(box.max.z));
        auto p5 = tg::plane3(tg::dir3(0,0,-1), tg::abs(box.min.z));


        lines[0] = (tg::abs(tg::dot(p0.normal, plane.normal)) < 0.9999)? tg::intersection(p0, (tg::plane3)plane): cc::optional<tg::line3>();
        lines[1] = (tg::abs(tg::dot(p1.normal, plane.normal)) < 0.9999)? tg::intersection(p1, (tg::plane3)plane): cc::optional<tg::line3>();
        lines[2] = (tg::abs(tg::dot(p2.normal, plane.normal)) < 0.9999)? tg::intersection(p2, (tg::plane3)plane): cc::optional<tg::line3>();
        lines[3] = (tg::abs(tg::dot(p3.normal, plane.normal)) < 0.9999)? tg::intersection(p3, (tg::plane3)plane): cc::optional<tg::line3>();
        lines[4] = (tg::abs(tg::dot(p4.normal, plane.normal)) < 0.9999)? tg::intersection(p4, (tg::plane3)plane): cc::optional<tg::line3>();
        lines[5] = (tg::abs(tg::dot(p5.normal, plane.normal)) < 0.9999)? tg::intersection(p5, (tg::plane3)plane): cc::optional<tg::line3>();

        return lines;

}
static cc::vector<tg::line3> get_valid_lines(cc::array<cc::optional<tg::line3>, 6>  &lines){
    auto v_list = cc::vector<tg::line3>();
    for (auto & l : lines)
        if (l.has_value())
            v_list.push_back(l.value());

    return v_list;
}
static cc::vector<tg::pos3> get_points(cc::vector<tg::line3> &lines, tg::aabb3 &box){

    auto points = cc::vector<tg::pos3>();

    for (auto & line : lines){
        auto s = tg::intersection(line, box);
        if (!s.has_value())  continue;
        auto value = s.value();
        if (tg::distance(value.pos0, value.pos1) < 0.0009) continue;
        points.push_back(value.pos0);
        points.push_back(value.pos1);
    }


    return points;

}
static cc::vector<tg::angle> get_angles_in_plane(tg::mat3 mat, cc::vector<tg::pos3> points, tg::pos3 center ){
    auto angs = cc::vector<tg::angle>();
    for (auto& p : points){
        auto v = (tg::vec3)tg::normalize(p-center);
        v = tg::inverse(mat) *v;
        auto a = tg::atan2(v.x, v.y);
        angs.push_back(a);
    }
    return angs;
}

static void make_unique(cc::vector<tg::pos3> & collection){
    auto set = std::set<tg::pos3>();

    for (auto & item : collection)
        set.insert(item);

    collection.clear();

    for (auto & item : set)
        collection.push_back(item);
}


namespace linkml {

    static cc::optional<std::vector<pm::face_handle>> crop_plane_with_aabb(pm::Mesh& m, pm::vertex_attribute<tg::pos3>& pos, tg::aabb3 box, Plane plane ){


        std::vector<pm::face_handle> faces;

        auto lines = get_lines(box, plane);
        auto valid_lines = get_valid_lines(lines);
        auto points = get_points(valid_lines, box);
        
        make_unique(points);

        if (points.size() < 3) return {};


        auto [mat, center] = get_csystem(points, plane);
        auto agles = get_angles_in_plane(mat, points, center);

        cc::vector<cc::pair<tg::angle, tg::pos3>> pairs;
        for (size_t i = 0; i < agles.size(); ++i) {
            pairs.emplace_back(agles[i], points[i]);
        }

        std::sort(pairs.begin(), pairs.end(), comparator);

        auto plist = cc::vector<tg::pos3>();
        for (auto & p : pairs)
            plist.push_back(p.second);



        // Add faces
        for (int i = 0; i< (int)plist.size()-1; ++i){

            const auto vh0 = m.vertices().add();
            const auto vh1 = m.vertices().add();
            const auto vh2 = m.vertices().add();

            pos[vh0] = plist[i];
            pos[vh1] = plist[i+1];
            pos[vh2] = center;

            faces.emplace_back(m.faces().add(vh0, vh1, vh2));
        }

        // Add last face
        const auto vh0 = m.vertices().add();
        const auto vh1 = m.vertices().add();
        const auto vh2 = m.vertices().add();

        pos[vh0] = plist[(int)plist.size()-1];
        pos[vh1] = plist[0];
        pos[vh2] = center;

        faces.emplace_back(m.faces().add(vh0, vh1, vh2));

        pm::deduplicate(m, pos);
        m.compactify();
    
        return faces;
    }
    
    static void crop_plane_with_aabb(linkml::CellComplex& cw, tg::aabb3& box, result_fit_planes& results ){


        // Intersect all planes wiht the bounding box to generate initila face candidates.
        for (int i = 0; i < (int)results.planes.size(); i++){
            auto faces = crop_plane_with_aabb(cw.m, cw.pos, box, results.planes[i]);
            if (!faces.has_value()) continue;
            for (auto h : faces.value()){
                if (!h.is_valid()) continue;
                cw.supporting_plans[h] = results.planes[i];
                cw.colors[h] = get_color_forom_angle(sample_circle(i));
            }
        }
    }
}