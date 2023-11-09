#include "linkml.h"
#include "cellcomplex.h"

#include <omp.h>

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

#include <typed-geometry/tg.hh>

#include <polymesh/pm.hh>
#include <polymesh/Mesh.hh>
#include <polymesh/algorithms.hh>
#include <polymesh/algorithms/deduplicate.hh>


// #include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
// #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Alpha_shape_2.h>
// #include <CGAL/Alpha_shape_vertex_base_2.h>
// #include <CGAL/Alpha_shape_face_base_2.h>
// #include <CGAL/Delaunay_triangulation_2.h>
// #include <CGAL/algorithm.h>




void foo(pm::Mesh& m, pm::vertex_attribute<tg::pos3>& pos){
  pm::deduplicate(m, pos);
}




namespace linkml {

    


    float THREASHHOLD = 0.001;


    static Cell ConstructFromAABB(tg::aabb3 box){

            auto cell = Cell();
            cell.box = box;
            
            auto min = cell.box.min;
            auto max = cell.box.max;

            auto pt_0 = tg::pos3(min.x, min.y, min.z);
            auto pt_1 = tg::pos3(max.x, min.y, min.z);
            auto pt_2 = tg::pos3(min.x, max.y, min.z);
            auto pt_3 = tg::pos3(max.x, max.y, min.z);
            auto pt_4 = tg::pos3(min.x, min.y, max.z);
            auto pt_5 = tg::pos3(max.x, min.y, max.z);
            auto pt_6 = tg::pos3(min.x, max.y, max.z);
            auto pt_7 = tg::pos3(max.x, max.y, max.z);

            cell.triangels.push_back(tg::triangle3(pt_0, pt_1, pt_5));
            cell.triangels.push_back(tg::triangle3(pt_5, pt_4, pt_0));
            cell.triangels.push_back(tg::triangle3(pt_1, pt_3, pt_5));
            cell.triangels.push_back(tg::triangle3(pt_5, pt_1, pt_3));
            cell.triangels.push_back(tg::triangle3(pt_3, pt_2, pt_6));
            cell.triangels.push_back(tg::triangle3(pt_6, pt_7, pt_3));
            cell.triangels.push_back(tg::triangle3(pt_2, pt_0, pt_6));
            cell.triangels.push_back(tg::triangle3(pt_6, pt_0, pt_4));
            cell.triangels.push_back(tg::triangle3(pt_0, pt_1, pt_3));
            cell.triangels.push_back(tg::triangle3(pt_3, pt_2, pt_0));
            cell.triangels.push_back(tg::triangle3(pt_4, pt_5, pt_6));
            cell.triangels.push_back(tg::triangle3(pt_6, pt_5, pt_7));


            return cell;
        };
    static std::tuple<std::vector<tg::pos3>, std::vector<tg::pos3>> split_vertecies(tg::triangle3 t, tg::plane3 p){

        auto pts_cell1 = std::vector<tg::pos3>();
        auto pts_cell2 = std::vector<tg::pos3>();

        //TODO: Try using signed distance

        auto d0 = tg::signed_distance(t.pos0, p);
        auto d1 = tg::signed_distance(t.pos1, p);
        auto d2 = tg::signed_distance(t.pos2, p);


        if (d0 >=0){pts_cell1.push_back(t.pos0);}else{pts_cell2.push_back(t.pos0);}
        if (d1 >=0){pts_cell1.push_back(t.pos1);}else{pts_cell2.push_back(t.pos1);}
        if (d2 >=0){pts_cell1.push_back(t.pos2);}else{pts_cell2.push_back(t.pos2);}

        return std::make_tuple(pts_cell1, pts_cell2);

    }
    static std::vector<tg::triangle3> triagles_from_points_and_segment(std::vector<tg::pos3> pts, tg::segment3 seg, tg::vec3 trig_norm){

        auto result = std::vector<tg::triangle3>();
        if (pts.size() == 1){
            auto pt = pts[0];

            auto norm = tg::cross(seg.pos0-pt, seg.pos1-pt);
            tg::triangle3 trig = (tg::dot(norm, trig_norm) > .5) ? tg::triangle3(pt, seg.pos0, seg.pos1) : tg::triangle3(pt, seg.pos1, seg.pos0);
            result.push_back(trig);
            
        } else if (pts.size() == 2) {

            auto pt0 = pts[0];
            auto pt1 = pts[1];

            auto norm0 = tg::cross(seg.pos0-pt0, seg.pos1-pt0);

            bool flip = tg::dot(norm0, trig_norm) > .5;
            tg::triangle3 trig0 = (flip) ? tg::triangle3(pt0, seg.pos0, seg.pos1) : tg::triangle3(pt0, seg.pos1, seg.pos0);
            result.push_back(trig0);

            auto set_pt = (flip) ? seg.pos1 : seg.pos0;
            auto norm1 = tg::cross(pt0-pt1, set_pt-pt1);
            tg::triangle3 trig1 = (tg::dot(norm1, trig_norm) > .5) ? tg::triangle3(pt1, pt0, set_pt) : tg::triangle3(pt1, set_pt, pt0);
            result.push_back(trig1);


        } else {
            auto s = pts.size();
            std::cout << "Number of points: " << pts.size() << std::endl;
            throw std::invalid_argument("can only construct triangles form one or two points");
        }

        return result;

    } 

    // static pm::Mesh plane_aabb_intersect_as_mesh(tg::aabb3 b, tg::plane3 p){

    //     pm::Mesh m = pm::Mesh();
    //     auto pos = pm::vertex_attribute<tg::pos3>(m);

    //     auto p0 = tg::plane3(tg::dir3(1,0,0), b.max.x );
    //     auto p1 = tg::plane3(tg::dir3(0,1,0), b.max.y );
    //     auto p2 = tg::plane3(tg::dir3(0,0,1), b.max.z );
    //     auto p3 = tg::plane3(tg::dir3(-1,0,0), b.min.x );
    //     auto p4 = tg::plane3(tg::dir3(0,-1,0), b.min.y );
    //     auto p5 = tg::plane3(tg::dir3(0,0,-1), b.min.z );

    //     auto s0 = tg::intersection(p0,p);
    //     auto s1 = tg::intersection(p1,p);
    //     auto s2 = tg::intersection(p2,p);
    //     auto s3 = tg::intersection(p3,p);
    //     auto s4 = tg::intersection(p4,p);
    //     auto s5 = tg::intersection(p5,p);

    //     auto b0 = tg::intersects(s0, b);
    //     auto b1 = tg::intersects(s1, b);
    //     auto b2 = tg::intersects(s2, b);
    //     auto b3 = tg::intersects(s3, b);
    //     auto b4 = tg::intersects(s4, b);
    //     auto b5 = tg::intersects(s5, b);

    //     std::vector<tg::line3> segments = std::vector<tg::line3>();

    //     if (b0) segments.push_back(s0);
    //     if (b1) segments.push_back(s1);
    //     if (b2) segments.push_back(s2);
    //     if (b3) segments.push_back(s3);
    //     if (b4) segments.push_back(s4);
    //     if (b5) segments.push_back(s5);





    // }

    std::vector<Cell> CreateCellComplex(linkml::point_cloud const &cloud, std::vector<linkml::Plane> const &planes){

        // polyscope::init();

        // polyscope::PointCloud* psCloud = polyscope::registerPointCloud("Cloud", cloud.pts);
        // psCloud->setPointRadius(0.001);
        // psCloud->setPointRenderMode(polyscope::PointRenderMode::Sphere);

        // // polyscope::show();



        auto cell = ConstructFromAABB(tg::aabb_of(cloud.pts)); //tg::aabb3(tg::min(cloud.pts)[0], tg::max(cloud.pts)[0]));
        std::vector<Cell> cells = std::vector<Cell>();
        cells.push_back(cell);

        // for (auto& plane : planes){
        for (int i = 0; i < 5; i++){

            auto plane = planes[i];


            for (int i = 0; i < (int)cells.size(); i++){

                // Check if the whole volume is being intersected.
                // Otherwise there is no need to check each face.
                bool ib = tg::intersects(cells[i].box, (tg::plane3)plane);

                if (ib){

                    auto cell1 = Cell();
                    auto cell2 = Cell();
                    auto segments = std::vector<tg::segment3>(); 

                    for (auto& t : cells[i].triangels){

                        auto dpos0 = tg::distance(t.pos0, plane) < THREASHHOLD;
                        auto dpos1 = tg::distance(t.pos1, plane) < THREASHHOLD;
                        auto dpos2 = tg::distance(t.pos2, plane) < THREASHHOLD;

                        // Contine of and point is just touching the plane
                        bool too_close = dpos0 | dpos1 | dpos2;

                        bool valid = (!too_close)? tg::intersects(t, plane): false;

                        if (valid){

                            cc::optional<tg::segment3> intersection = tg::intersection(t, plane);

                            // Find segment and add to collection
                            tg::segment3 seg =  intersection.value();
                            segments.push_back(seg);


                            // Normal of trangle to ensure new faces have 
                            auto trig_norm = tg::cross(t.pos1-t.pos0, t.pos2-t.pos0);


                            // Filter points based on the side of the plane they are on
                            auto [ pts_cell1, pts_cell2 ]= split_vertecies(t, plane);

                            // Construct and add triangle to sells
                            for (auto& t : triagles_from_points_and_segment(pts_cell1, seg, trig_norm))
                                cell1.triangels.push_back(t);
                            
                            for (auto& t : triagles_from_points_and_segment(pts_cell2, seg, trig_norm))
                                cell2.triangels.push_back(t);


                        } else {

                            // Sort triangles

                            auto comp = tg::bcomp3();
                            comp.comp0 = tg::dot(t.pos0, plane.normal) >=0;
                            comp.comp1 = tg::dot(t.pos1, plane.normal) >=0;
                            comp.comp2 = tg::dot(t.pos2, plane.normal) >=0;

                            if (tg::all(comp)){
                                cell1.triangels.push_back(t);
                            } else {
                                cell2.triangels.push_back(t);
                            }

                        }
                    }

                    // // Triangulate the last face between the volumes.
                    // auto pts = std::vector<tg::pos3>();

                    // for (auto& seg :segments){
                    //     pts.push_back(seg.pos0);
                    //     pts.push_back(seg.pos1);
                    // }

                    // tg::pos3 center;
                    // if( pts.size() > 1) center = tg::project(tg::average(pts), plane);

                    // for (auto& seg :segments){

                    //     auto trig1 = tg::triangle3(center, seg.pos0, seg.pos1);
                    //     auto trig2 = tg::triangle3(center, seg.pos1, seg.pos0);

                    //     auto norm = tg::cross(trig1.pos1-trig1.pos0, trig1.pos2-trig1.pos0);

                    //     if (tg::dot(norm, plane.normal) > .5){
                    //         cell1.triangels.push_back(trig1);
                    //         cell2.triangels.push_back(trig2);
                    //     } else {
                    //         cell1.triangels.push_back(trig2);
                    //         cell2.triangels.push_back(trig1);
                    //     }
                    // }
                    
                    // Add cells to cell complex
                    bool c1 = cell1.triangels.size() > 0;
                    bool c2 = cell2.triangels.size() > 0;


                    if (c1) cell1.box = tg::aabb_of(cell1.triangels);
                    if (c2) cell2.box = tg::aabb_of(cell2.triangels);

                    if (c1 & c2){
                        cells[i] = cell1;
                        cells.push_back(cell2);
                    } else if (c1 | c2) {
                        cells[i] = c1 ? cell1: cell2;
                    }

                }

            }            

        };

        for (auto& cell : cells){
            pm::Mesh m;
            auto pos = pm::vertex_attribute<tg::pos3>(m);

            for (auto& tri: cell.triangels){

                /// add vertices to topology
                const auto vh0 = m.vertices().add();
                const auto vh1 = m.vertices().add();
                const auto vh2 = m.vertices().add();

                /// add vertex positions
                pos[vh0] = tri.pos0;
                pos[vh1] = tri.pos1;
                pos[vh2] = tri.pos2;

                /// add face to topology
                m.faces().add(vh0, vh1, vh2);
            }
            
            pm::deduplicate(m, pos);
            m.compactify();

            // cell.triangels = m.faces().map([&](pm::face_handle f){
            //     return tg::triangle3(
            //         pos[f.vertices().to_vector()[0]],
            //         pos[f.vertices().to_vector()[1]],
            //         pos[f.vertices().to_vector()[2]]);
            // }).to_vector();

            cell.vertecies = m.all_vertices().map([&](pm::vertex_handle h){
                auto p = pos[h];
                std::array<float,3> arr = std::array<float,3>();
                arr[0] = p.x;
                arr[1] = p.y;
                arr[2] = p.z;
                return arr;
                }).to_vector();
            cell.faces = m.faces().map([&](pm::face_handle f){
                auto vx = f.vertices().to_vector();
                std::array<int, 3> indexis{int(vx[0]),int(vx[1]),int(vx[2])};
                return indexis;
                }).to_vector();
        }

        return cells;

    };
}