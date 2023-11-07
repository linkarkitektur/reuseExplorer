#include "linkml.h"
#include "cellcomplex.h"
#include <typed-geometry/tg.hh>
#include <omp.h>


namespace linkml {


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

        if (tg::dot(t.pos0, p.normal) >=0){pts_cell1.push_back(t.pos0);}else{pts_cell2.push_back(t.pos0);}
        if (tg::dot(t.pos1, p.normal) >=0){pts_cell1.push_back(t.pos1);}else{pts_cell2.push_back(t.pos1);}
        if (tg::dot(t.pos2, p.normal) >=0){pts_cell1.push_back(t.pos2);}else{pts_cell2.push_back(t.pos2);}

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
            throw std::invalid_argument("can only construct triangles form one or two points");
        }

        return result;

    } 

    std::vector<Cell> CreateCellComplex(linkml::point_cloud const &cloud, std::vector<linkml::Plane> const &planes){

        auto cell = ConstructFromAABB(tg::aabb_of(cloud.pts)); //tg::aabb3(tg::min(cloud.pts)[0], tg::max(cloud.pts)[0]));
        std::vector<Cell> cells = std::vector<Cell>();
        cells.push_back(cell);

        for (auto& plane : planes){

            for (int i = 0; i < (int)cells.size(); i++){

                // Check if the whole volume is being intersected.
                // Otherwise there is no need to check each face.
                bool ib = tg::intersects(cells[i].box, (tg::plane3)plane);

                if (ib){

                    auto cell1 = Cell();
                    auto cell2 = Cell();

                    for (auto& t : cells[i].triangels){
                        cc::optional<tg::segment3> intersection = tg::intersection(t, plane);

                        auto segments = std::vector<tg::segment3>(); 

                        if (intersection.has_value()){

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
                        // Triangulate the last face between the volumes.

                        auto pts = std::vector<tg::pos3>();

                        for (auto& seg :segments){
                            pts.push_back(seg.pos0);
                            pts.push_back(seg.pos1);
                        }

                        auto center = tg::project(tg::average(pts), plane);

                        for (auto& seg :segments){

                            auto trig1 = tg::triangle3(center, seg.pos0, seg.pos1);
                            auto trig2 = tg::triangle3(center, seg.pos1, seg.pos0);

                            auto norm = tg::cross(trig1.pos1-trig1.pos0, trig1.pos2-trig1.pos0);

                            if (tg::dot(norm, plane.normal) > .5){
                                cell1.triangels.push_back(trig1);
                                cell2.triangels.push_back(trig2);
                            } else {
                                cell1.triangels.push_back(trig2);
                                cell2.triangels.push_back(trig1);
                            }
                        }
                    }

                    // Add cells to cell complex
                    cell1.box = tg::aabb_of(cell1.triangels);
                    cell2.box = tg::aabb_of(cell2.triangels);

                    cells[i] = cell1;
                    cells.push_back(cell2);

                }

            }            

        };

        return cells;

    };
}