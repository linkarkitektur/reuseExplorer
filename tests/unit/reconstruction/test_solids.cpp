// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Typed solids (W6.1, reusex-z1v.8.1): a wall with a door and a window cut
// out of it, a slab, a space volume -- each closed, with the volume known in
// closed form.

#include <reusex/core/solid_model.hpp>
#include <reusex/reconstruction/solids.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Surface_mesh.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <string>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using geometry::build_solids;
using geometry::ElementSolid;
using geometry::SolidsOptions;
using geometry::SolidType;

namespace {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Mesh = CGAL::Surface_mesh<K::Point_3>;
namespace PMP = CGAL::Polygon_mesh_processing;

Mesh to_cgal(const pcl::PolygonMesh &m) {
  pcl::PointCloud<pcl::PointXYZ> pts;
  pcl::fromPCLPointCloud2(m.cloud, pts);
  Mesh out;
  std::vector<Mesh::Vertex_index> v;
  for (const auto &p : pts)
    v.push_back(out.add_vertex(K::Point_3(p.x, p.y, p.z)));
  for (const auto &f : m.polygons) {
    REQUIRE(f.vertices.size() == 3);
    REQUIRE(out.add_face(v[f.vertices[0]], v[f.vertices[1]],
                         v[f.vertices[2]]) != Mesh::null_face());
  }
  return out;
}

/// Inside / outside the closed mesh of @p s.
CGAL::Bounded_side side(const ElementSolid &s, double x, double y, double z) {
  const Mesh m = to_cgal(s.mesh);
  CGAL::Side_of_triangle_mesh<Mesh, K> inside(m);
  return inside(K::Point_3(x, y, z));
}

core::StoreyRecord storey(int idx, double z_floor, double z_ceiling) {
  core::StoreyRecord s;
  s.idx = idx;
  s.guid = "st" + std::to_string(idx);
  s.z_floor = z_floor;
  s.z_ceiling = z_ceiling;
  return s;
}

core::WallRecord wall(const std::string &guid, double x0, double y0, double x1,
                      double y1, double thickness, double height = 2.5) {
  core::WallRecord w;
  w.guid = guid;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.thickness = thickness;
  w.height = height;
  return w;
}

core::OpeningRecord opening(const std::string &guid, const std::string &host,
                            const std::string &kind, double s0, double s1,
                            double z0, double z1) {
  core::OpeningRecord o;
  o.guid = guid;
  o.wall = host;
  o.kind = kind;
  o.s_start = s0;
  o.s_end = s1;
  o.z_sill = z0;
  o.z_head = z1;
  return o;
}

core::SpaceRecord space(const std::string &guid,
                        std::vector<core::PlanPoint> outer, double z0,
                        double z1) {
  core::SpaceRecord s;
  s.guid = guid;
  s.polygon.outer = std::move(outer);
  s.z_floor = z0;
  s.z_ceiling = z1;
  s.kind = "room";
  return s;
}

} // namespace

TEST_CASE("Solids_Wall_IsAClosedBox", "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("w1", 0, 0, 4, 0, 0.2)};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 1);
  const auto &w = r.walls[0];
  CHECK(w.type == SolidType::wall);
  CHECK(w.source_guid == "w1");
  CHECK(w.predefined == "STANDARD");
  CHECK(w.closed);
  CHECK_THAT(w.volume, WithinAbs(4.0 * 0.2 * 2.5, 1e-6));
  CHECK(side(w, 2.0, 0.0, 1.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(w, 2.0, 0.2, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
  // The IFC-ready profile: a length x thickness rectangle, extruded by the
  // height, placed at the axis start on the floor.
  CHECK(w.profile.outer.size() == 4);
  CHECK_THAT(w.profile.depth, WithinAbs(2.5, 1e-9));
  CHECK(w.voids.empty());
}

TEST_CASE("Solids_WallWithDoorAndWindow_LeavesHoles",
          "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("w1", 0, 0, 4, 0, 0.2)};
  m.openings = {opening("d1", "w1", "door", 0.5, 1.4, 0.0, 2.1),
                opening("o1", "w1", "window", 2.5, 3.5, 0.9, 2.0)};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 1);
  const auto &w = r.walls[0];
  CHECK(r.openings_cut == 2);
  CHECK(r.openings_failed == 0);
  CHECK(w.closed);
  CHECK(w.voids_failed == 0);
  const double box = 4.0 * 0.2 * 2.5;
  const double door = 0.9 * 0.2 * 2.1;
  const double window = 1.0 * 0.2 * 1.1;
  CHECK_THAT(w.volume, WithinAbs(box - door - window, 1e-6));
  // Through the door and the window: no wall. Below the window's sill and
  // above the door's head: wall.
  CHECK(side(w, 1.0, 0.0, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(w, 1.0, 0.0, 0.01) == CGAL::ON_UNBOUNDED_SIDE); // door to floor
  CHECK(side(w, 3.0, 0.0, 1.5) == CGAL::ON_UNBOUNDED_SIDE);
  CHECK(side(w, 3.0, 0.0, 0.5) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(w, 1.0, 0.0, 2.3) == CGAL::ON_BOUNDED_SIDE);
  // The voids are kept for IFC, door first.
  REQUIRE(w.voids.size() == 2);
  CHECK(w.voids[0].guid == "d1");
  CHECK(w.voids[0].kind == "door");
  CHECK(w.voids[1].guid == "o1");
  CHECK(w.voids[0].profile.depth > 0.2);
}

TEST_CASE("Solids_Opening_OrphanOrOutside_IsSkipped",
          "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("w1", 0, 0, 4, 0, 0.2)};
  m.openings = {opening("x", "nope", "door", 0.5, 1.4, 0.0, 2.1),
                opening("y", "w1", "window", 5.0, 6.0, 0.9, 2.0)};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 1);
  CHECK(r.openings_skipped == 2);
  CHECK(r.walls[0].voids.empty());
  CHECK_THAT(r.walls[0].volume, WithinAbs(2.0, 1e-6));
}

TEST_CASE("Solids_UnknownThickness_GrowsAwayFromTheSpace",
          "[reconstruction][solids]") {
  // The wall's axis is its seen face at y = 0; the room is at y > 0, so the
  // body is at y < 0.
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("w1", 0, 0, 4, 0, -1.0)};
  m.spaces = {space("s1", {{0, 0}, {4, 0}, {4, 3}, {0, 3}}, 0.0, 2.5)};
  SolidsOptions o;
  o.unknown_wall_thickness_m = 0.1;
  const auto r = build_solids(m, o);
  REQUIRE(r.walls.size() == 1);
  CHECK(r.walls[0].closed);
  CHECK(side(r.walls[0], 2.0, -0.05, 1.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(r.walls[0], 2.0, 0.05, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
}

TEST_CASE("Solids_CornerJoin_ExtendsTheWallEnds", "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("a", 0, 0, 4, 0, 0.2), wall("b", 4, 0, 4, 3, 0.2)};
  m.wall_joins = {{"a", "b", "corner", "end", "start"}};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 2);
  // Both reach the outer corner (4.1, -0.1).
  CHECK(side(r.walls[0], 4.05, -0.05, 1.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(r.walls[1], 4.05, -0.05, 1.0) == CGAL::ON_BOUNDED_SIDE);
  SolidsOptions o;
  o.extend_at_joins = false;
  const auto r2 = build_solids(m, o);
  CHECK(side(r2.walls[0], 4.05, -0.05, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
}

TEST_CASE("Solids_Space_IsItsPolygonTimesItsHeight",
          "[reconstruction][solids]") {
  // An L-shaped room with a column hole.
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  auto s =
      space("s1", {{0, 0}, {6, 0}, {6, 3}, {3, 3}, {3, 5}, {0, 5}}, 0.0, 2.5);
  s.polygon.holes = {{{1, 1}, {1.5, 1}, {1.5, 1.5}, {1, 1.5}}};
  m.spaces = {s};
  const auto r = build_solids(m);
  REQUIRE(r.spaces.size() == 1);
  const auto &e = r.spaces[0];
  CHECK(e.type == SolidType::space);
  CHECK(e.source_guid == "s1");
  CHECK(e.predefined == "room");
  CHECK(e.closed);
  const double area = 6 * 3 + 3 * 2 - 0.25;
  CHECK_THAT(e.volume, WithinAbs(area * 2.5, 1e-6));
  CHECK(side(e, 4.0, 4.0, 1.0) == CGAL::ON_UNBOUNDED_SIDE);   // the L's notch
  CHECK(side(e, 1.25, 1.25, 1.0) == CGAL::ON_UNBOUNDED_SIDE); // the column
  CHECK(side(e, 2.0, 2.0, 1.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(e.profile.holes.size() == 1);
  CHECK_THAT(e.profile.depth, WithinAbs(2.5, 1e-9));
}

TEST_CASE("Solids_Slab_FloorPolygonTimesTheGapBelow",
          "[reconstruction][solids]") {
  // Two storeys: the upper floor's slab fills the gap from the lower
  // ceiling (2.6) to its floor (3.0); the ground floor and the roof get the
  // default thickness.
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.6), storey(1, 3.0, 5.6)};
  auto s0 = space("s0", {{0, 0}, {4, 0}, {4, 3}, {0, 3}}, 0.0, 2.6);
  auto s1 = space("s1", {{0, 0}, {4, 0}, {4, 3}, {0, 3}}, 3.0, 5.6);
  s1.storey = 1;
  m.spaces = {s0, s1};
  SolidsOptions o;
  o.slab_thickness_m = 0.25;
  const auto r = build_solids(m, o);
  REQUIRE(r.slabs.size() == 3);
  const auto &ground = r.slabs[0];
  const auto &upper = r.slabs[1];
  const auto &roof = r.slabs[2];
  CHECK(ground.source_guid == "st0");
  CHECK(ground.predefined == "FLOOR");
  CHECK(upper.source_guid == "st1");
  CHECK(roof.predefined == "ROOF");
  for (const auto *s : {&ground, &upper, &roof})
    CHECK(s->closed);
  CHECK_THAT(ground.volume, WithinAbs(12.0 * 0.25, 1e-6));
  CHECK_THAT(upper.volume, WithinAbs(12.0 * 0.4, 1e-6));
  CHECK_THAT(roof.volume, WithinAbs(12.0 * 0.25, 1e-6));
  CHECK(side(ground, 2, 1.5, -0.1) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(upper, 2, 1.5, 2.8) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(roof, 2, 1.5, 5.7) == CGAL::ON_BOUNDED_SIDE);
}

TEST_CASE("Solids_Slab_UnionOfSpacesAndWalls", "[reconstruction][solids]") {
  // Two rooms either side of a 0.2 m partition: the slab is one piece under
  // both rooms and the wall, 4.2 x 3.
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.spaces = {space("a", {{0, 0}, {2, 0}, {2, 3}, {0, 3}}, 0.0, 2.5),
              space("b", {{2.2, 0}, {4.2, 0}, {4.2, 3}, {2.2, 3}}, 0.0, 2.5)};
  m.walls = {wall("p", 2.1, 0, 2.1, 3, 0.2)};
  SolidsOptions o;
  o.roof = false;
  const auto r = build_solids(m, o);
  REQUIRE(r.slabs.size() == 1);
  CHECK_THAT(r.slabs[0].volume, WithinAbs(4.2 * 3 * 0.2, 1e-6));
}

TEST_CASE("Solids_Slab_StoreyHullWithoutSpaces", "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("a", 0, 0, 4, 0, 0.2), wall("b", 4, 0, 4, 3, 0.2),
             wall("c", 4, 3, 0, 3, 0.2), wall("d", 0, 3, 0, 0, 0.2)};
  SolidsOptions o;
  o.roof = false;
  o.extend_at_joins = false;
  const auto r = build_solids(m, o);
  REQUIRE(r.slabs.size() == 1);
  CHECK(r.slabs[0].closed);
  // The hull of the four footprints: 4.2 x 3.2 less the four 0.1 x 0.1
  // corner triangles the unextended walls leave.
  CHECK_THAT(r.slabs[0].volume, WithinAbs((4.2 * 3.2 - 0.02) * 0.2, 1e-6));
}

TEST_CASE("Solids_LevelFrame_IsMappedToWorld", "[reconstruction][solids]") {
  // world -> level: z - 10, so level z 0 is world z 10.
  core::SolidModel m;
  auto st = storey(0, 0.0, 2.5);
  st.level_transform[11] = -10.0;
  m.storeys = {st};
  m.walls = {wall("w1", 0, 0, 4, 0, 0.2)};
  const auto r = build_solids(m);
  REQUIRE(r.walls.size() == 1);
  CHECK(side(r.walls[0], 2.0, 0.0, 11.0) == CGAL::ON_BOUNDED_SIDE);
  CHECK(side(r.walls[0], 2.0, 0.0, 1.0) == CGAL::ON_UNBOUNDED_SIDE);
  // The placement carries the frame too.
  CHECK_THAT(r.walls[0].profile.placement[11], WithinAbs(10.0, 1e-9));
}

TEST_CASE("Solids_DegenerateElements_AreSkipped", "[reconstruction][solids]") {
  core::SolidModel m;
  m.storeys = {storey(0, 0.0, 2.5)};
  m.walls = {wall("z", 1, 1, 1, 1, 0.2), wall("h", 0, 0, 1, 0, 0.2, 0.0)};
  m.spaces = {space("flat", {{0, 0}, {1, 0}, {1, 1}}, 1.0, 1.0)};
  SolidsOptions o;
  o.roof = false;
  const auto r = build_solids(m, o);
  CHECK(r.walls.empty());
  CHECK(r.walls_skipped == 2);
  CHECK(r.spaces.empty());
  CHECK(r.spaces_skipped == 1);
}
