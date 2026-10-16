// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// W3.4 wall graph (reusex-z1v.5.4): snapping, collinear bridging, corner and
// tee joins, seen share -- on synthetic rooms run through the whole chain.

#include "wall_synth.hpp"

#include <reusex/reconstruction/walls.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::geometry::AxisFamily;
using reusex::geometry::join_walls;
using reusex::geometry::measure_walls;
using reusex::geometry::pair_faces;
using reusex::geometry::wall_faces;
using reusex::geometry::wall_records;
using reusex::geometry::WallEnd;
using reusex::geometry::WallGraph;
using reusex::geometry::WallJoinKind;
using reusex::geometry::WallOptions;
using reusex::geometry::WallPoints;
using namespace reusex::test_support::walls;

namespace {

std::vector<AxisFamily> grid() {
  AxisFamily x, y;
  x.index = 0;
  x.d = Eigen::Vector2d::UnitX();
  x.dominant = true;
  y.index = 1;
  y.d = Eigen::Vector2d::UnitY();
  y.angle_deg = 90.0;
  y.dominant = true;
  return {x, y};
}

WallGraph graph_of(const WallPoints &wp, const WallOptions &o = {}) {
  const auto fam = grid();
  const auto faces = wall_faces(wp, fam, o);
  const auto pairing = pair_faces(faces, wp, fam, o);
  return join_walls(measure_walls(faces, pairing, wp, fam, o), fam, wp, o);
}

int count(const WallGraph &g, WallJoinKind k) {
  return static_cast<int>(
      std::count_if(g.joins.begin(), g.joins.end(),
                    [&](const auto &j) { return j.kind == k; }));
}

/// A closed polygon room seen from inside: each edge a face whose normal
/// points into the room (the polygon is counter-clockwise).
void add_room(WallPoints &wp, const std::vector<Eigen::Vector2d> &poly,
              unsigned seed) {
  for (size_t i = 0; i < poly.size(); ++i) {
    const Eigen::Vector2d a = poly[i], b = poly[(i + 1) % poly.size()];
    const Eigen::Vector2d t = (b - a).normalized();
    add_face(wp, a, b, {-t.y(), t.x()}, 0.0, 2.5, 0.003,
             seed + static_cast<unsigned>(i));
  }
}

/// Where the corner joins put the wall ends: every end of a corner-joined
/// wall lies on the other wall's axis.
Eigen::Vector2d end_point(const reusex::geometry::WallSegment &w, WallEnd e) {
  const Eigen::Vector2d t(-w.d.y(), w.d.x());
  return w.axis_offset * w.d + (e == WallEnd::start ? w.s0 : w.s1) * t;
}

} // namespace

TEST_CASE("JoinWalls_RectangularRoom_FourCorners", "[walls][join_walls]") {
  auto wp = storey_points();
  add_room(wp, {{0, 0}, {4, 0}, {4, 3}, {0, 3}}, 40);
  const auto g = graph_of(wp);
  REQUIRE(g.walls.size() == 4);
  CHECK(count(g, WallJoinKind::corner) == 4);
  CHECK(count(g, WallJoinKind::tee) == 0);
  // One-sided walls: their axes are the seen (inner) faces, and the
  // corners meet on those (reusex-v8p).
  const double h = 0.0;
  for (const auto &j : g.joins) {
    const auto pa = end_point(g.walls[static_cast<size_t>(j.a)], j.at_a);
    const auto pb = end_point(g.walls[static_cast<size_t>(j.b)], j.at_b);
    CHECK((pa - pb).norm() < 1e-9);
    CHECK_THAT(std::min(std::abs(pa.x() + h), std::abs(pa.x() - 4 - h)),
               WithinAbs(0.0, 0.01));
    CHECK_THAT(std::min(std::abs(pa.y() + h), std::abs(pa.y() - 3 - h)),
               WithinAbs(0.0, 0.01));
  }
  for (const auto &w : g.walls) {
    CHECK(w.assumed);
    CHECK(w.is_external);
    CHECK(w.seen_share > 0.9); // the faces meet: nothing overhangs
    CHECK_FALSE(w.low_seen);
  }
}

TEST_CASE("JoinWalls_LShapedRoom_EveryCornerJoined", "[walls][join_walls]") {
  auto wp = storey_points();
  add_room(wp, {{0, 0}, {6, 0}, {6, 3}, {3, 3}, {3, 6}, {0, 6}}, 50);
  const auto g = graph_of(wp);
  REQUIRE(g.walls.size() == 6);
  CHECK(count(g, WallJoinKind::corner) == 6);
}

TEST_CASE("JoinWalls_PartitionedRoom_TwoTees", "[walls][join_walls]") {
  // A 6 x 3 m room split by a 0.1 m partition at x = 3: the long walls are
  // seen in two pieces either side of the partition and bridge into one;
  // the partition (measured, two faces) meets them in two tees.
  auto wp = storey_points();
  add_face(wp, {0, 0}, {0, 3}, {1, 0}, 0.0, 2.5, 0.003, 60);
  add_face(wp, {6, 0}, {6, 3}, {-1, 0}, 0.0, 2.5, 0.003, 61);
  add_face(wp, {0, 0}, {2.95, 0}, {0, 1}, 0.0, 2.5, 0.003, 62);
  add_face(wp, {3.05, 0}, {6, 0}, {0, 1}, 0.0, 2.5, 0.003, 63);
  add_face(wp, {0, 3}, {2.95, 3}, {0, -1}, 0.0, 2.5, 0.003, 64);
  add_face(wp, {3.05, 3}, {6, 3}, {0, -1}, 0.0, 2.5, 0.003, 65);
  add_face(wp, {2.95, 0}, {2.95, 3}, {-1, 0}, 0.0, 2.5, 0.003, 66);
  add_face(wp, {3.05, 0}, {3.05, 3}, {1, 0}, 0.0, 2.5, 0.003, 67);
  add_floor(wp, 0.0, 0.0, 2.95, 3.0);
  add_floor(wp, 3.05, 0.0, 6.0, 3.0);

  const auto g = graph_of(wp);
  REQUIRE(g.walls.size() == 5);
  CHECK(count(g, WallJoinKind::tee) == 2);
  CHECK(count(g, WallJoinKind::corner) == 4);
  const auto part = std::find_if(g.walls.begin(), g.walls.end(),
                                 [](const auto &w) { return !w.assumed; });
  REQUIRE(part != g.walls.end());
  CHECK_THAT(part->axis_offset, WithinAbs(3.0, 0.005));
  CHECK_THAT(part->thickness, WithinAbs(0.1, 0.01));
  CHECK_FALSE(part->is_external);
  for (const auto &j : g.joins)
    if (j.kind == WallJoinKind::tee) {
      CHECK(&g.walls[static_cast<size_t>(j.a)] == &*part);
      CHECK(j.at_b == WallEnd::path);
    }
}

TEST_CASE("JoinWalls_DoorwayInAWall_BridgesIntoOneWall",
          "[walls][join_walls]") {
  auto wp = storey_points();
  add_face(wp, {1, 0}, {1, 2}, {1, 0}, 0.0, 2.5, 0.003, 70);
  add_face(wp, {1, 3}, {1, 5}, {1, 0}, 0.0, 2.5, 0.003, 71);
  // Too far to bridge: 2 m further along.
  add_face(wp, {1, 7}, {1, 9}, {1, 0}, 0.0, 2.5, 0.003, 72);
  const auto g = graph_of(wp);
  REQUIRE(g.walls.size() == 2);
  CHECK_THAT(g.walls[0].s0, WithinAbs(0.0, 0.05));
  CHECK_THAT(g.walls[0].s1, WithinAbs(5.0, 0.05));
}

TEST_CASE("JoinWalls_HalfSeenWall_IsFlagged", "[walls][join_walls]") {
  auto wp = storey_points();
  add_room(wp, {{0, 0}, {4, 0}, {4, 3}, {0, 3}}, 80);
  // A second room's wall seen only up to 1 m (4 of 10 cell rows).
  add_face(wp, {8, 0}, {8, 3}, {1, 0}, 0.0, 1.0, 0.003, 85);
  const auto g = graph_of(wp);
  REQUIRE(g.walls.size() == 5);
  CHECK(g.low_seen == 1);
  const auto low = std::find_if(g.walls.begin(), g.walls.end(),
                                [](const auto &w) { return w.low_seen; });
  REQUIRE(low != g.walls.end());
  CHECK_THAT(low->seen_share, WithinAbs(0.4, 0.05));

  const auto [walls, joins] = wall_records(g, reusex::core::StoreyRecord{});
  REQUIRE(walls.size() == 5);
  const auto rec = std::find_if(walls.begin(), walls.end(), [](const auto &w) {
    return !w.metadata.empty();
  });
  REQUIRE(rec != walls.end());
  CHECK(rec->metadata == R"({"low_seen_share":true})");
  CHECK(joins.size() == g.joins.size());
}

TEST_CASE("JoinWalls_SlightlyDriftedWallRun_StaysOneJoinedWall",
          "[walls][join_walls]") {
  // A 4 x 3 room whose east wall drifted 2 deg (one plane): squared onto
  // the family about its centroid, it still closes the room at two corners.
  const double a = 2.0 * std::numbers::pi / 180.0;
  auto wp = storey_points();
  add_face(wp, {0, 0}, {0, 3}, {1, 0}, 0.0, 2.5, 0.003, 90);
  add_face(wp, {0, 0}, {4, 0}, {0, 1}, 0.0, 2.5, 0.003, 91);
  add_face(wp, {0, 3}, {4, 3}, {0, -1}, 0.0, 2.5, 0.003, 92);
  wp.plane.assign(wp.points.size(), -1);
  const size_t before = wp.points.size();
  const Eigen::Vector2d p0(4.0, 0.0);
  const Eigen::Vector2d p1 =
      p0 + 3.0 * Eigen::Vector2d(std::sin(a), std::cos(a));
  add_face(wp, p0, p1, {-std::cos(a), std::sin(a)}, 0.0, 2.5, 0.003, 93);
  wp.plane.resize(wp.points.size(), 4);
  (void)before;

  const auto g = graph_of(wp);
  REQUIRE(g.walls.size() == 4);
  CHECK(count(g, WallJoinKind::corner) == 4);
}

TEST_CASE("WallRecords_Graph_StableKeysAndJoinGuids", "[walls][join_walls]") {
  auto wp = storey_points();
  add_room(wp, {{0, 0}, {4, 0}, {4, 3}, {0, 3}}, 40);
  const auto g = graph_of(wp);
  reusex::core::StoreyRecord st;
  st.idx = 2;
  st.z_floor = 0.0;
  st.z_ceiling = 2.5;
  const auto [walls, joins] = wall_records(g, st);
  REQUIRE(walls.size() == 4);
  for (const auto &w : walls) {
    CHECK(w.guid.rfind("walls/2/", 0) == 0);
    CHECK(w.storey == 2);
    CHECK_THAT(w.height, WithinAbs(2.5, 1e-12));
    CHECK(w.assumed);
    CHECK(w.thickness_ci_lo == -1.0);
    // One-sided: the axis is the inner face, the thickness unknown
    // (reusex-v8p), as the reference plans draw external walls.
    CHECK(w.thickness == -1.0);
    const bool on_x = std::abs(w.x0 - w.x1) < 1e-9;
    if (on_x)
      CHECK_THAT(std::min(std::abs(w.x0), std::abs(w.x0 - 4.0)),
                 WithinAbs(0.0, 0.01));
    else
      CHECK_THAT(std::min(std::abs(w.y0), std::abs(w.y0 - 3.0)),
                 WithinAbs(0.0, 0.01));
  }
  CHECK(walls[0].guid != walls[1].guid);
  REQUIRE(joins.size() == 4);
  for (const auto &j : joins) {
    CHECK(j.kind == "corner");
    CHECK((j.at_a == "start" || j.at_a == "end"));
  }
  // Same input, same keys.
  const auto again = wall_records(graph_of(wp), st);
  for (size_t i = 0; i < walls.size(); ++i)
    CHECK(again.first[i].guid == walls[i].guid);
}

TEST_CASE("JoinWalls_NoWalls_ReturnsEmptyGraph", "[walls][join_walls]") {
  auto wp = storey_points();
  const auto g = join_walls({}, grid(), wp);
  CHECK(g.walls.empty());
  CHECK(g.joins.empty());
}

TEST_CASE("JoinWalls_FacadeWithWindowNiches_IsOneWallOnThePierFace",
          "[walls][join_walls]") {
  // An outer wall seen from inside (+x): 1.2 m piers at x = 1.0 and 0.8 m
  // window niches whose backs sit 0.35 m deeper, alternating.
  auto wp = storey_points();
  unsigned seed = 100;
  double y = 0.0;
  for (int k = 0; k < 4; ++k) {
    add_face(wp, {1.0, y}, {1.0, y + 1.2}, {1, 0}, 0.0, 2.5, 0.003, seed++);
    y += 1.2;
    if (k < 3) {
      add_face(wp, {0.65, y}, {0.65, y + 0.8}, {1, 0}, 0.9, 2.2, 0.003, seed++);
      y += 0.8;
    }
  }
  const auto g = graph_of(wp);
  REQUIRE(g.walls.size() == 1);
  CHECK_THAT(g.walls[0].face_hi, WithinAbs(1.0, 0.005));
  CHECK_THAT(g.walls[0].s0, WithinAbs(0.0, 0.05));
  CHECK_THAT(g.walls[0].s1, WithinAbs(y, 0.05));
}

TEST_CASE("JoinWalls_OneSidedWallSeenFromBothRooms_StaysOneWall",
          "[walls][join_walls]") {
  // A 0.2 m wall at x = 1.0 .. 1.2 whose west face is seen over y 0..2 and
  // whose east face over y 2.6..4.6 (two rooms, the doorway between them
  // hidden): two one-sided stretches whose seen faces differ by the
  // thickness. With axes on the faces they are still collinear (their bodies
  // coincide) and bridge into one wall.
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 2}, {-1, 0}, 0.0, 2.5, 0.003, 120);
  add_face(wp, {1.2, 2.6}, {1.2, 4.6}, {1, 0}, 0.0, 2.5, 0.003, 121);
  const auto g = graph_of(wp);
  REQUIRE(g.walls.size() == 1);
  CHECK_THAT(g.walls[0].s0, WithinAbs(0.0, 0.05));
  CHECK_THAT(g.walls[0].s1, WithinAbs(4.6, 0.05));
}

TEST_CASE("JoinWalls_OneSidedWall_SnapsToItsFaceNotIntoTheRoom",
          "[walls][join_walls]") {
  // Furniture faces 6 cm in front of a one-sided wall must not pull the
  // snapped face (and with it the axis) into the room.
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {1, 0}, 0.0, 2.5, 0.003, 130);
  add_face(wp, {1.06, 1}, {1.06, 3}, {1, 0}, 0.0, 0.9, 0.003, 131);
  const auto g = graph_of(wp);
  REQUIRE_FALSE(g.walls.empty());
  const auto w = std::max_element(
      g.walls.begin(), g.walls.end(),
      [](const auto &a, const auto &b) { return a.s1 - a.s0 < b.s1 - b.s0; });
  CHECK_THAT(w->axis_offset, WithinAbs(1.0, 0.01));
  CHECK_THAT(w->face_hi, WithinAbs(1.0, 0.01));
}
