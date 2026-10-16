// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// W3.3 face pairing (reusex-z1v.5.3): which faces are the two sides of one
// wall -- gap, shared length, empty valley, co-visibility.

#include "wall_synth.hpp"

#include <reusex/reconstruction/walls.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::geometry::AxisFamily;
using reusex::geometry::pair_faces;
using reusex::geometry::wall_faces;
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

/// A wall between x = x0 and x = x0 + t along y in [0, len]: its -x face
/// seen from -x, its +x face from +x.
void add_wall(WallPoints &wp, double x0, double t, double len,
              unsigned seed = 1, double noise = 0.005) {
  add_face(wp, {x0, 0}, {x0, len}, {-1, 0}, 0.0, 2.5, noise, seed);
  add_face(wp, {x0 + t, 0}, {x0 + t, len}, {1, 0}, 0.0, 2.5, noise, seed + 1);
}

} // namespace

TEST_CASE("PairFaces_TwoFaceWall_PairsThem", "[walls][pair_faces]") {
  auto wp = storey_points();
  add_wall(wp, 1.0, 0.12, 4.0);
  const auto fam = grid();
  const auto faces = wall_faces(wp, fam);
  REQUIRE(faces.size() == 2);
  const auto r = pair_faces(faces, wp, fam);
  REQUIRE(r.pairs.size() == 1);
  CHECK(r.singles.empty());
  const auto &p = r.pairs.front();
  CHECK(faces[static_cast<size_t>(p.face_lo)].offset <
        faces[static_cast<size_t>(p.face_hi)].offset);
  CHECK_THAT(p.gap, WithinAbs(0.12, 0.005));
  CHECK_THAT(p.s1 - p.s0, WithinAbs(3.95, 0.05));
  CHECK(p.valley_share <= 0.15);
  CHECK(p.co_visibility >= 0.95);
}

TEST_CASE("PairFaces_GapOutOfRange_LeavesTwoSingles", "[walls][pair_faces]") {
  const auto fam = grid();
  for (const double t : {0.6, 0.03}) {
    auto wp = storey_points();
    add_wall(wp, 1.0, t, 4.0, 1, 0.002);
    const auto faces = wall_faces(wp, fam);
    REQUIRE(faces.size() == 2);
    const auto r = pair_faces(faces, wp, fam);
    CHECK(r.pairs.empty());
    CHECK(r.singles.size() == 2);
  }
}

TEST_CASE("PairFaces_FacesLookingAtEachOther_AreARoomNotAWall",
          "[walls][pair_faces]") {
  // A 0.3 m niche: the lower face is seen from +x, the upper from -x.
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {1, 0}, 0.0, 2.5);
  add_face(wp, {1.3, 0}, {1.3, 4}, {-1, 0}, 0.0, 2.5);
  const auto fam = grid();
  CHECK(pair_faces(wall_faces(wp, fam), wp, fam).pairs.empty());
}

TEST_CASE("PairFaces_FilledValley_IsRejected", "[walls][pair_faces]") {
  // Two opposite faces 8 cm apart with points all the way between them (a
  // solid block / a lining with something behind it): no empty valley.
  auto wp = storey_points();
  add_wall(wp, 1.0, 0.08, 4.0, 1, 0.002);
  for (double x = 1.01; x < 1.075; x += 0.01)
    for (double y = 0.05; y < 4.0; y += 0.1)
      for (double z = 0.05; z < 2.5; z += 0.1)
        wp.others.emplace_back(x, y, z);
  const auto fam = grid();
  const auto faces = wall_faces(wp, fam);
  REQUIRE(faces.size() == 2);
  const auto r = pair_faces(faces, wp, fam);
  CHECK(r.pairs.empty());
  CHECK(r.rejected == 1);
}

TEST_CASE("PairFaces_LiningInFrontOfAWall_IsNotPairedWithIt",
          "[walls][pair_faces]") {
  // A lining sheet 3 cm in front of a wall face, both seen from the room.
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {-1, 0}, 0.0, 2.5, 0.002, 3);
  add_face(wp, {0.97, 0}, {0.97, 4}, {-1, 0}, 1.0, 2.0, 0.002, 4);
  const auto fam = grid();
  CHECK(pair_faces(wall_faces(wp, fam), wp, fam).pairs.empty());
}

TEST_CASE("PairFaces_OccludedFace_FailsCoVisibility", "[walls][pair_faces]") {
  // The +x face is only seen in the lowest 0.5 m: 2 of 10 cell rows.
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {-1, 0}, 0.0, 2.5, 0.003, 5);
  add_face(wp, {1.12, 0}, {1.12, 4}, {1, 0}, 0.0, 0.5, 0.003, 6);
  const auto fam = grid();
  WallOptions o;
  o.min_column_share = 0.1; // keep the low face a face
  const auto faces = wall_faces(wp, fam, o);
  REQUIRE(faces.size() == 2);
  const auto r = pair_faces(faces, wp, fam, o);
  CHECK(r.pairs.empty());
  CHECK(r.rejected == 1);
  CHECK(r.singles.size() == 2);
}

TEST_CASE("PairFaces_LongFaceAgainstTwoShortOnes_PairsBothStretches",
          "[walls][pair_faces]") {
  // A corridor wall seen whole from one side and from two rooms on the
  // other, with a 1 m stretch between them that only one side saw.
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 6}, {-1, 0}, 0.0, 2.5, 0.003, 7);
  add_face(wp, {1.1, 0}, {1.1, 2}, {1, 0}, 0.0, 2.5, 0.003, 8);
  add_face(wp, {1.1, 3}, {1.1, 6}, {1, 0}, 0.0, 2.5, 0.003, 9);
  const auto fam = grid();
  const auto faces = wall_faces(wp, fam);
  REQUIRE(faces.size() == 3);
  const auto r = pair_faces(faces, wp, fam);
  REQUIRE(r.pairs.size() == 2);
  CHECK_THAT(r.pairs[0].s1 - r.pairs[0].s0, WithinAbs(3.0, 0.1)); // longest
  CHECK_THAT(r.pairs[1].s1 - r.pairs[1].s0, WithinAbs(2.0, 0.1));
  REQUIRE(r.singles.size() == 1);
  CHECK_THAT(r.singles[0].s0, WithinAbs(2.0, 0.1));
  CHECK_THAT(r.singles[0].s1, WithinAbs(3.0, 0.1));
}

TEST_CASE("PairFaces_NoFaces_ReturnsEmpty", "[walls][pair_faces]") {
  auto wp = storey_points();
  const auto r = pair_faces({}, wp, grid());
  CHECK(r.pairs.empty());
  CHECK(r.singles.empty());
}

TEST_CASE("PairFaces_CrossedFacesOfADriftedWall_PairAsOneAssumedWall",
          "[walls][pair_faces]") {
  // A thin wall whose two rooms were registered 6 cm apart: the face seen
  // from +x lies 6 cm *below* the face seen from -x. No sensor sees into a
  // 6 cm gap, so it is one wall, not a room.
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {1, 0}, 0.0, 2.5, 0.003, 30);
  add_face(wp, {1.06, 0}, {1.06, 4}, {-1, 0}, 0.0, 2.5, 0.003, 31);
  add_floor(wp, -2.0, 0.0, 0.9, 4.0);
  add_floor(wp, 1.2, 0.0, 3.0, 4.0);
  const auto fam = grid();
  const auto faces = wall_faces(wp, fam);
  REQUIRE(faces.size() == 2);
  const auto r = pair_faces(faces, wp, fam);
  REQUIRE(r.pairs.size() == 1);
  CHECK(r.pairs[0].crossed);
  CHECK(r.singles.empty());
  const auto walls = reusex::geometry::measure_walls(faces, r, wp, fam);
  REQUIRE(walls.size() == 1);
  CHECK(walls[0].assumed);
  CHECK_THAT(walls[0].axis_offset, WithinAbs(1.03, 0.005));
  CHECK_FALSE(walls[0].is_external);

  // 0.3 m apart, the same faces bound a narrow room: no pair.
  auto wide = storey_points();
  add_face(wide, {1.0, 0}, {1.0, 4}, {1, 0}, 0.0, 2.5, 0.003, 30);
  add_face(wide, {1.3, 0}, {1.3, 4}, {-1, 0}, 0.0, 2.5, 0.003, 31);
  CHECK(pair_faces(wall_faces(wide, fam), wide, fam).pairs.empty());
}
