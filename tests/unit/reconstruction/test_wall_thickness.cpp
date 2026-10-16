// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// W3.3 thickness, CI, lean and the external test (reusex-z1v.5.3).

#include "wall_synth.hpp"

#include <reusex/reconstruction/walls.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::geometry::AxisFamily;
using reusex::geometry::measure_thickness;
using reusex::geometry::measure_walls;
using reusex::geometry::pair_faces;
using reusex::geometry::wall_faces;
using reusex::geometry::WallFace;
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

constexpr double kDeg = std::numbers::pi / 180.0;

} // namespace

TEST_CASE("MeasureThickness_ThinWallWithNoise_WithinOneCmAndCiCoversTruth",
          "[walls][thickness]") {
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {-1, 0}, 0.0, 2.5, 0.005, 11);
  add_face(wp, {1.12, 0}, {1.12, 4}, {1, 0}, 0.0, 2.5, 0.005, 12);
  const auto faces = wall_faces(wp, grid());
  REQUIRE(faces.size() == 2);
  const auto fit = measure_thickness(faces[0], faces[1], 0.0, 4.0);
  REQUIRE(fit.ok);
  CHECK_THAT(fit.thickness, WithinAbs(0.12, 0.01));
  CHECK(fit.ci_lo <= 0.12);
  CHECK(fit.ci_hi >= 0.12);
  CHECK(fit.ci_lo < fit.ci_hi);
  CHECK_THAT(fit.lean_deg, WithinAbs(0.0, 0.3));
  CHECK_THAT(fit.offset_lo, WithinAbs(1.0, 0.005));

  // Same seed, same interval.
  const auto again = measure_thickness(faces[0], faces[1], 0.0, 4.0);
  CHECK(again.ci_lo == fit.ci_lo);
  CHECK(again.ci_hi == fit.ci_hi);
}

TEST_CASE("MeasureThickness_LeaningWall_ReportsLeanAndMidHeightThickness",
          "[walls][thickness]") {
  // Both faces lean 2 deg toward +x: thickness unchanged, lean 2 deg.
  const double k = std::tan(2.0 * kDeg);
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {-1, 0}, 0.0, 2.5, 0.003, 13, 0.05, -k);
  add_face(wp, {1.12, 0}, {1.12, 4}, {1, 0}, 0.0, 2.5, 0.003, 14, 0.05, k);
  WallOptions o;
  o.face_window_m = 0.06; // the lean spreads each face over 8.7 cm
  const auto faces = wall_faces(wp, grid(), o);
  REQUIRE(faces.size() == 2);
  const auto fit = measure_thickness(faces[0], faces[1], 0.0, 4.0, o);
  REQUIRE(fit.ok);
  CHECK_THAT(fit.lean_deg, WithinAbs(2.0, 0.3));
  CHECK_THAT(fit.thickness, WithinAbs(0.12, 0.01));
}

TEST_CASE("MeasureThickness_FurnitureCellsOnOneFace_AreDroppedAsOutliers",
          "[walls][thickness]") {
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {-1, 0}, 0.0, 2.5, 0.003, 15);
  // A row of cabinets 2.5 cm proud of the +x face, 3 m wide, 0.75 m high,
  // hiding the wall behind it; seen from +x and inside that face's window,
  // so 36 of its 160 cells sit 2.5 cm off.
  add_face(wp, {1.12, 0}, {1.12, 4}, {1, 0}, 0.75, 2.5, 0.003, 16);
  add_face(wp, {1.12, 3}, {1.12, 4}, {1, 0}, 0.0, 0.75, 0.003, 21);
  add_face(wp, {1.145, 0.0}, {1.145, 3.0}, {1, 0}, 0.0, 0.75, 0.003, 17);
  const auto faces = wall_faces(wp, grid());
  REQUIRE(faces.size() == 2);
  const auto &lo = faces.front();
  const auto &hi = faces.back();
  const auto fit = measure_thickness(lo, hi, 0.0, 4.0);
  REQUIRE(fit.ok);
  CHECK_THAT(fit.thickness, WithinAbs(0.12, 0.01));
}

TEST_CASE("MeasureThickness_TooFewCells_IsNotOk", "[walls][thickness]") {
  WallFace a, b;
  CHECK_FALSE(measure_thickness(a, b, 0.0, 1.0).ok);
}

TEST_CASE("MeasureWalls_OneSidedFace_AssumedWithDefaultThickness",
          "[walls][thickness]") {
  // An outer wall seen only from inside (from +x), room floor at x > 1.
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {1, 0}, 0.0, 2.5, 0.003, 18);
  add_floor(wp, 1.0, 0.0, 4.0, 4.0);
  const auto fam = grid();
  const auto faces = wall_faces(wp, fam);
  const auto pairing = pair_faces(faces, wp, fam);
  const auto walls = measure_walls(faces, pairing, wp, fam);
  REQUIRE(walls.size() == 1);
  const auto &w = walls.front();
  WallOptions o;
  CHECK(w.assumed);
  CHECK_THAT(w.thickness, WithinAbs(o.default_thickness_m, 1e-12));
  CHECK_THAT(w.face_hi, WithinAbs(1.0, 0.003));
  CHECK_FALSE(std::isfinite(w.face_lo));
  // The body lies behind the seen face, away from the room.
  CHECK_THAT(w.axis_offset,
             WithinAbs(1.0 - 0.5 * o.default_thickness_m, 0.003));
  CHECK(w.is_external);
}

TEST_CASE("MeasureWalls_RoomOnBothSides_IsInternal", "[walls][thickness]") {
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 4}, {-1, 0}, 0.0, 2.5, 0.003, 19);
  add_face(wp, {1.12, 0}, {1.12, 4}, {1, 0}, 0.0, 2.5, 0.003, 20);
  add_floor(wp, -2.0, 0.0, 1.0, 4.0);
  const auto fam = grid();
  auto faces = wall_faces(wp, fam);
  auto walls = measure_walls(faces, pair_faces(faces, wp, fam), wp, fam);
  REQUIRE(walls.size() == 1);
  CHECK_FALSE(walls[0].assumed);
  CHECK_THAT(walls[0].thickness, WithinAbs(0.12, 0.01));
  CHECK(walls[0].is_external); // floor on one side only

  add_floor(wp, 1.12, 0.0, 3.0, 4.0);
  faces = wall_faces(wp, fam);
  walls = measure_walls(faces, pair_faces(faces, wp, fam), wp, fam);
  REQUIRE(walls.size() == 1);
  CHECK_FALSE(walls[0].is_external);
  CHECK_THAT(walls[0].axis_offset, WithinAbs(1.06, 0.005));
  CHECK(walls[0].points_lo > 0);
  CHECK(walls[0].points_hi > 0);
  CHECK(walls[0].co_visibility > 0.9);
}
