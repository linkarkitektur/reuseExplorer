// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// W5.1 (reusex-z1v.7.1): the (s, z) face raster with the occluder check and
// the rectangle fit, on synthetic wall faces.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/reconstruction/openings.hpp>

#include "opening_synth.hpp"

#include <algorithm>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using namespace reusex::geometry;
namespace os = reusex::test_support::openings;

namespace {

constexpr double kLen = 5.0, kH = 2.5;

std::vector<OpeningRect> detect(const std::vector<Eigen::Vector3d> &pts,
                                const OpeningOptions &opt = {}) {
  const auto w = os::wall(kLen, kH);
  const auto raster = face_raster(pts, w, 0.0, kH, opt);
  return fit_openings(raster, 0.0, kH, opt);
}

} // namespace

TEST_CASE("FaceRaster_CellStates", "[reconstruction][openings]") {
  // One face with a hole in it; something stands in front of the left half
  // of the hole.
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.05, kLen, kH, {{1.0, 2.0, 0.5, 1.5}});
  os::add_box(pts, 1.0, 1.5, 1.6, 0.05, 0.6);
  const auto r = face_raster(pts, os::wall(kLen, kH), 0.0, kH);
  CHECK(r.ns == 50);
  CHECK(r.nz == 24); // 0.05 .. 2.40
  CHECK_THAT(r.z0, WithinAbs(0.05, 1e-9));
  // s = 0.5, z = 1.0: wall.
  CHECK(r.at(5, 9) == FaceCellState::solid);
  // s = 1.75, z = 1.0: hole, nothing in front: open.
  CHECK(r.at(17, 9) == FaceCellState::open);
  // s = 1.25, z = 1.0: hole, but the box hides it: occluded.
  CHECK(r.at(12, 9) == FaceCellState::occluded);
}

TEST_CASE("FaceRaster_RejectsDegenerateInput", "[reconstruction][openings]") {
  const std::vector<Eigen::Vector3d> none;
  auto w = os::wall();
  w.x1 = w.x0;
  CHECK_THROWS_AS(face_raster(none, w, 0.0, kH), std::invalid_argument);
  CHECK_THROWS_AS(face_raster(none, os::wall(), 0.0, 0.1),
                  std::invalid_argument);
  OpeningOptions o;
  o.cell_m = 0.0;
  CHECK_THROWS_AS(face_raster(none, os::wall(), 0.0, kH, o),
                  std::invalid_argument);
}

// The bead's acceptance: a door gap and a cabinet in front of a solid part
// whose wall the sensor could not see -> one opening, not two.
TEST_CASE("Openings_DoorGap_AndCabinet_GiveOneOpening",
          "[reconstruction][openings]") {
  std::vector<Eigen::Vector3d> pts;
  // One-sided wall (seen from +y): the door gap, and the stretch hidden
  // behind the cabinet, have no wall points.
  const os::Hole door{1.0, 1.9, 0.0, 2.1};
  const os::Hole behind_cabinet{2.6, 3.8, 0.0, 1.8};
  os::add_face(pts, 0.05, kLen, kH, {door, behind_cabinet});
  os::add_box(pts, 2.6, 3.8, 1.8, 0.05, 0.65);
  os::add_floor_ceiling(pts, kLen, -3.0, 3.0, kH);

  const auto rects = detect(pts);
  REQUIRE(rects.size() == 1);
  const auto &r = rects[0];
  CHECK_THAT(r.s_start, WithinAbs(1.0, 0.11));
  CHECK_THAT(r.s_end, WithinAbs(1.9, 0.11));
  CHECK(r.at_floor);
  CHECK_FALSE(r.at_ceiling);
  CHECK_THAT(r.z_sill, WithinAbs(0.0, 1e-9));
  CHECK_THAT(r.z_head, WithinAbs(2.1, 0.11));
  CHECK(r.fill > 0.9);
  CHECK(r.jamb_start > 0.9);
  CHECK(r.jamb_end > 0.9);
}

TEST_CASE("Openings_OccluderOnTheFarSide_AlsoHides",
          "[reconstruction][openings]") {
  // A shelf on the other side (-y) of a hole in a one-sided wall.
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.05, kLen, kH, {{2.0, 3.0, 0.0, 1.8}});
  os::add_box(pts, 2.0, 3.0, 1.8, -0.05, -0.5);
  CHECK(detect(pts).empty());
}

TEST_CASE("Openings_ThingsFarInFront_DoNotHide", "[reconstruction][openings]") {
  // Furniture 2 m in front of a window stands beyond front_m: the window is
  // still open.
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.05, kLen, kH, {{2.0, 3.2, 0.9, 2.0}});
  os::add_box(pts, 2.0, 3.2, 2.2, 2.0, 2.4);
  const auto rects = detect(pts);
  REQUIRE(rects.size() == 1);
  CHECK_FALSE(rects[0].at_floor);
  CHECK_THAT(rects[0].z_sill, WithinAbs(0.9, 0.11));
  CHECK_THAT(rects[0].z_head, WithinAbs(2.0, 0.11));
  CHECK_THAT(rects[0].s_start, WithinAbs(2.0, 0.11));
  CHECK_THAT(rects[0].s_end, WithinAbs(3.2, 0.11));
}

TEST_CASE("Openings_WindowInATwoSidedWall", "[reconstruction][openings]") {
  // Both faces of a 10 cm wall, the window cut through both.
  std::vector<Eigen::Vector3d> pts;
  const os::Hole win{0.8, 2.0, 1.0, 2.2};
  os::add_face(pts, 0.05, kLen, kH, {win});
  os::add_face(pts, -0.05, kLen, kH, {win});
  os::add_floor_ceiling(pts, kLen, -2.0, 2.0, kH);
  const auto rects = detect(pts);
  REQUIRE(rects.size() == 1);
  CHECK_THAT(rects[0].z_sill, WithinAbs(1.0, 0.11));
  CHECK_THAT(rects[0].z_head, WithinAbs(2.2, 0.11));
}

TEST_CASE("Openings_SmallHoles_AndSparseHoles_AreDropped",
          "[reconstruction][openings]") {
  std::vector<Eigen::Vector3d> pts;
  // 30 x 30 cm hole: below min_width_m / min_height_m.
  os::add_face(pts, 0.05, kLen, kH, {{1.0, 1.3, 1.0, 1.3}});
  CHECK(detect(pts).empty());
}

TEST_CASE("Openings_UnseenTopBand_IsNotAnOpening",
          "[reconstruction][openings]") {
  // The sensor never saw the top 40 cm of the face.
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.05, kLen, kH, {{0.0, kLen, 1.9, kH}});
  CHECK(detect(pts).empty());
  OpeningOptions keep;
  keep.drop_top_bands = false;
  keep.min_jamb_share = 0.0;
  CHECK_FALSE(detect(pts, keep).empty());
}

TEST_CASE("Openings_TallWindowUpToTheCeiling_IsKept_LowGapAtFloorIsNot",
          "[reconstruction][openings]") {
  std::vector<Eigen::Vector3d> pts;
  // A window from 1.0 m up to the ceiling, and a 0.5 m gap under a
  // radiator at the floor.
  os::add_face(pts, 0.05, kLen, kH,
               {{1.0, 2.2, 1.0, kH}, {3.0, 4.0, 0.0, 0.5}});
  const auto rects = detect(pts);
  REQUIRE(rects.size() == 1);
  CHECK(rects[0].at_ceiling);
  CHECK_THAT(rects[0].z_sill, WithinAbs(1.0, 0.11));
}

TEST_CASE("Openings_UnframedHole_AtTheWallEnd_IsDropped",
          "[reconstruction][openings]") {
  // The face simply ends 1 m before the axis does: no jamb on that side.
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.05, kLen, kH, {{4.0, kLen, 0.0, kH}});
  CHECK(detect(pts).empty());
}

TEST_CASE("Openings_FloorToCeiling_KeepsBothEnds",
          "[reconstruction][openings]") {
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.05, kLen, kH, {{2.0, 3.5, 0.0, kH}});
  os::add_floor_ceiling(pts, kLen, -2.0, 2.0, kH);
  const auto rects = detect(pts);
  REQUIRE(rects.size() == 1);
  CHECK(rects[0].at_floor);
  CHECK(rects[0].at_ceiling);
  CHECK_THAT(rects[0].z_head, WithinAbs(kH, 1e-9));
}

TEST_CASE("Openings_AreDeterministic_AndIndependentOfPointOrder",
          "[reconstruction][openings]") {
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.05, kLen, kH,
               {{1.0, 1.9, 0.0, 2.1}, {3.0, 4.2, 0.9, 2.0}});
  const auto a = detect(pts);
  std::reverse(pts.begin(), pts.end());
  const auto b = detect(pts);
  REQUIRE(a.size() == 2);
  REQUIRE(b.size() == a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    CHECK(a[i].s_start == b[i].s_start);
    CHECK(a[i].z_head == b[i].z_head);
    CHECK(a[i].fill == b[i].fill);
  }
  CHECK(a[0].s_start < a[1].s_start);
}

TEST_CASE("Openings_OneSidedWall_IgnoresWhatIsBehindIt",
          "[reconstruction][openings]") {
  // An external wall seen from +y only: the window's reveal and glazing
  // 30 cm behind the face (-y) hide nothing from the sensor.
  std::vector<Eigen::Vector3d> pts;
  const os::Hole win{2.0, 3.2, 0.9, 2.0};
  os::add_face(pts, 0.05, kLen, kH, {win});
  for (double s = 2.025; s < 3.2; s += 0.05)
    for (double z = 0.925; z < 2.0; z += 0.05)
      pts.emplace_back(s, -0.35, z);
  auto w = os::wall(kLen, kH);
  w.face_a_points = 5000; // +t = +y
  w.face_b_points = 0;
  const auto rects = fit_openings(face_raster(pts, w, 0.0, kH), 0.0, kH);
  REQUIRE(rects.size() == 1);
  CHECK_THAT(rects[0].z_sill, WithinAbs(0.9, 0.11));
  // Seen from both sides (or unknown), the same points do hide it.
  CHECK(detect(pts).empty());
}

TEST_CASE("FaceRaster_OneSidedWallOfUnknownThickness_BodyLiesBehindTheFace",
          "[reconstruction][openings]") {
  // reusex-v8p: a one-sided wall's axis is its seen face (+y here) and its
  // thickness unknown (-1). Its body lies behind the face (-y), so the slab
  // is [-2 h - margin, +margin] about the face, not +-(h + margin): a sill
  // board 10 cm in front of the lower part of a window hides it (occluded),
  // it is not wall.
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.0, kLen, kH, {{2.0, 3.2, 0.9, 2.0}});
  for (double s = 2.025; s < 3.2; s += 0.05)
    for (double z = 0.925; z < 1.2; z += 0.05)
      pts.emplace_back(s, 0.10, z);
  auto w = os::wall(kLen, kH, -1.0);
  w.face_a_points = 5000; // seen from +t = +y
  w.face_b_points = 0;
  const auto r = face_raster(pts, w, 0.0, kH);
  // s = 2.5, z = 1.0: the board in front: occluded, not solid.
  CHECK(r.at(25, 9) == FaceCellState::occluded);
  const auto rects = fit_openings(r, 0.0, kH);
  REQUIRE(rects.size() == 1);
  CHECK_THAT(rects[0].z_sill, WithinAbs(0.9, 0.11));
}

TEST_CASE("Openings_DeskInFrontOfAWindow_StillAWindow",
          "[reconstruction][openings]") {
  // A desk top (0.75 m) standing in front of the lower part of a window
  // (sill 0.6 m), 20 cm off the wall: those cells are occluded, the rest is
  // seen through.
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.05, kLen, kH, {{1.5, 3.3, 0.6, 2.2}});
  os::add_box(pts, 1.3, 3.5, 0.75, 0.25, 0.9);
  const auto rects = detect(pts);
  REQUIRE(rects.size() == 1);
  CHECK(rects[0].open_share < rects[0].fill);
  CHECK_THAT(rects[0].z_sill, WithinAbs(0.6, 0.11));
  CHECK_THAT(rects[0].s_start, WithinAbs(1.5, 0.11));
  CHECK_THAT(rects[0].s_end, WithinAbs(3.3, 0.11));
}
