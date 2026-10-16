// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Solid-model benchmark metrics (reusex-z1v.2.5) on synthetic plans whose
// answers are known in closed form.

#include <reusex/core/solid_model.hpp>
#include <reusex/geometry/transform_utils.hpp>
#include <reusex/reconstruction/solid_metrics.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nlohmann/json.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cmath>
#include <string>

using Catch::Matchers::WithinAbs;
using reusex::core::OpeningRecord;
using reusex::core::SolidModel;
using reusex::core::SpaceRecord;
using reusex::core::StoreyRecord;
using reusex::core::WallRecord;
using reusex::geometry::compare_solid_models;
using reusex::geometry::fit_report;
using reusex::geometry::SolidMetricsOptions;
using reusex::geometry::SolidModelReport;

namespace {

WallRecord wall(const std::string &guid, double x0, double y0, double x1,
                double y1, double thickness) {
  WallRecord w;
  w.guid = guid;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.height = 2.6;
  w.thickness = thickness;
  return w;
}

SpaceRecord rect_space(const std::string &guid, double x0, double y0, double x1,
                       double y1) {
  SpaceRecord s;
  s.guid = guid;
  s.polygon.outer = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  return s;
}

OpeningRecord opening(const std::string &guid, const std::string &host,
                      double s0, double s1, double sill, double head) {
  OpeningRecord o;
  o.guid = guid;
  o.wall = host;
  o.kind = "window";
  o.s_start = s0;
  o.s_end = s1;
  o.z_sill = sill;
  o.z_head = head;
  return o;
}

/// A 6 x 4 m box with 0.3 m walls, one room and one window.
SolidModel box_reference() {
  SolidModel m;
  m.walls = {wall("s", 0, 0, 6, 0, 0.3), wall("e", 6, 0, 6, 4, 0.3),
             wall("n", 6, 4, 0, 4, 0.3), wall("w", 0, 4, 0, 0, 0.3)};
  m.spaces = {rect_space("r", 0.15, 0.15, 5.85, 3.85)};
  m.openings = {opening("o", "s", 2.0, 3.0, 1.0, 2.0)};
  return m;
}

} // namespace

TEST_CASE("CompareSolidModels_IdenticalModels_ScorePerfect",
          "[reconstruction][solid_metrics]") {
  const SolidModel ref = box_reference();
  const SolidModelReport r = compare_solid_models(ref, ref);
  REQUIRE(r.room_count_error.has_value());
  CHECK(*r.room_count_error == 0);
  CHECK_THAT(r.room_iou_mean.value(), WithinAbs(1.0, 1e-9));
  CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.0, 1e-12));
  CHECK_THAT(r.wall_thickness_err_mean_m.value(), WithinAbs(0.0, 1e-12));
  CHECK_THAT(r.wall_thickness_err_p95_m.value(), WithinAbs(0.0, 1e-12));
  CHECK(r.wall_recall.value() == 1.0);
  CHECK(r.wall_precision.value() == 1.0);
  CHECK(r.opening_recall.value() == 1.0);
  CHECK(r.opening_precision.value() == 1.0);
  CHECK_THAT(r.opening_iou_mean.value(), WithinAbs(1.0, 1e-12));
  CHECK(r.matched_walls == 4);
}

TEST_CASE("CompareSolidModels_EmptyModels_EveryMetricIsNa",
          "[reconstruction][solid_metrics]") {
  const SolidModelReport r =
      compare_solid_models(SolidModel{}, box_reference());
  CHECK_FALSE(r.room_count_error.has_value());
  CHECK_FALSE(r.room_iou_mean.has_value());
  CHECK_FALSE(r.wall_position_rms_m.has_value());
  CHECK_FALSE(r.wall_thickness_err_mean_m.has_value());
  CHECK_FALSE(r.wall_recall.has_value());
  CHECK_FALSE(r.wall_precision.has_value());
  CHECK_FALSE(r.opening_recall.has_value());
  CHECK_FALSE(r.opening_precision.has_value());
  CHECK_FALSE(r.opening_iou_mean.has_value());
  CHECK_FALSE(r.fit_mean_m.has_value());
  CHECK_FALSE(r.seen_share_mean.has_value());
  CHECK_FALSE(r.assumed_share.has_value());

  const auto j = nlohmann::json::parse(r.to_json());
  CHECK(j.at("wall_recall").is_null());
  CHECK(j.at("room_iou_mean").is_null());
  CHECK(j.at("fit_mean_m").is_null());
  CHECK(j.at("mesh_unsupported_share").is_null());
}

TEST_CASE("CompareSolidModels_ProvidedButEmptyDetection_ScoresAMiss",
          "[reconstruction][solid_metrics]") {
  SolidModel detected;
  detected.provided.walls = true;
  detected.provided.spaces = true;
  const SolidModelReport r = compare_solid_models(detected, box_reference());
  CHECK(r.wall_recall.value() == 0.0);
  CHECK_FALSE(r.wall_precision.has_value()); // 0 / 0
  CHECK_FALSE(r.wall_position_rms_m.has_value());
  CHECK(r.room_count_error.value() == -1);
  CHECK(r.room_iou_mean.value() == 0.0);
}

TEST_CASE("CompareSolidModels_WallsOffsetByThreeCentimetres_RmsIsTheOffset",
          "[reconstruction][solid_metrics]") {
  const SolidModel ref = box_reference();
  SolidModel det;
  // Every wall moved 3 cm outward, perpendicular to its axis.
  det.walls = {
      wall("a", 0, -0.03, 6, -0.03, 0.3), wall("b", 6.03, 0, 6.03, 4, 0.3),
      wall("c", 6, 4.03, 0, 4.03, 0.3), wall("d", -0.03, 4, -0.03, 0, 0.3)};
  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK(r.matched_walls == 4);
  CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.03, 1e-9));
}

TEST_CASE("CompareSolidModels_OneSidedWalls_ComparedFaceToFace",
          "[reconstruction][solid_metrics]") {
  // reusex-v8p: a wall of unknown thickness (-1) is drawn on its one seen
  // face. Against a wall of known thickness it is scored by its distance to
  // the nearer face of that wall, not to its centre line.
  SECTION("detected one-sided on the inner face of a 0.3 m reference wall") {
    const SolidModel ref = box_reference(); // south wall: faces y = +-0.15
    SolidModel det;
    det.walls = {wall("a", 0, 0.15, 6, 0.15, -1.0)};
    const auto r = compare_solid_models(det, ref);
    REQUIRE(r.matched_walls == 1);
    CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.0, 1e-9));
  }
  SECTION("reference one-sided on the inner face, detected measured") {
    SolidModel ref;
    ref.walls = {wall("s", 0, 0.15, 6, 0.15, -1.0)};
    SolidModel det;
    det.walls = {wall("a", 0, 0.0, 6, 0.0, 0.3)};
    const auto r = compare_solid_models(det, ref);
    REQUIRE(r.matched_walls == 1);
    CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.0, 1e-9));
  }
  SECTION("both one-sided: the faces themselves, 2 cm apart") {
    SolidModel ref;
    ref.walls = {wall("s", 0, 0.15, 6, 0.15, -1.0)};
    SolidModel det;
    det.walls = {wall("a", 0, 0.17, 6, 0.17, -1.0)};
    const auto r = compare_solid_models(det, ref);
    CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.02, 1e-9));
  }
  SECTION("an assumed (defaulted) thickness has no faces to compare") {
    SolidModel ref;
    ref.walls = {wall("s", 0, 0.15, 6, 0.15, -1.0)};
    SolidModel det;
    det.walls = {wall("a", 0, 0.0, 6, 0.0, 0.3)};
    det.walls[0].assumed = true;
    const auto r = compare_solid_models(det, ref);
    CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.15, 1e-9));
  }
}

TEST_CASE("CompareSolidModels_ThicknessErrors_MeanAndP95",
          "[reconstruction][solid_metrics]") {
  const SolidModel ref = box_reference();
  SolidModel det = ref;
  det.walls[0].thickness = 0.32; // +2 cm
  det.walls[1].thickness = 0.26; // -4 cm
  det.walls[2].thickness = -1.0; // unknown: not scored
  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK_THAT(r.wall_thickness_err_mean_m.value(),
             WithinAbs((0.02 + 0.04 + 0.0) / 3.0, 1e-9));
  CHECK_THAT(r.wall_thickness_err_p95_m.value(), WithinAbs(0.04, 1e-9));
}

TEST_CASE("CompareSolidModels_MissingAndSpuriousWalls_RecallAndPrecision",
          "[reconstruction][solid_metrics]") {
  const SolidModel ref = box_reference();
  SolidModel det = ref;
  det.openings.clear();
  det.walls.pop_back();                                // miss "w"
  det.walls.push_back(wall("x", 3, 1, 3, 3, 0.1));     // nothing there
  det.walls.push_back(wall("y", 0, 0.2, 6, 0.2, 0.3)); // a duplicate of "s"
  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK(r.matched_walls == 3);
  CHECK_THAT(r.wall_recall.value(), WithinAbs(3.0 / 4.0, 1e-12));
  CHECK_THAT(r.wall_precision.value(), WithinAbs(3.0 / 5.0, 1e-12));
}

TEST_CASE("CompareSolidModels_WallTooSteepOrTooFar_IsNotAMatch",
          "[reconstruction][solid_metrics]") {
  SolidModel ref;
  ref.walls = {wall("r", 0, 0, 4, 0, 0.2)};
  SolidModel det;
  SECTION("rotated 20 degrees") {
    const double a = 20.0 * M_PI / 180.0;
    det.walls = {wall("d", 0, 0, 4 * std::cos(a), 4 * std::sin(a), 0.2)};
  }
  SECTION("parallel but 0.8 m away") {
    det.walls = {wall("d", 0, 0.8, 4, 0.8, 0.2)};
  }
  SECTION("collinear but beyond the end") {
    det.walls = {wall("d", 5, 0, 9, 0, 0.2)};
  }
  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK(r.matched_walls == 0);
  CHECK(r.wall_recall.value() == 0.0);
}

TEST_CASE("CompareSolidModels_WallsOnDifferentStoreys_DoNotMatch",
          "[reconstruction][solid_metrics]") {
  SolidModel ref;
  ref.walls = {wall("r", 0, 0, 4, 0, 0.2)};
  SolidModel det = ref;
  det.walls[0].storey = 1;
  CHECK(compare_solid_models(det, ref).matched_walls == 0);
}

TEST_CASE("CompareSolidModels_HalfOverlappingSquares_IouIsOneThird",
          "[reconstruction][solid_metrics]") {
  SolidModel ref;
  ref.spaces = {rect_space("r", 0, 0, 1, 1)};
  SolidModel det;
  det.spaces = {rect_space("d", 0.5, 0, 1.5, 1)};
  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK(r.room_count_error.value() == 0);
  CHECK_THAT(r.room_iou_mean.value(), WithinAbs(1.0 / 3.0, 1e-9));
}

TEST_CASE("CompareSolidModels_SpaceWithHole_IouUsesTheHole",
          "[reconstruction][solid_metrics]") {
  SolidModel ref;
  ref.spaces = {rect_space("r", 0, 0, 2, 2)}; // area 4
  SolidModel det;
  det.spaces = {rect_space("d", 0, 0, 2, 2)};
  // A 1 x 1 hole, wound clockwise on purpose: the metric must not care.
  det.spaces[0].polygon.holes = {
      {{0.5, 0.5}, {0.5, 1.5}, {1.5, 1.5}, {1.5, 0.5}}};
  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK_THAT(r.room_iou_mean.value(), WithinAbs(3.0 / 4.0, 1e-9));
}

TEST_CASE("CompareSolidModels_OneRoomDetectedAsTwo_CountErrorAndIou",
          "[reconstruction][solid_metrics]") {
  SolidModel ref;
  ref.spaces = {rect_space("r", 0, 0, 4, 2)};
  SolidModel det;
  det.spaces = {rect_space("a", 0, 0, 2, 2), rect_space("b", 2, 0, 4, 2)};
  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK(r.room_count_error.value() == 1);
  CHECK(r.matched_spaces == 1);
  CHECK_THAT(r.room_iou_mean.value(), WithinAbs(0.5, 1e-9));
}

TEST_CASE("CompareSolidModels_NonSimpleSpacePolygon_Throws",
          "[reconstruction][solid_metrics]") {
  SolidModel ref;
  ref.spaces = {rect_space("r", 0, 0, 1, 1)};
  SolidModel det;
  det.spaces = {rect_space("bow", 0, 0, 1, 1)};
  det.spaces[0].polygon.outer = {{0, 0}, {1, 1}, {1, 0}, {0, 1}};
  CHECK_THROWS_AS(compare_solid_models(det, ref), std::invalid_argument);
}

TEST_CASE("CompareSolidModels_OpeningShiftedAlongTheWall_IouDecidesTheMatch",
          "[reconstruction][solid_metrics]") {
  const SolidModel ref = box_reference(); // window s 2..3, z 1..2 on "s"
  SolidModel det = ref;
  SECTION("shifted 0.25 m: IoU 0.6, matched") {
    det.openings[0].s_start = 2.25;
    det.openings[0].s_end = 3.25;
    const auto r = compare_solid_models(det, ref);
    CHECK(r.opening_recall.value() == 1.0);
    CHECK(r.opening_precision.value() == 1.0);
  }
  SECTION("shifted 0.5 m: IoU 1/3, missed") {
    det.openings[0].s_start = 2.5;
    det.openings[0].s_end = 3.5;
    const auto r = compare_solid_models(det, ref);
    CHECK(r.opening_recall.value() == 0.0);
    CHECK(r.opening_precision.value() == 0.0);
    // ...but the near miss shows in the IoU mean (no threshold).
    CHECK_THAT(r.opening_iou_mean.value(), WithinAbs(1.0 / 3.0, 1e-9));
  }
}

TEST_CASE("CompareSolidModels_OpeningOnAReversedHostWall_StillMatches",
          "[reconstruction][solid_metrics]") {
  // The detected host wall runs the other way, so its s axis is mirrored:
  // the comparison must happen in the reference wall's coordinates.
  const SolidModel ref = box_reference();
  SolidModel det = ref;
  det.walls[0] = wall("s", 6, 0, 0, 0, 0.3);
  det.openings[0].s_start = 3.0; // 6 - 3
  det.openings[0].s_end = 4.0;   // 6 - 2
  const auto r = compare_solid_models(det, ref);
  CHECK(r.opening_recall.value() == 1.0);
}

TEST_CASE("CompareSolidModels_OpeningOnUnknownHostWall_Throws",
          "[reconstruction][solid_metrics]") {
  const SolidModel ref = box_reference();
  SolidModel det = ref;
  det.openings[0].wall = "nope";
  CHECK_THROWS_AS(compare_solid_models(det, ref), std::invalid_argument);
}

TEST_CASE("CompareSolidModels_SeenAndAssumedShares_FromDetectedWalls",
          "[reconstruction][solid_metrics]") {
  SolidModel det = box_reference();
  det.walls[0].seen_share = 0.8;
  det.walls[1].seen_share = 0.4;
  det.walls[2].assumed = true;
  const auto r = compare_solid_models(det, SolidModel{});
  CHECK_THAT(r.seen_share_mean.value(), WithinAbs(0.6, 1e-12));
  CHECK_THAT(r.assumed_share.value(), WithinAbs(0.25, 1e-12));
  CHECK_FALSE(r.wall_recall.has_value()); // no reference walls
}

TEST_CASE("CompareSolidModels_SameInputTwice_SameJson",
          "[reconstruction][solid_metrics]") {
  SolidModel det = box_reference();
  det.walls[1].x0 += 0.02;
  det.walls.push_back(wall("dup", 0, 0.05, 6, 0.05, 0.25));
  const auto a = compare_solid_models(det, box_reference()).to_json();
  const auto b = compare_solid_models(det, box_reference()).to_json();
  CHECK(a == b);
}

TEST_CASE("FitReport_PointsAboveAQuad_MeanDistanceAndWithinShare",
          "[reconstruction][solid_metrics]") {
  pcl::PointCloud<pcl::PointXYZ> verts;
  verts.push_back({0, 0, 0});
  verts.push_back({2, 0, 0});
  verts.push_back({2, 2, 0});
  verts.push_back({0, 2, 0});
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(verts, mesh.cloud);
  pcl::Vertices quad;
  quad.vertices = {0, 1, 2, 3}; // a quad: fanned into two triangles
  mesh.polygons.push_back(quad);

  reusex::Cloud cloud;
  pcl::PointXYZRGB p;
  p.x = 1.0f;
  p.y = 1.0f;
  p.z = 0.01f;
  cloud.push_back(p);
  p.x = 1.5f;
  p.z = -0.05f;
  cloud.push_back(p);
  p.x = std::nanf("");
  cloud.push_back(p); // skipped

  SolidModelReport r;
  fit_report(cloud, mesh, SolidMetricsOptions{}, r);
  CHECK(r.fit_points == 2);
  CHECK_THAT(r.fit_mean_m.value(), WithinAbs(0.03, 1e-6));
  CHECK_THAT(r.fit_within_share.value(), WithinAbs(0.5, 1e-12));
}

TEST_CASE("FitReport_MeshWithoutScanPoints_UnsupportedShare",
          "[reconstruction][solid_metrics]") {
  // Two 1 x 1 quads; the scan covers only the first (points every 5 cm).
  // The second one stands where the cloud has nothing -- a fake wall.
  pcl::PointCloud<pcl::PointXYZ> verts;
  for (const float x0 : {0.0F, 5.0F}) {
    verts.push_back({x0, 0, 0});
    verts.push_back({x0 + 1, 0, 0});
    verts.push_back({x0 + 1, 1, 0});
    verts.push_back({x0, 1, 0});
  }
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(verts, mesh.cloud);
  for (std::uint32_t q : {0U, 4U}) {
    pcl::Vertices quad;
    quad.vertices = {q, q + 1, q + 2, q + 3};
    mesh.polygons.push_back(quad);
  }
  reusex::Cloud cloud;
  pcl::PointXYZRGB p;
  for (int i = 0; i <= 20; ++i)
    for (int j = 0; j <= 20; ++j) {
      p.x = 0.05F * i;
      p.y = 0.05F * j;
      p.z = 0.0F;
      cloud.push_back(p);
    }

  SolidModelReport r;
  fit_report(cloud, mesh, SolidMetricsOptions{}, r);
  CHECK_THAT(r.mesh_area_m2.value(), WithinAbs(2.0, 1e-9));
  CHECK_THAT(r.mesh_unsupported_share.value(), WithinAbs(0.5, 0.01));
  CHECK(r.support_samples > 0);
  const auto j = nlohmann::json::parse(r.to_json());
  CHECK_THAT(j.at("mesh_unsupported_share").get<double>(),
             WithinAbs(0.5, 0.01));
  CHECK(j.at("counts").at("support_samples").get<std::size_t>() ==
        r.support_samples);

  // With a 5 m support distance the second quad (4 m from the scan) counts
  // as supported too.
  SolidMetricsOptions far;
  far.support_dist_m = 5.0;
  SolidModelReport r2;
  fit_report(cloud, mesh, far, r2);
  CHECK_THAT(r2.mesh_unsupported_share.value(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("FitReport_MeshWithoutTriangles_Throws",
          "[reconstruction][solid_metrics]") {
  pcl::PolygonMesh mesh;
  reusex::Cloud cloud;
  cloud.push_back(pcl::PointXYZRGB());
  SolidModelReport r;
  CHECK_THROWS_AS(fit_report(cloud, mesh, SolidMetricsOptions{}, r),
                  std::invalid_argument);
}

TEST_CASE("CompareSolidModels_OpeningOnAnotherStorey_IsNotAMatch",
          "[reconstruction][solid_metrics]") {
  // The same window position one storey up: walls on different storeys
  // never match, and neither may their openings. The reference covers both
  // storeys, so nothing is filtered out before matching.
  SolidModel ref = box_reference();
  ref.storeys.resize(2);
  ref.storeys[1].idx = 1;
  SolidModel det = box_reference();
  for (auto &w : det.walls)
    w.storey = 1;
  const auto r = compare_solid_models(det, ref);
  CHECK(r.detected_openings == 1);
  CHECK(r.matched_openings == 0);
}

TEST_CASE("CompareSolidModels_ShortFragmentOnTheLine_DoesNotStealTheWall",
          "[reconstruction][solid_metrics]") {
  SolidModel ref;
  ref.walls = {wall("r", 0, 0, 4, 0, 0.2)};
  SolidModel det;
  det.walls = {wall("fragment", 1.8, 0.0, 2.2, 0.0, 0.2), // exactly on the line
               wall("full", 0, 0.05, 4, 0.05, 0.2)};      // the real wall
  const auto r = compare_solid_models(det, ref);
  CHECK(r.matched_walls == 1);
  CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.05, 1e-9));
  CHECK_THAT(r.wall_precision.value(), WithinAbs(0.5, 1e-12));
}

TEST_CASE("CompareSolidModels_MinOverlapOption_ControlsFragments",
          "[reconstruction][solid_metrics]") {
  SolidModel ref;
  ref.walls = {wall("r", 0, 0, 4, 0, 0.2)};
  SolidModel det;
  det.walls = {wall("half", 0, 0, 2, 0, 0.2)}; // covers 50 % of the wall
  SolidMetricsOptions opt;
  opt.wall_match_min_overlap = 0.6;
  CHECK(compare_solid_models(det, ref, opt).matched_walls == 0);
  opt.wall_match_min_overlap = 0.4;
  CHECK(compare_solid_models(det, ref, opt).matched_walls == 1);
}

TEST_CASE(
    "CompareSolidModels_DetectionsOnStoreysOutsideTheReference_AreIgnored",
    "[reconstruction][solid_metrics]") {
  // The reference covers storey 0 only; a detector that also models storey 1
  // must not be charged false positives (or extra rooms) for it.
  SolidModel ref = box_reference();
  ref.storeys.resize(1); // storey idx 0
  SolidModel det = box_reference();
  for (auto w : box_reference().walls) {
    w.guid += "-up";
    w.storey = 1;
    det.walls.push_back(w);
  }
  auto upper_room = rect_space("r-up", 0.15, 0.15, 5.85, 3.85);
  upper_room.storey = 1;
  det.spaces.push_back(upper_room);
  auto upper_window = opening("o-up", "s-up", 2.0, 3.0, 1.0, 2.0);
  det.openings.push_back(upper_window);

  const auto r = compare_solid_models(det, ref);
  CHECK(r.wall_precision.value() == 1.0);
  CHECK(r.detected_walls == 4);
  CHECK(r.room_count_error.value() == 0);
  CHECK(r.opening_precision.value() == 1.0);
  CHECK(r.detected_outside_reference == 6); // 4 walls, 1 space, 1 opening
}

TEST_CASE("CompareSolidModels_AssumedThickness_IsNotScored",
          "[reconstruction][solid_metrics]") {
  const SolidModel ref = box_reference();
  SolidModel det = ref;
  det.walls[0].thickness = 0.5; // a defaulted guess, 0.2 off
  det.walls[0].assumed = true;
  det.walls[1].thickness = 0.32;
  const auto r = compare_solid_models(det, ref);
  CHECK(r.thickness_pairs == 3);
  CHECK(r.thickness_assumed_excluded == 1);
  CHECK_THAT(r.wall_thickness_err_mean_m.value(), WithinAbs(0.02 / 3.0, 1e-9));
}

// ---------------------------------------------------------------------------
// Level frames (reusex-4kt): elements are compared in world, not in raw
// level-frame coordinates.

namespace {

constexpr double kDeg = M_PI / 180.0;

/// World -> level frame: a yaw of @p yaw_deg about +z, after moving
/// @p origin (world) to the level-frame origin.
Eigen::Affine3d level_frame(double yaw_deg, const Eigen::Vector3d &origin) {
  return Eigen::Affine3d(
             Eigen::AngleAxisd(yaw_deg * kDeg, Eigen::Vector3d::UnitZ())) *
         Eigen::Translation3d(-origin);
}

StoreyRecord storey(int idx, const Eigen::Affine3d &world_to_level,
                    double z_floor = 0.0) {
  StoreyRecord s;
  s.idx = idx;
  s.guid = "storey-" + std::to_string(idx);
  s.z_floor = z_floor;
  s.z_ceiling = z_floor + 2.6;
  s.level_transform = reusex::geometry::to_array16(world_to_level);
  return s;
}

/// @p m's walls, spaces and openings, given in the level frame
/// @p from (world -> level), re-expressed in level frame @p to. Only valid for
/// frames that differ by a yaw and a translation (s along a wall is kept).
SolidModel reexpress(SolidModel m, const Eigen::Affine3d &from,
                     const Eigen::Affine3d &to) {
  const Eigen::Affine3d T = to * from.inverse();
  auto map = [&](double x, double y, double z) {
    return Eigen::Vector3d(T * Eigen::Vector3d(x, y, z));
  };
  for (auto &w : m.walls) {
    const auto a = map(w.x0, w.y0, w.z_base), b = map(w.x1, w.y1, w.z_base);
    w.x0 = a.x();
    w.y0 = a.y();
    w.x1 = b.x();
    w.y1 = b.y();
    w.z_base = a.z();
  }
  for (auto &s : m.spaces)
    for (auto &q : s.polygon.outer) {
      const auto p = map(q[0], q[1], s.z_floor);
      q = {p.x(), p.y()};
    }
  for (auto &o : m.openings) {
    o.z_sill = map(0, 0, o.z_sill).z();
    o.z_head = map(0, 0, o.z_head).z();
  }
  return m;
}

/// The 6 x 4 box, placed in world at (100, 50) with its floor at world
/// z = 12, authored the way a reference plan is: world x/y, z shifted so the
/// floor is at 0 (level_transform = translation by -12 in z).
SolidModel world_box_reference(int idx = 0, double floor_z = 12.0) {
  const Eigen::Affine3d ref_frame(Eigen::Translation3d(0.0, 0.0, -floor_z));
  SolidModel m = box_reference();
  for (auto &w : m.walls)
    w.storey = idx;
  for (auto &s : m.spaces)
    s.storey = idx;
  // box_reference() is at the origin; put it at (100, 50) in world.
  m = reexpress(m, Eigen::Affine3d(Eigen::Translation3d(-100.0, -50.0, 0.0)),
                Eigen::Affine3d::Identity());
  m.storeys = {storey(idx, ref_frame)};
  return m;
}

} // namespace

TEST_CASE("CompareSolidModels_SamePlanInAnotherLevelFrame_ScoresPerfect",
          "[reconstruction][solid_metrics][level_frame]") {
  const SolidModel ref = world_box_reference();
  const Eigen::Affine3d ref_frame = Eigen::Affine3d(
      reusex::geometry::to_affine(ref.storeys[0].level_transform));

  Eigen::Affine3d det_frame;
  double det_floor = 0.0;
  SECTION("world-origin frame (identity, floor at z = 12)") {
    det_frame = Eigen::Affine3d::Identity();
    det_floor = 12.0;
  }
  SECTION("floor-origin frame, rotated 3 degrees") {
    // geometry::make_level_frame puts the origin on a storey-0 floor point,
    // which depends on the run's plane segmentation.
    det_frame = level_frame(3.0, {101.2, 52.7, 12.0});
  }
  SECTION("floor-origin frame, rotated 90 degrees") {
    det_frame = level_frame(90.0, {97.0, 48.5, 12.0});
  }
  SolidModel det = reexpress(ref, ref_frame, det_frame);
  det.storeys = {storey(0, det_frame, det_floor)};

  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK(r.detected_outside_reference == 0);
  CHECK(r.matched_walls == 4);
  CHECK(r.wall_recall.value() == 1.0);
  CHECK(r.wall_precision.value() == 1.0);
  CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.0, 1e-9));
  CHECK_THAT(r.room_iou_mean.value(), WithinAbs(1.0, 1e-9));
  CHECK(r.opening_recall.value() == 1.0);
  CHECK(r.opening_precision.value() == 1.0);
}

TEST_CASE("CompareSolidModels_ShiftedWallInAnotherLevelFrame_ShowsTrueOffset",
          "[reconstruction][solid_metrics][level_frame]") {
  const SolidModel ref = world_box_reference();
  const Eigen::Affine3d ref_frame =
      reusex::geometry::to_affine(ref.storeys[0].level_transform);
  // The south wall moved 0.1 m south in world, then the whole model
  // expressed in a rotated floor-origin frame.
  SolidModel shifted = ref;
  shifted.walls[0].y0 -= 0.1;
  shifted.walls[0].y1 -= 0.1;
  const Eigen::Affine3d det_frame = level_frame(-4.0, {103.0, 49.0, 12.0});
  SolidModel det = reexpress(shifted, ref_frame, det_frame);
  det.storeys = {storey(0, det_frame)};

  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK(r.matched_walls == 4);
  // One wall of four off by 0.1 m at both ends: sqrt(2 * 0.01 / 8).
  CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.05, 1e-9));
  CHECK(r.opening_recall.value() == 1.0);
}

TEST_CASE("CompareSolidModels_StoreysMatchedByWorldFloorHeight",
          "[reconstruction][solid_metrics][level_frame]") {
  // The reference models two storeys (floors at world z = 12 and 15), the
  // box on the upper one. The detector only saw the upper storey and calls
  // it idx 0; it also found a storey at world z = 18 the reference does not
  // have.
  SolidModel ref = world_box_reference(/*idx=*/1, /*floor_z=*/15.0);
  ref.storeys.insert(
      ref.storeys.begin(),
      storey(0, Eigen::Affine3d(Eigen::Translation3d(0.0, 0.0, -12.0))));
  const Eigen::Affine3d ref_frame =
      reusex::geometry::to_affine(ref.storeys[1].level_transform);

  // One frame for the whole scan (as detect_storeys writes it), origin on
  // the storey-0 floor; storey 1's floor is 3 m up in that frame.
  const Eigen::Affine3d det_frame = level_frame(2.0, {99.0, 51.0, 15.0});
  SolidModel det = reexpress(ref, ref_frame, det_frame);
  for (auto &w : det.walls)
    w.storey = 0;
  for (auto &s : det.spaces)
    s.storey = 0;
  det.storeys = {storey(0, det_frame, 0.0), storey(1, det_frame, 3.0)};
  WallRecord upstairs = wall("up", 0, 0, 3, 0, 0.2);
  upstairs.storey = 1;
  upstairs.z_base = 3.0;
  det.walls.push_back(upstairs);

  const SolidModelReport r = compare_solid_models(det, ref);
  CHECK(r.detected_outside_reference == 1); // the wall on the z = 18 storey
  CHECK(r.matched_walls == 4);
  CHECK(r.wall_precision.value() == 1.0);
  CHECK_THAT(r.wall_position_rms_m.value(), WithinAbs(0.0, 1e-9));
  CHECK_THAT(r.room_iou_mean.value(), WithinAbs(1.0, 1e-9));
  CHECK(r.opening_recall.value() == 1.0);
}

TEST_CASE("CompareSolidModels_SingularLevelTransform_Throws",
          "[reconstruction][solid_metrics][level_frame]") {
  SolidModel ref = world_box_reference();
  SolidModel det = ref;
  det.storeys[0].level_transform = {};
  CHECK_THROWS_AS(compare_solid_models(det, ref), std::invalid_argument);
}
