// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Pinhole projection of instance points into a sensor frame (#373; moved from
// vision for the resource view record).

#include <catch2/catch_test_macros.hpp>

#include <core/SensorIntrinsics.hpp>
#include <core/frame_projection.hpp>

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

using reusex::core::project_points_to_box;
using reusex::core::SensorIntrinsics;

namespace {

// Identity pose (camera at world origin, looking down +Z), simple intrinsics.
SensorIntrinsics makeIntrinsics() {
  SensorIntrinsics k;
  k.fx = 500;
  k.fy = 500;
  k.cx = 320;
  k.cy = 240;
  k.width = 640;
  k.height = 480;
  // local_transform defaults to identity.
  return k;
}

const std::array<double, 16> kIdentityPose = {1, 0, 0, 0, 0, 1, 0, 0,
                                              0, 0, 1, 0, 0, 0, 0, 1};

} // namespace

TEST_CASE("ProjectPointsToBox_PointOnOpticalAxis_LandsAtPrincipalPoint",
          "[core][projection]") {
  auto k = makeIntrinsics();
  std::vector<Eigen::Vector3f> pts = {{0.f, 0.f, 2.f}};
  auto box = project_points_to_box(pts, kIdentityPose, k, 640, 480);
  REQUIRE(box.has_value());
  CHECK(box->in_bounds == 1);
  CHECK(box->x_min == 320);
  CHECK(box->y_min == 240);
}

TEST_CASE("ProjectPointsToBox_ClusterOfPoints_ComputesEnclosingBox",
          "[core][projection]") {
  auto k = makeIntrinsics();
  // Points at z=2: x = px offset * z / fx. For px offsets +-100 -> x = +-0.4.
  std::vector<Eigen::Vector3f> pts = {
      {-0.4f, -0.24f, 2.f}, {0.4f, 0.24f, 2.f}, {0.f, 0.f, 2.f}};
  auto box = project_points_to_box(pts, kIdentityPose, k, 640, 480);
  REQUIRE(box.has_value());
  CHECK(box->in_bounds == 3);
  CHECK(box->x_min == 220);
  CHECK(box->x_max == 420);
  CHECK(box->y_min == 180);
  CHECK(box->y_max == 300);
}

TEST_CASE("ProjectPointsToBox_PointsBehindCamera_AreExcluded",
          "[core][projection]") {
  auto k = makeIntrinsics();
  std::vector<Eigen::Vector3f> pts = {{0.f, 0.f, -2.f}, {0.f, 0.f, 2.f}};
  auto box = project_points_to_box(pts, kIdentityPose, k, 640, 480);
  REQUIRE(box.has_value());
  CHECK(box->in_bounds == 1); // only the +Z point counts
}

TEST_CASE("ProjectPointsToBox_AllPointsOutOfFrame_ReturnsNullopt",
          "[core][projection]") {
  auto k = makeIntrinsics();
  // Far off to the side: x huge relative to z -> px well outside 0..640.
  std::vector<Eigen::Vector3f> pts = {{100.f, 0.f, 1.f}};
  auto box = project_points_to_box(pts, kIdentityPose, k, 640, 480);
  CHECK_FALSE(box.has_value());
}

TEST_CASE("ProjectPointsToBox_ScaledImageSize_ScalesIntrinsics",
          "[core][projection]") {
  auto k = makeIntrinsics();
  std::vector<Eigen::Vector3f> pts = {{0.f, 0.f, 2.f}};
  // Half resolution: principal point should halve to (160, 120).
  auto box = project_points_to_box(pts, kIdentityPose, k, 320, 240);
  REQUIRE(box.has_value());
  CHECK(box->x_min == 160);
  CHECK(box->y_min == 120);
}

TEST_CASE("ProjectPointsToBox_EmptyInputOrBadIntrinsics_ReturnsNullopt",
          "[core][projection]") {
  auto k = makeIntrinsics();
  CHECK_FALSE(
      project_points_to_box({}, kIdentityPose, k, 640, 480).has_value());

  SensorIntrinsics bad;
  std::vector<Eigen::Vector3f> pts = {{0.f, 0.f, 2.f}};
  CHECK_FALSE(
      project_points_to_box(pts, kIdentityPose, bad, 640, 480).has_value());
}

#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/ProjectDB.hpp>
#include <core/instance_evidence.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <pcl/point_types.h>

using Catch::Matchers::WithinAbs;
using reusex::ProjectDB;
using reusex::core::instance_points;
using reusex::core::project_instance_bbox;

namespace {
/// fx = fy = 100, principal point (64, 64), 128 x 128 — the instance
/// evidence fixture's camera.
SensorIntrinsics pinhole128() {
  SensorIntrinsics k;
  k.fx = k.fy = 100.0;
  k.cx = k.cy = 64.0;
  k.width = k.height = 128;
  return k;
}
void frame_at_origin(ProjectDB &db, int id) {
  db.save_sensor_frame(id, cv::Mat(128, 128, CV_8UC3, cv::Scalar(40, 90, 160)),
                       cv::Mat(), cv::Mat(), kIdentityPose, pinhole128(),
                       static_cast<double>(id), -1);
}
/// Instance 1: (-0.1, 0, 2) and (0.1, 0, 2); instance 2: (0, 0, -10)
/// (behind the camera); one unlabeled point and one NaN point of instance 1.
void seed_instances(ProjectDB &db) {
  reusex::Cloud positions;
  reusex::CloudL labels;
  auto add = [&](float x, float y, float z, std::uint32_t label) {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    positions.push_back(p);
    pcl::Label l;
    l.label = label;
    labels.push_back(l);
  };
  add(-0.1F, 0.0F, 2.0F, 1);
  add(0.1F, 0.0F, 2.0F, 1);
  add(std::numeric_limits<float>::quiet_NaN(), 0.0F, 2.0F, 1);
  add(0.0F, 0.0F, -10.0F, 2);
  add(5.0F, 5.0F, 5.0F, 0);
  db.save_point_cloud("cloud", positions, "test", "{}");
  db.save_point_cloud("instances", labels, "test", "{}");
}
} // namespace

TEST_CASE("InstancePoints_GroupsFinitePointsByLabel_SkipsUnlabeled",
          "[core][projection]") {
  const reusex::test_support::TempPath tmp("instance_points");
  ProjectDB db(tmp.path);
  seed_instances(db);
  const auto pts = instance_points(db, "instances");
  REQUIRE(pts.size() == 2);
  CHECK(pts.at(1).size() == 2); // the NaN point is dropped
  CHECK(pts.at(2).size() == 1);
  CHECK(pts.count(0) == 0);
  CHECK_THROWS_AS(instance_points(db, "nope"), std::invalid_argument);
  CHECK_THROWS_AS(instance_points(db, "instances", "nope"),
                  std::invalid_argument);
  reusex::CloudL longer;
  longer.resize(9);
  db.save_point_cloud("instances", longer, "test", "{}");
  CHECK_THROWS_AS(instance_points(db, "instances"),
                  reusex::core::CloudMisalignedError);
}

TEST_CASE("ProjectInstanceBbox_SeenInstance_ReturnsPaddedNormalisedRegion",
          "[core][projection]") {
  const reusex::test_support::TempPath tmp("instance_bbox");
  ProjectDB db(tmp.path);
  seed_instances(db);
  frame_at_origin(db, 1);
  const auto pts = instance_points(db, "instances");
  // u = 59 and 69, v = 64: box [59, 64, 11 px, 1 px] of 128, padded 15 %.
  const auto r = project_instance_bbox(db, 1, pts.at(1));
  REQUIRE(r);
  CHECK_THAT((*r)[0], WithinAbs(57.35 / 128.0, 1e-9));
  CHECK_THAT((*r)[1], WithinAbs(63.85 / 128.0, 1e-9));
  CHECK_THAT((*r)[2], WithinAbs(14.3 / 128.0, 1e-9));
  CHECK_THAT((*r)[3], WithinAbs(1.3 / 128.0, 1e-9));
  const auto unpadded = project_instance_bbox(db, 1, pts.at(1), 0.0);
  REQUIRE(unpadded);
  CHECK_THAT((*unpadded)[2], WithinAbs(11.0 / 128.0, 1e-9));
}

TEST_CASE("ProjectInstanceBbox_UnseenOrUnposed_ReturnsNullopt",
          "[core][projection]") {
  const reusex::test_support::TempPath tmp("instance_bbox_none");
  ProjectDB db(tmp.path);
  seed_instances(db);
  frame_at_origin(db, 1);
  const auto pts = instance_points(db, "instances");
  CHECK_FALSE(project_instance_bbox(db, 1, pts.at(2)));  // behind the camera
  CHECK_FALSE(project_instance_bbox(db, 99, pts.at(1))); // no such frame
  CHECK_FALSE(project_instance_bbox(db, 1, {}));         // no points
}

using reusex::core::complete_view_region;
using reusex::core::ResourceView;
using reusex::core::ViewKind;

TEST_CASE("CompleteViewRegion_InstanceInView_FillsPaddedBbox",
          "[core][projection][view]") {
  const reusex::test_support::TempPath tmp("complete_view_region");
  ProjectDB db(tmp.path);
  seed_instances(db);
  frame_at_origin(db, 1);
  const auto filled = complete_view_region(
      db, ResourceView{ViewKind::frame, 1, std::nullopt}, "instances", 1);
  CHECK(filled.kind == ViewKind::frame);
  CHECK(filled.id == 1);
  REQUIRE(filled.region);
  CHECK_THAT((*filled.region)[0], WithinAbs(57.35 / 128.0, 1e-9));
  CHECK_THAT((*filled.region)[1], WithinAbs(63.85 / 128.0, 1e-9));
  CHECK_THAT((*filled.region)[2], WithinAbs(14.3 / 128.0, 1e-9));
  CHECK_THAT((*filled.region)[3], WithinAbs(1.3 / 128.0, 1e-9));
  const auto expected =
      project_instance_bbox(db, 1, instance_points(db, "instances").at(1));
  CHECK(filled.region == expected);
}

TEST_CASE("CompleteViewRegion_RegionGivenOrPanorama_Unchanged",
          "[core][projection][view]") {
  const reusex::test_support::TempPath tmp("complete_view_region_kept");
  ProjectDB db(tmp.path);
  seed_instances(db);
  frame_at_origin(db, 1);
  const ResourceView given{ViewKind::frame, 1,
                           reusex::core::ViewRegion{0.1, 0.2, 0.3, 0.4}};
  CHECK(complete_view_region(db, given, "instances", 1) == given);
  const ResourceView pano{ViewKind::panorama, 1, std::nullopt};
  CHECK(complete_view_region(db, pano, "instances", 1) == pano);
}

TEST_CASE("CompleteViewRegion_NotInViewOrNoCloud_StaysNull",
          "[core][projection][view]") {
  const reusex::test_support::TempPath tmp("complete_view_region_null");
  ProjectDB db(tmp.path);
  seed_instances(db);
  frame_at_origin(db, 1);
  const ResourceView bare{ViewKind::frame, 1, std::nullopt};
  // Instance 2 is behind the camera; instance 7 has no points.
  CHECK(complete_view_region(db, bare, "instances", 2) == bare);
  CHECK(complete_view_region(db, bare, "instances", 7) == bare);
  // A frame that does not exist: unchanged, the caller's check refuses it.
  const ResourceView missing{ViewKind::frame, 99, std::nullopt};
  CHECK(complete_view_region(db, missing, "instances", 1) == missing);
  // Missing label or positions cloud: unchanged, no throw.
  CHECK(complete_view_region(db, bare, "nope", 1) == bare);
  CHECK(complete_view_region(db, bare, "instances", 1, "nope") == bare);
  // Misaligned clouds: unchanged, no throw.
  reusex::CloudL longer;
  longer.resize(9);
  db.save_point_cloud("instances", longer, "test", "{}");
  CHECK(complete_view_region(db, bare, "instances", 1) == bare);
}
