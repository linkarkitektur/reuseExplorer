// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Covers reusex::vision::project() — the 2D-label -> 3D-cloud projection
// behind `rux create project` (#205).
//
// The scene is a synthetic pinhole camera looking down +z with an identity
// world pose and an identity camera-to-base transform, so the projection
// reduces to px = fx*x/z + cx, py = fy*y/z + cy and every expected label can
// be computed in the test. The geometry is a fronto-parallel grid of points at
// a known constant depth, which also makes the internally-computed z-buffer
// (built at 1/16 resolution and resized back up) a constant field, so the
// 0.15 m occlusion test is satisfied everywhere the label image is defined.
//
// The assertions are the label contract of docs/STANDARDS.md §3:
//   * segmentation storage is CV_16U with a +1 offset, the ProjectDB API is
//     CV_32S with -1 = background, in-memory point labels are 0 = unlabeled;
//   * background (-1) must land as kUnlabeled (0), never as a wrapped or
//     off-by-one class id;
//   * points the projection never touches must stay 0, so a caller can rely
//     on `label >= 1` before indexing anything.
//
// UNTESTED HERE, and why:
//   * The visual observer branch (viewer_add_camera_frustum) — needs the
//     PCL/Qt viewer; get_visual_observer() is null headless, which is the
//     path taken here.
//   * Non-identity local_transform / rotated poses beyond the single
//     translated-camera case: rtabmap::CameraModel and util3d own that
//     algebra, and asserting it here would be testing RTABMap.
//   * The label *producer* (annotate / SAM3 / YOLO inference) — needs weights
//     and a GPU. This test writes segmentation images directly instead.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/label_semantics.hpp>
#include <types/point_types.hpp>
#include <vision/project.hpp>

#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using reusex::ProjectDB;
using reusex::vision::project;

namespace {

// 1/16 of these must still be a usable z-buffer: project() builds the buffer
// at intrinsics.width/16 x intrinsics.height/16 before resizing it back up.
constexpr int kWidth = 128;
constexpr int kHeight = 128;
constexpr double kFx = 128.0;
constexpr double kFy = 128.0;
constexpr double kCx = 64.0;
constexpr double kCy = 64.0;

/// Depth of the synthetic wall, in metres. Comfortably inside project()'s
/// hard-coded 7 m far cutoff.
constexpr float kDepth = 2.0F;

/// The labelled square, in pixels: [kPatchLo, kPatchHi) on both axes.
constexpr int kPatchLo = 40;
constexpr int kPatchHi = 88;
constexpr int kPatchClass = 7; // API-side class id (CV_32S, >= 0)

std::array<double, 16> identity4() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

reusex::core::SensorIntrinsics make_intrinsics() {
  reusex::core::SensorIntrinsics intr;
  intr.fx = kFx;
  intr.fy = kFy;
  intr.cx = kCx;
  intr.cy = kCy;
  intr.width = kWidth;
  intr.height = kHeight;
  intr.local_transform = identity4(); // camera frame == base frame
  return intr;
}

/// A label image that is background everywhere except one axis-aligned square.
cv::Mat make_label_image() {
  cv::Mat labels(kHeight, kWidth, CV_32S,
                 cv::Scalar(reusex::core::kBackgroundApi));
  labels(cv::Range(kPatchLo, kPatchHi), cv::Range(kPatchLo, kPatchHi))
      .setTo(kPatchClass);
  return labels;
}

/// A dense fronto-parallel grid of points at z = kDepth, spanning most of the
/// image. Dense enough that every 8x8 z-buffer cell receives points, so no
/// hole-filling artefacts perturb the depth test.
reusex::CloudPtr make_wall() {
  reusex::CloudPtr cloud(new reusex::Cloud);
  for (int i = 0; i < 73; ++i) {
    for (int j = 0; j < 73; ++j) {
      reusex::PointT p;
      p.x = -0.9F + 0.025F * static_cast<float>(i);
      p.y = -0.9F + 0.025F * static_cast<float>(j);
      p.z = kDepth;
      p.r = p.g = p.b = 200;
      cloud->push_back(p);
    }
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

/// Where a point lands in the image, using the same pinhole algebra project()
/// applies (identity pose and identity local transform make it exact).
std::pair<int, int> pixel_of(const reusex::PointT &p) {
  const int px =
      static_cast<int>(std::lround(kFx * static_cast<double>(p.x) / p.z + kCx));
  const int py =
      static_cast<int>(std::lround(kFy * static_cast<double>(p.y) / p.z + kCy));
  return {px, py};
}

bool inside_patch(int px, int py) {
  return px >= kPatchLo && px < kPatchHi && py >= kPatchLo && py < kPatchHi;
}

std::shared_ptr<ProjectDB> make_project(const cv::Mat &label_image,
                                        bool with_segmentation = true) {
  auto db = std::make_shared<ProjectDB>(":memory:");
  cv::Mat color(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0));
  db->save_sensor_frame(1, color, cv::Mat(), cv::Mat(), identity4(),
                        make_intrinsics());
  if (with_segmentation)
    db->save_segmentation_image(1, label_image);
  return db;
}

} // namespace

TEST_CASE("Project_PointsInsideLabelledSquare_GetSquareClassLabel",
          "[vision][project]") {
  auto db = make_project(make_label_image());
  auto cloud = make_wall();

  auto labels = project(*db, cloud);

  REQUIRE(labels);
  REQUIRE(labels->size() == cloud->size());

  size_t labelled = 0;
  size_t unlabelled = 0;
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto [px, py] = pixel_of(cloud->points[i]);
    const uint32_t got = labels->points[i].label;

    if (inside_patch(px, py)) {
      // Point label = API class + 1 (STANDARDS §3.1).
      REQUIRE(got == static_cast<uint32_t>(kPatchClass) + 1u);
      ++labelled;
    } else {
      // Background (-1) and anything the projection skipped are both
      // kUnlabeled — never a wrapped or off-by-one id.
      REQUIRE(got == reusex::core::kUnlabeled);
      ++unlabelled;
    }
  }

  // Guard against a vacuous pass in either direction: the square is ~48x48 px
  // of a 128x128 image, so both populations must be substantial.
  REQUIRE(labelled > 100);
  REQUIRE(unlabelled > 100);
}

TEST_CASE("Project_AllBackgroundLabelImage_LeavesAllPointsUnlabeled",
          "[vision][project]") {
  // An all-background label image must produce an all-zero CloudL, not a
  // cloud of some sentinel class.
  cv::Mat background(kHeight, kWidth, CV_32S,
                     cv::Scalar(reusex::core::kBackgroundApi));
  auto db = make_project(background);
  auto cloud = make_wall();

  auto labels = project(*db, cloud);

  REQUIRE(labels->size() == cloud->size());
  for (const auto &pt : labels->points)
    REQUIRE(pt.label == reusex::core::kUnlabeled);
}

TEST_CASE("Project_ClassIdZero_MapsToPointLabelOne", "[vision][project]") {
  // 0 is a valid API class id (only -1 is background): the first prompt
  // class. It must become point label 1, not 0 == kUnlabeled, or every point
  // of that class is silently dropped by segment_instances (reusex-z1v.2.1).
  cv::Mat all_zero(kHeight, kWidth, CV_32S, cv::Scalar(0));
  auto db = make_project(all_zero);
  auto cloud = make_wall();

  auto labels = project(*db, cloud);

  REQUIRE(labels->size() == cloud->size());
  size_t first_class = 0;
  for (const auto &pt : labels->points) {
    REQUIRE((pt.label == 1u || pt.label == reusex::core::kUnlabeled));
    if (pt.label == 1u)
      ++first_class;
  }
  REQUIRE(first_class > 100);
}

TEST_CASE("Project_FrameWithoutSegmentationImage_LeavesPointsUnlabeled",
          "[vision][project]") {
  auto db = make_project(cv::Mat(), /*with_segmentation=*/false);
  auto cloud = make_wall();

  auto labels = project(*db, cloud);

  REQUIRE(labels->size() == cloud->size());
  for (const auto &pt : labels->points)
    REQUIRE(pt.label == reusex::core::kUnlabeled);
}

// An empty cloud is a caller error, not a legitimate state: `rux create
// project` only makes sense after `rux create clouds`, and returning an empty
// CloudL would let the pipeline continue on nothing. project() therefore
// rejects it at the entry point with a std::runtime_error naming the stage
// (STANDARDS §5), instead of letting it reach
// rtabmap::util3d::projectCloudToCamera() and die on that function's internal
// UASSERT ("Condition (!laserScan->empty()) not met!"), which named neither
// this stage nor the empty input (#279).
TEST_CASE("Project_EmptyCloud_ThrowsStageNamedError", "[vision][project]") {
  auto db = make_project(make_label_image());
  reusex::CloudPtr empty(new reusex::Cloud);
  empty->width = 0;
  empty->height = 1;

  REQUIRE_THROWS_AS(project(*db, empty), std::runtime_error);

  // The message must identify the stage and the problem — the whole point of
  // the guard is that it does not read like an RTABMap assertion.
  try {
    project(*db, empty);
    FAIL("project() accepted an empty cloud");
  } catch (const std::runtime_error &e) {
    const std::string what = e.what();
    REQUIRE(what.find("project") != std::string::npos);
    REQUIRE(what.find("empty") != std::string::npos);
  }
}

TEST_CASE("Project_NullCloudPointer_Throws", "[vision][project]") {
  auto db = make_project(make_label_image());

  REQUIRE_THROWS_AS(project(*db, reusex::CloudPtr()), std::runtime_error);
}

TEST_CASE("Project_NoSensorFrames_LabelsNothing", "[vision][project]") {
  auto db = std::make_shared<ProjectDB>(":memory:");
  auto cloud = make_wall();

  auto labels = project(*db, cloud);

  REQUIRE(labels->size() == cloud->size());
  for (const auto &pt : labels->points)
    REQUIRE(pt.label == reusex::core::kUnlabeled);
}

TEST_CASE("Project_PointsBehindCamera_LeavesPointsUnlabeled",
          "[vision][project]") {
  auto db = make_project(make_label_image());

  // Same wall, mirrored to negative z. project() rejects pt_cam.z <= 0 before
  // it can divide by it, so nothing may be labelled.
  auto wall = make_wall();
  reusex::CloudPtr behind(new reusex::Cloud);
  for (const auto &p : wall->points) {
    reusex::PointT q = p;
    q.z = -kDepth;
    behind->push_back(q);
  }
  behind->width = behind->size();
  behind->height = 1;

  auto labels = project(*db, behind);

  REQUIRE(labels->size() == behind->size());
  for (const auto &pt : labels->points)
    REQUIRE(pt.label == reusex::core::kUnlabeled);
}

TEST_CASE("Project_PointsBeyondFarCutoff_LeavesPointsUnlabeled",
          "[vision][project]") {
  auto db = make_project(make_label_image());

  // project() hard-codes a 7 m maximum depth (see the TODO in project.cpp).
  auto wall = make_wall();
  reusex::CloudPtr far(new reusex::Cloud);
  for (const auto &p : wall->points) {
    reusex::PointT q = p;
    q.x = p.x * 5.0F; // keep the same image footprint at 5x the distance
    q.y = p.y * 5.0F;
    q.z = kDepth * 5.0F; // 10 m > 7 m
    far->push_back(q);
  }
  far->width = far->size();
  far->height = 1;

  auto labels = project(*db, far);

  REQUIRE(labels->size() == far->size());
  for (const auto &pt : labels->points)
    REQUIRE(pt.label == reusex::core::kUnlabeled);
}
