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
// default 0.15 m occlusion tolerance is satisfied everywhere the label image is
// defined.
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

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

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
/// default 7 m far cutoff (ProjectLabelsOptions::max_depth_m).
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

  // The default ProjectLabelsOptions::max_depth_m is 7 m.
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

// ── Multi-view vote (reusex-z1v.2.2) ──────────────────────────────────────
// Every frame below sits at the same identity pose, so each frame sees the
// same points at the same pixels and only the label images differ. project()
// used to be last-frame-wins: the frame order is chosen so that last-wins
// gives a different answer than the vote in every case.

namespace {

/// One frame per label image, ids 1..N in order, all at the identity pose.
std::shared_ptr<ProjectDB>
make_multi_frame_project(const std::vector<cv::Mat> &label_images) {
  auto db = std::make_shared<ProjectDB>(":memory:");
  cv::Mat color(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0));
  int id = 1;
  for (const auto &img : label_images) {
    db->save_sensor_frame(id, color, cv::Mat(), cv::Mat(), identity4(),
                          make_intrinsics());
    db->save_segmentation_image(id, img);
    ++id;
  }
  return db;
}

cv::Mat uniform_label_image(int api_class) {
  return cv::Mat(kHeight, kWidth, CV_32S, cv::Scalar(api_class));
}

/// Number of points whose label is @p label; also REQUIREs that every other
/// point is unlabeled, so a stray third value cannot hide.
size_t count_label(const reusex::CloudL &labels, uint32_t label) {
  size_t n = 0;
  for (const auto &pt : labels.points) {
    REQUIRE((pt.label == label || pt.label == reusex::core::kUnlabeled));
    if (pt.label == label)
      ++n;
  }
  return n;
}

} // namespace

TEST_CASE("Project_ClassThenBackgroundFrame_BackgroundNeverOverwritesClass",
          "[vision][project][vote]") {
  // Frame 1 sees the patch as class 7, frame 2 sees everything as background.
  cv::Mat background(kHeight, kWidth, CV_32S,
                     cv::Scalar(reusex::core::kBackgroundApi));
  auto db = make_multi_frame_project({make_label_image(), background});
  auto cloud = make_wall();

  auto labels = project(*db, cloud);

  REQUIRE(labels->size() == cloud->size());
  size_t labelled = 0;
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto [px, py] = pixel_of(cloud->points[i]);
    const uint32_t got = labels->points[i].label;
    if (inside_patch(px, py)) {
      REQUIRE(got == reusex::core::api_to_point_label(kPatchClass));
      ++labelled;
    } else {
      REQUIRE(got == reusex::core::kUnlabeled);
    }
  }
  REQUIRE(labelled > 100);
}

TEST_CASE("Project_TwoFramesAgainstOne_MajorityWins",
          "[vision][project][vote]") {
  // API class 3 twice, then class 5 once (last, so last-wins would pick 5).
  auto db = make_multi_frame_project(
      {uniform_label_image(3), uniform_label_image(3), uniform_label_image(5)});
  auto cloud = make_wall();

  auto labels = project(*db, cloud);

  REQUIRE(labels->size() == cloud->size());
  REQUIRE(count_label(*labels, reusex::core::api_to_point_label(3)) > 1000);
}

TEST_CASE("Project_ExactTie_LowestClassWins", "[vision][project][vote]") {
  // One vote each; the higher class comes last so last-wins would pick it.
  auto db = make_multi_frame_project(
      {uniform_label_image(2), uniform_label_image(9)});
  auto cloud = make_wall();

  auto labels = project(*db, cloud);

  REQUIRE(labels->size() == cloud->size());
  REQUIRE(count_label(*labels, reusex::core::api_to_point_label(2)) > 1000);
}

TEST_CASE("Project_RepeatedRuns_AreBitIdentical", "[vision][project][vote]") {
  // A mix of disagreeing frames, so the vote has real work to do.
  cv::Mat mixed = uniform_label_image(4);
  mixed(cv::Range(0, kHeight / 2), cv::Range::all())
      .setTo(reusex::core::kBackgroundApi);
  auto db = make_multi_frame_project(
      {make_label_image(), mixed, uniform_label_image(4), make_label_image()});
  auto cloud = make_wall();

  auto first = project(*db, cloud);
  REQUIRE(first->size() == cloud->size());
  for (int run = 0; run < 3; ++run) {
    auto again = project(*db, cloud);
    REQUIRE(again->size() == first->size());
    for (size_t i = 0; i < first->size(); ++i)
      REQUIRE(again->points[i].label == first->points[i].label);
  }
}

TEST_CASE("Project_MaxDepthOption_PointsBeyondItStayUnlabeled",
          "[vision][project][options]") {
  auto db = make_project(make_label_image());
  auto cloud = make_wall(); // at kDepth = 2 m

  reusex::vision::ProjectLabelsOptions near_only;
  near_only.max_depth_m = kDepth - 0.5F;
  auto labels = project(*db, cloud, near_only);
  REQUIRE(labels->size() == cloud->size());
  for (const auto &pt : labels->points)
    REQUIRE(pt.label == reusex::core::kUnlabeled);

  // The default (7 m) reaches the wall.
  auto defaults = project(*db, cloud, reusex::vision::ProjectLabelsOptions{});
  REQUIRE(count_label(*defaults,
                      reusex::core::api_to_point_label(kPatchClass)) > 100);
}

TEST_CASE("Project_InvalidOptions_Throw", "[vision][project][options]") {
  auto db = make_project(make_label_image());
  auto cloud = make_wall();

  reusex::vision::ProjectLabelsOptions bad_depth;
  bad_depth.max_depth_m = 0.0F;
  REQUIRE_THROWS_AS(project(*db, cloud, bad_depth), std::invalid_argument);

  reusex::vision::ProjectLabelsOptions bad_tol;
  bad_tol.depth_tolerance_m = -0.1F;
  REQUIRE_THROWS_AS(project(*db, cloud, bad_tol), std::invalid_argument);
}

TEST_CASE("ProjectLabelsParameters_CarryOptionsAndLabelEncoding",
          "[vision][project][options]") {
  reusex::vision::ProjectLabelsOptions opt;
  opt.max_depth_m = 4.5F;
  opt.depth_tolerance_m = 0.25F;

  const auto j =
      nlohmann::json::parse(reusex::vision::project_labels_parameters(opt));

  REQUIRE(j.at("max_depth_m").get<double>() == 4.5);
  REQUIRE(j.at("depth_tolerance_m").get<double>() == 0.25);
  REQUIRE(j.at(reusex::core::kLabelEncodingKey).get<std::string>() ==
          reusex::core::kLabelEncodingApiPlusOne);
}

TEST_CASE("Project_DifferentThreadCounts_AreBitIdentical",
          "[vision][project][vote]") {
#ifdef _OPENMP
  cv::Mat mixed = uniform_label_image(4);
  mixed(cv::Range(0, kHeight / 2), cv::Range::all()).setTo(1);
  auto db = make_multi_frame_project(
      {make_label_image(), mixed, uniform_label_image(1), make_label_image()});
  auto cloud = make_wall();

  const int saved = omp_get_max_threads();
  omp_set_num_threads(1);
  auto serial = project(*db, cloud);
  omp_set_num_threads(4);
  auto parallel = project(*db, cloud);
  omp_set_num_threads(saved);

  REQUIRE(serial->size() == parallel->size());
  for (size_t i = 0; i < serial->size(); ++i)
    REQUIRE(serial->points[i].label == parallel->points[i].label);
#else
  SKIP("test binary built without OpenMP");
#endif
}
