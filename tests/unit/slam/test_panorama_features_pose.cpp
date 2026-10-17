// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Degenerate-pose guard in `pano_detail::extract_frame_features()` (#336).
//
// This helper is the single source of the WORLD 3D points that both 360
// consumers resect a panorama against — `rux align 360`
// (slam/PanoramaAlignment.cpp) and `rux optimize --use-panoramas`
// (slam/PanoramaLoopEdges.cpp). It lifts keypoints to the frame's optical
// frame and stores `T_world_cam` from the frame's pose;
// `FrameFeatures::world()` composes the two on demand.
//
// A frame with no usable stored pose used to get `sensor_frame_pose()`'s
// identity fallback here, so its whole point set was placed at the world
// origin and handed to the resection as correspondences that are well-formed
// and geometrically wrong. It now contributes nothing, signalled through the
// empty `descriptors` every caller already checks.
//
// Includes the module-private `slam/panorama_features.hpp` directly, the same
// arrangement tests/CMakeLists.txt already provides for gsplat's
// `view_sampling.hpp` (STANDARDS §2: module-private headers are testable, not
// public).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>

#include "slam/panorama_features.hpp"

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <array>
#include <optional>

using namespace reusex;
using namespace reusex::test_support;
using Catch::Matchers::WithinAbs;

namespace {

// Big enough, and textured enough, for ORB to find corners deterministically:
// a checkerboard has a corner at every square intersection.
constexpr int kW = 160;
constexpr int kH = 120;
constexpr double kDepthM = 2.0;
constexpr float kMinDepth = 0.1F;
constexpr float kMaxDepth = 6.0F;
constexpr double kFarX = 15.0;

void seed_frame(ProjectDB &db, int node_id,
                const std::array<double, 16> &pose) {
  db.save_sensor_frame(node_id, make_checker(kW, kH),
                       make_depth(kW, kH, kDepthM), cv::Mat(), pose,
                       make_intrinsics(80.0, 80.0, kW / 2.0, kH / 2.0, kW, kH));
}

cv::Ptr<cv::ORB> orb() { return cv::ORB::create(500); }

} // namespace

TEST_CASE("ExtractFrameFeatures_PosedFrame_SeedsWorldPoseFromStoredTransform",
          "[slam][panorama][pose]") {
  // Positive control: features are found and the stored pose reaches
  // T_world_cam unmodified, so the guard test's empty result is attributable
  // to the pose and not to a textureless fixture.
  TempPath tmp("test_pano_features_posed");

  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(kFarX, 0.0, 0.0));

  const auto features = geometry::pano_detail::extract_frame_features(
      db, 1, orb(), kMinDepth, kMaxDepth);

  REQUIRE_FALSE(features.descriptors.empty());
  CHECK(features.node_id == 1);
  CHECK_THAT(features.T_world_cam(0, 3), WithinAbs(kFarX, 1e-9));
}

TEST_CASE("ExtractFrameFeatures_FrameWithUnusablePose_ContributesNoDescriptors",
          "[slam][panorama][pose]") {
  TempPath tmp("test_pano_features_poseless");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 1, translation_pose(kFarX, 0.0, 0.0));
  }

  SECTION("NULL transform") { clear_sensor_frame_pose(tmp.path, 1); }
  SECTION("all-zero transform") {
    set_raw_sensor_frame_pose(tmp.path, 1, zero_pose());
  }
  SECTION("NaN translation") {
    set_raw_sensor_frame_pose(tmp.path, 1, nan_translation_pose());
  }

  ProjectDB db(tmp.path);
  const auto features = geometry::pano_detail::extract_frame_features(
      db, 1, orb(), kMinDepth, kMaxDepth);

  // The old code returned a full descriptor set anchored at the world origin.
  CHECK(features.descriptors.empty());
  CHECK(features.keypoints.empty());
}

TEST_CASE(
    "ExtractFrameFeatures_RequirePoseFalse_PoselessFrameStillYieldsFeatures",
    "[slam][panorama][pose]") {
  // reusex-467: LoopClosure's detect_loop_edges() takes seed_poses as an
  // explicit argument and never reads T_world_cam, so it only needs
  // colour+depth+intrinsics, not a *stored* pose. require_pose=false must
  // keep extracting features for a frame whose stored transform is missing,
  // instead of the gated empty result the other two (world-pose) callers get.
  TempPath tmp("test_pano_features_poseless_looseok");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 1, translation_pose(kFarX, 0.0, 0.0));
  }
  clear_sensor_frame_pose(tmp.path, 1);

  ProjectDB db(tmp.path);
  REQUIRE_FALSE(db.has_sensor_frame_pose(1));

  const auto features = geometry::pano_detail::extract_frame_features(
      db, 1, orb(), kMinDepth, kMaxDepth, /*region=*/std::nullopt,
      /*require_pose=*/false);

  CHECK_FALSE(features.descriptors.empty());
  CHECK_FALSE(features.keypoints.empty());
  // T_world_cam is left at Identity: a require_pose=false caller must not
  // read it (LoopClosure never does).
  CHECK(features.T_world_cam.isApprox(Eigen::Matrix4d::Identity(), 1e-12));
}

TEST_CASE("ExtractFrameFeatures_FrameWithStoredIdentityPose_ReturnsDescriptors",
          "[slam][panorama][pose]") {
  // A stored identity pose is a legitimate placement at the origin, not the
  // absence of one, and must survive the guard.
  TempPath tmp("test_pano_features_identity");

  ProjectDB db(tmp.path);
  seed_frame(db, 1, identity16());

  const auto features = geometry::pano_detail::extract_frame_features(
      db, 1, orb(), kMinDepth, kMaxDepth);

  CHECK_FALSE(features.descriptors.empty());
  CHECK(features.T_world_cam.isApprox(Eigen::Matrix4d::Identity(), 1e-12));
}
