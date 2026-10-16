// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// reusex-rcv.1.15: `reconstruct_point_clouds()` stamps "cloud" (and
// "normals") with a pose fingerprint so a later pose change (`rux optimize`,
// `rux register`) can be detected without re-reading depth data.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_fingerprint.hpp>
#include <reusex/segmentation/reconstruct.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>

using namespace reusex;
using namespace reusex::test_support;

namespace {

constexpr int kW = 64;
constexpr int kH = 48;

geometry::ReconstructionParams test_params() {
  geometry::ReconstructionParams p;
  p.resolution = 0.05f;
  p.min_distance = 0.0f;
  p.max_distance = 4.0f;
  p.sampling_factor = 1;
  p.confidence_threshold = 2;
  return p;
}

void seed_frame(ProjectDB &db, int node_id,
                const std::array<double, 16> &pose) {
  db.save_sensor_frame(node_id, make_color(kW, kH), make_depth(kW, kH, 1.5),
                       cv::Mat(), pose,
                       make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH));
}

} // namespace

TEST_CASE("ReconstructPointClouds_FreshRun_StampsMatchingFingerprint",
          "[segmentation][reconstruct][pose_fingerprint]") {
  TempPath tmp("test_reconstruct_pose_fingerprint_fresh");
  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(0.0, 0.0, 0.0));
  seed_frame(db, 2, translation_pose(1.0, 0.0, 0.0));

  geometry::reconstruct_point_clouds(db, test_params());

  REQUIRE(db.has_point_cloud("cloud"));
  CHECK(core::cloud_pose_freshness(db, "cloud") == core::PoseFreshness::fresh);
}

TEST_CASE("ReconstructPointClouds_PoseChangedAfterward_FreshnessGoesStale",
          "[segmentation][reconstruct][pose_fingerprint]") {
  TempPath tmp("test_reconstruct_pose_fingerprint_stale");
  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(0.0, 0.0, 0.0));
  seed_frame(db, 2, translation_pose(1.0, 0.0, 0.0));

  geometry::reconstruct_point_clouds(db, test_params());
  REQUIRE(core::cloud_pose_freshness(db, "cloud") ==
          core::PoseFreshness::fresh);

  // A pose-graph solve (`rux optimize`/`rux register`) moves a frame after
  // the cloud was built — the cloud is now stale against the stored poses.
  db.update_sensor_frame_pose(2, translation_pose(1.0, 0.0, 0.3));

  CHECK(core::cloud_pose_freshness(db, "cloud") == core::PoseFreshness::stale);

  // Re-running `create clouds` (reconstruct) clears it.
  geometry::reconstruct_point_clouds(db, test_params());
  CHECK(core::cloud_pose_freshness(db, "cloud") == core::PoseFreshness::fresh);
}
