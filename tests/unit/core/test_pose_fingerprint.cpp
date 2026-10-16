// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core/pose_fingerprint.hpp (reusex-rcv.1.15): detecting a stored cloud built
// from sensor poses the project no longer has (e.g. `rux optimize`/`rux
// register` ran after `rux create clouds`).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_fingerprint.hpp>
#include <reusex/core/validate.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

using reusex::Cloud;
using reusex::CloudN;
using reusex::ProjectDB;
using reusex::test_support::TempPath;
using namespace reusex::core;
using namespace reusex::test_support;

namespace {

void seed_frame(ProjectDB &db, int id, std::array<double, 16> pose) {
  db.save_sensor_frame(id, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
                       pose, make_intrinsics(8, 8, 4, 4, 8, 8), 0.0);
}

Cloud makeRgbCloud(size_t n) {
  Cloud c;
  c.width = static_cast<uint32_t>(n);
  c.height = 1;
  c.is_dense = false;
  c.points.resize(n);
  return c;
}

CloudN makeNormalCloud(size_t n) {
  CloudN c;
  c.width = static_cast<uint32_t>(n);
  c.height = 1;
  c.is_dense = false;
  c.points.resize(n);
  return c;
}

std::string pose_fingerprint_params(const ProjectDB &db) {
  return nlohmann::json{
      {std::string(kPoseFingerprintKey), compute_pose_fingerprint(db)}}
      .dump();
}

bool has_check(const std::vector<ValidationIssue> &issues,
               const std::string &check) {
  return std::any_of(issues.begin(), issues.end(),
                     [&](const auto &i) { return i.check == check; });
}

} // namespace

TEST_CASE("ComputePoseFingerprint_SamePoses_IsStable",
          "[core][pose_fingerprint]") {
  TempPath tmp("test_pose_fingerprint_stable");
  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(0, 0, 0));
  seed_frame(db, 2, translation_pose(1, 0, 0));

  const auto a = compute_pose_fingerprint(db);
  const auto b = compute_pose_fingerprint(db);
  CHECK(a == b);
  CHECK_FALSE(a.empty());
}

TEST_CASE("ComputePoseFingerprint_PoseChanges_FingerprintChanges",
          "[core][pose_fingerprint]") {
  TempPath tmp("test_pose_fingerprint_changes");
  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(0, 0, 0));
  seed_frame(db, 2, translation_pose(1, 0, 0));

  const auto before = compute_pose_fingerprint(db);
  db.update_sensor_frame_pose(2,
                              translation_pose(1, 0, 5)); // optimize/register
  const auto after = compute_pose_fingerprint(db);
  CHECK(before != after);
}

TEST_CASE("CloudPoseFreshness_NoCloud_IsUnknown", "[core][pose_fingerprint]") {
  TempPath tmp("test_pose_fingerprint_no_cloud");
  ProjectDB db(tmp.path);
  CHECK(cloud_pose_freshness(db, "cloud") == PoseFreshness::unknown);
}

TEST_CASE("CloudPoseFreshness_LegacyCloudWithoutStamp_IsUnknownNotStale",
          "[core][pose_fingerprint]") {
  TempPath tmp("test_pose_fingerprint_legacy");
  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(0, 0, 0));

  // Written with no "pose_fingerprint" key — what every cloud made before
  // this feature looks like.
  db.save_point_cloud("cloud", makeRgbCloud(3), "reconstruct",
                      R"({"resolution":0.05})");

  CHECK(cloud_pose_freshness(db, "cloud") == PoseFreshness::unknown);
  // "unknown" must never be reported as "stale" (AC #2): no stage-input
  // warning for a legacy cloud.
  std::vector<ValidationIssue> issues;
  check_stale_cloud_poses(db, issues);
  CHECK(issues.empty());
}

TEST_CASE("CloudPoseFreshness_FingerprintMatchesCurrentPoses_IsFresh",
          "[core][pose_fingerprint]") {
  TempPath tmp("test_pose_fingerprint_fresh");
  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(0, 0, 0));
  seed_frame(db, 2, translation_pose(1, 0, 0));

  db.save_point_cloud("cloud", makeRgbCloud(3), "reconstruct",
                      pose_fingerprint_params(db));

  CHECK(cloud_pose_freshness(db, "cloud") == PoseFreshness::fresh);
  std::vector<ValidationIssue> issues;
  check_stale_cloud_poses(db, issues);
  CHECK(issues.empty());
}

TEST_CASE("CloudPoseFreshness_PoseChangedSinceStamp_IsStale",
          "[core][pose_fingerprint]") {
  TempPath tmp("test_pose_fingerprint_stale");
  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(0, 0, 0));
  seed_frame(db, 2, translation_pose(1, 0, 0));

  db.save_point_cloud("cloud", makeRgbCloud(3), "reconstruct",
                      pose_fingerprint_params(db));

  // A pose refinement (`rux optimize`/`rux register`) runs after the cloud
  // was built.
  db.update_sensor_frame_pose(2, translation_pose(1, 0, 5));

  CHECK(cloud_pose_freshness(db, "cloud") == PoseFreshness::stale);

  std::vector<ValidationIssue> issues;
  check_stale_cloud_poses(db, issues);
  REQUIRE(has_check(issues, "stale_cloud_poses"));
  for (const auto &issue : issues) {
    if (issue.check != "stale_cloud_poses")
      continue;
    CHECK(issue.severity == ValidationSeverity::warning);
    CHECK(issue.message.find("cloud") != std::string::npos);
    CHECK(issue.hint.find("rux create clouds") != std::string::npos);
  }

  // Re-running `create clouds` (simulated: re-stamp with the current
  // fingerprint) clears the warning.
  db.save_point_cloud("cloud", makeRgbCloud(3), "reconstruct",
                      pose_fingerprint_params(db));
  issues.clear();
  check_stale_cloud_poses(db, issues);
  CHECK(issues.empty());
}

TEST_CASE("CheckStageInputs_StaleCloud_WarnsForDownstreamStage",
          "[core][pose_fingerprint][validate][stage]") {
  TempPath tmp("test_pose_fingerprint_stage_planes");
  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(0, 0, 0));
  seed_frame(db, 2, translation_pose(1, 0, 0));

  const auto params = pose_fingerprint_params(db);
  db.save_point_cloud("cloud", makeRgbCloud(4), "reconstruct", params);
  db.save_point_cloud("normals", makeNormalCloud(4), "reconstruct", params);

  // Fresh: `planes`'s stage-input check is clean.
  auto fresh_report = validate_stage(db, PipelineStage::planes);
  CHECK_FALSE(has_check(fresh_report.issues, "stale_cloud_poses"));

  // `rux optimize`/`rux register` moves a pose after the cloud was built.
  db.update_sensor_frame_pose(2, translation_pose(1, 0, 5));

  auto stale_report = validate_stage(db, PipelineStage::planes);
  REQUIRE(has_check(stale_report.issues, "stale_cloud_poses"));
  // A warning, not an error — the stage can still run, just on stale input.
  CHECK(stale_report.ok());
}

TEST_CASE("CheckStageInputs_StageWithoutCloudInput_NeverChecksPoseStaleness",
          "[core][pose_fingerprint][validate][stage]") {
  TempPath tmp("test_pose_fingerprint_stage_import");
  ProjectDB db(tmp.path); // empty project: `import` has no `cloud` input
  auto report = validate_stage(db, PipelineStage::import);
  CHECK_FALSE(has_check(report.issues, "stale_cloud_poses"));
}
