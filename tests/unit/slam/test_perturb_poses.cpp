// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for synthetic trajectory drift (issue #338).
//
// This generator exists to make a benchmark trustworthy, so the properties it
// must have are exactly the ones a reader of the resulting table will assume
// without checking (docs/STANDARDS.md §7):
//
//   1. it is a pure, seeded function — same input, same drift, bit for bit,
//      and drift_scale 0 changes nothing at all (STANDARDS §6);
//   2. frame 0 stays exactly on the seed, because the pose graph's only
//      absolute anchor is its prior on frame 0 — drift that moved the whole
//      trajectory rigidly would be unrecoverable by construction and the
//      benchmark would be measuring an impossibility;
//   3. the requested drift magnitude is actually realised, since every row of
//      the resulting matrix is labelled with it;
//   4. the error accumulates SMOOTHLY along the relative chain rather than
//      jittering each pose independently — that is what makes it drift, and
//      §8.3 showed real drift on these captures has exactly that character
//      (no single bad edge for a robust kernel to find).

#include <reusex/slam/perturb_poses.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/slam/pose_graph.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using reusex::geometry::median_pair_disagreement;
using reusex::geometry::perturb_trajectory;
using reusex::geometry::PoseDriftOptions;
using reusex::geometry::PoseDriftResult;

namespace {

/// A plausible hand-held room capture: 240 frames walking the perimeter of a
/// 6 x 4 m room, camera yawing to face outward. Extent = sqrt(36 + 16) = 7.21
/// m.
std::vector<Eigen::Matrix4d> room_trajectory(int n = 240) {
  std::vector<Eigen::Matrix4d> poses;
  poses.reserve(n);
  for (int i = 0; i < n; ++i) {
    const double u = static_cast<double>(i) / n; // 0..1 around the loop
    const double perimeter = 2.0 * (6.0 + 4.0);
    double s = u * perimeter;
    double x = 0.0, y = 0.0;
    if (s < 6.0) {
      x = s;
    } else if (s < 10.0) {
      x = 6.0;
      y = s - 6.0;
    } else if (s < 16.0) {
      x = 6.0 - (s - 10.0);
      y = 4.0;
    } else {
      y = 4.0 - (s - 16.0);
    }
    const double yaw = 2.0 * M_PI * u;
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
    P.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    P(0, 3) = x;
    P(1, 3) = y;
    P(2, 3) = 1.4; // eye height
    poses.push_back(P);
  }
  return poses;
}

bool poses_bit_identical(const std::vector<Eigen::Matrix4d> &a,
                         const std::vector<Eigen::Matrix4d> &b) {
  if (a.size() != b.size())
    return false;
  for (size_t k = 0; k < a.size(); ++k)
    if (a[k] != b[k]) // exact comparison is the point
      return false;
  return true;
}

} // namespace

TEST_CASE("perturb_trajectory_FewerThanTwoPoses_IsNoOp", "[slam][drift]") {
  std::vector<Eigen::Matrix4d> one = {Eigen::Matrix4d::Identity()};
  const auto copy = one;
  const PoseDriftResult r = perturb_trajectory(one, PoseDriftOptions{});
  REQUIRE(r.frames == 1);
  REQUIRE(poses_bit_identical(one, copy));
}

TEST_CASE("perturb_trajectory_ZeroDriftScale_LeavesPosesBitIdentical",
          "[slam][drift][determinism]") {
  // The no-op guard the benchmark leans on: the drifted-variant harness must be
  // provably incapable of perturbing anything when asked not to.
  auto poses = room_trajectory();
  const auto seed_poses = poses;
  PoseDriftOptions opt;
  opt.drift_scale = 0.0;

  const PoseDriftResult r = perturb_trajectory(poses, opt);

  REQUIRE(poses_bit_identical(poses, seed_poses));
  REQUIRE(r.max_position_error == 0.0);
  REQUIRE(r.drift_ratio == 0.0);
  REQUIRE(r.calibrated);
}

TEST_CASE("perturb_trajectory_SameSeed_ReproducesDriftBitForBit",
          "[slam][drift][determinism]") {
  PoseDriftOptions opt;
  opt.seed = 7;

  auto a = room_trajectory();
  auto b = room_trajectory();
  const PoseDriftResult ra = perturb_trajectory(a, opt);
  const PoseDriftResult rb = perturb_trajectory(b, opt);

  REQUIRE(poses_bit_identical(a, b));
  REQUIRE(ra.amplitude == rb.amplitude);
  REQUIRE(ra.drift_ratio == rb.drift_ratio);
}

TEST_CASE("perturb_trajectory_DifferentSeed_ProducesDifferentRealisation",
          "[slam][drift]") {
  // Seeds are the ensemble knob: a conclusion drawn from one realisation of the
  // drift is not a conclusion, so different seeds must actually differ.
  PoseDriftOptions opt_a;
  opt_a.seed = 1;
  PoseDriftOptions opt_b;
  opt_b.seed = 2;

  auto a = room_trajectory();
  auto b = room_trajectory();
  perturb_trajectory(a, opt_a);
  perturb_trajectory(b, opt_b);

  REQUIRE_FALSE(poses_bit_identical(a, b));
}

TEST_CASE("perturb_trajectory_AnySeed_LeavesFirstFrameExactlyOnSeed",
          "[slam][drift]") {
  for (unsigned seed : {1u, 2u, 42u}) {
    auto poses = room_trajectory();
    const Eigen::Matrix4d first = poses.front();
    PoseDriftOptions opt;
    opt.seed = seed;
    perturb_trajectory(poses, opt);
    REQUIRE(poses.front() == first);
  }
}

TEST_CASE("perturb_trajectory_RequestedRatio_IsRealisedWithinTolerance",
          "[slam][drift]") {
  for (double scale : {0.25, 0.5, 1.0}) {
    auto poses = room_trajectory();
    PoseDriftOptions opt;
    opt.drift_scale = scale;
    const PoseDriftResult r = perturb_trajectory(poses, opt);

    INFO("drift_scale = " << scale);
    REQUIRE(r.calibrated);
    REQUIRE_THAT(r.drift_ratio,
                 WithinRel(opt.target_drift_ratio * scale, 1e-3));
  }
}

TEST_CASE("perturb_trajectory_LargerDriftScale_ProducesLargerPoseError",
          "[slam][drift]") {
  auto mild = room_trajectory();
  auto heavy = room_trajectory();
  PoseDriftOptions opt_mild;
  opt_mild.drift_scale = 0.25;
  PoseDriftOptions opt_heavy;
  opt_heavy.drift_scale = 1.0;

  const PoseDriftResult rm = perturb_trajectory(mild, opt_mild);
  const PoseDriftResult rh = perturb_trajectory(heavy, opt_heavy);

  REQUIRE(rh.max_position_error > rm.max_position_error);
  REQUIRE(rh.amplitude > rm.amplitude);
}

TEST_CASE("perturb_trajectory_HeavyDrift_AccumulatesSmoothlyAlongTheChain",
          "[slam][drift]") {
  // The defining property: the injected error lives in the RELATIVE chain and
  // integrates, so no single step is grossly wrong even when the trajectory
  // ends up metres away from the truth. If this failed, the synthetic drift
  // would be a bag of outliers — the one thing §8.3 measured real drift NOT to
  // be, and a robust kernel would trivially remove it.
  auto poses = room_trajectory();
  const auto seed_poses = poses;
  PoseDriftOptions opt;
  const PoseDriftResult r = perturb_trajectory(poses, opt);

  double worst_step_error = 0.0;
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const Eigen::Matrix4d rel_seed =
        seed_poses[i].inverse() * seed_poses[i + 1];
    const Eigen::Matrix4d rel_drift = poses[i].inverse() * poses[i + 1];
    worst_step_error = std::max(
        worst_step_error,
        (rel_seed.block<3, 1>(0, 3) - rel_drift.block<3, 1>(0, 3)).norm());
  }

  INFO("worst per-step error " << worst_step_error << " m vs max pose error "
                               << r.max_position_error << " m");
  REQUIRE(r.max_position_error > 1.0); // heavy drift really was injected
  REQUIRE(worst_step_error < 0.05 * r.max_position_error);
}

TEST_CASE("perturb_trajectory_AnyDrift_ReportsSeedTrajectoryGeometry",
          "[slam][drift]") {
  auto poses = room_trajectory();
  const PoseDriftResult r = perturb_trajectory(poses, PoseDriftOptions{});
  // 6 x 4 m room: bbox diagonal sqrt(52), perimeter 20 m.
  REQUIRE_THAT(r.trajectory_extent, WithinRel(std::sqrt(52.0), 1e-6));
  REQUIRE_THAT(r.path_length, WithinRel(20.0, 0.05));
}

TEST_CASE("median_pair_disagreement_IdenticalTrajectories_ReturnsZero",
          "[slam][drift]") {
  const auto poses = room_trajectory();
  REQUIRE(median_pair_disagreement(poses, poses, 50) == 0.0);
}

TEST_CASE("median_pair_disagreement_RigidlyTransformedTrajectory_ReturnsZero",
          "[slam][drift]") {
  // The statistic compares RELATIVE poses, so it is invariant to where the
  // trajectory sits in the world. That invariance is what makes it the right
  // measure of drift: a rigid offset is a gauge choice, not an error.
  const auto poses = room_trajectory();
  Eigen::Matrix4d G = Eigen::Matrix4d::Identity();
  G.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized())
          .toRotationMatrix();
  G.block<3, 1>(0, 3) = Eigen::Vector3d(11.0, -5.0, 2.0);

  std::vector<Eigen::Matrix4d> moved;
  moved.reserve(poses.size());
  for (const auto &P : poses)
    moved.push_back(G * P);

  REQUIRE_THAT(median_pair_disagreement(poses, moved, 50),
               WithinAbs(0.0, 1e-9));
}

TEST_CASE("median_pair_disagreement_ShortSequence_StillSamplesPairs",
          "[slam][drift]") {
  // A 40-frame sequence has no pairs 50 apart; the gap must clamp rather than
  // report a silent 0, which would let the calibration believe any target had
  // been met (STANDARDS §5).
  auto poses = room_trajectory(40);
  auto drifted = poses;
  PoseDriftOptions opt;
  opt.drift_scale = 0.5;
  perturb_trajectory(drifted, opt);

  REQUIRE(median_pair_disagreement(poses, drifted, 50) > 0.0);
}

TEST_CASE("perturb_sensor_poses_WithStoredGraph_ResnapshotsAndRederives",
          "[slam][drift][pose_graph]") {
  namespace pg = reusex::core::pose_graph;
  reusex::test_support::TempPath tmp("test_perturb_graph");
  reusex::ProjectDB db(tmp.path);
  const auto traj = room_trajectory(60);
  for (int k = 0; k < 60; ++k)
    db.save_sensor_frame(
        k + 1, reusex::test_support::make_color(8, 8), cv::Mat(), cv::Mat(),
        pg::row_major_from_matrix(traj[static_cast<size_t>(k)]),
        reusex::test_support::make_intrinsics(8, 8, 4, 4, 8, 8), k);
  reusex::slam::init_pose_graph(db);

  reusex::geometry::perturb_sensor_poses(db, PoseDriftOptions{},
                                         /*dry_run=*/false);

  bool any_changed = false;
  for (int k = 1; k <= 60; ++k) {
    REQUIRE(db.initial_pose(k).has_value());
    CHECK(*db.initial_pose(k) == db.sensor_frame_pose(k));
  }
  for (int k = 1; k < 60; ++k) {
    const auto e = db.pose_graph_edge(k, k + 1, "odometry");
    REQUIRE(e.has_value());
    const Eigen::Matrix4d drifted =
        pg::matrix_from_row_major(db.sensor_frame_pose(k)).inverse() *
        pg::matrix_from_row_major(db.sensor_frame_pose(k + 1));
    CHECK((pg::matrix_from_row_major(*e->transform) - drifted).norm() < 1e-9);
    const Eigen::Matrix4d original =
        traj[static_cast<size_t>(k - 1)].inverse() *
        traj[static_cast<size_t>(k)];
    any_changed = any_changed || (drifted - original).norm() > 1e-6;
  }
  CHECK(any_changed);
}

TEST_CASE("perturb_sensor_poses_WithoutGraph_OnlySnapshots",
          "[slam][drift][pose_graph]") {
  reusex::test_support::TempPath tmp("test_perturb_nograph");
  reusex::ProjectDB db(tmp.path);
  const auto traj = room_trajectory(60);
  for (int k = 0; k < 60; ++k)
    db.save_sensor_frame(
        k + 1, reusex::test_support::make_color(8, 8), cv::Mat(), cv::Mat(),
        reusex::core::pose_graph::row_major_from_matrix(
            traj[static_cast<size_t>(k)]),
        reusex::test_support::make_intrinsics(8, 8, 4, 4, 8, 8), k);
  reusex::geometry::perturb_sensor_poses(db, PoseDriftOptions{}, false);
  CHECK(db.list_pose_graph_edges().empty());
  for (int k = 1; k <= 60; ++k)
    CHECK(*db.initial_pose(k) == db.sensor_frame_pose(k));
}

// ---------------------------------------------------------------------------
// Yaw-drift mode (reusex-xka, reusex-rcv.1.7). The default drift is
// tilt-dominated on single-room scans (41069048 d0.25: median tilt 5.6 deg vs
// yaw 1.1 deg), which is not how the houses fail: they drift in heading across
// rooms they never revisit, while gravity keeps roll and pitch honest. The yaw
// mode must inject drift ONLY about the world up axis and only horizontally,
// so the camera's tilt and height stay exactly on the seed.
// ---------------------------------------------------------------------------

namespace {

/// room_trajectory() with real OPTICAL cameras (x right, y down, z forward),
/// looking horizontally: camera y maps to world -z. The plain fixture's camera
/// z IS world up, which would make "tilt" and "yaw" indistinguishable.
std::vector<Eigen::Matrix4d> optical_room_trajectory() {
  auto poses = room_trajectory();
  const Eigen::Matrix3d level =
      Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  for (auto &P : poses)
    P.block<3, 3>(0, 0) = P.block<3, 3>(0, 0) * level;
  return poses;
}

/// The world up axis as seen in the camera frame: a pose's tilt.
Eigen::Vector3d up_in_camera(const Eigen::Matrix4d &P) {
  return P.block<3, 3>(0, 0).transpose() * Eigen::Vector3d::UnitZ();
}

} // namespace

TEST_CASE("perturb_trajectory_YawMode_KeepsTiltAndHeightOnSeed",
          "[slam][drift]") {
  auto poses = optical_room_trajectory();
  const auto seed_poses = poses;
  PoseDriftOptions opt;
  opt.mode = reusex::geometry::DriftMode::yaw;
  const PoseDriftResult r = perturb_trajectory(poses, opt);

  REQUIRE(r.calibrated);
  REQUIRE(r.max_position_error > 0.1); // drift really was injected
  for (size_t k = 0; k < poses.size(); ++k) {
    INFO("frame " << k);
    REQUIRE((up_in_camera(poses[k]) - up_in_camera(seed_poses[k])).norm() <
            1e-9);
    REQUIRE_THAT(poses[k](2, 3), WithinAbs(seed_poses[k](2, 3), 1e-9));
  }
}

TEST_CASE("perturb_trajectory_YawMode_RealisesRequestedRatio",
          "[slam][drift]") {
  for (double scale : {0.25, 1.0}) {
    auto poses = optical_room_trajectory();
    PoseDriftOptions opt;
    opt.mode = reusex::geometry::DriftMode::yaw;
    opt.drift_scale = scale;
    const PoseDriftResult r = perturb_trajectory(poses, opt);
    INFO("drift_scale = " << scale);
    REQUIRE(r.calibrated);
    REQUIRE_THAT(r.drift_ratio,
                 WithinRel(opt.target_drift_ratio * scale, 1e-3));
  }
}

TEST_CASE("perturb_trajectory_FullMode_TiltsTheCameras", "[slam][drift]") {
  // The contrast that makes the yaw mode worth having: the default model
  // perturbs all three rotation axes, so the cameras' tilt drifts too.
  auto poses = optical_room_trajectory();
  const auto seed_poses = poses;
  perturb_trajectory(poses, PoseDriftOptions{});
  double worst_tilt = 0.0;
  for (size_t k = 0; k < poses.size(); ++k)
    worst_tilt =
        std::max(worst_tilt,
                 (up_in_camera(poses[k]) - up_in_camera(seed_poses[k])).norm());
  REQUIRE(worst_tilt > 0.01);
}
