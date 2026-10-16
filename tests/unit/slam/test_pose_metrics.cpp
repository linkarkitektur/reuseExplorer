// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ATE / RPE scoring (reusex-rcv.1.6): compare_trajectories against synthetic
// trajectories with exactly known offsets, so the expected numbers are hand
// computable rather than merely plausible (docs/STANDARDS.md §7).

#include <reusex/core/trajectory.hpp>
#include <reusex/slam/pose_metrics.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <numbers>

using Catch::Matchers::WithinAbs;
using reusex::core::StampedPose;
using reusex::core::Trajectory;
using reusex::slam::compare_trajectories;
using reusex::slam::PoseMetrics;
using reusex::slam::PoseMetricsOptions;

namespace {

Eigen::Matrix4d rigid(double tx, double ty, double tz, double yaw_rad = 0.0) {
  Eigen::Matrix4d p = Eigen::Matrix4d::Identity();
  p.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p(0, 3) = tx;
  p(1, 3) = ty;
  p(2, 3) = tz;
  return p;
}

/// A straight line of `n` poses along x, 1 m apart, identity rotation,
/// timestamps 0..n-1.
Trajectory straight_line(int n) {
  Trajectory t;
  for (int i = 0; i < n; ++i)
    t.push_back(
        {i, static_cast<double>(i), rigid(static_cast<double>(i), 0, 0)});
  return t;
}

Trajectory translate_all(const Trajectory &in, double dx, double dy,
                         double dz) {
  Trajectory out = in;
  for (auto &sp : out)
    sp.pose.topRightCorner<3, 1>() += Eigen::Vector3d(dx, dy, dz);
  return out;
}

Trajectory scale_all(const Trajectory &in, double s) {
  Trajectory out = in;
  for (auto &sp : out)
    sp.pose.topRightCorner<3, 1>() *= s;
  return out;
}

} // namespace

TEST_CASE("compare_trajectories_IdenticalTrajectories_EverythingZero",
          "[slam][pose_metrics]") {
  const auto ref = straight_line(10);
  const auto m = compare_trajectories(ref, ref);
  CHECK(m.matched == 10);
  CHECK_THAT(m.ate_rmse_m, WithinAbs(0.0, 1e-12));
  CHECK_THAT(m.ate_median_m, WithinAbs(0.0, 1e-12));
  CHECK_THAT(m.ate_max_m, WithinAbs(0.0, 1e-12));
  CHECK_THAT(m.rot_rmse_deg, WithinAbs(0.0, 1e-9));
  CHECK_THAT(m.rpe_trans_m, WithinAbs(0.0, 1e-12));
  CHECK_THAT(m.rpe_trans_frames_m, WithinAbs(0.0, 1e-12));
}

TEST_CASE("compare_trajectories_UniformTranslationOffset_MatchesByHand",
          "[slam][pose_metrics]") {
  const auto ref = straight_line(9); // divides evenly into 3 segments
  const auto est = translate_all(ref, 0.3, 0.4, 0.0); // offset norm = 0.5

  PoseMetricsOptions opt;
  opt.segments = 3;
  const auto m = compare_trajectories(est, ref, opt);

  CHECK(m.matched == 9);
  CHECK_THAT(m.ate_rmse_m, WithinAbs(0.5, 1e-9));
  CHECK_THAT(m.ate_median_m, WithinAbs(0.5, 1e-9));
  CHECK_THAT(m.ate_max_m, WithinAbs(0.5, 1e-9));
  CHECK_THAT(m.rot_rmse_deg, WithinAbs(0.0, 1e-9));
  // The error is the same everywhere, so every segment sees it too.
  REQUIRE(m.segment_ate_m.size() == 3);
  for (double s : m.segment_ate_m)
    CHECK_THAT(s, WithinAbs(0.5, 1e-9));

  // RPE cancels a shared global offset: relative motion is unaffected.
  CHECK_THAT(m.rpe_trans_m, WithinAbs(0.0, 1e-9));
  CHECK_THAT(m.rpe_trans_frames_m, WithinAbs(0.0, 1e-9));
}

TEST_CASE("compare_trajectories_Se3Alignment_RecoversRigidOffset",
          "[slam][pose_metrics]") {
  const auto ref = straight_line(12);
  const auto est = translate_all(ref, 1.0, -2.0, 0.5);

  PoseMetricsOptions unaligned;
  const auto before = compare_trajectories(est, ref, unaligned);
  CHECK(before.ate_rmse_m > 1.0); // uncorrected, the raw offset dominates

  PoseMetricsOptions aligned;
  aligned.align = PoseMetricsOptions::Align::se3;
  const auto after = compare_trajectories(est, ref, aligned);
  CHECK_THAT(after.ate_rmse_m, WithinAbs(0.0, 1e-6));
  CHECK_THAT(after.scale, WithinAbs(1.0, 1e-12)); // se3 never rescales
}

TEST_CASE("compare_trajectories_Sim3Alignment_RecoversScale",
          "[slam][pose_metrics]") {
  const auto ref = straight_line(12);
  // est is ref shrunk by 1/2 about the origin, so the alignment that maps
  // est onto ref must recover scale 2.0.
  const auto est = scale_all(ref, 0.5);

  PoseMetricsOptions opt;
  opt.align = PoseMetricsOptions::Align::sim3;
  const auto m = compare_trajectories(est, ref, opt);

  CHECK_THAT(m.scale, WithinAbs(2.0, 1e-6));
  CHECK_THAT(m.ate_rmse_m, WithinAbs(0.0, 1e-6));
}

TEST_CASE("compare_trajectories_Rpe_MatchesHandComputedRotationError",
          "[slam][pose_metrics]") {
  // Two poses, 1 m apart, identity rotation on the reference; the estimate's
  // SECOND pose is rotated an extra 5 deg about Z relative to the first, so
  // the one relative-pose error IS that 5 deg rotation, exactly.
  Trajectory ref{{0, 0.0, rigid(0, 0, 0)}, {1, 1.0, rigid(1, 0, 0)}};
  const double five_deg = 5.0 * std::numbers::pi / 180.0;
  Eigen::Matrix4d est1 = rigid(1, 0, 0);
  est1.topLeftCorner<3, 3>() =
      est1.topLeftCorner<3, 3>() *
      Eigen::AngleAxisd(five_deg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Trajectory est{{0, 0.0, rigid(0, 0, 0)}, {1, 1.0, est1}};

  PoseMetricsOptions opt;
  opt.rpe_frames = 1;
  opt.rpe_distance_m = 0.5; // reached by the only step (1 m)
  opt.segments = 1;
  const auto m = compare_trajectories(est, ref, opt);

  CHECK(m.matched == 2);
  CHECK_THAT(m.rpe_trans_frames_m, WithinAbs(0.0, 1e-9));
  CHECK_THAT(m.rpe_rot_frames_deg, WithinAbs(5.0, 1e-6));
  CHECK_THAT(m.rpe_trans_m, WithinAbs(0.0, 1e-9));
  CHECK_THAT(m.rpe_rot_deg, WithinAbs(5.0, 1e-6));
}

TEST_CASE("compare_trajectories_FewerThanTwoMatches_EverythingZeroNoThrow",
          "[slam][pose_metrics]") {
  Trajectory ref{{0, 0.0, rigid(0, 0, 0)}};
  Trajectory est{{0, 0.0, rigid(5, 0, 0)}}; // same timestamp, one pair only
  const auto m = compare_trajectories(est, ref);
  CHECK(m.matched == 1);
  CHECK_THAT(m.ate_rmse_m, WithinAbs(0.0, 1e-12));
}

TEST_CASE("compare_trajectories_DisjointTimestamps_ZeroMatches",
          "[slam][pose_metrics]") {
  Trajectory ref{{0, 0.0, rigid(0, 0, 0)}, {1, 1.0, rigid(1, 0, 0)}};
  Trajectory est{{0, 100.0, rigid(0, 0, 0)}, {1, 101.0, rigid(1, 0, 0)}};
  const auto m = compare_trajectories(est, ref);
  CHECK(m.matched == 0);
  CHECK_THAT(m.ate_rmse_m, WithinAbs(0.0, 1e-12));
}

TEST_CASE("compare_trajectories_RejectsInvalidOptions",
          "[slam][pose_metrics]") {
  const auto ref = straight_line(5);
  PoseMetricsOptions bad_distance;
  bad_distance.rpe_distance_m = 0.0;
  REQUIRE_THROWS_AS(compare_trajectories(ref, ref, bad_distance),
                    std::invalid_argument);

  PoseMetricsOptions bad_frames;
  bad_frames.rpe_frames = 0;
  REQUIRE_THROWS_AS(compare_trajectories(ref, ref, bad_frames),
                    std::invalid_argument);

  PoseMetricsOptions bad_segments;
  bad_segments.segments = 0;
  REQUIRE_THROWS_AS(compare_trajectories(ref, ref, bad_segments),
                    std::invalid_argument);
}
