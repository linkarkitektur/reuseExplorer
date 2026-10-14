// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the scale-relative loop-edge seed-disagreement gate (#339).
//
// The gate decides whether a loop edge carries drift-correction information or
// merely restates the seed. Expressed in absolute metres it does not transfer
// between capture scales: the shipped 0.50 m value was tuned on an 18 m scan
// with ~16 m of drift, and applied unchanged to a 1.8 m ARKitScenes room scan
// it stops selecting informative edges and starts selecting the MOST
// disagreeing — i.e. the wrong — ones, measured at F@50mm 0.29
// (docs/research/registration-improvements.md §9.4).
//
// What these tests pin is therefore the arithmetic that the measured failure
// blamed, and the two defaults that must not move without a re-measurement:
// the office scan's historical gate must be reproduced exactly, and a short
// trajectory must scale down instead of inheriting it.

#include <reusex/slam/LoopClosure.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/loop_candidates.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Core>

#include <vector>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using reusex::geometry::LoopClosureOptions;
using reusex::geometry::seed_disagreement_gate;
using reusex::geometry::trajectory_extent;
using reusex::slam::LoopCandidateOptions;

namespace {

/// Pose whose camera centre is (x, y, z); rotation is irrelevant to extent.
Eigen::Matrix4d pose_at(double x, double y, double z) {
  Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
  P(0, 3) = x;
  P(1, 3) = y;
  P(2, 3) = z;
  return P;
}

/// Measured reference scales, registration-improvements.md §9.5.
constexpr double kOfficeExtent = 18.01;     // m
constexpr double kArkitScenesExtent = 1.82; // m, scan 41069048

} // namespace

TEST_CASE("trajectory_extent_FewerThanTwoPoses_ReturnsZero",
          "[slam][loop][gate]") {
  REQUIRE(trajectory_extent({}) == 0.0);
  REQUIRE(trajectory_extent({pose_at(3.0, 4.0, 0.0)}) == 0.0);
}

TEST_CASE("trajectory_extent_TwoPoses_ReturnsSeparationOfCameraCentres",
          "[slam][loop][gate]") {
  const std::vector<Eigen::Matrix4d> poses = {pose_at(0.0, 0.0, 0.0),
                                              pose_at(3.0, 4.0, 0.0)};
  REQUIRE_THAT(trajectory_extent(poses), WithinRel(5.0, 1e-12));
}

TEST_CASE("trajectory_extent_BoxTrajectory_ReturnsBoundingBoxDiagonal",
          "[slam][loop][gate]") {
  // Eight corners of a 2 x 3 x 6 box: diagonal = sqrt(4 + 9 + 36) = 7.
  std::vector<Eigen::Matrix4d> poses;
  for (double x : {-1.0, 1.0})
    for (double y : {0.0, 3.0})
      for (double z : {10.0, 16.0})
        poses.push_back(pose_at(x, y, z));
  REQUIRE_THAT(trajectory_extent(poses), WithinRel(7.0, 1e-12));
}

TEST_CASE("trajectory_extent_PathWalkedTwice_MeasuresPlacesNotPathLength",
          "[slam][loop][gate]") {
  // A 5 m corridor walked out and back is a 5 m capture, not a 10 m one — the
  // gate must not double because the operator retraced their steps. This is
  // why extent is the bbox diagonal and not the cumulative path length.
  std::vector<Eigen::Matrix4d> out_and_back;
  for (double x : {0.0, 2.5, 5.0, 2.5, 0.0})
    out_and_back.push_back(pose_at(x, 0.0, 0.0));
  REQUIRE_THAT(trajectory_extent(out_and_back), WithinRel(5.0, 1e-12));
}

TEST_CASE("trajectory_extent_ReorderedPoses_IsOrderIndependent",
          "[slam][loop][gate][determinism]") {
  const std::vector<Eigen::Matrix4d> forward = {
      pose_at(0.0, 0.0, 0.0), pose_at(1.0, 2.0, 3.0), pose_at(-4.0, 1.0, 0.5)};
  const std::vector<Eigen::Matrix4d> shuffled = {forward[2], forward[0],
                                                 forward[1]};
  REQUIRE(trajectory_extent(forward) == trajectory_extent(shuffled));
}

TEST_CASE("seed_disagreement_gate_LongTrajectory_UsesRelativeTerm",
          "[slam][loop][gate]") {
  // fraction * extent = 0.05 * 20 = 1.0 m, above the 0.10 m floor.
  REQUIRE_THAT(seed_disagreement_gate(20.0, 0.05, 0.10), WithinRel(1.0, 1e-12));
}

TEST_CASE("seed_disagreement_gate_ShortTrajectory_FallsBackToAbsoluteFloor",
          "[slam][loop][gate]") {
  // fraction * extent = 0.05 * 1.0 = 0.05 m, below the 0.10 m floor. The floor
  // is a matcher/depth-noise property and does NOT shrink with the room, so it
  // wins — which is what keeps loop closure a no-op on a small well-posed scan
  // rather than injecting noise.
  REQUIRE_THAT(seed_disagreement_gate(1.0, 0.05, 0.10), WithinRel(0.10, 1e-12));
}

TEST_CASE("seed_disagreement_gate_ZeroFractionAndFloor_DisablesGate",
          "[slam][loop][gate]") {
  REQUIRE(seed_disagreement_gate(18.01, 0.0, 0.0) == 0.0);
}

TEST_CASE("seed_disagreement_gate_NegativeInputs_ClampToZero",
          "[slam][loop][gate]") {
  REQUIRE(seed_disagreement_gate(18.01, -0.5, -1.0) == 0.0);
  REQUIRE_THAT(seed_disagreement_gate(18.01, -0.5, 0.25),
               WithinRel(0.25, 1e-12));
}

TEST_CASE("seed_disagreement_gate_ZeroExtent_UsesFloorAlone",
          "[slam][loop][gate]") {
  // A single-pose (or degenerate) trajectory has no scale to be relative to;
  // the gate must not collapse to "keep every edge".
  REQUIRE_THAT(seed_disagreement_gate(0.0, 0.0278, 0.10),
               WithinRel(0.10, 1e-12));
}

TEST_CASE("LoopClosureOptions_OfficeScaleTrajectory_ReproducesShipped100mmGate",
          "[slam][loop][gate][regression]") {
  // The internal ORB path's 0.10 m gate is the honka/office-calibrated
  // depth-noise floor. The default fraction is chosen so that at the office
  // scan's own extent the floor still wins — i.e. the relative term only ever
  // STRENGTHENS this gate, on captures longer than ~18 m, and every measured
  // scan to date is unchanged by #339.
  const LoopClosureOptions opt;
  const double gate =
      seed_disagreement_gate(kOfficeExtent, opt.min_seed_disagreement_fraction,
                             opt.min_seed_disagreement);
  // 1e-6 rather than 1e-9: LoopClosureOptions stores these as float, so 0.10f
  // widens to 0.10000000149. The tolerance is about the storage type, not
  // about how tightly the default is pinned.
  REQUIRE_THAT(gate, WithinAbs(0.10, 1e-6));
}

TEST_CASE("LoopClosureOptions_ShortTrajectory_KeepsNoiseFloorGate",
          "[slam][loop][gate]") {
  const LoopClosureOptions opt;
  const double gate = seed_disagreement_gate(kArkitScenesExtent,
                                             opt.min_seed_disagreement_fraction,
                                             opt.min_seed_disagreement);
  REQUIRE_THAT(gate, WithinAbs(0.10, 1e-6)); // float storage, see above
}

TEST_CASE("LoopClosureOptions_VeryLongTrajectory_ScalesGateUp",
          "[slam][loop][gate]") {
  const LoopClosureOptions opt;
  // 100 m capture: 0.00555 * 100 = 0.555 m, well above the floor.
  const double gate = seed_disagreement_gate(
      100.0, opt.min_seed_disagreement_fraction, opt.min_seed_disagreement);
  REQUIRE(gate > opt.min_seed_disagreement);
  REQUIRE_THAT(gate, WithinRel(0.555, 1e-3));
}

TEST_CASE("PlaneGraphOptions_OfficeScaleTrajectory_ReproducesShipped500mmGate",
          "[slam][loop][gate][regression]") {
  // The whole point of the #339 default: on the capture the absolute 0.50 m
  // gate was tuned on, the relative default must reproduce it, so the office
  // loop-closure numbers recorded in §9 stay valid.
  const LoopCandidateOptions opt;
  const double gate = seed_disagreement_gate(
      kOfficeExtent, opt.loop_edges_min_seed_disagreement_fraction,
      opt.loop_edges_min_seed_disagreement);
  REQUIRE_THAT(gate, WithinAbs(0.50, 0.01));
}

TEST_CASE("PlaneGraphOptions_ShortTrajectory_ScalesGateDownFromOfficeValue",
          "[slam][loop][gate][regression]") {
  // The measured footgun, pinned. On 41069048's 1.82 m extent the shipped
  // absolute gate admitted only the most-disagreeing (wrong) edges and dropped
  // F@50mm to 0.29; the relative gate lands at ~0.05 m instead — the setting
  // §9.4 measured at 0.73-0.76 on the same scans.
  const LoopCandidateOptions opt;
  const double gate = seed_disagreement_gate(
      kArkitScenesExtent, opt.loop_edges_min_seed_disagreement_fraction,
      opt.loop_edges_min_seed_disagreement);
  REQUIRE(gate < 0.50);
  REQUIRE_THAT(gate, WithinAbs(0.05, 0.01));
}

TEST_CASE("PlaneGraphOptions_ExplicitAbsoluteFloor_PinsGateOnShortTrajectory",
          "[slam][loop][gate]") {
  // An operator who knows their capture's scale can still pin an absolute gate
  // by raising the floor; the relative term never lowers a gate below it.
  LoopCandidateOptions opt;
  opt.loop_edges_min_seed_disagreement = 0.50;
  const double gate = seed_disagreement_gate(
      kArkitScenesExtent, opt.loop_edges_min_seed_disagreement_fraction,
      opt.loop_edges_min_seed_disagreement);
  REQUIRE_THAT(gate, WithinAbs(0.50, 1e-9));
}
