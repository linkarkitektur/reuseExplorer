// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// slam::frame_pair_matches (reusex-xlg.2): the correspondence
// alignment behind Rediger's match editor. rigid_from_matches is tested on
// synthetic point pairs; match_frame_pair / lift_pixel on a real ProjectDB
// with synthetic frames (below the first section).

#include <catch2/catch_test_macros.hpp>

#include <reusex/slam/frame_pair_matches.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

using reusex::slam::PointPair;
using reusex::slam::rigid_from_matches;
using reusex::slam::RigidFitFailure;

namespace {

using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

Eigen::Matrix4d M(const std::array<double, 16> &a) {
  return Eigen::Map<const RowMajor4d>(a.data());
}

std::array<double, 16> A(const Eigen::Matrix4d &m) {
  std::array<double, 16> a{};
  Eigen::Map<RowMajor4d>(a.data()) = m;
  return a;
}

/// Row-major pose: translate(tx, ty, tz) * Rz(yaw_deg).
std::array<double, 16> pose(double tx, double ty, double tz, double yaw_deg) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  m.topRightCorner<3, 1>() = Eigen::Vector3d(tx, ty, tz);
  return A(m);
}

double translation_error(const std::array<double, 16> &a,
                         const Eigen::Matrix4d &b) {
  return (M(a).topRightCorner<3, 1>() - b.topRightCorner<3, 1>()).norm();
}

double angle_error_deg(const std::array<double, 16> &a,
                       const Eigen::Matrix4d &b) {
  const Eigen::Matrix3d r =
      M(a).topLeftCorner<3, 3>().transpose() * b.topLeftCorner<3, 3>();
  return std::acos(std::clamp((r.trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 /
         M_PI;
}

reusex::slam::Point3 P(const Eigen::Vector3d &v) {
  return {v.x(), v.y(), v.z()};
}

/// n points in a 2 m box in A's frame, and the same world points in B's
/// frame when the TRUE world correction applied to B is `delta`:
/// T_A p_a = delta T_B p_b  =>  p_b = (delta T_B)^-1 T_A p_a.
std::vector<PointPair> synthetic_pairs(const std::array<double, 16> &pa,
                                       const std::array<double, 16> &pb,
                                       const Eigen::Matrix4d &delta, int n,
                                       unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const Eigen::Matrix4d b_from_a = (delta * M(pb)).inverse() * M(pa);
  std::vector<PointPair> out;
  for (int k = 0; k < n; ++k) {
    const Eigen::Vector3d p(u(rng), u(rng), 1.5 + 0.5 * u(rng));
    const Eigen::Vector3d q = (b_from_a * p.homogeneous()).head<3>();
    out.emplace_back(P(p), P(q));
  }
  return out;
}

const std::array<double, 16> kPoseA = pose(1.0, 2.0, 0.0, 10.0);
const std::array<double, 16> kPoseB = pose(1.3, 2.1, 0.05, 16.0);

Eigen::Matrix4d true_delta() {
  return M(pose(0.12, -0.05, 0.02, 3.0)); // the drift to undo
}

} // namespace

TEST_CASE("RigidFromMatches_ExactPairs_RecoverTheWorldDelta",
          "[slam][matches]") {
  const auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 40, 3);
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  REQUIRE(fit.ok);
  CHECK(fit.failure == RigidFitFailure::none);
  CHECK(fit.inliers.size() == pairs.size());
  CHECK(fit.rms_m < 1e-9);
  CHECK(translation_error(fit.world_delta, true_delta()) < 1e-6);
  CHECK(angle_error_deg(fit.world_delta, true_delta()) < 1e-6);
  // The convention, literally: world_delta * T_B * p_b == T_A * p_a.
  for (const auto &[pa, pb] : pairs) {
    const Eigen::Vector3d lhs =
        (M(fit.world_delta) * M(kPoseB) *
         Eigen::Vector3d(pb[0], pb[1], pb[2]).homogeneous())
            .head<3>();
    const Eigen::Vector3d rhs =
        (M(kPoseA) * Eigen::Vector3d(pa[0], pa[1], pa[2]).homogeneous())
            .head<3>();
    CHECK((lhs - rhs).norm() < 1e-9);
  }
}

TEST_CASE("RigidFromMatches_InjectedOutliers_AreRejected", "[slam][matches]") {
  auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 30, 5);
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(0.5, 1.5);
  for (int k = 0; k < 8; ++k) { // pairs 30..37 point 0.5–1.5 m off
    auto bad = pairs[static_cast<std::size_t>(k)];
    bad.second[0] += u(rng);
    bad.second[1] -= u(rng);
    pairs.push_back(bad);
  }
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  REQUIRE(fit.ok);
  CHECK(fit.inliers.size() == 30);
  for (int i : fit.inliers)
    CHECK(i < 30);
  CHECK(translation_error(fit.world_delta, true_delta()) < 1e-6);
}

TEST_CASE("RigidFromMatches_FewerThanThreePairs_IsRefusedWithAReason",
          "[slam][matches]") {
  const auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 2, 1);
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  CHECK_FALSE(fit.ok);
  CHECK(fit.failure == RigidFitFailure::too_few_pairs);
  CHECK(fit.reason == "3 matches with depth needed, 2 given");
}

TEST_CASE("RigidFromMatches_CollinearPoints_AreRefusedAsDegenerate",
          "[slam][matches]") {
  // Five points on a door edge: a rotation about that line is free.
  std::vector<PointPair> pairs;
  const Eigen::Matrix4d b_from_a =
      (true_delta() * M(kPoseB)).inverse() * M(kPoseA);
  for (int k = 0; k < 5; ++k) {
    const Eigen::Vector3d p(0.3, -0.8 + 0.4 * k, 1.6);
    pairs.emplace_back(P(p), P((b_from_a * p.homogeneous()).head<3>()));
  }
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  CHECK_FALSE(fit.ok);
  CHECK(fit.failure == RigidFitFailure::degenerate);
  CHECK(fit.reason.find("collinear") != std::string::npos);
}

TEST_CASE("RigidFromMatches_NoConsensus_IsRefused", "[slam][matches]") {
  // Every pair disagrees with every other by metres.
  std::vector<PointPair> pairs;
  for (int k = 0; k < 6; ++k)
    pairs.push_back({{0.0 + k, 0.0, 1.0}, {5.0 * k * k, -3.0 * k, 9.0 - k}});
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  CHECK_FALSE(fit.ok);
  CHECK(fit.failure == RigidFitFailure::no_consensus);
}

TEST_CASE("RigidFromMatches_FixedSeed_IsDeterministic", "[slam][matches]") {
  auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 25, 11);
  pairs.push_back({{0, 0, 1}, {3, 3, 3}});
  pairs.push_back({{1, 0, 1}, {-3, 2, 0}});
  const auto a = rigid_from_matches(kPoseA, kPoseB, pairs, 0.05, 500, 7);
  const auto b = rigid_from_matches(kPoseA, kPoseB, pairs, 0.05, 500, 7);
  REQUIRE(a.ok);
  CHECK(a.world_delta == b.world_delta);
  CHECK(a.inliers == b.inliers);
  CHECK(a.rms_m == b.rms_m);
}

TEST_CASE("RigidFromMatches_InvalidParameters_Throw", "[slam][matches]") {
  const auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 10, 2);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  CHECK_THROWS_AS(rigid_from_matches(kPoseA, kPoseB, pairs, 0.05, 0),
                  std::invalid_argument);
  CHECK_THROWS_AS(rigid_from_matches(kPoseA, kPoseB, pairs, 0.05, -3),
                  std::invalid_argument);
  CHECK_THROWS_AS(rigid_from_matches(kPoseA, kPoseB, pairs, -0.01),
                  std::invalid_argument);
  CHECK_THROWS_AS(rigid_from_matches(kPoseA, kPoseB, pairs, nan),
                  std::invalid_argument);
  CHECK_THROWS_AS(rigid_from_matches(kPoseA, kPoseB, pairs, inf),
                  std::invalid_argument);
  auto bad = kPoseB;
  bad[3] = nan;
  CHECK_THROWS_AS(rigid_from_matches(kPoseA, bad, pairs),
                  std::invalid_argument);
  CHECK_THROWS_AS(rigid_from_matches(bad, kPoseB, pairs),
                  std::invalid_argument);
}

TEST_CASE("RigidFromMatches_InliersAndRms_AreThoseOfTheReturnedTransform",
          "[slam][matches]") {
  // Noisy pairs: whatever RANSAC's hypothesis counted, the reported inliers
  // and RMS are measured under the refit transform actually returned.
  auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 60, 21);
  std::mt19937 rng(4);
  std::normal_distribution<double> n(0.0, 0.02);
  for (auto &pr : pairs)
    for (int i = 0; i < 3; ++i)
      pr.second[static_cast<std::size_t>(i)] += n(rng);
  const double thr = 0.04;
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs, thr, 300, 3);
  REQUIRE(fit.ok);
  const Eigen::Matrix4d W = M(fit.world_delta);
  std::vector<int> expect;
  double sq = 0.0;
  for (std::size_t k = 0; k < pairs.size(); ++k) {
    const auto &[pa, pb] = pairs[k];
    const Eigen::Vector3d e =
        (W * M(kPoseB) * Eigen::Vector3d(pb[0], pb[1], pb[2]).homogeneous())
            .head<3>() -
        (M(kPoseA) * Eigen::Vector3d(pa[0], pa[1], pa[2]).homogeneous())
            .head<3>();
    if (e.squaredNorm() <= thr * thr) {
      expect.push_back(static_cast<int>(k));
      sq += e.squaredNorm();
    }
  }
  CHECK(fit.inliers == expect);
  CHECK(std::abs(fit.rms_m -
                 std::sqrt(sq / static_cast<double>(expect.size()))) < 1e-9);
}
