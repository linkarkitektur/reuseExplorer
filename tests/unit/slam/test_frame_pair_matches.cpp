// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// slam::frame_pair_matches (reusex-xlg.2): the correspondence
// alignment behind Rediger's match editor. rigid_from_matches is tested on
// synthetic point pairs; match_frame_pair / lift_pixel on a real ProjectDB
// with synthetic frames (below the first section).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/slam/frame_pair_icp.hpp>
#include <reusex/slam/frame_pair_matches.hpp>

#include "../../support/temp_path.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <sqlite3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
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

TEST_CASE("RigidFromMatches_PartialConsensus_IsReported", "[slam][matches]") {
  // Three pairs, the third stretched 0.1 m: the one hypothesis' Umeyama fit
  // leaves residuals 0.042, 0.019 and 0.061 m, so 2 of 3 agree within
  // 0.05 m. The refusal reports those 2, not 0.
  const auto I = pose(0, 0, 0, 0);
  const std::vector<PointPair> pairs = {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
                                        {{2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}},
                                        {{0.0, 2.1, 0.0}, {0.0, 2.0, 0.0}}};
  const auto fit = rigid_from_matches(I, I, pairs);
  CHECK_FALSE(fit.ok);
  CHECK(fit.failure == RigidFitFailure::no_consensus);
  CHECK(fit.inliers == std::vector<int>{0, 1});
  CHECK(fit.reason.find("2 of 3") != std::string::npos);
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

// --------------------------------------------------- database-backed tests --

using reusex::slam::FrameMatchOptions;
using reusex::slam::lift_pixel;
using reusex::slam::match_frame_pair;
using reusex::test_support::TempPath;

namespace {

constexpr int kW = 320, kH = 240;
constexpr double kF = 300.0;

/// RTABMap's optical -> base rotation: a non-identity local_transform, so a
/// keypoint's xyz must be local · optical to land where frame_world_points
/// puts the pixel.
reusex::core::SensorIntrinsics intrinsics() {
  reusex::core::SensorIntrinsics k;
  k.fx = k.fy = kF;
  k.cx = kW / 2.0;
  k.cy = kH / 2.0;
  k.width = kW;
  k.height = kH;
  k.local_transform = {0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1};
  return k;
}

/// Blocky random colour: strong, distinctive ORB corners.
cv::Mat texture() {
  cv::RNG rng(12345);
  cv::Mat small(kH / 8, kW / 8, CV_8UC3);
  rng.fill(small, cv::RNG::UNIFORM, 0, 256);
  cv::Mat big;
  cv::resize(small, big, cv::Size(kW, kH), 0, 0, cv::INTER_NEAREST);
  return big;
}

/// A wedge (1.2 m rising to ~1.84 m left to right), a box 0.3 m nearer,
/// and a 20x20 hole without depth at (20..39, 20..39): non-planar, so ICP
/// is well-posed, and a hole for lift_pixel's snap.
uint16_t depth_mm(int u, int v) {
  if (u >= 20 && u < 40 && v >= 20 && v < 40)
    return 0;
  int z = 1200 + 2 * u;
  if (u >= 200 && u < 260 && v >= 60 && v < 140)
    z -= 300;
  return static_cast<uint16_t>(z);
}

/// The depth image at cols/kW of the colour resolution.
cv::Mat depth(int cols, int rows) {
  cv::Mat d(rows, cols, CV_16UC1);
  const int s = kW / cols;
  for (int v = 0; v < rows; ++v)
    for (int u = 0; u < cols; ++u)
      d.at<uint16_t>(v, u) = depth_mm(u * s, v * s);
  return d;
}

/// Frames 1 (A) and 2 (B): the SAME colour and depth, different stored
/// poses — the true correction is world_delta = T_A · T_B^-1. Frame 3: A's
/// view with half-resolution depth. Frame 4: colour, no depth. Frame 5:
/// colour and depth, no pose.
void write_fixture(const std::string &path) {
  reusex::ProjectDB db(path, /*readOnly=*/false);
  const auto k = intrinsics();
  const cv::Mat c = texture();
  const cv::Mat conf(kH, kW, CV_8UC1, cv::Scalar(255));
  db.save_sensor_frame(1, c, depth(kW, kH), conf, kPoseA, k, 0.0);
  db.save_sensor_frame(2, c, depth(kW, kH), conf, kPoseB, k, 1.0);
  db.save_sensor_frame(3, c, depth(kW / 2, kH / 2),
                       cv::Mat(kH / 2, kW / 2, CV_8UC1, cv::Scalar(255)),
                       kPoseA, k, 2.0);
  db.save_sensor_frame(4, c, cv::Mat{}, conf, kPoseA, k, 3.0);
  db.save_sensor_frame(5, c, depth(kW, kH), conf, kPoseA, k, 4.0);
}

/// No public API stores a frame with depth but no pose
/// (tests/unit/io/test_update_sensor_frame_pose.cpp says why), so NULL the
/// transform with raw SQL — after the ProjectDB above has closed (WAL).
void clear_column(const std::string &path, int node_id,
                  const std::string &column) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.c_str(), &raw) == SQLITE_OK);
  const std::string sql = "UPDATE sensor_frames SET " + column +
                          " = NULL WHERE node_id = " + std::to_string(node_id) +
                          ";";
  REQUIRE(sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, nullptr) ==
          SQLITE_OK);
  REQUIRE(sqlite3_changes(raw) == 1);
  sqlite3_close(raw);
}

void clear_pose(const std::string &path, int node_id) {
  clear_column(path, node_id, "transform");
}

/// The fixture as every database test opens it.
void make_fixture(const std::string &path) {
  write_fixture(path);
  clear_pose(path, 5);
  reusex::ProjectDB db(path, /*readOnly=*/true);
  REQUIRE_FALSE(db.has_sensor_frame_pose(5));
  REQUIRE(db.has_sensor_frame_pose(1));
}

std::vector<PointPair> depth_pairs(const reusex::slam::FramePairMatches &m) {
  std::vector<PointPair> out;
  for (const auto &mm : m.matches) {
    const auto &ka = m.keypoints_a[static_cast<std::size_t>(mm.a)];
    const auto &kb = m.keypoints_b[static_cast<std::size_t>(mm.b)];
    if (ka.xyz && kb.xyz)
      out.emplace_back(*ka.xyz, *kb.xyz);
  }
  return out;
}

const Eigen::Matrix4d kTruth = M(kPoseA) * M(kPoseB).inverse();

/// A rigid transform's inverse, row-major.
std::array<double, 16> rigid_inverse(const std::array<double, 16> &t) {
  Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R = M(t).topLeftCorner<3, 3>();
  inv.topLeftCorner<3, 3>() = R.transpose();
  inv.topRightCorner<3, 1>() = -R.transpose() * M(t).topRightCorner<3, 1>();
  return A(inv);
}

} // namespace

TEST_CASE("FramePairMatches_IdenticalViews_RecoverTheStoredPoseDifference",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_recover");
  make_fixture(project.path.string());
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto m = match_frame_pair(db, 1, 2);
  REQUIRE(m.matches.size() >= 50);
  for (const auto &mm : m.matches) {
    REQUIRE(mm.a >= 0);
    REQUIRE(mm.a < static_cast<int>(m.keypoints_a.size()));
    REQUIRE(mm.b >= 0);
    REQUIRE(mm.b < static_cast<int>(m.keypoints_b.size()));
  }
  // The hole: keypoints there are kept, without xyz.
  const auto fit = rigid_from_matches(kPoseA, kPoseB, depth_pairs(m));
  REQUIRE(fit.ok);
  CHECK(translation_error(fit.world_delta, kTruth) < 0.01);
  CHECK(angle_error_deg(fit.world_delta, kTruth) < 0.5);
}

TEST_CASE("FramePairMatches_Region_DropsKeypointsOutside", "[slam][matches]") {
  const TempPath project("frame_pair_matches_region");
  make_fixture(project.path.string());
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  FrameMatchOptions o;
  o.region_a = reusex::slam::PixelRect{0, 0, kW / 2, kH};
  const auto m = match_frame_pair(db, 1, 2, o);
  REQUIRE_FALSE(m.keypoints_a.empty());
  // ORB's mask test rounds a keypoint to its pixel: allow half a pixel.
  for (const auto &k : m.keypoints_a)
    CHECK(k.u < kW / 2.0f + 0.5f);
  const bool b_right =
      std::any_of(m.keypoints_b.begin(), m.keypoints_b.end(),
                  [](const auto &k) { return k.u > kW / 2.0f; });
  CHECK(b_right); // B unrestricted
}

TEST_CASE("FramePairMatches_RegionPartlyOutside_KeepsTheIntersection",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_region_out");
  make_fixture(project.path.string());
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  FrameMatchOptions o;
  o.region_b = reusex::slam::PixelRect{kW / 2, -50, kW, kH + 100};
  const auto m = match_frame_pair(db, 1, 2, o);
  REQUIRE_FALSE(m.keypoints_b.empty());
  for (const auto &k : m.keypoints_b)
    CHECK(k.u >= kW / 2.0f - 0.5f);
  o.region_b = reusex::slam::PixelRect{kW + 10, 0, 50, 50}; // all outside
  const auto none = match_frame_pair(db, 1, 2, o);
  CHECK(none.keypoints_b.empty());
  CHECK(none.matches.empty());
}

TEST_CASE("FramePairMatches_FrameWithoutDepthOrPose_ThrowsNamingIt",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_missing");
  make_fixture(project.path.string());
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto message = [&](int a, int b) -> std::string {
    try {
      match_frame_pair(db, a, b);
    } catch (const std::runtime_error &e) {
      return e.what();
    }
    return {};
  };
  CHECK(message(1, 4).find("frame 4") != std::string::npos);
  CHECK(message(5, 1).find("frame 5") != std::string::npos);
  CHECK_THROWS_AS(match_frame_pair(db, 2, 2), std::invalid_argument);
}

TEST_CASE("FramePairMatches_FrameWithoutColour_ThrowsNamingIt",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_no_colour");
  make_fixture(project.path.string());
  clear_column(project.path.string(), 2, "color"); // depth and pose kept
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto message = [](auto &&f) -> std::string {
    try {
      f();
    } catch (const std::runtime_error &e) {
      return e.what();
    }
    return {};
  };
  CHECK(message([&] { match_frame_pair(db, 1, 2); }) ==
        "frame 2 has no stored colour image");
  CHECK(message([&] { lift_pixel(db, 2, 100.0f, 100.0f); }) ==
        "frame 2 has no stored colour image");
}

TEST_CASE("FramePairMatches_RatioTestKeepsNone_WarnsWithCountsAndRatio",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_ratio_none");
  make_fixture(project.path.string());
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  // Disjoint halves and a near-zero ratio: keypoints on both sides, every
  // match refused.
  FrameMatchOptions o;
  o.region_a = reusex::slam::PixelRect{0, 0, kW / 2, kH};
  o.region_b = reusex::slam::PixelRect{kW / 2, 0, kW / 2, kH};
  o.ratio_test = 0.01;
  std::vector<std::string> warnings;
  const auto saved = reusex::core::get_log_level();
  reusex::core::set_log_handler(
      [&](reusex::core::LogLevel l, std::string_view m) {
        if (l == reusex::core::LogLevel::warn)
          warnings.emplace_back(m);
      });
  reusex::core::set_log_level(reusex::core::LogLevel::warn);
  const auto m = match_frame_pair(db, 1, 2, o);
  reusex::core::reset_log_handler();
  reusex::core::set_log_level(saved);
  REQUIRE_FALSE(m.keypoints_a.empty());
  REQUIRE_FALSE(m.keypoints_b.empty());
  CHECK(m.matches.empty());
  REQUIRE(warnings.size() == 1);
  const std::string &w = warnings.front();
  CHECK(w.find(std::to_string(m.keypoints_a.size())) != std::string::npos);
  CHECK(w.find(std::to_string(m.keypoints_b.size())) != std::string::npos);
  CHECK(w.find("0.01") != std::string::npos);
}

TEST_CASE("LiftPixel_SnapsToNearestDepth_AndRefusesFarFromAny",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_lift");
  make_fixture(project.path.string());
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto direct = lift_pixel(db, 1, 100.0f, 80.0f);
  REQUIRE(direct);
  CHECK(direct->u == 100.0f);
  CHECK(direct->v == 80.0f);
  REQUIRE(direct->xyz);
  const auto snapped = lift_pixel(db, 1, 22.0f, 30.0f); // 3 px into the hole
  REQUIRE(snapped);
  CHECK(snapped->u == 19.0f);
  CHECK(snapped->v == 30.0f);
  CHECK_FALSE(lift_pixel(db, 1, 30.0f, 30.0f)); // 10+ px from any depth
  CHECK_FALSE(lift_pixel(db, 1, -5.0f, 30.0f)); // outside the image
  CHECK_THROWS_AS(lift_pixel(db, 4, 10.0f, 10.0f), std::runtime_error);
}

TEST_CASE("LiftPixel_HalfResolutionDepth_MatchesFullResolutionLift",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_halfres");
  make_fixture(project.path.string());
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto full = lift_pixel(db, 1, 100.0f, 80.0f);
  const auto half = lift_pixel(db, 3, 100.0f, 80.0f);
  REQUIRE(full);
  REQUIRE(half);
  CHECK(half->u == 100.0f);
  CHECK(half->v == 80.0f);
  for (int i = 0; i < 3; ++i)
    CHECK(std::abs((*half->xyz)[static_cast<std::size_t>(i)] -
                   (*full->xyz)[static_cast<std::size_t>(i)]) < 1e-9);
}

TEST_CASE("LiftPixel_PoseTimesXyz_IsTheOverlaysWorldPoint", "[slam][matches]") {
  // The stored-pose-frame convention: T_A · xyz must be a point of the very
  // cloud the overlay draws (frame_world_points, every pixel).
  const TempPath project("frame_pair_matches_world");
  make_fixture(project.path.string());
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto k = lift_pixel(db, 1, 150.0f, 100.0f);
  REQUIRE(k);
  REQUIRE(k->xyz);
  const Eigen::Vector3d w =
      (M(kPoseA) *
       Eigen::Vector3d((*k->xyz)[0], (*k->xyz)[1], (*k->xyz)[2]).homogeneous())
          .head<3>();
  reusex::slam::FramePairIcpOptions o;
  o.pixel_step = 1;
  o.max_depth_m = 5.0f;
  double best = 1e9;
  for (const auto &p : reusex::slam::frame_world_points(db, 1, o))
    best = std::min(best, (Eigen::Vector3d(p[0], p[1], p[2]) - w).norm());
  CHECK(best < 1e-4);
}

TEST_CASE("RigidFromMatches_WorldDelta_RoundTripsWithIcpAndEdgeMeasurement",
          "[slam][matches]") {
  // The matches' world_delta is the ICP refinement's seed and, without ICP,
  // the manual edge. ICP seeded with it stays there, and the manual A -> B
  // edge T_A^-1 ΔT T_B is the inverse of ICP's B -> A edge_transform.
  const TempPath project("frame_pair_matches_icp");
  make_fixture(project.path.string());
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto fit = rigid_from_matches(kPoseA, kPoseB,
                                      depth_pairs(match_frame_pair(db, 1, 2)));
  REQUIRE(fit.ok);
  reusex::slam::FramePairIcpOptions o;
  o.initial_guess = fit.world_delta;
  const auto icp = reusex::slam::refine_frame_pair_icp(db, 2, 1, o);
  REQUIRE(icp.converged);
  CHECK(translation_error(icp.world_delta, M(fit.world_delta)) < 0.01);
  CHECK(angle_error_deg(icp.world_delta, M(fit.world_delta)) < 0.5);
  // The manual edge (A -> B), computed as the Qt client's
  // edge_measurement does: T_A^-1 · ΔT · T_B.
  const Eigen::Matrix4d manual =
      M(kPoseA).inverse() * M(fit.world_delta) * M(kPoseB);
  const auto icp_a_to_b = rigid_inverse(icp.edge_transform);
  CHECK(translation_error(icp_a_to_b, manual) < 0.01);
  CHECK(angle_error_deg(icp_a_to_b, manual) < 0.5);
}
