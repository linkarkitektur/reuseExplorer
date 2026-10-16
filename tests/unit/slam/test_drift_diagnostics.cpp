// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// slam::diagnose_drift (reusex-rcv.1.1) on a synthetic two-pass capture of a
// box room: the planes are ray-cast into depth images from KNOWN poses, and a
// rigid break (2 deg yaw + 0.10 m) is injected into the STORED poses of a
// later stretch. The detector must see the break in the segment matrix,
// localise it to the broken segment and suggest the inverse move; with no
// break the pass-to-pass residual must sit at the noise floor.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/slam/drift_diagnostics.hpp>

#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using reusex::test_support::TempPath;
namespace slam = reusex::slam;

namespace {

constexpr int kW = 80;
constexpr int kH = 60;
constexpr int kFramesPerPass = 24;
// The room: x in [-2.5, 2.5], y in [-2, 2], z in [0, 2.6].
constexpr double kRoomX = 2.5, kRoomY = 2.0, kRoomZ = 2.6;

reusex::core::SensorIntrinsics intrinsics() {
  reusex::core::SensorIntrinsics k;
  k.fx = k.fy = kW / 2.0; // 90 deg field of view
  k.cx = kW / 2.0;
  k.cy = kH / 2.0;
  k.width = kW;
  k.height = kH;
  return k; // local_transform: identity, the pose IS the optical frame
}

/// Optical camera (z forward, x right, y down) at @p c looking along yaw
/// @p yaw (radians, world z up).
Eigen::Matrix4d camera_pose(const Eigen::Vector3d &c, double yaw) {
  const Eigen::Vector3d z(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d x(std::sin(yaw), -std::cos(yaw), 0.0);
  const Eigen::Vector3d y(0.0, 0.0, -1.0);
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<3, 1>(0, 0) = x;
  m.block<3, 1>(0, 1) = y;
  m.block<3, 1>(0, 2) = z;
  m.block<3, 1>(0, 3) = c;
  return m;
}

/// True pose of frame @p k (0 .. 2 * kFramesPerPass - 1): two passes panning
/// a full turn each, on slightly different circles, the second one later.
Eigen::Matrix4d true_pose(int k) {
  const int pass = k / kFramesPerPass;
  const int i = k % kFramesPerPass;
  const double turn = 2.0 * M_PI * i / kFramesPerPass;
  const double r = pass == 0 ? 0.3 : 0.5;
  const double yaw0 = pass == 0 ? 0.0 : 7.0 * M_PI / 180.0;
  const Eigen::Vector3d c(r * std::cos(turn), r * std::sin(turn), 1.4);
  return camera_pose(c, yaw0 + turn);
}

double timestamp(int k) {
  const int pass = k / kFramesPerPass;
  return (pass == 0 ? 0.0 : 100.0) + (k % kFramesPerPass);
}

/// Depth (mm) of the box room seen from @p pose, with seeded 2 mm noise.
cv::Mat render_depth(const Eigen::Matrix4d &pose, std::mt19937 &rng) {
  std::normal_distribution<double> noise(0.0, 0.002);
  const auto k = intrinsics();
  cv::Mat depth(kH, kW, CV_16UC1, cv::Scalar(0));
  const Eigen::Matrix3d r = pose.topLeftCorner<3, 3>();
  const Eigen::Vector3d o = pose.topRightCorner<3, 1>();
  for (int v = 0; v < kH; ++v)
    for (int u = 0; u < kW; ++u) {
      const Eigen::Vector3d ray_c((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
      const Eigen::Vector3d d = r * ray_c; // optical z component == 1
      double t = std::numeric_limits<double>::infinity();
      const std::array<double, 3> lo{-kRoomX, -kRoomY, 0.0};
      const std::array<double, 3> hi{kRoomX, kRoomY, kRoomZ};
      for (int a = 0; a < 3; ++a) {
        if (std::abs(d[a]) < 1e-12)
          continue;
        for (double bound : {lo[a], hi[a]}) {
          const double ta = (bound - o[a]) / d[a];
          if (ta > 0 && ta < t)
            t = ta;
        }
      }
      const double z = t + noise(rng); // depth along optical z (ray_c.z = 1)
      if (std::isfinite(z) && z > 0)
        depth.at<std::uint16_t>(v, u) =
            static_cast<std::uint16_t>(std::lround(z * 1000.0));
    }
  return depth;
}

std::array<double, 16> to_array(const Eigen::Matrix4d &m) {
  std::array<double, 16> a{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      a[static_cast<size_t>(i * 4 + j)] = m(i, j);
  return a;
}

/// 2 deg yaw about the room centre plus 0.10 m in x.
Eigen::Matrix4d injected_break() {
  Eigen::Matrix4d b = Eigen::Matrix4d::Identity();
  b.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  b(0, 3) = 0.10;
  return b;
}

/// A project of the two passes; frames with index >= @p break_from (node
/// ids 1..) store pose = injected_break() * true pose.
void make_project(const std::string &path, int break_from,
                  int frames = 2 * kFramesPerPass) {
  reusex::ProjectDB db(path, /*readOnly=*/false);
  std::mt19937 rng(7);
  const cv::Mat color(kH, kW, CV_8UC3, cv::Scalar(128, 128, 128));
  const cv::Mat conf(kH, kW, CV_8UC1, cv::Scalar(255));
  for (int k = 0; k < frames; ++k) {
    const Eigen::Matrix4d truth = true_pose(k);
    const Eigen::Matrix4d stored =
        k >= break_from ? Eigen::Matrix4d(injected_break() * truth) : truth;
    db.save_sensor_frame(k + 1, color, render_depth(truth, rng), conf,
                         to_array(stored), intrinsics(), timestamp(k));
  }
}

slam::DriftDiagnosticsOptions test_options(int segments) {
  slam::DriftDiagnosticsOptions o;
  o.depth_stride = 2;
  o.segments = segments;
  o.min_frame_points = 20;
  o.min_cell_points = 20;
  o.doubled_walls = false;
  return o;
}

double rotation_deg(const Eigen::Matrix4d &m) {
  const double tr = m(0, 0) + m(1, 1) + m(2, 2);
  return std::acos(std::clamp((tr - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
}

const slam::SegmentPair *find_pair(const slam::DriftReport &r, int a, int b) {
  for (const auto &p : r.pairs)
    if (p.a == a && p.b == b)
      return &p;
  return nullptr;
}

} // namespace

TEST_CASE("DriftDiagnostics_NoBreak_StaysAtNoiseFloor", "[slam][drift]") {
  const TempPath project("drift_nobreak");
  make_project(project.path.string(), /*break_from=*/1 << 30);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto r = slam::diagnose_drift(db, test_options(4));

  REQUIRE(r.noise_floor_m);
  REQUIRE(r.residual_median_m);
  INFO("floor " << *r.noise_floor_m << " pass-to-pass "
                << *r.residual_median_m);
  CHECK(r.timestamps);
  CHECK(r.frames_used == 2 * kFramesPerPass);
  CHECK(std::abs(*r.residual_median_m - *r.noise_floor_m) <=
        0.10 * *r.noise_floor_m);
  CHECK(r.warnings.empty());
  // No segment wants to move.
  REQUIRE(r.suggestions.size() == 4);
  for (const auto &s : r.suggestions) {
    CHECK(s.rotation_deg < 0.2);
    CHECK(s.centroid_shift_m < 0.01);
  }
}

TEST_CASE("DriftDiagnostics_PassBreak_AcrossPairsLightUp", "[slam][drift]") {
  const TempPath project("drift_passbreak");
  make_project(project.path.string(), /*break_from=*/kFramesPerPass);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto r = slam::diagnose_drift(db, test_options(4));

  REQUIRE(r.halves.within_m);
  REQUIRE(r.halves.across_m);
  INFO("within " << *r.halves.within_m << " across " << *r.halves.across_m);
  CHECK(*r.halves.across_m >= 3.0 * *r.halves.within_m);

  // Two segments, one per pass: the second pass's suggestion undoes the
  // break.
  const auto r2 = slam::diagnose_drift(db, test_options(2));
  REQUIRE(r2.suggestions.size() == 2);
  const auto &s = r2.suggestions[1];
  CHECK(s.first_node == kFramesPerPass + 1);
  CHECK(s.last_node == 2 * kFramesPerPass);
  const Eigen::Matrix4d err = s.delta * injected_break();
  const double err_t = err.block(0, 3, 3, 1).norm();
  INFO("residual rotation " << rotation_deg(err) << " deg, translation "
                            << err_t << " m");
  CHECK(rotation_deg(err) <= 0.2);
  CHECK(err_t <= 0.01);
  CHECK(s.after_m < 0.5 * s.before_m);
}

TEST_CASE("DriftDiagnostics_BreakInLastSegment_IsLocalised", "[slam][drift]") {
  // Break the last quarter only (segment 3 of 4).
  const TempPath project("drift_lastseg");
  make_project(project.path.string(),
               /*break_from=*/kFramesPerPass + kFramesPerPass / 2);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto r = slam::diagnose_drift(db, test_options(4));

  // Every pair that involves segment 3 is worse than every pair that does
  // not.
  double worst_clean = 0.0, best_broken = 1e9;
  for (const auto &p : r.pairs) {
    if (p.a == 3 || p.b == 3)
      best_broken = std::min(best_broken, p.residual_m);
    else
      worst_clean = std::max(worst_clean, p.residual_m);
  }
  INFO("worst clean pair " << worst_clean << " best broken pair "
                           << best_broken);
  CHECK(best_broken > 2.0 * worst_clean);
  REQUIRE(find_pair(r, 3, 0) != nullptr);

  // ... and segment 3's suggestion is the inverse break.
  REQUIRE(r.suggestions.size() == 4);
  const auto &s = r.suggestions[3];
  REQUIRE(s.segment == 3);
  const Eigen::Matrix4d err = s.delta * injected_break();
  const double err_t = err.block(0, 3, 3, 1).norm();
  INFO("residual rotation " << rotation_deg(err) << " deg, translation "
                            << err_t << " m");
  CHECK(rotation_deg(err) <= 0.2);
  CHECK(err_t <= 0.01);
  // The largest suggested move is segment 3's.
  for (const auto &o : r.suggestions)
    if (o.segment != 3)
      CHECK(o.rotation_deg < s.rotation_deg);

  // The drift cells and frame residuals are filled.
  CHECK(!r.cells.empty());
  CHECK(r.frames.size() == 2 * kFramesPerPass);
  CHECK(std::is_sorted(r.cells.begin(), r.cells.end(),
                       [](const auto &a, const auto &b) {
                         return a.x != b.x ? a.x < b.x : a.y < b.y;
                       }));
}

TEST_CASE("DriftDiagnostics_SinglePass_WarnsNoOverlap", "[slam][drift]") {
  const TempPath project("drift_singlepass");
  make_project(project.path.string(), 1 << 30, kFramesPerPass);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto r = slam::diagnose_drift(db, test_options(4));
  CHECK(!r.residual_median_m);
  CHECK(r.revisit_points == 0);
  REQUIRE(!r.warnings.empty());
  CHECK(r.warnings.front().find("pass") != std::string::npos);
  CHECK(r.noise_floor_m); // the floor is still measured
}

TEST_CASE("DriftDiagnostics_PoseOverride_ReplacesStoredPoses",
          "[slam][drift]") {
  // Stored poses broken; the override hands in the truth -> no break.
  const TempPath project("drift_override");
  make_project(project.path.string(), kFramesPerPass);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  std::vector<Eigen::Matrix4d> truth;
  for (int k = 0; k < 2 * kFramesPerPass; ++k)
    truth.push_back(true_pose(k));
  const auto r = slam::diagnose_drift(db, test_options(4), nullptr, truth);
  REQUIRE(r.halves.within_m);
  REQUIRE(r.halves.across_m);
  CHECK(*r.halves.across_m < 1.5 * *r.halves.within_m);

  truth.pop_back();
  CHECK_THROWS_AS(slam::diagnose_drift(db, test_options(4), nullptr, truth),
                  std::invalid_argument);
}

TEST_CASE("DriftDiagnostics_Deterministic_JsonHasAllFields", "[slam][drift]") {
  const TempPath project("drift_json");
  make_project(project.path.string(), kFramesPerPass + kFramesPerPass / 2);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto a =
      slam::drift_report_to_json(slam::diagnose_drift(db, test_options(4)));
  const auto b =
      slam::drift_report_to_json(slam::diagnose_drift(db, test_options(4)));
  CHECK(a == b);
  for (const char *key :
       {"\"noise_floor_m\"", "\"residual_median_m\"", "\"residual_p90_m\"",
        "\"frames\"", "\"segments\"", "\"pairs\"", "\"halves\"",
        "\"suggestions\"", "\"cells\"", "\"crossings\"", "\"doubled_walls\"",
        "\"warnings\"", "\"delta\"", "\"frames_flagged\""})
    CHECK(a.find(key) != std::string::npos);
}

TEST_CASE("DriftDiagnostics_Cells_AreKeyedByStorey", "[slam][drift]") {
  // reusex-98o: two storeys stacked in one plan footprint must not share a
  // heat cell. The box room is split at z = 1.3 into two "storeys" whose
  // level frame is world shifted by +10 m in x, so a cell's x tells which
  // frame it was binned in.
  const TempPath project("drift_storeys");
  make_project(project.path.string(), kFramesPerPass + kFramesPerPass / 2);
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    std::vector<reusex::core::StoreyRecord> st(2);
    for (int i = 0; i < 2; ++i) {
      st[static_cast<std::size_t>(i)].guid = "storey-" + std::to_string(i);
      st[static_cast<std::size_t>(i)].idx = i;
      st[static_cast<std::size_t>(i)].z_floor = i == 0 ? 0.0 : 1.3;
      st[static_cast<std::size_t>(i)].z_ceiling = i == 0 ? 1.3 : 2.6;
      st[static_cast<std::size_t>(i)].level_transform[3] = 10.0;
    }
    db.save_storeys(st);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  auto o = test_options(4);
  const auto r = slam::diagnose_drift(db, o);
  REQUIRE(r.storeys.size() == 2);
  CHECK(r.storeys[1].idx == 1);
  CHECK(r.storeys[1].z_floor == 1.3);
  CHECK(r.storeys[1].level_transform[3] == 10.0);
  REQUIRE(!r.cells.empty());
  bool seen[2] = {false, false};
  for (const auto &c : r.cells) {
    REQUIRE((c.storey == 0 || c.storey == 1));
    seen[c.storey] = true;
    // Binned in the level frame: the room spans x in [7.5, 12.5] there.
    CHECK(c.x > 7.0);
    CHECK(c.x < 13.0);
    // The low z of a storey-1 cell is above the split.
    if (c.storey == 1)
      CHECK(c.z_m > 1.2);
  }
  CHECK(seen[0]);
  CHECK(seen[1]);
  CHECK(std::is_sorted(
      r.cells.begin(), r.cells.end(), [](const auto &a, const auto &b) {
        return std::tie(a.storey, a.x, a.y) < std::tie(b.storey, b.x, b.y);
      }));
  const auto json = slam::drift_report_to_json(r);
  CHECK(json.find("\"storeys\"") != std::string::npos);
  CHECK(json.find("\"storey\"") != std::string::npos);
  CHECK(json.find("\"z_m\"") != std::string::npos);

  // Switched off (or no storeys): world cells over all heights.
  o.cells_by_storey = false;
  const auto w = slam::diagnose_drift(db, o);
  CHECK(w.storeys.empty());
  REQUIRE(!w.cells.empty());
  for (const auto &c : w.cells) {
    CHECK(c.storey == -1);
    CHECK(c.x < 3.0);
  }
}

TEST_CASE("DriftDiagnostics_WallCrossings_OpeningVsWall", "[slam][drift]") {
  reusex::core::SolidModel plan;
  reusex::core::StoreyRecord st;
  st.idx = 0;
  st.z_floor = 0.0;
  st.z_ceiling = 2.6;
  plan.storeys.push_back(st);
  reusex::core::WallRecord w;
  w.guid = "w-x0";
  w.storey = 0;
  w.x0 = 0.0;
  w.y0 = -5.0;
  w.x1 = 0.0;
  w.y1 = 5.0;
  plan.walls.push_back(w);
  reusex::core::OpeningRecord door;
  door.wall = "w-x0";
  door.kind = "door";
  door.s_start = 4.5; // y in [-0.5, 0.5]
  door.s_end = 5.5;
  plan.openings.push_back(door);

  auto at = [](double x, double y) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m(0, 3) = x;
    m(1, 3) = y;
    m(2, 3) = 1.4;
    return m;
  };
  // Through the door, back through the wall at y = 3, a long jump across
  // (ignored), and a step on another storey height (ignored).
  const std::vector<Eigen::Matrix4d> poses{
      at(-0.4, 0.0), at(0.4, 0.1),  at(0.4, 3.0), at(-0.4, 3.0),
      at(-0.4, 3.5), at(3.0, -3.0), at(3.0, -3.0)};
  std::vector<Eigen::Matrix4d> high = poses;
  const std::vector<std::int64_t> ids{1, 2, 3, 4, 5, 6, 7};
  const auto c = slam::trajectory_wall_crossings(ids, poses, plan);
  REQUIRE(c.crossings.size() == 2);
  CHECK(c.crossings[0].from_node == 1);
  CHECK(c.crossings[0].at_opening);
  CHECK(c.crossings[1].from_node == 3);
  CHECK(!c.crossings[1].at_opening);
  CHECK(c.crossings[1].wall_guid == "w-x0");
  CHECK(std::abs(c.crossings[1].s_m - 8.0) < 1e-9);
  CHECK(c.through_wall() == 1);
  CHECK(c.steps == 4); // the 2.9 m and 7.4 m jumps are not steps

  for (auto &m : high)
    m(2, 3) = 5.0; // above the storey
  CHECK(slam::trajectory_wall_crossings(ids, high, plan).steps == 0);

  CHECK_THROWS_AS(slam::trajectory_wall_crossings(
                      std::span<const std::int64_t>(ids).first(3), poses, plan),
                  std::invalid_argument);
}
