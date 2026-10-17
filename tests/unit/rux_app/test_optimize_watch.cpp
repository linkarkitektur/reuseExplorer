// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux optimize --edits e.yaml --watch` (reusex-rcv.1.5): the pose loop,
// driven one pass at a time -- no sleeping.
//
// The capture: two passes round a box room (as test_drift_diagnostics.cpp),
// the second one stored with a rigid break (2 deg yaw + 0.10 m), the graph
// initialised from the broken poses. The loop must report the break, print
// a suggestion that pastes into the edits file and undoes it, keep the last
// good solve when the file breaks, repeat itself bit for bit, and show the
// shear a move alone causes on the pass no edit names (reusex-sb2).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <global-params.hpp>
#include <optimize.hpp>
#include <optimize_watch.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/io/pose_edits.hpp>
#include <reusex/slam/drift_diagnostics.hpp>
#include <reusex/slam/pose_graph.hpp>

#include "../../support/temp_path.hpp"

#include <CLI/CLI.hpp>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace reusex::test_support;
using Catch::Matchers::ContainsSubstring;
namespace fs = std::filesystem;
namespace pg = reusex::core::pose_graph;

namespace {

constexpr int kW = 80;
constexpr int kH = 60;
constexpr int kPerPass = 24;
constexpr double kRoomX = 2.5, kRoomY = 2.0, kRoomZ = 2.6;

reusex::core::SensorIntrinsics intrinsics() {
  reusex::core::SensorIntrinsics k;
  k.fx = k.fy = kW / 2.0;
  k.cx = kW / 2.0;
  k.cy = kH / 2.0;
  k.width = kW;
  k.height = kH;
  return k;
}

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

Eigen::Matrix4d true_pose(int k) {
  const int pass = k / kPerPass;
  const int i = k % kPerPass;
  const double turn = 2.0 * M_PI * i / kPerPass;
  const double r = pass == 0 ? 0.3 : 0.5;
  const double yaw0 = pass == 0 ? 0.0 : 7.0 * M_PI / 180.0;
  const Eigen::Vector3d c(r * std::cos(turn), r * std::sin(turn), 1.4);
  return camera_pose(c, yaw0 + turn);
}

cv::Mat render_depth(const Eigen::Matrix4d &pose, std::mt19937 &rng) {
  std::normal_distribution<double> noise(0.0, 0.002);
  const auto k = intrinsics();
  cv::Mat depth(kH, kW, CV_16UC1, cv::Scalar(0));
  const Eigen::Matrix3d r = pose.topLeftCorner<3, 3>();
  const Eigen::Vector3d o = pose.topRightCorner<3, 1>();
  for (int v = 0; v < kH; ++v)
    for (int u = 0; u < kW; ++u) {
      const Eigen::Vector3d d =
          r * Eigen::Vector3d((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);
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
      const double z = t + noise(rng);
      if (std::isfinite(z) && z > 0)
        depth.at<std::uint16_t>(v, u) =
            static_cast<std::uint16_t>(std::lround(z * 1000.0));
    }
  return depth;
}

Eigen::Matrix4d injected_break() {
  Eigen::Matrix4d b = Eigen::Matrix4d::Identity();
  b.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  b(0, 3) = 0.10;
  return b;
}

/// Node ids 1..48; the second pass (25..48) stored broken; the pose graph
/// initialised from the stored poses (initial = broken).
void make_project(const fs::path &path) {
  reusex::ProjectDB db(path, /*readOnly=*/false);
  std::mt19937 rng(7);
  const cv::Mat color(kH, kW, CV_8UC3, cv::Scalar(128, 128, 128));
  const cv::Mat conf(kH, kW, CV_8UC1, cv::Scalar(255));
  for (int k = 0; k < 2 * kPerPass; ++k) {
    const Eigen::Matrix4d truth = true_pose(k);
    const Eigen::Matrix4d stored =
        k >= kPerPass ? Eigen::Matrix4d(injected_break() * truth) : truth;
    const double ts = (k < kPerPass ? 0.0 : 100.0) + (k % kPerPass);
    db.save_sensor_frame(k + 1, color, render_depth(truth, rng), conf,
                         pg::row_major_from_matrix(stored), intrinsics(), ts);
  }
  reusex::slam::init_pose_graph(db);
}

void write(const fs::path &p, const std::string &text) {
  std::ofstream(p, std::ios::trunc) << text;
}

const std::string kHeader = "schema: reusex.pose_edits.v1\nframe: world\n";

SubcommandOptimizeOptions watch_options(const fs::path &edits) {
  SubcommandOptimizeOptions o;
  o.edits = edits.string();
  o.watch = true;
  o.fast = true;
  o.drift_depth_stride = 2;
  o.drift_segments = 2; // one per pass
  // No plane landmark can form: the reference (the no-edit full solve,
  // reusex-rcv.1.21) keeps the break, so these tests see it to fix it.
  o.min_plane_inliers = 1000000;
  return o;
}

std::vector<std::array<double, 16>> stored_poses(const fs::path &project) {
  reusex::ProjectDB db(project, /*readOnly=*/true);
  std::vector<std::array<double, 16>> out;
  for (int id : db.sensor_frame_ids())
    out.push_back(db.sensor_frame_pose(id));
  return out;
}

} // namespace

TEST_CASE("pose_change and fit_rigid_delta", "[rux_app][optimize][watch]") {
  Eigen::Matrix4d d = injected_break();
  d(1, 3) = -0.05;
  std::vector<Eigen::Matrix4d> from, to;
  for (int k = 0; k < 5; ++k) {
    from.push_back(true_pose(k));
    to.push_back(d * true_pose(k));
  }
  const Eigen::Matrix4d fit = rux::fit_rigid_delta(from, to);
  CHECK((fit - d).cwiseAbs().maxCoeff() < 1e-9);
  // One pose is enough to fix the rotation.
  const Eigen::Matrix4d one = rux::fit_rigid_delta({from[0]}, {to[0]});
  CHECK((one - d).cwiseAbs().maxCoeff() < 1e-9);

  const std::vector<std::int64_t> ids{1, 2, 3, 4, 5};
  const auto c = rux::pose_change(ids, to, from);
  CHECK(c.nodes == 5);
  CHECK(c.max_m > 0.0);
  CHECK(c.max_rot_deg > 1.99);
  std::vector<bool> mask{true, false, false, false, false};
  CHECK(rux::pose_change(ids, to, from, &mask).nodes == 1);
}

TEST_CASE("storey_runs splits a segment at a storey change",
          "[rux_app][optimize][watch]") {
  reusex::core::StoreyRecord s0, s1;
  s0.idx = 0;
  s0.z_floor = 0.0;
  s0.z_ceiling = 2.5;
  s1.idx = 1;
  s1.z_floor = 2.8;
  s1.z_ceiling = 5.3;
  std::vector<std::int64_t> nodes;
  std::vector<Eigen::Vector3d> pos;
  for (int k = 0; k < 20; ++k) {
    nodes.push_back(100 + k);
    // Upstairs for 12 frames, one stray low frame, then downstairs.
    const double z = k < 12 ? 4.2 : 1.4;
    pos.emplace_back(0.0, 0.0, k == 5 ? 1.4 : z);
  }
  const auto runs = rux::storey_runs(nodes, pos, {s0, s1});
  REQUIRE(runs.size() == 2);
  CHECK(runs[0].first == 100);
  CHECK(runs[0].last == 111);
  CHECK(runs[0].storey == 1);
  CHECK(runs[0].frames == 12);
  CHECK(runs[1].first == 112);
  CHECK(runs[1].storey == 0);
  // Without storeys: one run.
  CHECK(rux::storey_runs(nodes, pos, {}).size() == 1);
}

TEST_CASE("edited_nodes names every node an edit constrains",
          "[rux_app][optimize][watch]") {
  const auto e = reusex::io::pose_edits_from_yaml(
      kHeader + "edits:\n"
                "  - id: m\n    kind: move\n    nodes: [3, 5]\n"
                "    delta: {xyz: [0.1, 0, 0]}\n"
                "  - id: a\n    kind: anchor\n    node: 9\n    pose: current\n"
                "  - id: x\n    kind: reject_edge\n    a: 1\n    b: 7\n"
                "    edge_type: loop_closure\n");
  const std::vector<std::int64_t> ids{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  CHECK(rux::edited_nodes(e, ids) == std::vector<std::int64_t>{3, 4, 5, 9});
}

TEST_CASE("OptimizeWatchSession: suggestion pastes, re-solves, keeps the last "
          "good solve, repeats itself",
          "[rux_app][optimize][watch]") {
  TempDir dir("optimize_watch");
  const fs::path project = dir.path / "p.rux";
  const fs::path edits = dir.path / "e.yaml";
  make_project(project);
  const auto before = stored_poses(project);
  write(edits, kHeader + "edits: []\n");

  RuxOptions global;
  global.project_db = project;
  rux::OptimizeWatchSession session(watch_options(edits), global);
  CHECK(session.watched_paths() == std::vector<fs::path>{edits});

  // Cycle 0: no edits. The break shows across the two segments, and the
  // second segment's ICP suggestion is printed as YAML.
  const auto p0 = session.pass(true);
  INFO(p0.report);
  REQUIRE(p0.solved);
  CHECK(p0.edits_ok);
  CHECK_THAT(p0.report, ContainsSubstring("ok: 0 edits"));
  CHECK_THAT(p0.report, ContainsSubstring("frame step 1, depth stride 2"));
  CHECK_THAT(p0.report, ContainsSubstring("shear (untouched)"));
  CHECK_THAT(p0.report, ContainsSubstring("not written (preview only"));
  // reusex-rcv.1.17: the depth is read once, into the frame cache.
  CHECK_THAT(p0.report, ContainsSubstring("frame cache           48 frames"));
  REQUIRE(p0.cross_segment_median_m);
  CHECK(*p0.cross_segment_median_m > 0.03);
  CHECK(p0.shear.untouched == 2 * kPerPass);
  CHECK(p0.shear.change.max_m < 1e-9); // nothing moved without edits
  REQUIRE(p0.poses.size() == 2 * kPerPass);
  CHECK_THAT(p0.suggestions_yaml, ContainsSubstring("kind: move"));
  REQUIRE_FALSE(p0.suggestion_items.empty());

  // Every suggestion pastes under `edits:` and parses (acceptance 4) ...
  write(edits, kHeader + "edits:\n" + p0.suggestions_yaml);
  const auto all = reusex::io::read_pose_edits(edits);
  CHECK(all.edits.size() == p0.suggestion_items.size());
  // ... and the best one, saved alone, is the second pass's move.
  const std::string best = p0.suggestion_items.front();
  write(edits, kHeader + "edits:\n" + best);
  const auto pasted = reusex::io::read_pose_edits(edits);
  REQUIRE(pasted.edits.size() == 1);
  CHECK(pasted.edits.front().kind == reusex::core::PoseEditKind::move);

  // ... and, saved, re-solves: a pass moves, the break closes.
  const auto p1 = session.pass(true);
  INFO(p1.report);
  REQUIRE(p1.solved);
  CHECK(p1.edits_ok);
  // The edit's line carries its verdict (reusex-rcv.1.28). Against the one
  // link at the break the block's RMS ends ~1.3 cm (2.7 sigma) off its
  // target here, yet the move is realised: judged by the share of its
  // rigid delta, with the bend reported apart, it is honoured (reusex-9ge).
  CHECK_THAT(p1.report, ContainsSubstring("edit m-"));
  CHECK_THAT(p1.report, ContainsSubstring("(rms "));
  CHECK_THAT(p1.report, ContainsSubstring("realised "));
  CHECK_THAT(p1.report, ContainsSubstring("block bent "));
  CHECK_THAT(p1.report, !ContainsSubstring("NOT HONOURED"));
  CHECK_THAT(p1.json, ContainsSubstring("\"honoured\": true"));
  CHECK_THAT(p1.json, ContainsSubstring("\"share_trans\""));
  REQUIRE(p1.cross_segment_median_m);
  CHECK(*p1.cross_segment_median_m < 0.5 * *p0.cross_segment_median_m);
  double moved = 0.0;
  for (int k = 0; k < 2 * kPerPass; ++k)
    moved = std::max(moved, (p1.poses[k].topRightCorner<3, 1>() -
                             p0.poses[k].topRightCorner<3, 1>())
                                .norm());
  CHECK(moved > 0.05);
  // The cached cycle measures what an uncached diagnosis of the same
  // candidate poses measures, bit for bit (reusex-rcv.1.17).
  {
    reusex::ProjectDB db(project, /*readOnly=*/true);
    reusex::slam::DriftDiagnosticsOptions d;
    d.depth_stride = 2;
    d.segments = 2;
    d.doubled_walls = false;
    const auto u = reusex::slam::diagnose_drift(db, d, nullptr, p1.poses);
    std::vector<double> cross;
    for (const auto &p : u.pairs)
      if (p.a != p.b)
        cross.push_back(p.residual_m);
    std::sort(cross.begin(), cross.end());
    REQUIRE(!cross.empty());
    const double pos = 0.5 * static_cast<double>(cross.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(pos));
    const auto hi = std::min(lo + 1, cross.size() - 1);
    const double med = cross[lo] + (cross[hi] - cross[lo]) * (pos - lo);
    CHECK(*p1.cross_segment_median_m == med);
  }
  // A move alone drags the pass no edit names along (reusex-sb2): the shear
  // indicator shows it and the suggestions offer a guard anchor.
  CHECK(p1.shear.untouched == static_cast<std::size_t>(kPerPass));
  CHECK(p1.shear.change.max_m > 0.0);
  if (p1.shear.flagged > 0) {
    CHECK_THAT(p1.report, ContainsSubstring("<- SHEAR"));
    CHECK_THAT(p1.suggestions_yaml, ContainsSubstring("kind: anchor"));
  }

  // A broken file: the last good solve stays on screen, the error names
  // the edit.
  write(edits, kHeader + "edits:\n  - id: m-bad\n    kind: move\n"
                         "    nodes: [30, 40]\n    delta: {xyz: [1, 2]}\n");
  const auto p2 = session.pass(true);
  INFO(p2.report);
  CHECK_FALSE(p2.edits_ok);
  CHECK_FALSE(p2.solved);
  CHECK_THAT(p2.edits_error, ContainsSubstring("m-bad"));
  CHECK_THAT(p2.report, ContainsSubstring("INVALID: "));
  CHECK_THAT(p2.report,
             ContainsSubstring("keeping the last good solve (cycle 1)"));
  CHECK(p2.shown_solve == 1);
  CHECK(p2.poses == p1.poses);

  // An edit that parses but cannot be lowered (no node of it is solved) is
  // refused the same way.
  write(edits, kHeader + "edits:\n  - id: a-ghost\n    kind: anchor\n"
                         "    node: 999\n    pose: current\n");
  const auto p3 = session.pass(true);
  CHECK_FALSE(p3.edits_ok);
  CHECK_THAT(p3.edits_error, ContainsSubstring("a-ghost"));
  CHECK(p3.shown_solve == 1);

  // Repeatability (acceptance 3): the same file again gives bit-identical
  // poses.
  write(edits, kHeader + "edits:\n" + best);
  const auto p4 = session.pass(true);
  REQUIRE(p4.solved);
  CHECK(p4.poses == p1.poses);
  CHECK(p4.cross_segment_median_m == p1.cross_segment_median_m);

  // The project was never written (no --apply).
  CHECK(stored_poses(project) == before);
}

TEST_CASE("shear_indicator: a carried tail or a better fit is not shear",
          "[rux_app][optimize][watch]") {
  // 30 nodes on a line, three segments of ten; an edit names 11..20.
  std::vector<std::int64_t> ids;
  std::vector<Eigen::Matrix4d> ref;
  for (int k = 1; k <= 30; ++k) {
    ids.push_back(k);
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m(0, 3) = 0.1 * k;
    ref.push_back(m);
  }
  std::vector<bool> untouched(30, true);
  for (int k = 10; k < 20; ++k)
    untouched[static_cast<std::size_t>(k)] = false;
  auto shifted = [&](int lo, int hi, double dy) {
    auto out = ref;
    for (int k = lo; k <= hi; ++k)
      out[static_cast<std::size_t>(k - 1)](1, 3) += dy;
    return out;
  };
  auto fits = [](std::optional<double> up, std::optional<double> up_ref,
                 std::optional<double> down, std::optional<double> down_ref) {
    return std::vector<rux::ShearSegmentFit>{{0, 1, 10, up, up_ref},
                                             {1, 11, 20, 0.005, 0.005},
                                             {2, 21, 30, down, down_ref}};
  };

  SECTION("the tail after the move is carried: expected, not shear") {
    const auto sh =
        rux::shear_indicator(ids, shifted(11, 30, 0.1), ref, untouched,
                             fits(0.005, 0.005, 0.005, 0.005), 0.05);
    CHECK(sh.untouched == 20);
    CHECK(sh.downstream_moved == 10);
    CHECK(sh.upstream_moved == 0);
    CHECK(sh.flagged == 0);
    REQUIRE(sh.segments.size() == 2);
    CHECK(sh.segments[1].downstream);
    CHECK_FALSE(sh.segments[1].sheared);
  }
  SECTION("upstream moved to a worse fit: shear") {
    const auto sh =
        rux::shear_indicator(ids, shifted(1, 10, 0.1), ref, untouched,
                             fits(0.009, 0.005, 0.005, 0.005), 0.05);
    CHECK(sh.upstream_moved == 10);
    CHECK(sh.flagged == 10);
    REQUIRE(sh.segments.size() == 2);
    CHECK(sh.segments[0].sheared);
    CHECK_FALSE(sh.segments[0].downstream);
  }
  SECTION("upstream moved to a better fit: a spread correction, not shear") {
    const auto sh =
        rux::shear_indicator(ids, shifted(1, 10, 0.1), ref, untouched,
                             fits(0.004, 0.005, 0.005, 0.005), 0.05);
    CHECK(sh.upstream_moved == 10);
    CHECK(sh.flagged == 0);
  }
  SECTION("worse by less than 1 mm is noise") {
    const auto sh =
        rux::shear_indicator(ids, shifted(1, 10, 0.1), ref, untouched,
                             fits(0.0058, 0.005, 0.005, 0.005), 0.05);
    CHECK(sh.flagged == 0);
  }
  SECTION("without residuals the upstream movement is flagged") {
    const auto sh = rux::shear_indicator(ids, shifted(1, 30, 0.1), ref,
                                         untouched, {}, 0.05);
    CHECK(sh.upstream_moved == 10);
    CHECK(sh.downstream_moved == 10);
    CHECK(sh.flagged == 10);
  }
}

TEST_CASE("fast_graph_at: the no-edit fast solve is the reference",
          "[rux_app][optimize][watch]") {
  TempDir dir("optimize_watch_fast_graph");
  const fs::path project = dir.path / "p.rux";
  make_project(project);
  reusex::ProjectDB db(project, /*readOnly=*/true);
  SubcommandOptimizeOptions o;
  auto options = plane_graph_options(o);
  const auto graph = reusex::geometry::load_stored_pose_graph(db, options);
  // The reference: the true poses (the break undone).
  std::vector<Eigen::Matrix4d> reference;
  for (const auto &f : graph.frames)
    reference.push_back(true_pose(f.node_id - 1));
  const auto fast = rux::fast_graph_at(graph, reference);
  REQUIRE(fast.frames.size() == graph.frames.size());
  CHECK(fast.seed_stored == reference);
  for (const auto &f : fast.frames)
    CHECK((f.points == nullptr || f.points->empty()));
  options.use_plane_factors = false;
  options.manhattan = false;
  const auto solve =
      reusex::geometry::solve_stored_pose_graph(db, fast, options);
  REQUIRE(solve.poses.size() == reference.size());
  double worst = 0.0;
  for (std::size_t k = 0; k < reference.size(); ++k)
    worst =
        std::max(worst, (solve.poses[k] - reference[k]).cwiseAbs().maxCoeff());
  CHECK(worst < 1e-6);
}

TEST_CASE("OptimizeWatchSession: --fast starts from plain optimize, shows it, "
          "never suggests the gauge segment",
          "[rux_app][optimize][watch]") {
  TempDir dir("optimize_watch_reference");
  const fs::path project = dir.path / "p.rux";
  const fs::path edits = dir.path / "e.yaml";
  make_project(project);
  write(edits, kHeader + "edits: []\n");
  RuxOptions global;
  global.project_db = project;

  // Plane factors on (the default): the reference is plain optimize.
  auto opt = watch_options(edits);
  opt.min_plane_inliers = SubcommandOptimizeOptions{}.min_plane_inliers;
  auto full_opt = opt;
  full_opt.fast = false;
  rux::OptimizeWatchSession full(full_opt, global);
  const auto f0 = full.pass(true);
  INFO(f0.report);
  REQUIRE(f0.solved);
  rux::OptimizeWatchSession fast(opt, global);
  const auto p0 = fast.pass(true);
  INFO(p0.report);
  REQUIRE(p0.solved);
  // reusex-rcv.1.21: the fast no-edit solve IS the full one, not the drift.
  REQUIRE(p0.poses.size() == f0.poses.size());
  double worst = 0.0;
  for (std::size_t k = 0; k < p0.poses.size(); ++k)
    worst = std::max(worst, (p0.poses[k].topRightCorner<3, 1>() -
                             f0.poses[k].topRightCorner<3, 1>())
                                .norm());
  CHECK(worst < 1e-4);
  // reusex-rcv.1.24: the reference is on every block.
  CHECK_THAT(p0.report, ContainsSubstring("vs plain optimize"));
  REQUIRE(p0.reference_cross_segment_median_m);
  REQUIRE(p0.cross_segment_median_m);
  CHECK(std::abs(*p0.reference_cross_segment_median_m -
                 *p0.cross_segment_median_m) < 1e-3);
  CHECK(p0.shear.change.max_m < 1e-4);

  // reusex-rcv.1.23: with the break kept (no plane landmark), segment 0 --
  // which holds the gauge, node 1 -- is never suggested.
  rux::OptimizeWatchSession drifted(watch_options(edits), global);
  const auto d0 = drifted.pass(true);
  INFO(d0.report);
  REQUIRE(d0.solved);
  REQUIRE_FALSE(d0.suggestion_items.empty());
  for (const auto &item : d0.suggestion_items)
    CHECK_THAT(item, !ContainsSubstring("nodes: [1, "));
  CHECK_THAT(d0.report, ContainsSubstring("holds the gauge"));
  REQUIRE(d0.beats_reference);
  CHECK_FALSE(*d0.beats_reference); // no edits: equal, not better

  // The verdict against the reference follows the cross-segment median.
  write(edits, kHeader + "edits:\n" + d0.suggestion_items.front());
  const auto d1 = drifted.pass(true);
  INFO(d1.report);
  REQUIRE(d1.solved);
  REQUIRE(d1.reference_cross_segment_median_m);
  REQUIRE(d1.cross_segment_median_m);
  REQUIRE(d1.beats_reference);
  CHECK(*d1.beats_reference ==
        (*d1.cross_segment_median_m < *d1.reference_cross_segment_median_m));
}

TEST_CASE("candidate_graph_edges colours by the candidate solve",
          "[rux_app][optimize][watch]") {
  // reusex-rcv.1.18: three stored rows (one rejected) plus one odometry
  // edge derived in memory; the solve reports residuals for two of them.
  reusex::geometry::StoredPoseGraph g;
  const auto edge = [](int a, int b, const char *type) {
    reusex::geometry::StoredPoseGraph::Edge e;
    e.from_node_id = a;
    e.to_node_id = b;
    e.edge_type = type;
    return e;
  };
  g.edges = {edge(1, 2, "odometry"), edge(1, 5, "loop_closure"),
             edge(2, 6, "loop_closure"), edge(5, 6, "odometry")};
  reusex::geometry::PlaneGraphResult res;
  res.edges.push_back({0, 1, "odometry", 0.25, 1.0, 0});
  res.edges.push_back({0, 4, "loop_closure", 42.0, 1.0, 1});
  res.edges.push_back({0, 4, "loop_closure", 7.0, 1.0, -1}); // untagged
  res.edges.push_back({0, 4, "loop_closure", 9.0, 1.0, 17}); // out of range
  const auto out =
      rux::candidate_graph_edges(g, {"auto", "confirmed", "rejected"}, res);
  REQUIRE(out.size() == 4);
  CHECK(out[0].residual == 0.25);
  CHECK(out[1].residual == 42.0);
  CHECK(out[1].status == "confirmed");
  CHECK(std::isnan(out[2].residual)); // not solved: grey
  CHECK(out[2].status == "rejected");
  CHECK(out[3].status == "auto"); // in-memory odometry
  CHECK(std::isnan(out[3].residual));
  CHECK(out[1].from_node_id == 1);
  CHECK(out[1].to_node_id == 5);
  CHECK(out[1].edge_type == "loop_closure");
}

TEST_CASE("OptimizeWatchSession --apply writes transform, never initial",
          "[rux_app][optimize][watch]") {
  TempDir dir("optimize_watch_apply");
  const fs::path project = dir.path / "p.rux";
  const fs::path edits = dir.path / "e.yaml";
  make_project(project);
  std::vector<std::array<double, 16>> initial_before;
  {
    reusex::ProjectDB db(project, true);
    for (int id : db.sensor_frame_ids())
      initial_before.push_back(*db.initial_pose(id));
  }
  write(edits, kHeader + "edits:\n  - id: m\n    kind: move\n"
                         "    nodes: [25, 48]\n    delta: {xyz: [-0.1, 0, "
                         "0]}\n    sigma: {trans_m: 0.002, rot_deg: 0.05}\n");
  auto opt = watch_options(edits);
  opt.apply = true;
  RuxOptions global;
  global.project_db = project;
  {
    rux::OptimizeWatchSession session(opt, global);
    const auto p = session.pass(true);
    INFO(p.report);
    REQUIRE(p.solved);
    CHECK(p.applied);
    CHECK_THAT(p.report, ContainsSubstring("WRITTEN"));
  }
  reusex::ProjectDB db(project, true);
  const auto ids = db.sensor_frame_ids();
  const auto p30 = pg::matrix_from_row_major(db.sensor_frame_pose(30));
  const auto i30 = pg::matrix_from_row_major(*db.initial_pose(30));
  CHECK((p30.topRightCorner<3, 1>() - i30.topRightCorner<3, 1>()).norm() >
        0.05);
  for (std::size_t k = 0; k < ids.size(); ++k)
    CHECK(*db.initial_pose(ids[k]) == initial_before[k]);
}

TEST_CASE("rux optimize --watch --watch-cycles 0 solves once and returns",
          "[rux_app][optimize][watch][cli]") {
  TempDir dir("optimize_watch_cli");
  const fs::path project = dir.path / "p.rux";
  const fs::path edits = dir.path / "e.yaml";
  make_project(project);
  write(edits, kHeader + "edits: []\n");
  const auto before = stored_poses(project);

  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_optimize(app, global);
  std::vector<std::string> args{"-p",
                                project.string(),
                                "optimize",
                                "--edits",
                                edits.string(),
                                "--watch",
                                "--fast",
                                "--watch-cycles",
                                "0",
                                "--segments",
                                "2",
                                "--drift-depth-stride",
                                "4",
                                "--json",
                                (dir.path / "m.json").string(),
                                "--render",
                                (dir.path / "img" / "q.png").string(),
                                "--render-width",
                                "160",
                                "--render-height",
                                "120"};
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  REQUIRE_NOTHROW(app.parse(reversed));
  CHECK(fs::exists(dir.path / "m.json"));
  if (!fs::exists(dir.path / "img" / "q.passes.png"))
    WARN("No offscreen GL here (or the render failed): images not checked");
  CHECK(stored_poses(project) == before);

  // --render without --watch is refused, as is --apply with --dry-run.
  CLI::App app2{"rux"};
  auto global2 = std::make_shared<RuxOptions>();
  setup_subcommand_optimize(app2, global2);
  app2.get_subcommand("optimize")->callback([] {});
  std::vector<std::string> bad{"x.png", "--render", "optimize"};
  CHECK_THROWS_AS(app2.parse(bad), CLI::ParseError);
  CLI::App app3{"rux"};
  auto global3 = std::make_shared<RuxOptions>();
  setup_subcommand_optimize(app3, global3);
  app3.get_subcommand("optimize")->callback([] {});
  std::vector<std::string> bad2{"--dry-run",    "--apply", "--watch",
                                edits.string(), "--edits", "optimize"};
  CHECK_THROWS_AS(app3.parse(bad2), CLI::ParseError);
}

// ── reusex-rcv.1.28 ─────────────────────────────────────────────────────────

namespace {

reusex::core::PoseEdit move_edit(const std::string &id, std::int64_t first,
                                 std::int64_t last, double x) {
  reusex::core::PoseEdit e;
  e.id = id;
  e.kind = reusex::core::PoseEditKind::move;
  e.nodes.first = first;
  e.nodes.last = last;
  e.delta.xyz = {x, 0.0, 0.0};
  e.delta.yaw_deg = 1.5;
  e.delta.pivot = std::array<double, 3>{1.0, 2.0, 1.4};
  e.sigma_trans_m = 0.005;
  e.sigma_rot_deg = 0.1;
  return e;
}

reusex::geometry::EditResidual residual(const std::string &tag, double rms_t,
                                        double rms_r, bool inlier = true) {
  reusex::geometry::EditResidual r;
  r.tag = tag;
  r.rms_trans_m = rms_t;
  r.rms_rot_deg = rms_r;
  r.trans_m = 1.5 * rms_t;
  r.rot_deg = 1.5 * rms_r;
  r.gnc_inlier = inlier;
  r.factors = 320;
  return r;
}

} // namespace

TEST_CASE("judge_edit: a move is honoured by its block's residual, not by "
          "GNC alone",
          "[rux_app][optimize][watch]") {
  const auto m = move_edit("m-2565-2884", 2565, 2884, 0.02);
  // The calibration case (41069051-full-d0.25): every factor a GNC inlier,
  // the block 12 cm / 6.6 deg off its target at sigma 5 mm / 0.1 deg.
  const auto bad = rux::judge_edit(residual(m.id, 0.12, 6.632), &m);
  CHECK_FALSE(bad.honoured);
  CHECK_THAT(bad.why, ContainsSubstring("2 sigma"));
  // Within 2 sigma in both: honoured.
  CHECK(rux::judge_edit(residual(m.id, 0.009, 0.19), &m).honoured);
  // Off in rotation only, or in translation only: not honoured.
  CHECK_FALSE(rux::judge_edit(residual(m.id, 0.002, 0.25), &m).honoured);
  CHECK_FALSE(rux::judge_edit(residual(m.id, 0.011, 0.01), &m).honoured);
  // GNC dropped it: not honoured whatever the residual.
  const auto dropped = rux::judge_edit(residual(m.id, 0.0, 0.0, false), &m);
  CHECK_FALSE(dropped.honoured);
  CHECK_THAT(dropped.why, ContainsSubstring("GNC"));
  // An anchor is judged against its own sigma the same way.
  reusex::core::PoseEdit a;
  a.id = "a-2885";
  a.kind = reusex::core::PoseEditKind::anchor;
  a.sigma_trans_m = 0.01;
  a.sigma_rot_deg = 0.2;
  CHECK_FALSE(rux::judge_edit(residual(a.id, 0.0473, 1.928), &a).honoured);
  CHECK(rux::judge_edit(residual(a.id, 0.0059, 0.071), &a).honoured);
  // same_surface: its residual is not in its sigma's units, GNC only.
  reusex::core::PoseEdit ss;
  ss.id = "s-wall";
  ss.kind = reusex::core::PoseEditKind::same_surface;
  CHECK(rux::judge_edit(residual(ss.id, 0.5, 5.0), &ss).honoured);
  // No edit to read a sigma from: GNC only.
  CHECK(rux::judge_edit(residual("x", 0.5, 5.0), nullptr).honoured);
}

// ── reusex-9ge ──────────────────────────────────────────────────────────────

namespace {

/// A block of 11 poses along a 2 m line, each yawed a little, so the rigid
/// fit has a rotation to find.
std::vector<Eigen::Matrix4d> block_poses() {
  std::vector<Eigen::Matrix4d> out;
  for (int k = 0; k <= 10; ++k) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(0.05 * k, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    m.topRightCorner<3, 1>() = Eigen::Vector3d(0.2 * k, 1.0, 1.4);
    out.push_back(m);
  }
  return out;
}

/// [R_z(yaw), t] about @p c: p' = c + t + R (p - c).
Eigen::Matrix4d delta_about(const Eigen::Vector3d &t, double yaw_deg,
                            const Eigen::Vector3d &c) {
  Eigen::Matrix4d d = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d r =
      Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  d.topLeftCorner<3, 3>() = r;
  d.topRightCorner<3, 1>() = c + t - r * c;
  return d;
}

std::vector<Eigen::Matrix4d> moved_by(const Eigen::Matrix4d &d,
                                      const std::vector<Eigen::Matrix4d> &in) {
  std::vector<Eigen::Matrix4d> out;
  for (const auto &m : in)
    out.push_back(d * m);
  return out;
}

} // namespace

TEST_CASE("move_realisation: how much of a move's rigid delta the solve "
          "realised, apart from the block's own bend",
          "[rux_app][optimize][watch]") {
  const auto initial = block_poses();
  const Eigen::Vector3d c(1.0, 1.0, 1.4); // the block's centroid
  const Eigen::Matrix4d d = delta_about({0.3, 0.0, 0.0}, 10.0, c);

  // All of it.
  const auto all = rux::move_realisation(initial, moved_by(d, initial), d);
  CHECK(all.nodes == 11);
  CHECK(std::abs(all.move_trans_m - 0.3) < 1e-9);
  CHECK(std::abs(all.move_rot_deg - 10.0) < 1e-9);
  CHECK(all.remaining_trans_m < 1e-9);
  CHECK(all.remaining_rot_deg < 1e-9);
  CHECK(std::abs(all.share_trans - 1.0) < 1e-9);
  CHECK(std::abs(all.share_rot - 1.0) < 1e-9);
  CHECK(all.bend_trans_m < 1e-9);

  // None of it: the block stayed where it was.
  const auto none = rux::move_realisation(initial, initial, d);
  CHECK(std::abs(none.share_trans) < 1e-9);
  CHECK(std::abs(none.share_rot) < 1e-9);
  CHECK(std::abs(none.remaining_trans_m - 0.3) < 1e-9);
  CHECK(std::abs(none.remaining_rot_deg - 10.0) < 1e-9);

  // Half of it.
  const auto half = rux::move_realisation(
      initial, moved_by(delta_about({0.15, 0.0, 0.0}, 5.0, c), initial), d);
  CHECK(std::abs(half.share_trans - 0.5) < 1e-9);
  CHECK(std::abs(half.share_rot - 0.5) < 1e-9);

  // Pushed the other way: a negative share.
  const auto back = rux::move_realisation(
      initial, moved_by(delta_about({-0.1, 0.0, 0.0}, 0.0, c), initial), d);
  CHECK(back.share_trans < 0.0);

  // All of it, but the block bent +-1 cm about its rigid pose (the fast
  // graph measures plain optimize's shape, not the initial one's): the
  // move is realised and the bend is reported apart.
  auto bent = moved_by(d, initial);
  for (std::size_t k = 0; k < bent.size(); ++k)
    bent[k](2, 3) += (k % 2 == 0 ? 0.01 : -0.01);
  const auto b = rux::move_realisation(initial, bent, d);
  CHECK(b.share_trans > 0.99);
  CHECK(b.share_rot > 0.99);
  CHECK(std::abs(b.bend_trans_m - 0.01) < 1e-3);
}

TEST_CASE("judge_edit: a move is honoured by the share of it the solve "
          "realised, not by the block's bend",
          "[rux_app][optimize][watch]") {
  const auto m = move_edit("m-1283-1603", 1283, 1603, 0.3);
  // The calibration case (41069051-full-d1.0, round 1, the move alone): the
  // block RMS 15.5 mm / 0.29 deg against 2 sigma = 10 mm / 0.2 deg, but 99%
  // / 98% of a 0.306 m / 17.5 deg move realised; the RMS is the block's
  // bend (15.1 mm / 0.45 deg), which no rigid move of it can remove.
  rux::MoveRealisation cal;
  cal.nodes = 321;
  cal.move_trans_m = 0.3059;
  cal.move_rot_deg = 17.51;
  cal.remaining_trans_m = 0.0022;
  cal.remaining_rot_deg = 0.32;
  cal.share_trans = 0.99;
  cal.share_rot = 0.98;
  cal.bend_trans_m = 0.0151;
  cal.bend_rot_deg = 0.45;
  const auto r = residual(m.id, 0.0155, 0.288);
  CHECK_FALSE(rux::judge_edit(r, &m).honoured); // the RMS rule alone
  CHECK(rux::judge_edit(r, &m, &cal).honoured);

  // Fought off: 30% of the move realised, 0.21 m / 12 deg from its target.
  auto fought = cal;
  fought.share_trans = 0.3;
  fought.share_rot = 0.31;
  fought.remaining_trans_m = 0.214;
  fought.remaining_rot_deg = 12.1;
  const auto f = rux::judge_edit(r, &m, &fought);
  CHECK_FALSE(f.honoured);
  CHECK_THAT(f.why, ContainsSubstring("30%"));
  CHECK_THAT(f.why, ContainsSubstring("0.214 m"));
  CHECK_THAT(f.why, ContainsSubstring("12.10 deg"));
  CHECK_THAT(f.why, ContainsSubstring("from its target"));
  // Short in rotation only.
  auto rot = cal;
  rot.share_rot = 0.5;
  rot.remaining_rot_deg = 8.7;
  const auto fr = rux::judge_edit(r, &m, &rot);
  CHECK_FALSE(fr.honoured);
  CHECK_THAT(fr.why, ContainsSubstring("50%"));
  // Pushed back past where it started.
  auto back = cal;
  back.share_trans = -0.2;
  back.remaining_trans_m = 0.367;
  CHECK_FALSE(rux::judge_edit(r, &m, &back).honoured);
  // A small move: a low share, but it ended within 2 sigma of its target.
  auto small = cal;
  small.move_trans_m = 0.004;
  small.remaining_trans_m = 0.003;
  small.share_trans = 0.25;
  CHECK(rux::judge_edit(r, &m, &small).honoured);
  // Exactly at the share bound: honoured.
  auto edge = cal;
  edge.share_trans = rux::kHonouredMoveShare;
  edge.remaining_trans_m = 0.06;
  CHECK(rux::judge_edit(r, &m, &edge).honoured);
  // GNC dropped it: not honoured, however much was realised.
  CHECK_FALSE(
      rux::judge_edit(residual(m.id, 0.0155, 0.288, false), &m, &cal).honoured);
  // A move restricted to some DoFs keeps the RMS rule (its residual counts
  // only the constrained directions; a rigid share would count all six).
  auto masked = m;
  masked.dofs = reusex::core::Dof::x | reusex::core::Dof::y;
  CHECK_FALSE(rux::judge_edit(r, &masked, &cal).honoured);
  // The edit line's summary.
  const std::string s = rux::describe_realisation(cal);
  CHECK_THAT(s, ContainsSubstring("realised 99% / 98% of 0.306 m / 17.51 deg"));
  CHECK_THAT(s, ContainsSubstring("2.2 mm / 0.32 deg from target"));
  CHECK_THAT(s, ContainsSubstring("bent 15.1 mm / 0.45 deg"));
}

TEST_CASE("blocked_replacement: no REPLACES ping-pong on an unrealised move",
          "[rux_app][optimize][watch]") {
  // m-2565-2884-2 as the loop printed it on 41069051-full-d0.25: segment 8,
  // predicted 8.2 -> 4.8 mm. The next cycle measured 8.9 mm.
  auto pasted = move_edit("m-2565-2884-2", 2565, 2884, 0.020);
  pasted.note = "cycle 0: seg 8 (2565-2884) into the rest 8.2 -> 4.8 mm "
                "(segment ICP on the candidate solve); replaces m-2565-2884";
  const auto p = rux::move_prediction(pasted);
  REQUIRE(p);
  CHECK(p->segment == 8);
  CHECK(std::abs(p->before_m - 0.0082) < 1e-12);
  CHECK(std::abs(p->after_m - 0.0048) < 1e-12);

  const auto why = rux::blocked_replacement(pasted, {{8, 0.0089}});
  REQUIRE(why);
  CHECK_THAT(*why, ContainsSubstring("m-2565-2884-2"));
  CHECK_THAT(*why, ContainsSubstring("8.2 -> 4.8 mm"));
  CHECK_THAT(*why, ContainsSubstring("8.9 mm"));
  CHECK_THAT(*why, ContainsSubstring("not realised"));
  CHECK(rux::move_realised(pasted, {{8, 0.0089}}) == false);

  // Realised (8.2 -> 5.5 mm is more than half of the predicted 3.4 mm): a
  // further move of the segment may replace it.
  CHECK(rux::move_realised(pasted, {{8, 0.0055}}) == true);
  CHECK_FALSE(rux::blocked_replacement(pasted, {{8, 0.0055}}));
  // Less than half of the gain: not realised.
  CHECK_FALSE(rux::move_realised(pasted, {{8, 0.0070}}).value());
  // No residual for its segment this cycle: nothing to judge by.
  CHECK_FALSE(rux::blocked_replacement(pasted, {{7, 0.0089}}));
  // A hand-written move (no prediction in its note) is never held to one.
  auto hand = pasted;
  hand.note = "the north wall, by eye";
  CHECK_FALSE(rux::move_prediction(hand));
  CHECK_FALSE(rux::blocked_replacement(hand, {{8, 0.0089}}));
  // Only moves carry a prediction.
  reusex::core::PoseEdit a;
  a.kind = reusex::core::PoseEditKind::anchor;
  a.note = pasted.note;
  CHECK_FALSE(rux::move_prediction(a));
}

TEST_CASE("OptimizeWatchSession: a held move is NOT HONOURED and its "
          "replacement is held back",
          "[rux_app][optimize][watch]") {
  TempDir dir("optimize_watch_pingpong");
  const fs::path project = dir.path / "p.rux";
  const fs::path edits = dir.path / "e.yaml";
  make_project(project);
  write(edits, kHeader + "edits: []\n");
  RuxOptions global;
  global.project_db = project;
  rux::OptimizeWatchSession session(watch_options(edits), global);

  const auto p0 = session.pass(true);
  INFO(p0.report);
  REQUIRE(p0.solved);
  REQUIRE_FALSE(p0.suggestion_items.empty());
  const std::string best = p0.suggestion_items.front();
  REQUIRE_THAT(best, ContainsSubstring("kind: move"));

  // The suggested move, pasted with an anchor that holds the broken pass
  // where it is: the block cannot reach its target, its predicted gain does
  // not materialise.
  write(edits, kHeader + "edits:\n" + best +
                   "  - id: a-hold\n    kind: anchor\n    node: 36\n"
                   "    pose: current\n"
                   "    sigma: {trans_m: 0.001, rot_deg: 0.02}\n");
  // A new session, as the calibration harness runs one process per cycle:
  // the prediction travels in the move's note, not in session memory.
  rux::OptimizeWatchSession next(watch_options(edits), global);
  const auto p1 = next.pass(true);
  INFO(p1.report);
  REQUIRE(p1.solved);
  CHECK_THAT(p1.report, ContainsSubstring("NOT HONOURED"));
  // reusex-9ge: a real conflict, not a bend -- the anchor fights most of
  // the move off (33% / 58% of it realised here).
  CHECK_THAT(p1.report, ContainsSubstring("fought the move off"));
  // The same segment's next move would REPLACE the pasted one: held back,
  // and the block says why.
  CHECK_THAT(p1.suggestions_yaml,
             !ContainsSubstring("REPLACES") && !ContainsSubstring("replaces"));
  CHECK_THAT(p1.report, ContainsSubstring("not realised"));
  CHECK_THAT(p1.json, ContainsSubstring("held_back"));
}

TEST_CASE("pinning_anchors: the anchors a move's block would override",
          "[rux_app][optimize][watch]") {
  reusex::core::PoseEdits edits;
  edits.edits.push_back(move_edit("m-1283-1603", 1283, 1603, 0.02));
  auto guard = [](const std::string &id, std::int64_t node) {
    reusex::core::PoseEdit a;
    a.id = id;
    a.kind = reusex::core::PoseEditKind::anchor;
    a.nodes.first = a.nodes.last = node;
    return a;
  };
  edits.edits.push_back(guard("a-1764", 1764));
  edits.edits.push_back(guard("a-2083", 2083));
  // 41069051-full-d1.0 (reusex-r5z): m-1604-1923 covers guard a-1764.
  CHECK(rux::pinning_anchors(edits, 1604, 1923) ==
        std::vector<std::string>{"a-1764"});
  CHECK(rux::pinning_anchors(edits, 1604, 2083) ==
        (std::vector<std::string>{"a-1764", "a-2083"}));
  // The ends count; a move is never an anchor.
  CHECK(rux::pinning_anchors(edits, 1764, 1764) ==
        std::vector<std::string>{"a-1764"});
  CHECK(rux::pinning_anchors(edits, 1283, 1603).empty());
  CHECK(rux::pinning_anchors(edits, 1765, 2082).empty());
}

TEST_CASE("OptimizeWatchSession: a suggested move over a pinned node says it "
          "overrides the anchor",
          "[rux_app][optimize][watch]") {
  TempDir dir("optimize_watch_pinned");
  const fs::path project = dir.path / "p.rux";
  const fs::path edits = dir.path / "e.yaml";
  make_project(project);
  // A shear guard as the loop writes one, pinning node 36 of the broken
  // pass (25..48) where it is.
  write(edits, kHeader + "edits:\n"
                         "  - id: a-36\n    kind: anchor\n    node: 36\n"
                         "    pose: current\n"
                         "    sigma: {trans_m: 0.01, rot_deg: 0.2}\n");
  RuxOptions global;
  global.project_db = project;
  rux::OptimizeWatchSession session(watch_options(edits), global);
  const auto p0 = session.pass(true);
  INFO(p0.report);
  REQUIRE(p0.solved);
  REQUIRE_FALSE(p0.suggestion_items.empty());
  const std::string move = p0.suggestion_items.front();
  REQUIRE_THAT(move, ContainsSubstring("kind: move"));
  // reusex-r5z: the block covers node 36. The move and the guard conflict
  // by construction, so the comment says the move overrides the guard and
  // to drop it, the note records it, and the JSON lists it.
  CHECK_THAT(move, ContainsSubstring("OVERRIDES a-36"));
  CHECK_THAT(move, ContainsSubstring("drop a-36"));
  CHECK_THAT(move, ContainsSubstring("overrides a-36"));
  CHECK_THAT(p0.json, ContainsSubstring("\"overrides\""));

  // Pasted with the guard dropped, as the comment says: the move is
  // realised and honoured.
  write(edits, kHeader + "edits:\n" + move);
  rux::OptimizeWatchSession next(watch_options(edits), global);
  const auto p1 = next.pass(true);
  INFO(p1.report);
  REQUIRE(p1.solved);
  CHECK_THAT(p1.report, !ContainsSubstring("NOT HONOURED"));
  REQUIRE(p1.cross_segment_median_m);
  REQUIRE(p0.cross_segment_median_m);
  CHECK(*p1.cross_segment_median_m < 0.5 * *p0.cross_segment_median_m);
}
