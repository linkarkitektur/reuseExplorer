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
#include <reusex/slam/pose_graph.hpp>

#include "../../support/temp_path.hpp"

#include <CLI/CLI.hpp>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
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
  CHECK_THAT(p1.report, ContainsSubstring("honoured"));
  REQUIRE(p1.cross_segment_median_m);
  CHECK(*p1.cross_segment_median_m < 0.5 * *p0.cross_segment_median_m);
  double moved = 0.0;
  for (int k = 0; k < 2 * kPerPass; ++k)
    moved = std::max(moved, (p1.poses[k].topRightCorner<3, 1>() -
                             p0.poses[k].topRightCorner<3, 1>())
                                .norm());
  CHECK(moved > 0.05);
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
