// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux optimize` on the stored pose graph (spec 2026-10-13 §4.3): seeds from
// initial_transform, never replaces edges, writes per-edge metrics, and is
// repeatable. Frames look at one wall 1.5 m away so they yield surfels and a
// plane landmark.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/core/pose_edits.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/pose_graph.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::test_support;
namespace pg = reusex::core::pose_graph;
namespace geo = reusex::geometry;
namespace slam = reusex::slam;

namespace {

constexpr int kW = 64;
constexpr int kH = 48;
constexpr int kFrames = 5;

void seed_wall_frame(ProjectDB &db, int id,
                     const std::array<double, 16> &pose) {
  db.save_sensor_frame(id, make_color(kW, kH), make_depth(kW, kH, 1.5),
                       cv::Mat(), pose,
                       make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH),
                       static_cast<double>(id));
}

std::array<double, 16> truth(int k) { return translation_pose(0.1 * k, 0, 0); }

geo::PlaneGraphOptions wall_options() {
  geo::PlaneGraphOptions o;
  o.surfel.min_distance = 0.0f;
  o.surfel.max_distance = 4.0f;
  o.surfel.sampling_factor = 2;
  o.surfel.confidence_threshold = 2;
  o.surfel.normal_radius = 0.1f;
  o.surfel.voxel_size = 0.0f;
  o.surfel.apply_depth_filters = false;
  o.min_plane_inliers = 50;
  o.min_landmark_observations = 2;
  o.min_landmark_spread_ratio = 0.0f;
  o.assoc_overlap_margin = 2.0f;
  o.assoc_rounds = 1;
  return o;
}

void make_project(ProjectDB &db) {
  for (int k = 0; k < kFrames; ++k)
    seed_wall_frame(db, k, truth(k));
}

double dist(const std::array<double, 16> &a, const std::array<double, 16> &b) {
  return (pg::matrix_from_row_major(a) - pg::matrix_from_row_major(b)).norm();
}

/// Captures `warn` lines from the library for the duration of a test.
class LogCapture {
    public:
  LogCapture() : previous_(reusex::core::get_log_level()) {
    reusex::core::set_log_level(reusex::core::LogLevel::info);
    reusex::core::set_log_handler(
        [this](reusex::core::LogLevel level, std::string_view message) {
          if (level == reusex::core::LogLevel::warn)
            warnings_.emplace_back(message);
        });
  }
  ~LogCapture() {
    reusex::core::reset_log_handler();
    reusex::core::set_log_level(previous_);
  }
  bool saw(std::string_view needle) const {
    for (const auto &w : warnings_)
      if (w.find(needle) != std::string::npos)
        return true;
    return false;
  }

    private:
  reusex::core::LogLevel previous_;
  std::vector<std::string> warnings_;
};

} // namespace

TEST_CASE("OptimizeStoredGraph_SeedsFromInitialPoses_NotFromTransform",
          "[slam][optimize][stored_graph]") {
  TempPath tmp("test_optimize_stored_seed");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db); // initial = truth, odometry from truth
  for (int k = 0; k < kFrames; ++k)
    db.update_sensor_frame_pose(k, translation_pose(5.0 + k, 3.0, 0.0));

  auto o = wall_options();
  o.use_plane_factors = false; // odometry + gauge: the solve IS the seed
  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  for (int k = 0; k < kFrames; ++k) {
    INFO("frame " << k);
    CHECK(dist(db.sensor_frame_pose(k), truth(k)) < 1e-6);
    CHECK(*db.initial_pose(k) == truth(k)); // never written by optimize
  }
}

TEST_CASE("OptimizeStoredGraph_RunTwice_IsRepeatable",
          "[slam][optimize][stored_graph]") {
  TempPath tmp("test_optimize_stored_repeat");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db); // odometry measured on the true trajectory
  // Seed frame 3 off the wall so the solve has work to do.
  db.set_initial_pose(3, translation_pose(0.3, 0.0, 0.05));
  const auto o = wall_options();

  const auto r1 = geo::optimize_sensor_poses(db, o);
  REQUIRE(r1.converged);
  std::vector<std::array<double, 16>> first;
  for (int k = 0; k < kFrames; ++k)
    first.push_back(db.sensor_frame_pose(k));
  const auto edges1 = db.list_pose_graph_edges();

  const auto r2 = geo::optimize_sensor_poses(db, o);
  REQUIRE(r2.converged);
  for (int k = 0; k < kFrames; ++k) {
    INFO("frame " << k);
    CHECK(dist(db.sensor_frame_pose(k), first[static_cast<size_t>(k)]) < 1e-9);
  }
  const auto edges2 = db.list_pose_graph_edges();
  REQUIRE(edges2.size() == edges1.size());
  for (size_t i = 0; i < edges2.size(); ++i) {
    CHECK(edges2[i].id == edges1[i].id);
    CHECK(edges2[i].status == edges1[i].status);
    CHECK(edges2[i].transform == edges1[i].transform);
    CHECK(std::isfinite(edges2[i].residual));
    CHECK(edges2[i].deviation_m.has_value());
    CHECK(edges2[i].seed_disagreement_m.has_value());
  }
  CHECK(r1.edges_with_metrics == kFrames - 1);
}

TEST_CASE("OptimizeStoredGraph_NeverRemovesEdgesOrChangesStatus",
          "[slam][optimize][stored_graph]") {
  LogCapture log;
  TempPath tmp("test_optimize_stored_keep");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  auto loop = [&](int a, int b, const char *status, bool measured) {
    ProjectDB::PoseGraphEdge e;
    e.from_node_id = a;
    e.to_node_id = b;
    e.edge_type = "loop_closure";
    if (measured)
      e.transform = pg::row_major_from_matrix(
          pg::matrix_from_row_major(truth(a)).inverse() *
          pg::matrix_from_row_major(truth(b)));
    e.source = measured ? "manual" : "legacy";
    e.status = status;
    REQUIRE(db.upsert_pose_graph_edge(e));
  };
  loop(0, 4, "confirmed", true);
  loop(1, 3, "auto", true);
  REQUIRE(db.set_pose_graph_edge_status(1, 3, "loop_closure", "rejected"));
  loop(0, 2, "auto", false);

  auto o = wall_options();
  o.use_plane_factors = false;
  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  CHECK(r.graph_edges == kFrames); // 4 odometry + the confirmed loop
  CHECK(db.list_pose_graph_edges().size() == 7);

  const auto confirmed = db.pose_graph_edge(0, 4, "loop_closure");
  const auto rejected = db.pose_graph_edge(1, 3, "loop_closure");
  const auto legacy = db.pose_graph_edge(0, 2, "loop_closure");
  CHECK(confirmed->status == "confirmed");
  CHECK(std::isfinite(confirmed->residual));
  CHECK(rejected->status == "rejected");
  CHECK(std::isnan(rejected->residual));
  CHECK(rejected->deviation_m.has_value()); // still measured against the solve
  CHECK(legacy->status == "auto");
  CHECK(std::isnan(legacy->residual));
  CHECK_FALSE(legacy->deviation_m.has_value());
  CHECK(log.saw("carry no measurement"));
}

TEST_CASE("OptimizeStoredGraph_RejectedOdometry_IsBridgedAndWarned",
          "[slam][optimize][stored_graph]") {
  // Review focus 3.
  LogCapture log;
  TempPath tmp("test_optimize_stored_bridge");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  REQUIRE(db.set_pose_graph_edge_status(2, 3, "odometry", "rejected"));

  auto o = wall_options();
  o.use_plane_factors = false;
  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  CHECK(r.bridge_edges == 1);
  CHECK(log.saw("bridged"));
  CHECK(db.pose_graph_edge(2, 3, "odometry")->status == "rejected");
  for (int k = 0; k < kFrames; ++k)
    CHECK(dist(db.sensor_frame_pose(k), truth(k)) < 1e-6);
}

namespace {

/// reusex-39h: a chain whose odometry link 2-3 is wrong. The current poses
/// (from which 'optimize' derives odometry when there is no stored graph)
/// have frames 3 and 4 turned 10 deg about z and shifted 0.05 m, so the
/// derived 2-3 link carries the jump while 3-4 stays right.
void make_broken_chain_project(ProjectDB &db) {
  Eigen::Matrix4d B = Eigen::Matrix4d::Identity();
  B.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  B.topRightCorner<3, 1>() = Eigen::Vector3d(0.0, 0.05, 0.0);
  for (int k = 0; k < kFrames; ++k) {
    Eigen::Matrix4d p = pg::matrix_from_row_major(truth(k));
    if (k >= 3)
      p = B * p;
    seed_wall_frame(db, k, pg::row_major_from_matrix(p));
  }
}

reusex::core::PoseEdit reject_2_3() {
  reusex::core::PoseEdit x;
  x.id = "x-bad-odometry";
  x.kind = reusex::core::PoseEditKind::reject_edge;
  x.nodes.first = x.nodes.last = 2;
  x.nodes_b.first = x.nodes_b.last = 3;
  x.edge_type = std::string(pg::kTypeOdometry);
  return x;
}

} // namespace

TEST_CASE("OptimizeStoredGraph_RejectEdgeOnDerivedOdometry_IsNotRebridged",
          "[slam][optimize][stored_graph][pose_edits]") {
  // reusex-39h: no stored pose graph, so optimize derives the odometry from
  // the current poses. The wrong link 2-3 is rejected by an edit, and a
  // relative edit 1 -> 3 (the true 0.2 m along x) keeps the tail tied to the
  // rest. The connectivity bridge must not put 2-3 back from the seeds:
  // the tail then lands where the relative edit says.
  TempPath tmp("test_optimize_reject_derived_odometry");
  ProjectDB db(tmp.path);
  make_broken_chain_project(db);
  REQUIRE(db.list_pose_graph_edges().empty());

  reusex::core::PoseEdits edits;
  edits.edits.push_back(reject_2_3());
  reusex::core::PoseEdit r;
  r.id = "r-1-3";
  r.kind = reusex::core::PoseEditKind::relative;
  r.nodes.first = r.nodes.last = 1;
  r.nodes_b.first = r.nodes_b.last = 3;
  r.pose.source = reusex::core::PoseSpec::Source::given;
  r.pose.value.matrix = translation_pose(0.2, 0.0, 0.0);
  r.sigma_trans_m = 0.001;
  r.sigma_rot_deg = 0.01;
  edits.edits.push_back(r);

  auto o = wall_options();
  o.use_plane_factors = false;
  const auto res = geo::optimize_sensor_poses(db, o, false, &edits);
  REQUIRE(res.converged);
  CHECK(res.edges_rejected_by_edits == 1);
  CHECK(res.bridge_edges == 0);
  for (int k = 0; k < kFrames; ++k) {
    INFO("frame " << k);
    CHECK(dist(db.sensor_frame_pose(k), truth(k)) < 1e-3);
  }
}

TEST_CASE("OptimizeStoredGraph_RejectEdgeThatSplitsTheGraph_FailsLoudly",
          "[slam][optimize][stored_graph][pose_edits]") {
  // reusex-39h: with nothing else tying frames 3-4 to the rest, honouring
  // the rejection leaves them floating. optimize refuses, naming the edit
  // and the pair, rather than silently bridging the rejected pair again.
  TempPath tmp("test_optimize_reject_splits");
  ProjectDB db(tmp.path);
  make_broken_chain_project(db);
  reusex::core::PoseEdits edits;
  edits.edits.push_back(reject_2_3());

  auto o = wall_options();
  o.use_plane_factors = false;
  try {
    geo::optimize_sensor_poses(db, o, /*dry_run=*/true, &edits);
    FAIL("optimize_sensor_poses did not throw");
  } catch (const std::runtime_error &e) {
    const std::string what = e.what();
    INFO(what);
    CHECK(what.find("x-bad-odometry") != std::string::npos);
    CHECK(what.find("2-3") != std::string::npos);
  }
}

TEST_CASE("OptimizeStoredGraph_RegisterAfterInit_WarnsToReinit",
          "[slam][optimize][stored_graph]") {
  // Review focus 5.
  TempPath tmp("test_optimize_stored_register");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  db.log_pipeline_end(db.log_pipeline_start("graph_init", "{}"), true);
  auto o = wall_options();
  o.use_plane_factors = false;

  SECTION("a register run newer than the init warns") {
    db.log_pipeline_end(db.log_pipeline_start("pose_refinement_jpr", "{}"),
                        true);
    LogCapture log;
    REQUIRE(geo::optimize_sensor_poses(db, o).converged);
    CHECK(log.saw("'rux register' ran after"));
  }
  SECTION("no register run, no warning") {
    LogCapture log;
    REQUIRE(geo::optimize_sensor_poses(db, o).converged);
    CHECK_FALSE(log.saw("'rux register' ran after"));
  }
}

TEST_CASE("OptimizeStoredGraph_NoGraph_AutoInitialisesButDryRunWritesNothing",
          "[slam][optimize][stored_graph]") {
  TempPath tmp("test_optimize_stored_autoinit");
  ProjectDB db(tmp.path);
  make_project(db);
  auto o = wall_options();
  o.use_plane_factors = false;

  const auto dry = geo::optimize_sensor_poses(db, o, /*dry_run=*/true);
  CHECK(dry.converged);
  CHECK(db.list_pose_graph_edges().empty());
  CHECK_FALSE(db.has_initial_pose(0));

  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  CHECK(r.graph_initialised);
  const auto edges = db.list_pose_graph_edges();
  CHECK(slam::count_usable_odometry_edges(edges) == kFrames - 1);
  for (const auto &e : edges)
    CHECK(e.source == "odometry_init");
  CHECK(db.has_initial_pose(0));
}

TEST_CASE("OptimizeStoredGraph_LegacyOdometryRows_StillAutoInitialise",
          "[slam][optimize][stored_graph]") {
  // Review focus 4: a project carrying rows from a pre-v28 optimize.
  TempPath tmp("test_optimize_stored_legacy");
  ProjectDB db(tmp.path);
  make_project(db);
  for (int k = 0; k < 2; ++k) {
    ProjectDB::PoseGraphEdge legacy;
    legacy.from_node_id = k;
    legacy.to_node_id = k + 1;
    legacy.edge_type = "odometry";
    legacy.residual = 0.1;
    legacy.weight = 100.0;
    legacy.source = "legacy";
    REQUIRE(db.upsert_pose_graph_edge(legacy));
  }
  auto o = wall_options();
  o.use_plane_factors = false;
  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  CHECK(r.graph_initialised);
  const auto e = db.pose_graph_edge(0, 1, "odometry");
  CHECK(e->source == "odometry_init");
  CHECK(e->transform.has_value());
}
