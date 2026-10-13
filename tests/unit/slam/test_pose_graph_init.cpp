// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// slam::init_pose_graph — `rux graph init` (spec §4.3, D3).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/slam/pose_graph.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>

using reusex::ProjectDB;
using namespace reusex::test_support;
namespace pg = reusex::core::pose_graph;
namespace slam = reusex::slam;

namespace {

std::array<double, 16> pose_k(int k) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(0.1 * k, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T(0, 3) = 0.3 * k;
  T(1, 3) = 0.05 * k * k;
  return pg::row_major_from_matrix(T);
}

void seed(ProjectDB &db, int id, const std::array<double, 16> &pose,
          int scan_id = -1) {
  db.save_sensor_frame(id, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
                       pose, make_intrinsics(8, 8, 4, 4, 8, 8), id, scan_id);
}

Eigen::Matrix4d rel(const ProjectDB &db, int a, int b) {
  return pg::matrix_from_row_major(db.sensor_frame_pose(a)).inverse() *
         pg::matrix_from_row_major(db.sensor_frame_pose(b));
}

} // namespace

TEST_CASE("InitPoseGraph_FreshProject_WritesOdometryChainAndSnapshots",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_fresh");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 5; ++k)
    seed(db, k, pose_k(k));

  const auto r = slam::init_pose_graph(db);
  CHECK(r.frames == 5);
  CHECK(r.snapshots_written == 5);
  CHECK(r.odometry_edges_written == 4);
  CHECK(r.scans == 1);

  const auto edges = db.list_pose_graph_edges();
  REQUIRE(edges.size() == 4);
  for (int k = 1; k <= 4; ++k) {
    INFO("edge " << k);
    const auto e = db.pose_graph_edge(k, k + 1, "odometry");
    REQUIRE(e.has_value());
    CHECK(e->source == "odometry_init");
    CHECK(e->status == "auto");
    CHECK_FALSE(e->information.has_value());
    CHECK(*e->seed_disagreement_m == 0.0);
    CHECK(
        (pg::matrix_from_row_major(*e->transform) - rel(db, k, k + 1)).norm() <
        1e-12);
  }
  for (int k = 1; k <= 5; ++k)
    CHECK(*db.initial_pose(k) == db.sensor_frame_pose(k));
  CHECK(slam::count_usable_odometry_edges(edges) == 4);
}

TEST_CASE("InitPoseGraph_Twice_RefusesWithoutForce",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_twice");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 3; ++k)
    seed(db, k, pose_k(k));
  slam::init_pose_graph(db);
  CHECK_THROWS_AS(slam::init_pose_graph(db), slam::PoseGraphExistsError);
}

TEST_CASE("InitPoseGraph_Force_ReplacesOnlyInitEdges",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_force");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 5; ++k)
    seed(db, k, pose_k(k));
  slam::init_pose_graph(db);
  const auto old_23 = *db.pose_graph_edge(2, 3, "odometry")->transform;

  ProjectDB::PoseGraphEdge loop;
  loop.from_node_id = 1;
  loop.to_node_id = 5;
  loop.edge_type = "loop_closure";
  loop.transform = pg::row_major_from_matrix(rel(db, 1, 5));
  loop.source = "manual";
  loop.status = "confirmed";
  REQUIRE(db.upsert_pose_graph_edge(loop));
  REQUIRE(db.set_pose_graph_edge_status(2, 3, "odometry", "rejected"));

  // Move every frame differently (register/perturb), then re-init.
  for (int k = 1; k <= 5; ++k)
    db.update_sensor_frame_pose(k, pose_k(k + 10));
  const auto r = slam::init_pose_graph(db, {.force = true});
  CHECK(r.odometry_edges_removed ==
        3); // (1,2), (3,4), (4,5); (2,3) is a tombstone
  CHECK(r.snapshots_written == 5);
  CHECK(r.odometry_edges_written == 3);
  CHECK(r.pairs_with_existing_odometry == 1);

  CHECK(*db.initial_pose(3) == pose_k(13));
  CHECK((pg::matrix_from_row_major(
             *db.pose_graph_edge(1, 2, "odometry")->transform) -
         rel(db, 1, 2))
            .norm() < 1e-12);
  const auto tomb = db.pose_graph_edge(2, 3, "odometry");
  CHECK(tomb->status == "rejected");
  CHECK(*tomb->transform == old_23);
  CHECK(db.pose_graph_edge(1, 5, "loop_closure")->status == "confirmed");
}

TEST_CASE("InitPoseGraph_TwoScans_DoesNotChainAcrossScans",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_scans");
  ProjectDB db(tmp.path);
  const auto a = db.create_scan("/data/scan_a.db");
  const auto b = db.create_scan("/data/scan_b.db");
  for (int k = 1; k <= 3; ++k)
    seed(db, k, pose_k(k), a.id);
  for (int k = 4; k <= 5; ++k)
    seed(db, k, pose_k(k), b.id);

  const auto r = slam::init_pose_graph(db);
  CHECK(r.scans == 2);
  CHECK(r.odometry_edges_written == 3);
  CHECK(db.pose_graph_edge(1, 2, "odometry").has_value());
  CHECK(db.pose_graph_edge(2, 3, "odometry").has_value());
  CHECK_FALSE(db.pose_graph_edge(3, 4, "odometry").has_value());
  CHECK(db.pose_graph_edge(4, 5, "odometry").has_value());
}

TEST_CASE("InitPoseGraph_PoselessFrame_IsSkipped", "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_poseless");
  {
    ProjectDB db(tmp.path);
    for (int k = 1; k <= 4; ++k)
      seed(db, k, pose_k(k));
  }
  clear_sensor_frame_pose(tmp.path, 3);
  ProjectDB db(tmp.path);
  const auto r = slam::init_pose_graph(db);
  CHECK(r.frames == 3);
  CHECK(db.pose_graph_edge(1, 2, "odometry").has_value());
  CHECK(db.pose_graph_edge(2, 4, "odometry").has_value());
  CHECK_FALSE(db.has_initial_pose(3));
}

TEST_CASE("InitPoseGraph_LegacyNullTransformOdometry_IsReplaced",
          "[slam][pose_graph][init]") {
  // Review focus 4: rows left by a pre-v27 `rux optimize` must neither block
  // init nor survive it.
  TempPath tmp("test_graph_init_legacy");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 3; ++k)
    seed(db, k, pose_k(k));
  ProjectDB::PoseGraphEdge legacy;
  legacy.from_node_id = 1;
  legacy.to_node_id = 2;
  legacy.edge_type = "odometry";
  legacy.residual = 0.1;
  legacy.weight = 100.0;
  legacy.source = "legacy";
  REQUIRE(db.upsert_pose_graph_edge(legacy));
  REQUIRE(slam::count_usable_odometry_edges(db.list_pose_graph_edges()) == 0);

  slam::init_pose_graph(db); // must not throw PoseGraphExistsError
  const auto e = db.pose_graph_edge(1, 2, "odometry");
  CHECK(e->source == "odometry_init");
  CHECK(e->transform.has_value());
  CHECK(db.list_pose_graph_edges().size() == 2);
}

TEST_CASE("OdometryInitEdges_FromCurrentTransform_IgnoresTheSnapshot",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_plan");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 2; ++k)
    seed(db, k, pose_k(k));
  db.set_initial_pose(2, pose_k(7));
  const auto from_initial = slam::odometry_init_edges(db, false);
  const auto from_current = slam::odometry_init_edges(db, true);
  REQUIRE(from_initial.size() == 1);
  REQUIRE(from_current.size() == 1);
  CHECK((pg::matrix_from_row_major(*from_current[0].transform) - rel(db, 1, 2))
            .norm() < 1e-12);
  CHECK((pg::matrix_from_row_major(*from_initial[0].transform) - rel(db, 1, 2))
            .norm() > 1e-3);
  CHECK(db.list_pose_graph_edges().empty()); // planning writes nothing
}
