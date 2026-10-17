// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// TUM trajectory I/O (reusex-rcv.1.6): the file format a pose reference is
// frozen into.

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/core/trajectory.hpp>
#include <reusex/io/trajectory.hpp>
#include <reusex/slam/pose_metrics.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Geometry>

#include <fstream>

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using reusex::core::StampedPose;
using reusex::core::Trajectory;
using reusex::io::read_tum;
using reusex::io::trajectory_from_project;
using reusex::io::write_tum;

namespace {

Eigen::Matrix4d pose_at(double x, double y, double z, double yaw_rad) {
  Eigen::Matrix4d p = Eigen::Matrix4d::Identity();
  p.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p(0, 3) = x;
  p(1, 3) = y;
  p(2, 3) = z;
  return p;
}

Trajectory sample_trajectory() {
  Trajectory t;
  t.push_back({1, 0.0, pose_at(0.0, 0.0, 1.2, 0.0)});
  t.push_back({2, 1.5, pose_at(1.0, 0.0, 1.2, 0.3)});
  t.push_back({3, 3.0, pose_at(1.0, 2.0, 1.25, 1.9)});
  return t;
}

} // namespace

TEST_CASE("write_tum_then_read_tum_RoundTrips", "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory", ".tum");
  const auto original = sample_trajectory();

  write_tum(original, tmp.path);
  const auto back = read_tum(tmp.path);

  REQUIRE(back.size() == original.size());
  for (std::size_t i = 0; i < original.size(); ++i) {
    CHECK_THAT(back[i].timestamp, WithinAbs(original[i].timestamp, 1e-12));
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c)
        CHECK_THAT(back[i].pose(r, c), WithinAbs(original[i].pose(r, c), 1e-9));
    // node_id is not a TUM column: read_tum numbers rows 0, 1, 2, ...
    CHECK(back[i].node_id == static_cast<std::int64_t>(i));
  }
}

TEST_CASE("read_tum_SkipsCommentsAndBlankLines", "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_comments", ".tum");
  {
    std::ofstream out(tmp.path);
    out << "# timestamp tx ty tz qx qy qz qw\n";
    out << "\n";
    out << "0.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n";
    out << "   \n";
    out << "1.0 1.0 0.0 0.0 0.0 0.0 0.0 1.0\n";
  }
  const auto t = read_tum(tmp.path);
  REQUIRE(t.size() == 2);
  CHECK_THAT(t[1].pose(0, 3), WithinAbs(1.0, 1e-12));
}

TEST_CASE("read_tum_RejectsWrongColumnCount", "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_bad_cols", ".tum");
  {
    std::ofstream out(tmp.path);
    out << "0.0 0.0 0.0 0.0 0.0 0.0 0.0\n"; // missing qw
  }
  REQUIRE_THROWS_WITH(read_tum(tmp.path), ContainsSubstring("line 1"));
}

TEST_CASE("read_tum_RejectsNonFiniteValue", "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_nan", ".tum");
  {
    std::ofstream out(tmp.path);
    out << "0.0 nan 0.0 0.0 0.0 0.0 0.0 1.0\n";
  }
  REQUIRE_THROWS_WITH(read_tum(tmp.path), ContainsSubstring("non-finite"));
}

TEST_CASE("read_tum_RejectsNearZeroQuaternion", "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_zeroquat", ".tum");
  {
    std::ofstream out(tmp.path);
    out << "0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n";
  }
  REQUIRE_THROWS_WITH(read_tum(tmp.path), ContainsSubstring("zero norm"));
}

TEST_CASE("trajectory_from_project_ExportsStoredPosesByNodeId",
          "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_project");
  reusex::ProjectDB db(tmp.path);
  auto intr = reusex::test_support::make_intrinsics(8, 8, 4, 4, 8, 8);
  const auto color = reusex::test_support::make_color(8, 8);
  for (int k : {3, 1, 2}) // inserted out of order; export must sort by id
    db.save_sensor_frame(k, color, cv::Mat(), cv::Mat(),
                         reusex::core::pose_graph::row_major_from_matrix(
                             pose_at(k * 1.0, 0.0, 1.0, 0.0)),
                         intr, /*timestamp=*/100.0 + k);

  const auto traj = trajectory_from_project(db, /*use_initial=*/false);
  REQUIRE(traj.size() == 3);
  for (int i = 0; i < 3; ++i) {
    const int node_id = i + 1;
    CHECK(traj[static_cast<std::size_t>(i)].node_id == node_id);
    CHECK_THAT(traj[static_cast<std::size_t>(i)].timestamp,
               WithinAbs(100.0 + node_id, 1e-12));
    CHECK_THAT(traj[static_cast<std::size_t>(i)].pose(0, 3),
               WithinAbs(node_id * 1.0, 1e-9));
  }
}

TEST_CASE("trajectory_from_project_FallsBackToNodeIdWhenNoTimestamp",
          "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_no_ts");
  reusex::ProjectDB db(tmp.path);
  auto intr = reusex::test_support::make_intrinsics(8, 8, 4, 4, 8, 8);
  const auto color = reusex::test_support::make_color(8, 8);
  // timestamp defaults to -1.0 (unset) when not passed.
  db.save_sensor_frame(
      7, color, cv::Mat(), cv::Mat(),
      reusex::core::pose_graph::row_major_from_matrix(pose_at(0, 0, 0, 0)),
      intr);

  const auto traj = trajectory_from_project(db, false);
  REQUIRE(traj.size() == 1);
  CHECK_THAT(traj[0].timestamp, WithinAbs(7.0, 1e-12));
}

TEST_CASE("trajectory_from_project_UsesInitialTransformWhenRequested",
          "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_initial");
  reusex::ProjectDB db(tmp.path);
  auto intr = reusex::test_support::make_intrinsics(8, 8, 4, 4, 8, 8);
  const auto color = reusex::test_support::make_color(8, 8);
  db.save_sensor_frame(
      1, color, cv::Mat(), cv::Mat(),
      reusex::core::pose_graph::row_major_from_matrix(pose_at(5, 0, 0, 0)),
      intr, 0.0);
  db.set_initial_pose(
      1, reusex::core::pose_graph::row_major_from_matrix(pose_at(9, 0, 0, 0)));

  const auto current = trajectory_from_project(db, false);
  const auto initial = trajectory_from_project(db, true);
  REQUIRE(current.size() == 1);
  REQUIRE(initial.size() == 1);
  CHECK_THAT(current[0].pose(0, 3), WithinAbs(5.0, 1e-9));
  CHECK_THAT(initial[0].pose(0, 3), WithinAbs(9.0, 1e-9));
}

TEST_CASE("trajectory_from_project_SkipsFramesWithNoUsablePose",
          "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_skip");
  {
    reusex::ProjectDB db(tmp.path);
    auto intr = reusex::test_support::make_intrinsics(8, 8, 4, 4, 8, 8);
    const auto color = reusex::test_support::make_color(8, 8);
    db.save_sensor_frame(
        1, color, cv::Mat(), cv::Mat(),
        reusex::core::pose_graph::row_major_from_matrix(pose_at(1, 0, 0, 0)),
        intr, 0.0);
    db.save_sensor_frame(
        2, color, cv::Mat(), cv::Mat(),
        reusex::core::pose_graph::row_major_from_matrix(pose_at(2, 0, 0, 0)),
        intr, 1.0);
  } // closed: WAL must be flushed before the raw-sqlite fixture mutates it
  reusex::test_support::clear_sensor_frame_pose(tmp.path, 2);

  reusex::ProjectDB db(tmp.path, /*readOnly=*/true);
  const auto traj = trajectory_from_project(db, false);
  REQUIRE(traj.size() == 1);
  CHECK(traj[0].node_id == 1);
}

TEST_CASE("trajectory_from_project_ThrowsWithNoSensorFrames",
          "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_empty");
  reusex::ProjectDB db(tmp.path);
  REQUIRE_THROWS_AS(trajectory_from_project(db, false), std::runtime_error);
}

// reusex-dh8: a project scored against its own `export trajectory` must read
// 0. Stored poses imported from float sources (RTABMap) carry a rotation
// that is orthonormal only to float precision; write_tum projects it onto a
// unit quaternion, so the two sides differ by ~1e-8 in the rotation matrix.
// acos((trace - 1) / 2) turns that into ~1e-4 rad (0.009 deg on house2).
TEST_CASE("trajectory_from_project_ScoredAgainstItsOwnExport_ReadsZero",
          "[io][trajectory]") {
  reusex::test_support::TempPath tmp("test_trajectory_self_score");
  reusex::test_support::TempPath tum("test_trajectory_self_score", ".tum");
  reusex::ProjectDB db(tmp.path);
  auto intr = reusex::test_support::make_intrinsics(8, 8, 4, 4, 8, 8);
  const auto color = reusex::test_support::make_color(8, 8);
  for (int k = 1; k <= 6; ++k) {
    Eigen::Matrix4d p = Eigen::Matrix4d::Identity();
    p.topLeftCorner<3, 3>() =
        (Eigen::AngleAxisd(0.37 * k, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(0.11 * k, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    p.topRightCorner<3, 1>() = Eigen::Vector3d(0.8 * k, 0.1 * k, 1.3);
    // Round through float, as a pose imported from RTABMap is.
    const Eigen::Matrix4d stored = p.cast<float>().cast<double>();
    db.save_sensor_frame(
        k, color, cv::Mat(), cv::Mat(),
        reusex::core::pose_graph::row_major_from_matrix(stored), intr,
        /*timestamp=*/100.0 + k);
  }

  const auto project = trajectory_from_project(db, /*use_initial=*/false);
  write_tum(project, tum.path);
  const auto exported = read_tum(tum.path);

  const auto m = reusex::slam::compare_trajectories(project, exported, {});
  REQUIRE(m.matched == 6);
  CHECK_THAT(m.ate_rmse_m, WithinAbs(0.0, 1e-9));
  CHECK_THAT(m.rot_rmse_deg, WithinAbs(0.0, 1e-6));
  CHECK_THAT(m.rpe_rot_frames_deg, WithinAbs(0.0, 1e-6));
  CHECK_THAT(m.rpe_rot_deg, WithinAbs(0.0, 1e-6));
}
