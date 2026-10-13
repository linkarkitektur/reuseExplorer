// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Stored (sensor_frames.transform) <-> optical (optimizer) edge conventions,
// spec §4.1.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/slam/pose_graph_conventions.hpp>

#include <Eigen/Geometry>

namespace pg = reusex::core::pose_graph;
using namespace reusex::slam;
using Catch::Matchers::WithinRel;

namespace {

Eigen::Matrix4d make_pose(double angle, const Eigen::Vector3d &axis,
                          const Eigen::Vector3d &t) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  T.topRightCorner<3, 1>() = t;
  return T;
}

Eigen::Matrix<double, 6, 1> small_log(const Eigen::Matrix4d &M) {
  const Eigen::AngleAxisd aa(Eigen::Matrix3d(M.topLeftCorner<3, 3>()));
  Eigen::Matrix<double, 6, 1> v;
  v.head<3>() = aa.angle() * aa.axis();
  v.tail<3>() = M.topRightCorner<3, 1>();
  return v;
}

// An iPad-like local transform: base -> optical rotation plus a lever arm.
const Eigen::Matrix4d kLi =
    make_pose(M_PI / 2, {1, -1, 1}, {0.02, -0.05, 0.01});
const Eigen::Matrix4d kLj =
    make_pose(M_PI / 2, {1, -1, 1}, {0.02, -0.05, 0.01});

} // namespace

TEST_CASE("PoseGraphConventions_Transform_RoundTrips", "[slam][pose_graph]") {
  const Eigen::Matrix4d T = make_pose(0.4, {0, 0, 1}, {1.0, 0.2, -0.3});
  const Eigen::Matrix4d opt = stored_to_optical(T, kLi, kLj);
  CHECK((optical_to_stored(opt, kLi, kLj) - T).norm() < 1e-12);
}

TEST_CASE("PoseGraphConventions_Transform_MatchesOpticalPoses",
          "[slam][pose_graph]") {
  // Stored poses W, optical poses W*L: the optical measurement must equal
  // (W_i L_i)^-1 (W_j L_j).
  const Eigen::Matrix4d W_i = make_pose(0.3, {0, 1, 0}, {2, 1, 0});
  const Eigen::Matrix4d W_j = make_pose(-0.2, {1, 0, 0}, {3, -1, 0.5});
  const Eigen::Matrix4d T = W_i.inverse() * W_j;
  const Eigen::Matrix4d expected = (W_i * kLi).inverse() * (W_j * kLj);
  CHECK((stored_to_optical(T, kLi, kLj) - expected).norm() < 1e-12);
}

TEST_CASE("PoseGraphConventions_Information_PreservesChiSquare",
          "[slam][pose_graph]") {
  const Eigen::Matrix4d T = make_pose(0.6, {1, 2, 0}, {1.0, -0.5, 0.2});
  const Eigen::Matrix4d W_i = make_pose(0.1, {0, 0, 1}, {0.5, 0.5, 0});
  const Eigen::Matrix4d D = make_pose(1e-4, {1, 0, 1}, {2e-4, -1e-4, 1e-4});
  const Eigen::Matrix4d W_j = W_i * T * D;
  pg::Matrix6d L = pg::information_from_sigmas(0.02, 0.05);
  L(1, 3) = L(3, 1) = 40.0;

  const auto e_stored = small_log(T.inverse() * W_i.inverse() * W_j);
  const Eigen::Matrix4d T_opt = stored_to_optical(T, kLi, kLj);
  const auto e_opt =
      small_log(T_opt.inverse() * (W_i * kLi).inverse() * (W_j * kLj));
  const pg::Matrix6d L_opt = information_stored_to_optical(L, kLj);
  CHECK_THAT(e_opt.dot(L_opt * e_opt),
             WithinRel(e_stored.dot(L * e_stored), 1e-3));
  CHECK((information_optical_to_stored(L_opt, kLj) - L).norm() <
        1e-9 * L.norm());
}
