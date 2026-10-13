// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core/pose_graph.hpp: the conventions every pose_graph_edges writer shares
// (spec §3.1): row-major blobs, GTSAM (rot, trans) tangent order, and the
// information of a reversed edge.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/pose_graph.hpp>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <limits>

namespace pg = reusex::core::pose_graph;
using Catch::Matchers::WithinAbs;
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

/// First-order log of a near-identity transform, GTSAM order (rot, trans).
Eigen::Matrix<double, 6, 1> small_log(const Eigen::Matrix4d &M) {
  const Eigen::AngleAxisd aa(Eigen::Matrix3d(M.topLeftCorner<3, 3>()));
  Eigen::Matrix<double, 6, 1> v;
  v.head<3>() = aa.angle() * aa.axis();
  v.tail<3>() = M.topRightCorner<3, 1>();
  return v;
}

pg::Matrix6d spd_information() {
  pg::Matrix6d A;
  A << 4, 1, 0, 0, 0.5, 0, //
      1, 5, 0, 0, 0, 0,    //
      0, 0, 6, 0.2, 0, 0,  //
      0, 0, 0.2, 90, 3, 0, //
      0.5, 0, 0, 3, 80, 1, //
      0, 0, 0, 0, 1, 70;
  return 0.5 * (A + A.transpose()) + 10.0 * pg::Matrix6d::Identity();
}

} // namespace

TEST_CASE("PoseGraphMath_RowMajorBlobs_RoundTrip", "[core][pose_graph]") {
  const Eigen::Matrix4d T = make_pose(0.3, {1, 2, 3}, {4, -5, 6});
  const auto a = pg::row_major_from_matrix(T);
  CHECK(a[3] == 4.0); // row 0, col 3: translation x
  CHECK(pg::matrix_from_row_major(a) == T);

  const pg::Matrix6d I = spd_information();
  CHECK(pg::information_from_row_major(pg::row_major_from_information(I)) == I);
}

TEST_CASE("PoseGraphMath_IsRigidTransform_RejectsBrokenBlobs",
          "[core][pose_graph]") {
  CHECK(pg::is_rigid_transform(
      pg::row_major_from_matrix(make_pose(1.0, {0, 0, 1}, {1, 2, 3}))));
  std::array<double, 16> zero{};
  CHECK_FALSE(pg::is_rigid_transform(zero));
  auto nan = pg::row_major_from_matrix(Eigen::Matrix4d::Identity());
  nan[7] = std::numeric_limits<double>::quiet_NaN();
  CHECK_FALSE(pg::is_rigid_transform(nan));
  auto reflection = pg::row_major_from_matrix(Eigen::Matrix4d::Identity());
  reflection[0] = -1.0; // det(R) = -1
  CHECK_FALSE(pg::is_rigid_transform(reflection));
  auto scaled = pg::row_major_from_matrix(Eigen::Matrix4d::Identity());
  scaled[0] = scaled[5] = scaled[10] = 2.0; // not orthonormal
  CHECK_FALSE(pg::is_rigid_transform(scaled));
}

TEST_CASE("PoseGraphMath_IsValidInformation_RejectsAsymmetricOrNonFinite",
          "[core][pose_graph]") {
  CHECK(pg::is_valid_information(
      pg::row_major_from_information(spd_information())));
  auto bad = pg::row_major_from_information(spd_information());
  bad[1] += 1.0; // (0,1) != (1,0)
  CHECK_FALSE(pg::is_valid_information(bad));
  bad = pg::row_major_from_information(spd_information());
  bad[0] = std::numeric_limits<double>::infinity();
  CHECK_FALSE(pg::is_valid_information(bad));
}

TEST_CASE("PoseGraphMath_ReverseEdgeInformation_PreservesChiSquare",
          "[core][pose_graph]") {
  // X_j = X_i * T_ij * D with a small D: the forward error is ~log(D); the
  // reversed edge (T_ji = T_ij^-1) sees the same disagreement from j's side.
  // The chi-square of the two must agree to first order.
  const Eigen::Matrix4d T_ij = make_pose(0.7, {0.2, 1, 0.3}, {1.5, -0.4, 2.0});
  const Eigen::Matrix4d X_i = make_pose(-0.4, {1, 0, 1}, {3, 1, -2});
  const Eigen::Matrix4d D =
      make_pose(2e-4, {0.3, -1, 0.5}, {1e-4, -2e-4, 3e-4});
  const Eigen::Matrix4d X_j = X_i * T_ij * D;
  const pg::Matrix6d L = spd_information();

  const auto e_ij = small_log(T_ij.inverse() * X_i.inverse() * X_j);
  const Eigen::Matrix4d T_ji = T_ij.inverse();
  const auto e_ji = small_log(T_ji.inverse() * X_j.inverse() * X_i);
  const pg::Matrix6d L_ji = pg::reverse_edge_information(L, T_ij);

  const double chi_ij = e_ij.dot(L * e_ij);
  const double chi_ji = e_ji.dot(L_ji * e_ji);
  REQUIRE(chi_ij > 0.0);
  CHECK_THAT(chi_ji, WithinRel(chi_ij, 1e-3));
}

TEST_CASE("PoseGraphMath_ReverseTwice_IsIdentity", "[core][pose_graph]") {
  const Eigen::Matrix4d T = make_pose(1.1, {1, 1, 0}, {0.5, 2, -1});
  const pg::Matrix6d L = spd_information();
  const pg::Matrix6d back = pg::reverse_edge_information(
      pg::reverse_edge_information(L, T), T.inverse());
  CHECK((back - L).norm() < 1e-9 * L.norm());
}

TEST_CASE("PoseGraphMath_InformationFromSigmas_IsDiagonalInGtsamOrder",
          "[core][pose_graph]") {
  const pg::Matrix6d L = pg::information_from_sigmas(0.1, 0.5);
  CHECK_THAT(L(0, 0), WithinRel(100.0, 1e-12)); // rotation first
  CHECK_THAT(L(5, 5), WithinRel(4.0, 1e-12));   // translation last
  CHECK(L(0, 5) == 0.0);
}

TEST_CASE("PoseGraphMath_TransformError_IsZeroForAgreeingPoses",
          "[core][pose_graph]") {
  const Eigen::Matrix4d P_i = make_pose(0.4, {0, 0, 1}, {1, 2, 0});
  const Eigen::Matrix4d T = make_pose(0.2, {0, 1, 0}, {0.5, 0, 0});
  const auto ok = pg::transform_error(T, P_i, P_i * T);
  CHECK_THAT(ok.translation_m, WithinAbs(0.0, 1e-12));
  CHECK_THAT(ok.rotation_deg, WithinAbs(0.0, 1e-6));
  const Eigen::Matrix4d off = make_pose(0.0, {0, 0, 1}, {0, 0.3, 0});
  const auto bad = pg::transform_error(T, P_i, P_i * T * off);
  CHECK_THAT(bad.translation_m, WithinAbs(0.3, 1e-12));
}

TEST_CASE("PoseGraphMath_Vocabulary_StatusValidation", "[core][pose_graph]") {
  CHECK(pg::is_valid_status("auto"));
  CHECK(pg::is_valid_status("confirmed"));
  CHECK(pg::is_valid_status("rejected"));
  CHECK_FALSE(pg::is_valid_status("deleted"));
  CHECK_FALSE(pg::is_valid_status(""));
}
