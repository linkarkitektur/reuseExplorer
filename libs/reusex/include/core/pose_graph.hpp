// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Vocabulary and SE(3) helpers for the stored pose graph (`pose_graph_edges`,
// schema v28). Header-only and Eigen-only so core (ProjectDB), io (the RTABMap
// importer) and slam share ONE definition of the conventions (STANDARDS §1:
// core may not reach slam or geometry_common).
//
// Conventions (spec §3.1):
//  * transform   T_ij = T_from^-1 * T_to in the sensor_frames.transform
//                convention, 16 doubles row-major.
//  * information 6x6 in GTSAM Pose3 tangent order (rx, ry, rz, tx, ty, tz),
//                36 doubles row-major.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>

namespace reusex::core::pose_graph {

inline constexpr std::string_view kTypeOdometry = "odometry";
inline constexpr std::string_view kTypeLoopClosure = "loop_closure";
inline constexpr std::string_view kTypePanorama = "panorama";

inline constexpr std::string_view kStatusAuto = "auto";
inline constexpr std::string_view kStatusConfirmed = "confirmed";
inline constexpr std::string_view kStatusRejected = "rejected";

inline constexpr std::string_view kSourceRtabmap = "rtabmap";
inline constexpr std::string_view kSourceOdometryInit = "odometry_init";
inline constexpr std::string_view kSourceOrb = "orb";
inline constexpr std::string_view kSourcePanorama = "panorama";
inline constexpr std::string_view kSourceFile = "file";
inline constexpr std::string_view kSourceIcp = "icp";
inline constexpr std::string_view kSourceManual = "manual";
inline constexpr std::string_view kSourceLegacy = "legacy";

/// project_settings key of the persistent display-only view yaw (spec D7).
inline constexpr std::string_view kSettingViewYawDeg = "view.yaw_deg";

using Matrix6d = Eigen::Matrix<double, 6, 6>;

inline bool is_valid_status(std::string_view status) {
  return status == kStatusAuto || status == kStatusConfirmed ||
         status == kStatusRejected;
}

inline Eigen::Matrix4d matrix_from_row_major(const std::array<double, 16> &a) {
  Eigen::Matrix4d m;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m(r, c) = a[static_cast<size_t>(r * 4 + c)];
  return m;
}

inline std::array<double, 16> row_major_from_matrix(const Eigen::Matrix4d &m) {
  std::array<double, 16> a{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      a[static_cast<size_t>(r * 4 + c)] = m(r, c);
  return a;
}

inline Matrix6d information_from_row_major(const std::array<double, 36> &a) {
  Matrix6d m;
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      m(r, c) = a[static_cast<size_t>(r * 6 + c)];
  return m;
}

inline std::array<double, 36> row_major_from_information(const Matrix6d &m) {
  std::array<double, 36> a{};
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      a[static_cast<size_t>(r * 6 + c)] = m(r, c);
  return a;
}

/// Finite, bottom row [0 0 0 1], orthonormal rotation block with det > 0.
/// Tolerances admit float-precision imports (RTABMap stores floats).
inline bool is_rigid_transform(const std::array<double, 16> &a) {
  for (double v : a)
    if (!std::isfinite(v))
      return false;
  if (std::abs(a[12]) > 1e-6 || std::abs(a[13]) > 1e-6 ||
      std::abs(a[14]) > 1e-6 || std::abs(a[15] - 1.0) > 1e-6)
    return false;
  const Eigen::Matrix3d R = matrix_from_row_major(a).topLeftCorner<3, 3>();
  return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-3 &&
         R.determinant() > 0.0;
}

/// Finite and symmetric (relative tolerance 1e-9).
inline bool is_valid_information(const std::array<double, 36> &a) {
  for (double v : a)
    if (!std::isfinite(v))
      return false;
  const Matrix6d m = information_from_row_major(a);
  return (m - m.transpose()).norm() <= 1e-9 * std::max(1.0, m.norm());
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d s;
  s << 0, -v.z(), v.y(), //
      v.z(), 0, -v.x(),  //
      -v.y(), v.x(), 0;
  return s;
}

/// Adjoint of T in GTSAM's (rotation, translation) tangent order:
/// [[R, 0], [t^ R, R]].
inline Matrix6d pose3_adjoint(const Eigen::Matrix4d &T) {
  const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
  const Eigen::Vector3d t = T.topRightCorner<3, 1>();
  Matrix6d A = Matrix6d::Zero();
  A.topLeftCorner<3, 3>() = R;
  A.bottomLeftCorner<3, 3>() = skew(t) * R;
  A.bottomRightCorner<3, 3>() = R;
  return A;
}

/// Information of the reversed edge j -> i. The BetweenFactor error of the
/// reversed measurement is e_ji = -Ad(T_ij) e_ij, so
/// L_ji = Ad(T_ji)^T L_ij Ad(T_ji) with T_ji = T_ij^-1. Symmetrised.
inline Matrix6d reverse_edge_information(const Matrix6d &info_ij,
                                         const Eigen::Matrix4d &T_ij) {
  const Matrix6d A = pose3_adjoint(T_ij.inverse());
  const Matrix6d out = A.transpose() * info_ij * A;
  return 0.5 * (out + out.transpose());
}

/// Diagonal information diag(1/sr^2 x3, 1/st^2 x3).
inline Matrix6d information_from_sigmas(double sigma_rot, double sigma_trans) {
  Matrix6d m = Matrix6d::Zero();
  for (int k = 0; k < 3; ++k) {
    m(k, k) = 1.0 / (sigma_rot * sigma_rot);
    m(k + 3, k + 3) = 1.0 / (sigma_trans * sigma_trans);
  }
  return m;
}

struct TransformError {
  double translation_m = 0.0;
  double rotation_deg = 0.0;
};

/// How far poses P_i, P_j disagree with the measurement T_ij:
/// E = T_ij^-1 * (P_i^-1 * P_j); returns |t(E)| and the rotation angle of E.
/// Used for seed_disagreement_* (P = initial poses) and deviation_*
/// (P = solved poses).
inline TransformError transform_error(const Eigen::Matrix4d &T_ij,
                                      const Eigen::Matrix4d &P_i,
                                      const Eigen::Matrix4d &P_j) {
  const Eigen::Matrix4d E = T_ij.inverse() * (P_i.inverse() * P_j);
  const double c =
      std::clamp((E.topLeftCorner<3, 3>().trace() - 1.0) / 2.0, -1.0, 1.0);
  return {E.topRightCorner<3, 1>().norm(), std::acos(c) * 180.0 / M_PI};
}

} // namespace reusex::core::pose_graph
