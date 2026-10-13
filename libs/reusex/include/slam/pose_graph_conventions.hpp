// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The one place stored pose-graph measurements (sensor_frames.transform
// convention, spec §3.1) are converted to and from the optimizer's optical
// convention (world_pose = worldTf * localTf, surfel_extraction.hpp).
//
//   T_optical = L_i^-1 * T_stored * L_j
//   e_optical = Ad(L_j^-1) * e_stored  =>  Info_opt = Ad(L_j)^T Info Ad(L_j)
//
// Header-only and GTSAM-free (STANDARDS §2).

#include <reusex/core/pose_graph.hpp>

#include <Eigen/Core>

namespace reusex::slam {

inline Eigen::Matrix4d stored_to_optical(const Eigen::Matrix4d &T_ij,
                                         const Eigen::Matrix4d &local_i,
                                         const Eigen::Matrix4d &local_j) {
  return local_i.inverse() * T_ij * local_j;
}

inline Eigen::Matrix4d optical_to_stored(const Eigen::Matrix4d &T_ij_optical,
                                         const Eigen::Matrix4d &local_i,
                                         const Eigen::Matrix4d &local_j) {
  return local_i * T_ij_optical * local_j.inverse();
}

inline core::pose_graph::Matrix6d
information_stored_to_optical(const core::pose_graph::Matrix6d &info,
                              const Eigen::Matrix4d &local_j) {
  const core::pose_graph::Matrix6d A = core::pose_graph::pose3_adjoint(local_j);
  const core::pose_graph::Matrix6d out = A.transpose() * info * A;
  return 0.5 * (out + out.transpose());
}

inline core::pose_graph::Matrix6d
information_optical_to_stored(const core::pose_graph::Matrix6d &info,
                              const Eigen::Matrix4d &local_j) {
  const core::pose_graph::Matrix6d A =
      core::pose_graph::pose3_adjoint(local_j.inverse());
  const core::pose_graph::Matrix6d out = A.transpose() * info * A;
  return 0.5 * (out + out.transpose());
}

} // namespace reusex::slam
