// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Depth-cloud ICP between two stored sensor frames (#465): the "refine" of a
// pose-graph link. Both the web GUI's POST /api/v1/posegraph/icp (through the
// app's IcpRefineFn) and the Qt client's pair strip call this, so the two
// surfaces report the same numbers for the same pair.

#include <array>
#include <optional>

namespace reusex {
class ProjectDB;
}

namespace reusex::slam {

/// Knobs of refine_frame_pair_icp(). The defaults mirror the merged-cloud
/// pipeline (segmentation/reconstruct.cpp), so ICP sees the density and depth
/// range `rux create clouds` uses.
struct FramePairIcpOptions {
  int pixel_step = 4;                ///< sample every Nth depth pixel
  float min_depth_m = 0.3f;          ///< drop closer returns
  float max_depth_m = 4.0f;          ///< drop farther returns
  int max_iterations = 50;           ///< STANDARDS §6: a deterministic bound
  double max_correspondence_m = 0.5; ///< initial correspondence window
  float inlier_threshold_m = 0.05f;  ///< gate for inlier_fraction
  /// World-space correction applied to the "from" cloud before ICP starts
  /// (row-major 4x4), e.g. a manual alignment from the pair panel. The
  /// result's world_delta INCLUDES it. nullopt = identity (seeded by the
  /// stored poses alone).
  std::optional<std::array<double, 16>> initial_guess;
};

struct FramePairIcpResult {
  /// Row-major 4x4 relative pose T_to^{-1} * T_icp_delta * T_from, between
  /// the two frames' stored POSE frames (the device/base frames the
  /// `transform` blob describes; the camera's local_transform is not part of
  /// it): maps a point from "from"'s pose frame into "to"'s pose frame.
  std::array<double, 16> relative_pose{1, 0, 0, 0, 0, 1, 0, 0,
                                       0, 0, 1, 0, 0, 0, 0, 1};
  /// The ICP correction applied to the "from" cloud in world space
  /// (identity when the stored poses already agree), row-major 4x4.
  std::array<double, 16> world_delta{1, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 1};
  /// How far the correction moves the "from" camera's optical centre (m):
  /// |delta * c - c|. This is the shift a user means; the translation part of
  /// world_delta also folds in the rotation's lever arm about the WORLD
  /// origin, so it grows with the distance from the origin.
  double source_center_shift_m = 0.0;
  /// Rotation angle of world_delta (degrees).
  double rotation_deg = 0.0;
  double fitness = 0.0;         ///< RMS correspondence error after ICP (m)
  double inlier_fraction = 0.0; ///< share of "from" points within the gate
  bool converged = false;
  int source_points = 0; ///< back-projected points of the "from" frame
  int target_points = 0; ///< back-projected points of the "to" frame
  /// pose_graph_edges.transform for the edge (from_id -> to_id) in the
  /// sensor_frames.transform convention: (world_delta * T_from)^-1 * T_to,
  /// i.e. relative_pose^-1.
  std::array<double, 16> edge_transform{1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};
};

/// Back-project each frame's stored depth into world space (its stored pose
/// and intrinsics) and run point-to-point ICP of @p from_id onto @p to_id,
/// seeded by the stored poses. Read-only: nothing is written to @p db.
///
/// @throws std::runtime_error when a frame has no depth or no valid pose, or
///         its depth has no return inside the range (the message names the
///         frame and the reason).
/// @throws std::invalid_argument when @p from_id == @p to_id.
FramePairIcpResult refine_frame_pair_icp(const ProjectDB &db, int from_id,
                                         int to_id,
                                         const FramePairIcpOptions &opts = {});

/// Diagonal 6x6 information (GTSAM order rx, ry, rz, tx, ty, tz, row-major)
/// for a pose-graph edge measured by ICP: sigma_t = max(fitness, 0.01) m and
/// sigma_r = max(fitness / 2.0, 0.005) rad (the RMS point error over a ~2 m
/// lever arm).
std::array<double, 36> icp_edge_information(const FramePairIcpResult &result);

} // namespace reusex::slam
