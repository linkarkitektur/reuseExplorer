// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Depth-cloud ICP between two stored sensor frames (#465): the "refine" of a
// pose-graph link. Both the web GUI's POST /api/v1/posegraph/icp (through the
// app's IcpRefineFn) and the Qt client's pair strip call this, so the two
// surfaces report the same numbers for the same pair.

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

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
  /// RMS nearest-neighbour distance (m) after ICP over ALL "from" points,
  /// including those outside the two views' overlap, which have no true
  /// counterpart: a loose measure that grows with the non-overlap.
  /// inlier_fraction / inlier_rms_m describe the overlap itself.
  double fitness = 0.0;
  double inlier_fraction = 0.0; ///< share of "from" points within the gate
  /// RMS point-to-plane distance (m) of the points counted by
  /// inlier_fraction, to their "to" neighbour's tangent plane: how well the
  /// overlapping surfaces agree, independent of the sampling spacing.
  double inlier_rms_m = 0.0;
  bool converged = false;
  /// Directions (of 6) the point-to-plane normal equations could not
  /// constrain in the last iteration, e.g. 3 for a bare wall (the slide
  /// along it and the turn about its normal). ICP leaves B unmoved along
  /// them rather than letting noise move it.
  int unconstrained_dofs = 0;

  /// The same overlap measures at the START pose (initial_guess, or the
  /// stored poses), so a caller can see whether ICP improved on its seed.
  double seed_inlier_fraction = 0.0;
  double seed_inlier_rms_m = 0.0;
  /// How far ICP moved the "from" camera's optical centre away from the
  /// START pose (m), and the rotation of that move (degrees). Without an
  /// initial_guess these equal source_center_shift_m / rotation_deg.
  double seed_shift_m = 0.0;
  double seed_rotation_deg = 0.0;
  /// icp_worse_than_seed(): ICP moved B far from its start and fits the
  /// overlap worse than the start did. A warning is logged; the caller
  /// should keep the seed rather than apply world_delta.
  bool worse_than_seed = false;
  int source_points = 0; ///< back-projected points of the "from" frame
  int target_points = 0; ///< back-projected points of the "to" frame
  /// pose_graph_edges.transform for the edge (from_id -> to_id) in the
  /// sensor_frames.transform convention: (world_delta * T_from)^-1 * T_to,
  /// i.e. relative_pose^-1.
  std::array<double, 16> edge_transform{1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};
};

/// Back-project each frame's stored depth into world space (its stored pose
/// and intrinsics) and run point-to-plane ICP of @p from_id onto @p to_id,
/// seeded by the stored poses (and initial_guess). Read-only: nothing is
/// written to @p db.
///
/// Point-to-plane, not point-to-point (reusex-xlg.12): on a low-structure
/// pair (walls, a floor, a few small objects) whose views only partly
/// overlap, point-to-point ICP pairs the points past the overlap with the
/// other cloud's border and drags B along the walls, away from a good seed.
/// Point-to-plane only pulls along surface normals, and directions no
/// surface constrains are left unmoved (unconstrained_dofs).
///
/// @throws std::runtime_error when a frame has no depth or no valid pose, or
///         its depth has no return inside the range (the message names the
///         frame and the reason).
/// @throws std::invalid_argument when @p from_id == @p to_id.
FramePairIcpResult refine_frame_pair_icp(const ProjectDB &db, int from_id,
                                         int to_id,
                                         const FramePairIcpOptions &opts = {});

/// The depth image of @p node_id back-projected into WORLD space with its
/// stored pose and intrinsics: exactly the cloud refine_frame_pair_icp()
/// aligns (the Qt client's pair overlay draws it).
/// @throws std::runtime_error when the frame has no depth or no valid pose,
///         or its depth has no return inside the range.
std::vector<std::array<float, 3>>
frame_world_points(const ProjectDB &db, int node_id,
                   const FramePairIcpOptions &opts = {});

/// frame_world_points() with each point's colour (reusex-xlg.10, the Qt
/// overlay's RGB mode).
struct FrameWorldCloud {
  /// Exactly frame_world_points(db, node_id, opts): same pixels, same order.
  std::vector<std::array<float, 3>> points;
  /// RGB (0..255) of the stored colour image at each point's depth pixel,
  /// index-aligned with @c points; EMPTY when the frame has no usable colour
  /// image (missing or undecodable), so the caller can fall back to a tint.
  std::vector<std::array<std::uint8_t, 3>> colors;
};

/// frame_world_points() plus colours. A depth pixel (u, v) samples the
/// colour image at (u * colour_w / depth_w, v * colour_h / depth_h): the two
/// may differ in resolution (an iPad stores depth at 192x256 against
/// 720x960 colour). A frame without a usable colour image logs a warning
/// naming it and returns its points with no colours.
/// @throws std::runtime_error as frame_world_points().
FrameWorldCloud
frame_world_colored_points(const ProjectDB &db, int node_id,
                           const FramePairIcpOptions &opts = {});

/// Moving the "from" camera centre farther than this from the start pose,
/// or rotating it more than kIcpSeedDriftDeg, is "far" for
/// icp_worse_than_seed() (the shift threshold is the inlier gate,
/// FramePairIcpOptions::inlier_threshold_m).
inline constexpr double kIcpSeedDriftDeg = 2.0;
/// A rise of the inlier RMS by more than this share counts as worse.
inline constexpr double kIcpSeedRmsTolerance = 0.10;

/// True when ICP moved B far from its start (seed_shift_m above the inlier
/// gate, or seed_rotation_deg above kIcpSeedDriftDeg) AND the overlap got
/// worse: fewer inliers, or an inlier RMS more than kIcpSeedRmsTolerance
/// higher. A far move that improves the overlap is a correction and is not
/// flagged; a small move never is.
bool icp_worse_than_seed(const FramePairIcpResult &result,
                         const FramePairIcpOptions &opts);

/// Diagonal 6x6 information (GTSAM order rx, ry, rz, tx, ty, tz, row-major)
/// for a pose-graph edge measured by ICP: sigma_t = max(fitness, 0.01) m and
/// sigma_r = max(fitness / 2.0, 0.005) rad (the RMS point error over a ~2 m
/// lever arm).
std::array<double, 36> icp_edge_information(const FramePairIcpResult &result);

} // namespace reusex::slam
