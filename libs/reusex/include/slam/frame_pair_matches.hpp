// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Feature correspondences between two stored sensor frames and the rigid
// alignment they give (reusex-xlg.2, Rediger's match editor): ORB
// keypoints matched with a Lowe ratio test — the loop-closure front-end's
// matcher — lifted to 3D through the stored depth, and a RANSAC rigid fit
// of B onto A. Read-only on the project; GTSAM- and OpenCV-free interface.
//
// Frames: a keypoint's xyz is in its frame's STORED-POSE frame
// (sensor_frames.transform's frame: local_transform · optical point), so
// T · xyz is exactly where slam::frame_world_points puts that pixel.
//
// RigidFit::world_delta is a WORLD-space correction applied to B:
//   world_delta · T_B · p_b ≈ T_A · p_a
// — the ΔT the Qt pair overlay applies to B's cloud and the
// FramePairIcpOptions::initial_guess the ICP refinement accepts (refine B onto
// A).

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::slam {

/// A pixel rectangle in a frame's colour image; w or h <= 0 = whole image.
struct PixelRect {
  int x = 0, y = 0, w = 0, h = 0;
};

struct FrameMatchOptions {
  int max_features = 3000; ///< ORB features per frame (LoopClosureOptions')
  /// Lowe ratio: best / second-best distance. 0.8, deliberately stricter
  /// than LoopClosureOptions' 0.85: these matches are drawn and fitted
  /// as-is, with no PCM stage behind them to vote outliers out.
  double ratio_test = 0.8;
  /// Keypoints are lifted only within [min_depth_m, max_depth_m]
  /// (LoopClosureOptions' range); outside, they are kept without xyz.
  float min_depth_m = 0.3f;
  float max_depth_m = 5.0f;
  /// Detect only inside these colour-image rectangles (clamped to the
  /// image); nullopt = the whole image.
  std::optional<PixelRect> region_a, region_b;
};

struct FrameKeypoint {
  float u = 0, v = 0; ///< pixel in the stored colour image
  /// Stored-pose-frame 3D point; nullopt = no valid depth (drawn hollow,
  /// never fitted).
  std::optional<std::array<double, 3>> xyz;
};

/// One correspondence: indices into keypoints_a / keypoints_b.
struct FrameMatch {
  int a = -1, b = -1;
  float distance = 0; ///< Hamming distance of the ORB descriptors
};

struct FramePairMatches {
  std::vector<FrameKeypoint> keypoints_a, keypoints_b;
  std::vector<FrameMatch> matches; ///< in B's keypoint order
};

/// ORB keypoints of both frames and their ratio-test matches (B queried
/// against A). Deterministic for a given project and options.
/// @throws std::invalid_argument when @p a_id == @p b_id.
/// @throws std::runtime_error naming the frame when it has no colour image,
///         no depth image, no stored pose, or unusable intrinsics.
FramePairMatches match_frame_pair(const ProjectDB &db, int a_id, int b_id,
                                  const FrameMatchOptions &opts = {});

/// A user-added correspondence end: the colour pixel (@p u, @p v) lifted
/// through the frame's depth, snapping to the nearest valid depth pixel
/// within @p snap_px colour pixels (scaled to the depth grid). The result's
/// u, v are the snapped pixel's position in colour coordinates.
/// @returns nullopt when no depth in range lies within the snap radius, or
///          the pixel is outside the image.
/// @throws std::runtime_error naming the frame when it has no colour or no
///         depth image or unusable intrinsics.
std::optional<FrameKeypoint> lift_pixel(const ProjectDB &db, int frame_id,
                                        float u, float v, int snap_px = 4,
                                        float min_depth_m = 0.3f,
                                        float max_depth_m = 5.0f);

using Point3 = std::array<double, 3>;
/// (point in A's stored-pose frame, point in B's stored-pose frame).
using PointPair = std::pair<Point3, Point3>;

enum class RigidFitFailure {
  none,
  too_few_pairs, ///< fewer than 3 pairs given
  no_consensus,  ///< fewer than 3 pairs agree within inlier_dist_m
  degenerate,    ///< the inliers are (nearly) collinear
};

/// Inliers whose RMS distance off their best-fit line is below this (m)
/// are collinear.
inline constexpr double kMinFitSpreadM = 0.01;

struct RigidFit {
  bool ok = false;
  /// World-space correction applied to B (row-major 4x4):
  /// world_delta · T_B · p_b ≈ T_A · p_a.
  std::array<double, 16> world_delta{1, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 1};
  /// Indices into the `pairs` argument, ascending: the pairs within
  /// inlier_dist_m under the returned world_delta.
  std::vector<int> inliers;
  double rms_m = 0;   ///< RMS 3D residual over the inliers, under world_delta
  std::string reason; ///< when !ok, e.g. "3 matches with depth needed, 2 given"
  RigidFitFailure failure = RigidFitFailure::none;
};

/// RANSAC rigid fit of B onto A from 3D correspondences (fixed @p seed:
/// deterministic, STANDARDS §6). @p pose_a / @p pose_b are the frames'
/// current stored poses (row-major).
/// @throws std::invalid_argument when a pair or a pose has a non-finite
///         value, @p iterations <= 0, or @p inlier_dist_m is negative or
///         not finite.
RigidFit rigid_from_matches(const std::array<double, 16> &pose_a,
                            const std::array<double, 16> &pose_b,
                            const std::vector<PointPair> &pairs,
                            double inlier_dist_m = 0.05, int iterations = 2000,
                            std::uint32_t seed = 1);

} // namespace reusex::slam
