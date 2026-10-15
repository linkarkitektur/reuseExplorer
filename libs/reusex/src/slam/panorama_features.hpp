// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Internal (NOT public API) helpers shared by the two panorama front-ends:
//   - PanoramaAlignment.cpp  — resects ONE pooled panorama pose in WORLD space
//                              (`rux align 360`, placement refinement)
//   - PanoramaLoopEdges.cpp  — resects the panorama SEPARATELY in each matched
//                              frame's own optical frame (issue #236, loop
//                              edges)
//
// Both slice an equirect into perspective views, ORB-match those slices against
// sensor frames, lift the frame keypoints to metric 3D via the stored depth,
// and refine a bearing-space pose with the same Gauss-Newton step. Only the
// COORDINATE FRAME of the 3D points and the grouping of correspondences differ,
// so the geometry primitives live here rather than being duplicated.
//
// This header deliberately sits in src/ (not include/): it is an implementation
// detail of the slam module, not part of the stable API (docs/STANDARDS.md §2).

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <array>
#include <optional>
#include <vector>

#include "core/SensorIntrinsics.hpp"

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry::pano_detail {

// --- small pose/rotation utilities ----------------------------------------

/// Row-major std::array<16> (the sensor_frame_pose convention) -> Eigen 4x4.
Eigen::Matrix4d to_matrix4(const std::array<double, 16> &a);

/// Eigen 4x4 -> row-major std::array<16>.
std::array<double, 16> from_matrix4(const Eigen::Matrix4d &M);

/// so(3) exponential (Rodrigues) of a rotation vector.
Eigen::Matrix3d exp_so3(const Eigen::Vector3d &w);

/// Skew-symmetric cross-product matrix.
Eigen::Matrix3d skew(const Eigen::Vector3d &v);

/// Angular error (rad) between the observed unit bearing @p b_obs and the
/// bearing of point @p X under the pano_from_reference transform (@p Q, @p t).
double bearing_angle(const Eigen::Matrix3d &Q, const Eigen::Vector3d &t,
                     const Eigen::Vector3d &X, const Eigen::Vector3d &b_obs);

// --- frame features --------------------------------------------------------

/// ORB features of one sensor frame with each keypoint lifted to metric 3D.
///
/// The 3D points are stored in the frame's OWN OPTICAL coordinates. That choice
/// is what makes the loop-edge front-end pose-independent: a panorama resected
/// against `local` sees nothing of `sensor_frame_pose`, so the relative pose it
/// implies between two frames is a genuine measurement rather than a
/// restatement of the (drifted) seed trajectory. Callers that want world
/// coordinates — the alignment path, which resects a single pooled world pose —
/// compose with `T_world_cam` via world().
struct FrameFeatures {
  cv::Mat descriptors;                 ///< Nx32 CV_8U
  std::vector<cv::KeyPoint> keypoints; ///< pixel locations (figures/debug)
  std::vector<Eigen::Vector3d> local;  ///< 3D per keypoint, OPTICAL frame
  std::vector<char> valid;             ///< depth was finite and in range
  Eigen::Matrix4d T_world_cam = Eigen::Matrix4d::Identity(); ///< seed pose
  int node_id = -1;

  /// The k-th keypoint's 3D point in world coordinates (seed-pose dependent).
  Eigen::Vector3d world(size_t k) const {
    return T_world_cam.block<3, 3>(0, 0) * local[k] +
           T_world_cam.block<3, 1>(0, 3);
  }
};

/// Detect ORB features in a frame's colour image and lift them to 3D using the
/// stored depth map and intrinsics. Returns an empty result (no descriptors)
/// when the frame has no colour/depth or degenerate intrinsics. @p region
/// restricts detection to that colour-image rectangle (clamped; nullopt =
/// the whole image) — Rediger's match editor (frame_pair_matches.cpp).
FrameFeatures
extract_frame_features(const ProjectDB &db, int node_id,
                       const cv::Ptr<cv::ORB> &orb, float min_depth,
                       float max_depth,
                       const std::optional<cv::Rect> &region = std::nullopt);

/// Pinhole intrinsics scaled to a depth grid of @p depth_cols x
/// @p depth_rows (the colour calibration is intr.width x intr.height).
struct DepthPinhole {
  double fx = 0, fy = 0, cx = 0, cy = 0;
};
DepthPinhole depth_pinhole(const core::SensorIntrinsics &intr, int depth_cols,
                           int depth_rows);

/// Optical-frame 3D point of depth pixel (@p ud, @p vd) of @p depth_m
/// (CV_32F metres); nullopt outside the grid or when the depth is not
/// finite or outside [min_depth, max_depth].
std::optional<Eigen::Vector3d> lift_depth_pixel(const cv::Mat &depth_m,
                                                const DepthPinhole &k, int ud,
                                                int vd, float min_depth,
                                                float max_depth);

// --- bearing-space pose refinement ----------------------------------------

/// Parameters of refine_bearing_pose.
struct BearingRefineOptions {
  double ang_gate = 0.01; ///< inlier angular tolerance (rad)
  int iterations = 10;    ///< Gauss-Newton steps
  int min_inliers = 25;   ///< abandon below this many gated correspondences

  /// Override for the INITIAL inlier gate only (the first call to the gate
  /// lambda before any Gauss-Newton step runs). 0.0 means "same as ang_gate".
  ///
  /// Used by the fix-translation path in PanoramaAlignment: the rotation seed
  /// Q_best was estimated by PnP using the PnP's OWN translation, so forcing
  /// the centre to fixed_centre makes most correspondences exceed the tight
  /// production ang_gate (~0.02 rad) before GN has had a chance to run. A
  /// looser seed gate lets GN start from a non-empty inlier set; the tight
  /// ang_gate is used for every subsequent re-gate so final quality is
  /// unchanged. (See issue #364.)
  double initial_ang_gate = 0.0;
};

/// Gauss-Newton refinement of a pano_from_reference pose (@p Q, @p t) over
/// point<->bearing correspondences, re-gating the inlier set as the pose
/// improves.
///
/// @p points and @p bearings are parallel arrays: `bearings[k]` is the unit
/// bearing, in the PANORAMA frame, under which `points[k]` (in the reference
/// frame — world for alignment, frame-local for loop edges) was observed.
///
/// Left perturbation `T <- exp(xi) T`; residual `e = b_hat - b_obs`.
///
/// @param out_initial_inliers  if non-null, receives the inlier count of the
///                             INITIAL pose (before refinement), so callers can
///                             report why a resection was abandoned.
/// @returns indices into @p points of the final inlier set, or empty when the
///          initial or final gate falls below `min_inliers`.
std::vector<int>
refine_bearing_pose(const std::vector<Eigen::Vector3d> &points,
                    const std::vector<Eigen::Vector3d> &bearings,
                    const BearingRefineOptions &opt, Eigen::Matrix3d &Q,
                    Eigen::Vector3d &t, int *out_initial_inliers = nullptr);

/// Rotation-only variant of refine_bearing_pose: the panorama centre @p c_world
/// (in world coordinates) is held fixed. Only the rotation @p Q is updated.
///
/// The centre enters the residual as `t = -Q * c_world` at each step,
/// recomputed from the current rotation rather than held as a separate
/// variable, so the translation DOF is fully suppressed.
///
/// @param c_world  fixed panorama centre in world space (from the seed frame
/// pose)
/// @param out_initial_inliers  same semantics as refine_bearing_pose
/// @returns final inlier indices, or empty when below `min_inliers`.
std::vector<int> refine_bearing_pose_rotation_only(
    const std::vector<Eigen::Vector3d> &points,
    const std::vector<Eigen::Vector3d> &bearings,
    const Eigen::Vector3d &c_world, const BearingRefineOptions &opt,
    Eigen::Matrix3d &Q, int *out_initial_inliers = nullptr);

// --- per-slice PnP and its noise floor (reusex-bx2.4) ----------------------

/// The 2D<->3D correspondences of one perspective slice of a panorama.
/// `pixels[k]` (slice pixel) and `world[k]` (3D point) are parallel arrays.
struct SlicePnPInput {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity(); ///< slice intrinsics
  /// maps slice-camera rays into the panorama frame
  Eigen::Matrix3d R_pano_from_view = Eigen::Matrix3d::Identity();
  std::vector<cv::Point2f> pixels;
  std::vector<Eigen::Vector3d> world;
};

/// solvePnPRansac parameters shared by the real resection and its null.
struct SlicePnPOptions {
  int min_correspondences = 6; ///< PnP a slice only with at least this many
  int ransac_iterations = 500;
  float ransac_reproj_px = 5.0f;
};

/// The strongest single-slice resection of a panorama.
struct SliceResection {
  int best_slice = -1;  ///< -1: no slice reached `min_correspondences` inliers
  int best_inliers = 0; ///< solvePnPRansac inliers of the best slice
  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity(); ///< pano_from_world
  Eigen::Vector3d t = Eigen::Vector3d::Zero();     ///< pano_from_world
  /// per slice: PnP inliers, 0 when PnP failed, -1 when not attempted (fewer
  /// than `min_correspondences` correspondences)
  std::vector<int> slice_inliers;
};

/// PnP every slice with solvePnPRansac (EPnP) and keep the one with the most
/// inliers (at least `min_correspondences`) as the panorama's initial pose.
SliceResection resect_best_slice(const std::vector<SlicePnPInput> &slices,
                                 const SlicePnPOptions &opt);

/// Null model of resect_best_slice()'s `best_inliers`: in each of @p trials
/// trials, every slice's correspondences are grouped by slice location (2 px
/// cells) and the groups' locations are permuted, so each group's world
/// points now pair with another group's pixels (same counts, same pixel and
/// world distributions, the matcher's repeated near-identical pairs kept
/// together, no geometry). Returns the maximum PnP inlier count over every
/// slice and trial — the support pure noise reaches on THIS panorama's
/// correspondences. The permutation is a Fisher-Yates driven by
/// std::mt19937(@p seed), so the result is reproducible (STANDARDS §6).
/// 0 when no slice has enough correspondences or @p trials <= 0.
int shuffled_best_slice_inliers(const std::vector<SlicePnPInput> &slices,
                                const SlicePnPOptions &opt, int trials,
                                unsigned seed);

/// The best-slice PnP inlier count a panorama needs to beat the noise floor:
/// strictly more than @p margin x @p null_best, and never fewer than
/// @p min_correspondences.
int noise_floor_threshold(int null_best, double margin,
                          int min_correspondences);

} // namespace reusex::geometry::pano_detail
