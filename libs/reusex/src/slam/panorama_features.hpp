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
  /// OpenCV's USAC framework (MAGSAC++ scoring, PROSAC-free uniform sampler,
  /// local optimisation) instead of the classic EPnP RANSAC loop. Seeded by
  /// @ref usac_seed, so it is as reproducible as the classic path.
  bool usac = false;
  int usac_seed = 0;
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

// --- the correspondence pool (reusex-bx2.5) --------------------------------
//
// The original pool let every slice keypoint pick its 1-NN in every candidate
// frame, so slices facing away from all frames still contributed 40-50 % of
// their keypoints as junk, overlapping slices entered the same panorama
// keypoint twice, and the nadir (operator, hand, stick) contributed 10 % of
// the pool and no inliers (docs/research/panorama-alignment-failure-modes.md).
// These helpers build the pool from the panorama's keypoints pooled over the
// whole sphere instead.

/// ORB keypoints of one perspective slice, as detected.
struct SliceKeypoints {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity(); ///< slice intrinsics
  Eigen::Matrix3d R_pano_from_view = Eigen::Matrix3d::Identity();
  cv::Size size;                   ///< slice image size (px)
  std::vector<cv::Point2f> pixels; ///< detected keypoints
  cv::Mat descriptors;             ///< one row per pixel
};

/// Which slice keypoints enter the sphere pool.
struct SpherePoolOptions {
  /// Keep a keypoint only in the slice that owns its bearing (owning_slice():
  /// each direction of the sphere is owned by exactly one slice), so a
  /// keypoint seen by two overlapping slices enters the pool once.
  bool dedupe_overlap = true;
  /// Drop keypoints whose bearing is within this angle (deg) of straight down
  /// (+y of the panorama frame) — the operator, hand and stick of a hand-held
  /// capture. 0 keeps everything.
  double nadir_mask_deg = 0.0;
};

/// The panorama's keypoints over the whole sphere. Parallel arrays.
struct SphereKeypoints {
  std::vector<cv::Point2f> pixels;       ///< pixel in the owning slice
  std::vector<int> slice;                ///< owning slice
  std::vector<Eigen::Vector3d> bearings; ///< unit bearing, panorama frame
  cv::Mat descriptors;                   ///< one row per keypoint
  int dropped_overlap = 0;               ///< owned by another slice
  int dropped_pole = 0;                  ///< inside the nadir mask
};

/// Unit bearing (panorama frame) of slice pixel @p px.
Eigen::Vector3d slice_bearing(const Eigen::Matrix3d &K,
                              const Eigen::Matrix3d &R_pano_from_view,
                              const cv::Point2f &px);

/// Index of the slice that owns @p bearing: among the slices whose image
/// contains the bearing, the one whose optical axis is closest to it (ties go
/// to the lower index). -1 when no slice sees it.
int owning_slice(const std::vector<SliceKeypoints> &slices,
                 const Eigen::Vector3d &bearing);

/// Pixel of panorama @p bearing in slice @p s; false when it falls outside
/// the slice image (or behind it).
bool project_to_slice(const SliceKeypoints &s, const Eigen::Vector3d &bearing,
                      cv::Point2f *px);

/// Pool the slices' keypoints over the sphere (slice order, then keypoint
/// order — deterministic).
SphereKeypoints pool_sphere_keypoints(const std::vector<SliceKeypoints> &slices,
                                      const SpherePoolOptions &opt);

/// One frame keypoint matched to one sphere keypoint.
struct SphereMatch {
  int frame_kp = -1;
  int sphere_kp = -1;
  float hamming = 0.0f;
};

/// Frame-driven matching against the whole sphere: every frame keypoint takes
/// its nearest sphere keypoint, and keeps it only when it beats, by Lowe's
/// @p ratio, the nearest sphere keypoint at a DIFFERENT place on the sphere
/// (bearing more than @p distinct_deg away). The competitor is searched among
/// the @p k nearest descriptors; when all of them sit at the same place
/// (ORB's duplicate detections of one corner), the match is kept.
std::vector<SphereMatch> match_frame_to_sphere(const cv::Mat &frame_desc,
                                               const SphereKeypoints &sphere,
                                               float ratio, double distinct_deg,
                                               int k = 4);

/// The legacy direction: every sphere keypoint takes its nearest frame
/// keypoint, Lowe-tested against the frame's second nearest.
std::vector<SphereMatch> match_sphere_to_frame(const cv::Mat &frame_desc,
                                               const SphereKeypoints &sphere,
                                               float ratio);

/// Indices (ascending) of the entries to keep so that each sphere keypoint
/// appears once: the lowest @p hamming wins, ties go to the earlier entry.
/// A panorama keypoint is one bearing constraint however many near-identical
/// candidate frames it matched; keeping all of them would let one wrong match
/// count several times in every PnP inlier tally.
std::vector<int> one_per_sphere_keypoint(const std::vector<int> &sphere_kp,
                                         const std::vector<float> &hamming);

} // namespace reusex::geometry::pano_detail
