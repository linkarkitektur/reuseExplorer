// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Issue #236: wide-baseline pose-graph loop edges derived from 360 panoramas.
//
// WHY A PANORAMA IS THE RIGHT SENSOR FOR THIS
// -------------------------------------------
// The plane-landmark back-end (PlaneGraphOptimizer) makes frames that
// co-observe the same wall mutually consistent, but two temporally distant
// views of the same place that share no plane-landmark chain are left
// unconstrained — the "basin problem" of
// docs/research/registration-improvements.md. The ORB front-end in
// LoopClosure.hpp attacks that by matching frame PAIRS, which requires the two
// narrow-FOV frames to overlap each other.
//
// A 360 panorama sees every direction at once, so a SINGLE panorama routinely
// matches frames that do not overlap each other at all — including frames from
// opposite ends of the capture. Each such panorama is therefore a hub that ties
// a whole group of temporally distant frames together.
//
// WHAT MAKES THE EDGE INFORMATIVE (the one thing to get right)
// ------------------------------------------------------------
// `PanoramaAlignment` resects ONE panorama pose in WORLD coordinates, from
// frame keypoints already transformed to world by their (drifted) seed poses.
// An edge built from that pose would be circular: `T_pano_A^-1 · T_pano_B`
// would reproduce `seed(A)^-1 · seed(B)` and carry exactly ZERO
// drift-correction information — it would restate the drift rather than measure
// it.
//
// This module therefore resects the panorama INDEPENDENTLY against each matched
// frame, using that frame's keypoints in the frame's OWN optical coordinates:
//
//   T_pano_A  :  pano-from-A, from A's correspondences alone
//   T_pano_B  :  pano-from-B, from B's correspondences alone
//   T_AB = T_pano_A^-1 · T_pano_B          (== pose(A)^-1 · pose(B))
//
// Neither resection reads `sensor_frame_pose`, so `T_AB` is a genuine metric
// measurement of where B sits relative to A. The panorama is a shared, rigid
// intermediate coordinate frame — it never has to be correctly placed in world.
// (A useful consequence: `rux align 360` is NOT a prerequisite. Panorama poses
// are neither read nor written here.)
//
// GUARDRAILS: nothing new is invented. The emitted LoopEdges are the SAME type
// the ORB front-end emits, are gated by the SAME LoopClosureOptions knobs
// (`min_frame_gap`, `min_seed_disagreement`, `max_seed_disagreement`), are
// PCM-filtered as part of the UNION with the other edge sources, and enter the
// SAME GncOptimizer graph — so a wrong panorama edge is down-weighted rather
// than corrupting the solution.
//
// This header carries no GTSAM and no OpenCV include (docs/STANDARDS.md §2).

#pragma once

#include "reusex/slam/LoopClosure.hpp"

#include <Eigen/Core>

#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// Parameters for panorama-derived loop-edge detection.
///
/// The ORB / slicing defaults deliberately mirror `PanoramaAlignmentOptions` so
/// the two front-ends see the same features; the frame-selection and
/// edge-emission knobs are specific to loop closure.
struct PanoramaLoopOptions {
  /// Off by default; opt in with `rux graph discover --use-panoramas`.
  bool enable = false;

  // --- candidate frames ----------------------------------------------------
  /// Panorama alignment matches a *temporal* window around the timestamp-seed
  /// frame (`PanoramaAlignmentOptions::candidate_window`), every pair of which
  /// is closer than `min_frame_gap` — so it could never yield a loop edge.
  /// Loop-edge detection instead sweeps the WHOLE trajectory, evenly
  /// subsampled to this many frames. That global sweep is what lets one
  /// panorama tie temporally-distant revisits together; it is also the dominant
  /// cost (slices x frames descriptor matches), hence the budget.
  int max_frames = 240;
  /// Skip frames whose colour/depth cannot be read rather than failing; a scan
  /// with fewer than this many usable candidates cannot support loop edges.
  int min_candidate_frames = 8;

  // --- panorama slicing (geometry/EquirectProjection) ----------------------
  int n_yaw = 8;         ///< perspective slices around the equator
  double fov_deg = 90.0; ///< per-slice horizontal FOV (overlapping)
  int slice = 1008;      ///< slice size (px, square)

  // --- ORB front-end -------------------------------------------------------
  int max_features = 3000;  ///< ORB features per image
  float ratio_test = 0.85f; ///< Lowe ratio threshold
  float min_depth = 0.3f;   ///< ignore frame keypoints outside [min,max] (m)
  float max_depth = 6.0f;

  // --- per-frame resection -------------------------------------------------
  /// A frame needs this many slice correspondences before resection is
  /// attempted. Lower than the alignment path's pooled requirement because the
  /// pool here is ONE frame, not all candidates.
  int min_frame_correspondences = 10;
  /// A single (slice, frame) pair needs this many correspondences to seed the
  /// resection with `solvePnPRansac` (which needs >= 4 for EPnP; 6 gives it
  /// margin). Mirrors PanoramaAlignmentOptions::min_slice_correspondences.
  int min_slice_correspondences = 6;
  /// Accept a frame's independent resection above this many gated inliers. The
  /// binding quality gate: a weak resection makes a wrong edge, and unlike the
  /// alignment path there is no second chance from other frames' points.
  int min_frame_inliers = 20;
  float ransac_reproj_px = 5.0f; ///< solvePnPRansac reprojection threshold
  int ransac_iterations = 500;
  int refine_iterations = 10; ///< bearing-space Gauss-Newton steps
  /// Reject a resection whose panorama centre sits further than this from the
  /// frame (m). Bearing resection from a narrow match cone is weakly
  /// constrained in translation and can slide the centre far away at a still
  /// small angular residual; the panorama was physically captured within a room
  /// of the frames it matches. Mirrors PanoramaAlignmentOptions::
  /// max_correction_m. <= 0 disables.
  double max_pano_distance = 8.0;

  // --- edge emission -------------------------------------------------------
  /// Cap the edges contributed per panorama (highest joint inlier support
  /// first). One panorama matching many frames would otherwise dominate the
  /// graph with O(F^2) mutually-derived edges that share the same resection
  /// errors and are therefore NOT independent measurements.
  int max_edges_per_panorama = 24;
  /// Base sigmas at `min_frame_inliers` support, shrinking as
  /// sqrt(min_frame_inliers / inliers) down to the floors — the same
  /// inlier-scaling law LoopClosureOptions uses. Deliberately looser than the
  /// ORB pair front-end: a panorama edge chains TWO independent resections, so
  /// its error is the composition of both.
  float base_sigma_trans = 0.15f; ///< m
  float base_sigma_rot = 0.06f;   ///< rad
  float min_sigma_trans = 0.05f;  ///< floor on the translational std (m)
  float min_sigma_rot = 0.025f;   ///< floor on the rotational std (rad)

  unsigned seed = 42; ///< RANSAC determinism (docs/STANDARDS.md §6)

  /// Memberwise, covering every field above — see SurfelExtractionParams's
  /// operator== for why (integration review finding 4).
  bool operator==(const PanoramaLoopOptions &) const = default;
};

/// Statistics from panorama loop-edge detection. Every drop reason is counted
/// so an empty result is explainable rather than mysterious (STANDARDS §5).
struct PanoramaLoopResult {
  int panoramas = 0;         ///< panoramas in the project
  int panoramas_matched = 0; ///< panoramas that resected >= 2 frames
  int frames_resected = 0;   ///< accepted independent per-frame resections
  int proposed = 0;          ///< candidate frame pairs before gating
  int dropped_gap = 0;       ///< rejected: closer than min_frame_gap
  int dropped_seed_gate = 0; ///< rejected: min/max seed-disagreement
  int dropped_cap = 0;       ///< rejected: max_edges_per_panorama
  int edges = 0;             ///< emitted edges
  int total_inliers = 0;     ///< summed supporting inliers
};

/// Detect panorama-derived wide-baseline loop edges.
///
/// Deterministic given the same database and options (STANDARDS §6): panoramas
/// and frames are processed in sorted id order, the OpenCV RNG is re-seeded per
/// panorama, and edge ordering/capping breaks ties on (i, j).
///
/// A project with no panoramas returns an empty vector and is NOT an error —
/// the caller decides whether the user explicitly asked for panorama edges and
/// should therefore be warned. `out_result->panoramas` reports the count either
/// way.
///
/// @param db          project database (read-only: panorama images, frame
///                    colour/depth/intrinsics; panorama POSES are never read)
/// @param node_ids    database node id of each frame, in frame-index order, so
///                    LoopEdge::i/j index straight into the caller's vector
/// @param seed_poses  optical->world seed pose of each frame. Used ONLY by the
///                    seed-disagreement gates — never to build the measurement.
/// @param options     detection parameters
/// @param gates       `min_frame_gap` / `min_seed_disagreement` /
///                    `max_seed_disagreement` reused from the ORB front-end, so
///                    one set of CLI knobs governs every edge source
/// @param out_result  optional statistics
/// @returns           accepted loop edges (possibly empty)
std::vector<LoopEdge>
detect_panorama_loop_edges(ProjectDB &db, const std::vector<int> &node_ids,
                           const std::vector<Eigen::Matrix4d> &seed_poses,
                           const PanoramaLoopOptions &options,
                           const LoopClosureOptions &gates,
                           PanoramaLoopResult *out_result = nullptr);

/// Internals exposed ONLY so the geometry and gating stages can be regression-
/// tested without a database and without RGB-D fixtures (docs/STANDARDS.md §7;
/// same rationale as `LoopClosure`'s `detail` namespace). Not a stable API.
namespace detail {

/// One panorama's independent resection against one frame.
struct PanoResection {
  int frame = -1; ///< index into the optimizer's frame vector
  /// pano-from-frame: maps a point in the frame's optical coordinates into the
  /// panorama's coordinates. Built from that frame's correspondences ALONE.
  Eigen::Matrix4d T_pano_frame = Eigen::Matrix4d::Identity();
  int inliers = 0; ///< gated bearing inliers supporting this resection
};

/// Turn one panorama's per-frame resections into gated loop edges.
///
/// For each pair (A, B) with A < B by frame index:
///   `T_AB = T_pano_A^-1 · T_pano_B`, i.e. pose(A)^-1 · pose(B).
///
/// Applies, in order: `min_frame_gap`, the seed-disagreement gates, and the
/// per-panorama cap (highest joint support first, ties broken on (i, j) so the
/// output is deterministic). Sigmas scale with joint inlier support.
///
/// @p stats is accumulated into when non-null (counters are added, not reset),
/// so a caller can sum across panoramas.
std::vector<LoopEdge>
edges_from_resections(std::vector<PanoResection> resections,
                      const std::vector<Eigen::Matrix4d> &seed_poses,
                      const PanoramaLoopOptions &opt,
                      const LoopClosureOptions &gates,
                      PanoramaLoopResult *stats = nullptr);

} // namespace detail

} // namespace reusex::geometry
