// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Content-based alignment of 360 panoramas to the scan.
//
// A panorama is imported and placed on the trajectory purely by nearest EXIF
// timestamp (`rux import 360`), inheriting the matched sensor frame's pose.
// That is only as good as the clock sync and gives the panorama no independent
// orientation. This module refines the panorama's 6-DoF pose from image
// content:
//
//   1. render virtual pinhole slices of the equirect
//   (geometry/EquirectProjection)
//   2. ORB-match each slice against nearby sensor frames — the SAME
//   license-clean
//      OpenCV ORB + Lowe-ratio front-end used by LoopClosure
//   3. lift frame matches to metric 3D via the frame's stored depth + pose
//   4. resect the panorama pose per slice with solvePnPRansac (each slice is a
//      true pinhole with known K and known slice->panorama rotation), then
//      refine over all slices' inliers in panorama-bearing space
//
// The refined pose is a panorama-optical->world transform in the SAME
// convention as sensor_frame_pose, so it drops straight into the viewer /
// exporters and (as a follow-up, issue #236) into the pose graph as loop
// constraints.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <limits>
#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// Parameters for content-based panorama alignment.
struct PanoramaAlignmentOptions {
  // --- candidate sensor frames -------------------------------------------
  /// Try the timestamp-seed frame plus this many frames on each side of it (in
  /// sensor-frame-id order). A wider window co-observes the panorama from more
  /// directions, which conditions the translation in bearing resection.
  int candidate_window = 25;
  /// Cap the number of candidate frames actually matched (nearest-to-seed
  /// first), bounding the O(slices x frames) matcher cost.
  int max_candidates = 20;

  // --- panorama slicing (geometry/EquirectProjection) --------------------
  int n_yaw = 8;         ///< perspective slices around the equator
  double fov_deg = 90.0; ///< per-slice horizontal FOV (overlap for matching)
  int slice = 1008;      ///< slice size (px, square)

  // --- ORB front-end (mirrors LoopClosureOptions) ------------------------
  int max_features = 3000;  ///< ORB features per image
  float ratio_test = 0.85f; ///< Lowe ratio threshold
  float min_depth = 0.3f;   ///< ignore frame keypoints outside [min,max] depth
  float max_depth = 6.0f;

  // --- PnP resection -----------------------------------------------------
  int min_slice_correspondences = 6; ///< need this many to PnP a slice
  int min_inliers = 25;              ///< accept alignment above this (total)
  float ransac_reproj_px = 5.0f;     ///< solvePnPRansac reprojection threshold
  int ransac_iterations = 500;
  int refine_iterations = 10; ///< bearing-space Gauss-Newton refinement steps

  /// Plausibility gate: reject an alignment whose refined centre is further
  /// than this from the timestamp-seed frame. Bearing-only resection is weakly
  /// constrained in translation when the matched frames span a narrow cone, and
  /// can slide the centre far away with a still-small angular residual. The
  /// timestamp seed is a good spatial prior (the panorama was captured in the
  /// same session, metres from the nearest frame), so a large disagreement is
  /// an ill-conditioned solve, not a real correction. Set <= 0 to disable.
  double max_correction_m = 5.0;
  unsigned seed = 42;

  /// Fix the panorama centre to the timestamp-seed frame's stored position and
  /// solve for rotation only.
  ///
  /// When set:
  ///   - The centre (t in pano_from_world) is seeded directly from
  ///     `sensor_frame_pose(seed_node_id)` and held fixed throughout.
  ///   - The Gauss-Newton bearing refinement updates ONLY the rotation block;
  ///     the translation degree of freedom is suppressed.
  ///   - `delta_from_seed_m` is always 0 — the centre is definitionally the
  ///     seed, so the `max_correction_m` plausibility gate is vacuous and the
  ///     dry-run output says "translation fixed (seed node position)" instead.
  ///
  /// Default false for backward-compatibility. Recommended when EXIF timestamps
  /// are reliable (the panorama was captured at the same location as the
  /// matched sensor frame, so the seed position is an accurate prior; only
  /// orientation is uncertain).
  bool fix_translation = false;

  /// If non-empty, write an ORB-correspondence figure (panorama slice <-> the
  /// best-matching sensor frame, inlier matches drawn) for each aligned
  /// panorama to `<debug_dir>/<debug_name>_matches.jpg`.
  std::string debug_dir;
  std::string debug_name;

  /// Keep every depth-backed correspondence in
  /// PanoramaAlignmentDiagnostics::correspondences (off by default: a few
  /// thousand rows per panorama).
  bool collect_correspondences = false;
};

/// Which stage of align_panorama() decided the outcome. Every early exit has
/// its own value, so a rejected panorama says *why* it kept its timestamp
/// placement instead of a blanket "insufficient matches" (reusex-bx2.1).
enum class PanoramaAlignmentOutcome {
  aligned,        ///< pose accepted
  no_image,       ///< the stored panorama image did not decode
  seed_not_found, ///< the timestamp-seed node is not a sensor frame
  /// no slice reached `min_slice_correspondences` PnP inliers (or the pool of
  /// depth-backed correspondences was empty)
  no_slice_pnp,
  /// the best slice's PnP rotation is upside down against the seed frame's
  /// gravity direction (#364)
  upside_down,
  /// the best slice's pose, applied to the whole pool, gates fewer than
  /// `min_inliers` correspondences (the 6-DoF path's initial gate)
  too_few_pooled_inliers,
  /// the Gauss-Newton re-gating dropped below `min_inliers` (or, with
  /// `fix_translation`, the final rotation-only count did)
  refinement_collapsed,
  /// the refined centre is further than `max_correction_m` from the seed
  correction_gate,
  /// fix_translation was requested but the seed frame has no usable pose
  no_seed_pose,
};

/// Stable snake_case name of @p o ("aligned", "no_slice_pnp", ...), used in
/// logs and in the diagnostics JSON.
const char *to_string(PanoramaAlignmentOutcome o);

/// Matching funnel of one perspective slice of the panorama.
struct PanoramaSliceStats {
  double yaw_deg = 0.0;   ///< slice centre azimuth in the panorama frame
  double pitch_deg = 0.0; ///< slice centre elevation (+90 = up cap)
  int keypoints = 0;      ///< ORB keypoints detected in the slice
  /// slice keypoints whose best frame match passed the Lowe ratio test,
  /// summed over all candidate frames (before the depth check)
  int ratio_matches = 0;
  /// of those, the ones whose frame keypoint has valid depth in
  /// [min_depth, max_depth] — the slice's PnP input
  int correspondences = 0;
  /// solvePnPRansac inliers of this slice alone; -1 when PnP was not attempted
  /// (fewer than `min_slice_correspondences` correspondences)
  int pnp_inliers = -1;
  /// correspondences of this slice in the final inlier set (0 unless the
  /// refinement ran)
  int final_inliers = 0;
};

/// Matching funnel of one candidate sensor frame.
struct PanoramaFrameStats {
  int node_id = -1;
  /// frame timestamp minus the seed frame's (s); 0 for the seed itself, NaN
  /// when either timestamp is unknown
  double dt_from_seed_s = 0.0;
  /// frame centre's distance from the seed frame's centre (m); -1 when either
  /// pose is unusable
  double dist_from_seed_m = -1.0;
  int keypoints = 0;            ///< ORB keypoints detected in the frame
  int keypoints_with_depth = 0; ///< of those, with depth in range
  int ratio_matches = 0;   ///< Lowe-passing matches from all slices (pre-depth)
  int correspondences = 0; ///< of those, depth-backed
  int final_inliers = 0;   ///< correspondences in the final inlier set
};

/// One depth-backed slice<->frame correspondence, kept only when
/// PanoramaAlignmentOptions::collect_correspondences is set (debugging and
/// research: e.g. where on the sphere the matches land, relative to the
/// dual-fisheye stitch seam).
struct PanoramaCorrespondence {
  int slice = -1;
  int frame_node = -1;
  double azimuth_deg = 0.0;   ///< panorama bearing azimuth, atan2(x, z)
  double elevation_deg = 0.0; ///< panorama bearing elevation, asin(-y)
  float slice_x = 0, slice_y = 0;
  float frame_x = 0, frame_y = 0;
  float hamming = 0;         ///< descriptor distance of the match
  bool final_inlier = false; ///< in the final inlier set
};

/// Everything align_panorama() measured on the way to its outcome. Counters
/// a run never reached stay at their defaults (-1 / 0 / NaN).
struct PanoramaAlignmentDiagnostics {
  std::vector<PanoramaSliceStats> slices;
  std::vector<PanoramaFrameStats> frames; ///< candidate frames, match order
  int correspondences_total = 0; ///< pooled depth-backed correspondences
  int best_slice = -1;           ///< slice whose PnP seeded the refinement
  int best_slice_inliers = 0;
  /// pano_down . seed_down of the PnP seed rotation (gravity check, #364);
  /// NaN when not evaluated
  double down_dot = std::numeric_limits<double>::quiet_NaN();
  int initial_inliers = -1; ///< pool gated by the PnP seed pose
  int final_inliers = -1;   ///< after Gauss-Newton re-gating
  double rms_deg = -1.0;    ///< angular RMS of the final inliers
  /// refined centre's distance from the seed (m), also for a pose the
  /// plausibility gate rejected; -1 when not measured
  double delta_from_seed_m = -1.0;
  int inlier_frames = 0; ///< distinct frames among the final inliers
  int inlier_slices = 0; ///< distinct slices among the final inliers
  std::vector<PanoramaCorrespondence> correspondences;
};

/// Outcome of aligning one panorama.
struct PanoramaAlignmentResult {
  int pano_id = -1;
  int seed_node = -1;
  bool aligned = false;
  int inliers = 0;       ///< pooled PnP inliers supporting the pose
  double rms_deg = -1.0; ///< angular reprojection RMS over inliers (deg)
  /// panorama-optical->world, row-major 4x4 (same convention as
  /// ProjectDB::sensor_frame_pose). Identity when !aligned.
  std::array<double, 16> pose = {1, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1};
  /// Translation delta from the timestamp-seed placement (m); -1 when there is
  /// no seed, or when the seed frame carries no usable stored pose to measure
  /// against (#336) — in which case the `max_correction_m` plausibility gate
  /// is skipped and a warning names the frame.
  /// Always 0.0 when `PanoramaAlignmentOptions::fix_translation` was set.
  double delta_from_seed_m = -1.0;
  /// True when the panorama centre was fixed to the seed frame's stored
  /// position (PanoramaAlignmentOptions::fix_translation). Callers use this
  /// to emit "translation fixed (seed node position)" in dry-run output
  /// rather than a misleading delta of 0.
  bool translation_fixed = false;
  /// Which gate decided the result (`aligned` iff @ref aligned).
  PanoramaAlignmentOutcome outcome = PanoramaAlignmentOutcome::no_image;
  /// The per-slice / per-frame matching funnel behind @ref outcome.
  PanoramaAlignmentDiagnostics diagnostics;
};

/// One JSON object (as text) holding @p r's outcome, pose summary and
/// PanoramaAlignmentDiagnostics — the record `rux align 360 --diagnostics`
/// writes per panorama.
std::string
panorama_alignment_diagnostics_json(const PanoramaAlignmentResult &r);

/// Align one panorama against nearby sensor frames. Read-only on @p db (does
/// not write the pose back — the caller decides, e.g. `rux align 360
/// --dry-run`).
///
/// @param db            project database
/// @param pano_id       panoramic_images.id
/// @param seed_node_id  timestamp-matched sensor frame node id (>=0), used to
///                      seed the candidate window and measure the correction
/// @param opt           parameters
PanoramaAlignmentResult align_panorama(ProjectDB &db, int pano_id,
                                       int seed_node_id,
                                       const PanoramaAlignmentOptions &opt);

} // namespace reusex::geometry
