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
///
/// These defaults are the ONE source of truth: `rux align 360` binds its
/// flags to a default-constructed instance (STANDARDS §4).
struct PanoramaAlignmentOptions {
  // --- candidate sensor frames -------------------------------------------
  /// Try the timestamp-seed frame plus this many frames on each side of it (in
  /// sensor-frame-id order). In principle a wider window co-observes the
  /// panorama from more directions; on NewOffice it mostly added junk faster
  /// than signal (docs/research/panorama-alignment-failure-modes.md), so the
  /// window is narrow by default.
  int candidate_window = 12;
  /// Cap the number of candidate frames actually matched (nearest-to-seed
  /// first), bounding the O(slices x frames) matcher cost.
  int max_candidates = 8;

  // --- panorama slicing (geometry/EquirectProjection) --------------------
  int n_yaw = 8;         ///< perspective slices around the equator
  double fov_deg = 90.0; ///< per-slice horizontal FOV (overlap for matching)
  int slice = 1008;      ///< slice size (px, square)

  // --- ORB front-end (mirrors LoopClosureOptions) ------------------------
  int max_features = 3000;  ///< ORB features per image
  float ratio_test = 0.85f; ///< Lowe ratio threshold
  float min_depth = 0.3f;   ///< ignore frame keypoints outside [min,max] depth
  float max_depth = 6.0f;

  // --- correspondence pool (reusex-bx2.5) --------------------------------
  // Each of these was measured on NewOffice (18 panoramas, 4 seeds, with the
  // shuffled null): with all five on, 5.0 panoramas per run clear the noise
  // floor and the 1.5 m prior against 2.0 before, and the null accepts none.
  // Turning any one off loses accepts or makes them disagree across seeds
  // (docs/research/panorama-alignment-failure-modes.md).
  /// Keep each panorama keypoint once: only in the slice whose optical axis
  /// is closest to it among the slices that see it. Adjacent slices overlap
  /// 45 deg at 8 x 90 deg, so without this the same keypoint enters the pool
  /// (and every inlier tally) twice.
  bool dedupe_overlap = true;
  /// Drop panorama keypoints within this angle (deg) of straight down: the
  /// operator, hand and stick of a hand-held capture, which gave 10 % of the
  /// NewOffice pool and no inliers. 45 deg removes the down cap's own region
  /// (30 and 55 deg both measured worse). 0 keeps them.
  double nadir_mask_deg = 45.0;
  /// Frame-driven matching: each frame keypoint takes its nearest panorama
  /// keypoint over the whole sphere, Lowe-tested against the nearest one at
  /// a different place (> `distinct_deg` away). When false, each panorama
  /// keypoint takes its nearest keypoint in each frame (the original
  /// slice-driven matching, which passed 40-50 % of the keypoints of slices
  /// no frame sees).
  bool frame_to_sphere = true;
  /// Two panorama keypoints closer than this (deg) are the same place (ORB's
  /// duplicate detections across pyramid levels) and never each other's Lowe
  /// competitor.
  double distinct_deg = 1.0;
  /// Keep one correspondence per panorama keypoint (the lowest Hamming
  /// distance over all candidate frames), so a keypoint matched in several
  /// near-identical frames counts once.
  bool one_per_keypoint = true;

  // --- PnP resection -----------------------------------------------------
  int min_slice_correspondences = 6; ///< need this many to PnP a slice
  /// Floor of the pooled gate: the best slice's pose must explain at least
  /// this many correspondences of the whole pool, before and after the
  /// bearing refinement. With `pooled_null_margin` (the default) the gate is
  /// the larger of this and the shuffled null's pooled support x margin; with
  /// the null off (or `fix_translation`) it is the whole gate. 12 (twice
  /// the slice PnP minimum) never bound on NewOffice: the calibrated count
  /// was 19 or more for every accepted panorama.
  int min_inliers = 12;
  float ransac_reproj_px = 5.0f; ///< solvePnPRansac reprojection threshold
  int ransac_iterations = 500;
  /// Resect each slice with OpenCV's USAC (MAGSAC++ scoring, local
  /// optimisation; seeded by `seed`) instead of the classic EPnP RANSAC loop.
  /// The noise-floor null uses the same estimator.
  bool usac = true;
  int refine_iterations = 10; ///< bearing-space Gauss-Newton refinement steps

  // --- noise-floor gate (reusex-bx2.4) -----------------------------------
  /// Before any pose is trusted, the per-slice PnP is re-run this many times
  /// on the panorama's own correspondences with each slice's world points
  /// randomly permuted (a null model: same counts and distributions, no
  /// geometry). The best of those null counts is the support pure noise
  /// reaches on this panorama; on NewOffice it was 0-17 inliers per panorama
  /// with 8 candidate frames and up to 27 with 20, well above the fixed
  /// `min_slice_correspondences`. 0 disables the gate.
  int null_trials = 5;
  /// The real best slice must have strictly more than `null_margin` x the
  /// null's best-slice inliers (and at least `min_slice_correspondences`),
  /// else the panorama is rejected as `below_noise_floor`.
  double null_margin = 1.5;
  /// Pooled gate against the same null (reusex-bx2.8): each null slice pose
  /// is applied to the whole permuted pool and refined like the real one,
  /// and the real pooled count (before and after refinement) must beat the
  /// null's best by this factor. A fixed count could not be right for every
  /// panorama: after the deduplicated pool (reusex-bx2.5) NewOffice 007 and
  /// 012 beat their best-slice noise floor 3.8x and 5.1x but held 30 and 36
  /// pooled inliers against a fixed 40. 2 was measured on NewOffice (4
  /// seeds): 1.5 also accepts 008 and 009 at centres that move 1.0-1.9 m
  /// between seeds; 2.5 loses 011 and 013. With the best-slice gate relaxed
  /// to 1.0, every shuffled-null pose that reached this gate stopped here,
  /// where a fixed 20 would have let two through. 0 keeps only
  /// `min_inliers`.
  double pooled_null_margin = 2.0;

  /// Hand-held position prior: reject an alignment whose refined centre is
  /// further than this from the timestamp-seed frame. Bearing-only resection
  /// is weakly constrained in translation when the matched frames span a
  /// narrow cone, and can slide the centre metres away at a still-small
  /// angular residual. A hand-held 360 capture is taken next to the scanning
  /// device (on NewOffice the iPad is visible in the nadir, and every
  /// panorama lies within 1.7 s of its seed frame), so its centre is within
  /// about an arm's span of the seed frame: 1.5 m allows for the span between
  /// the two hands plus the seed frame's timing. Raise it (or set <= 0 to
  /// disable) for a 360 camera that was NOT carried with the scanner.
  double max_correction_m = 1.5;
  unsigned seed = 42; ///< seeds the noise-floor permutations (STANDARDS §6)

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
  /// the best slice's PnP inliers do not beat the shuffled-correspondence
  /// null by `null_margin` (reusex-bx2.4): the matches carry no geometric
  /// signal distinguishable from noise
  below_noise_floor,
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
  /// keypoints of this slice that entered the sphere pool (after the
  /// overlap dedupe and the nadir mask)
  int pooled_keypoints = 0;
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
  int sphere_keypoints = 0;      ///< panorama keypoints in the sphere pool
  int dropped_overlap = 0;       ///< keypoints dropped as another slice's
  int dropped_pole = 0;          ///< keypoints inside the nadir mask
  /// depth-backed matches dropped as repeats of a panorama keypoint already
  /// matched in another frame (`one_per_keypoint`)
  int dropped_repeats = 0;
  int best_slice = -1; ///< slice whose PnP seeded the refinement
  int best_slice_inliers = 0;
  /// best-slice PnP inliers under the shuffled-correspondence null (max over
  /// `null_trials`); -1 when the gate did not run
  int null_best_slice_inliers = -1;
  /// best-slice inliers needed to pass the noise floor; -1 when not run
  int noise_floor_threshold = -1;
  /// pooled support of the shuffled null's poses, before / after refinement
  /// (reusex-bx2.8); -1 when the pooled null did not run
  int null_pooled_initial = -1;
  int null_pooled_final = -1;
  /// pooled inliers needed before / after refinement; -1 when not reached
  int pooled_threshold = -1;
  int final_threshold = -1;
  /// pano_down . seed_down of the PnP seed rotation (gravity check, #364);
  /// NaN when not evaluated
  double down_dot = std::numeric_limits<double>::quiet_NaN();
  int initial_inliers = -1; ///< pool gated by the PnP seed pose
  int final_inliers = -1;   ///< after Gauss-Newton re-gating
  double rms_deg = -1.0;    ///< angular RMS of the final inliers
  /// refined centre's distance from the seed (m), also for a pose the
  /// plausibility gate rejected; -1 when not measured
  double delta_from_seed_m = -1.0;
  /// the position-prior gate the delta was held against (m); <= 0 when off
  double max_correction_m = -1.0;
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

/// One line naming @p r's outcome and the numbers that decided it, e.g.
/// "below_noise_floor: best slice 13 PnP inliers, needs >= 23 (shuffled null
/// 15)"
/// or "correction_gate: centre 2.49 m from seed > 1.50 m". Callers log it
/// for every rejected panorama (STANDARDS §5).
std::string panorama_alignment_summary(const PanoramaAlignmentResult &r);

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
