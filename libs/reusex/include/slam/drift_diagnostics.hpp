// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Drift diagnostics (reusex-rcv.1.1, `rux analyze drift`): WHERE do the
// stored poses of a capture disagree with themselves?
//
// Every measure here is GT-free and compares PASSES, never the whole cloud
// against itself (docs/research/visual-drift-loop.md §3):
//
//  - pass-to-pass residual: each back-projected depth point against the
//    nearest point of a frame captured more than gap_s (or gap_frames) away,
//    point-to-plane; the same measure against frames 1..floor_max_frames away
//    is the sensor NOISE FLOOR. Drift is the gap between the two (§3.1).
//  - the segment x segment matrix: the trajectory cut into `segments` equal
//    time segments, each pair's points matched only against each other. A
//    1-2 deg pass break that pooled numbers cannot see lights up here (§3.2).
//  - per-segment suggestions: each segment rigidly aligned (robust
//    point-to-plane ICP) to the union of the others: a ready-made edit with
//    its residual before and after (§3.3).
//  - per-cell map of the pass-to-pass residual (the drift heatmap).
//  - through-wall crossings of the camera path against a reference plan
//    (§3.5): drift OR a plan error, either way a place to look.
//  - doubled-wall patches on the fused cloud (§3.4): drawn, weakly scored.
//
// Deterministic (STANDARDS §6): fixed segment boundaries, no sampling, no
// randomness, parallel loops write per-index results that are reduced in
// index order; every output vector is sorted.

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}
namespace reusex::core {
struct SolidModel;
}

namespace reusex::slam {

/// Which stored pose the frames are placed with.
enum class DriftPoseSource {
  current, ///< sensor_frames.transform (what the clouds are built from)
  initial, ///< sensor_frames.initial_transform (what optimize seeds from);
           ///< a frame without one falls back to its current pose
};

/// Every tunable of diagnose_drift() (STANDARDS §4). The defaults reproduce
/// the research prototypes of docs/research/visual-drift-loop.md §3.
struct DriftDiagnosticsOptions {
  // --- frames and back-projection -----------------------------------------
  DriftPoseSource pose_source = DriftPoseSource::current;
  int frame_step = 1;         ///< use every Nth sensor frame (by node order)
  int depth_stride = 8;       ///< sample every Nth depth pixel
  double min_depth_m = 0.2;   ///< drop closer returns
  double max_depth_m = 4.0;   ///< drop farther returns
  int normal_neighbours = 12; ///< k of the PCA surface normal

  // --- pass-to-pass residual and noise floor ------------------------------
  /// Two frames are different passes when their timestamps are more than
  /// this apart ...
  double gap_s = 30.0;
  /// ... or, when the capture has no timestamps, their node-order indices
  /// (before frame_step) are more than this apart.
  int gap_frames = 30;
  /// Noise floor: matched against frames 1..N away in the sampled order,
  /// close enough in time that drift cannot have accumulated.
  int floor_max_frames = 3;
  int revisit_neighbours = 24;   ///< candidates searched per point
  double revisit_radius_m = 0.3; ///< ... within this distance
  /// A frame's residual is reported when it has at least this many matches.
  int min_frame_points = 50;
  /// Frames whose median residual exceeds this are counted as flagged.
  double frame_flag_m = 0.03;

  // --- segments -----------------------------------------------------------
  int segments = 12;            ///< equal time segments of the sampled frames
  double match_radius_m = 0.15; ///< segment-pair NN gate
  int min_pair_points = 300;    ///< a pair needs this many matches

  // --- per-segment rigid ICP suggestions ----------------------------------
  double icp_max_corr_m = 0.2;     ///< correspondence gate (also the overlap)
  double huber_m = 0.02;           ///< Huber threshold of the robust weights
  int icp_iterations = 30;         ///< deterministic bound (STANDARDS §6)
  int min_suggestion_points = 500; ///< overlap points needed to suggest

  // --- heatmap cells ------------------------------------------------------
  double cell_m = 0.5;      ///< world x/y cell size
  int min_cell_points = 50; ///< a cell needs this many residuals

  // --- through-wall crossings (needs a plan) ------------------------------
  double crossing_max_step_m = 1.5; ///< longer steps are jumps, not walks
  /// A crossing within this distance of an opening's extent is "at" it.
  double crossing_opening_tol_m = 0.25;
  /// The camera belongs to a storey when its level-frame z lies in
  /// (z_floor - below, z_ceiling + above).
  double crossing_storey_below_m = 0.5;
  double crossing_storey_above_m = 0.3;

  // --- doubled-wall patches (needs storeys: plan or project) --------------
  bool doubled_walls = true;
  double doubled_voxel_m = 0.02;  ///< fused cloud thinned to one point/voxel
  double doubled_band_lo_m = 1.2; ///< wall band above the storey floor
  double doubled_band_hi_m = 2.3;
  double doubled_ceiling_clear_m = 0.15; ///< band ends this far below ceiling
  double doubled_max_abs_nz = 0.3;       ///< |n_z| below this is a wall point
  double doubled_cell_m = 0.5;           ///< patch cell size
  double doubled_azimuth_bin_deg = 20.0; ///< patch normal-azimuth bin
  int doubled_min_patch_points = 40;
  double doubled_hist_bin_m = 0.01;       ///< offset histogram bin
  double doubled_hist_half_m = 0.35;      ///< histogram spans +- this
  double doubled_min_separation_m = 0.04; ///< two layers at least this apart
  double doubled_valley_ratio = 0.6;      ///< valley <= ratio * smaller peak
  double doubled_peak_ratio = 0.25;       ///< a peak is >= ratio * the highest
};

/// One sampled frame's pass-to-pass residual.
struct FrameDrift {
  std::int64_t node_id = 0;
  /// Median pass-to-pass residual of its points [m]; nullopt when fewer than
  /// min_frame_points of them found a point of another pass.
  std::optional<double> residual_m;
  double revisit_share = 0.0; ///< share of its points that found one
  std::size_t points = 0;     ///< back-projected points
};

/// One time segment of the sampled frames.
struct DriftSegment {
  int index = 0;
  std::int64_t first_node = 0, last_node = 0;
  std::size_t frames = 0, points = 0;
};

/// Segment @c a's points against segment @c b's (ordered: a is matched INTO
/// b, with b's normals).
struct SegmentPair {
  int a = 0, b = 0;
  double residual_m = 0.0; ///< median point-to-plane distance
  std::size_t points = 0;  ///< matches within match_radius_m
};

/// A rigid move of one segment that makes it agree with all the others.
struct SegmentSuggestion {
  int segment = 0;
  std::int64_t first_node = 0, last_node = 0;
  /// World-frame correction: new_pose = delta * pose for every frame of the
  /// segment.
  Eigen::Matrix4d delta = Eigen::Matrix4d::Identity();
  double before_m = 0.0;  ///< median residual of the overlap before
  double after_m = 0.0;   ///< ... and after the move
  std::size_t points = 0; ///< overlap points (within icp_max_corr_m)
  double shift_m = 0.0;   ///< |translation of delta| (about the world origin)
  /// How far delta moves the segment's own centroid: the shift a person
  /// means, free of the world-origin lever arm.
  double centroid_shift_m = 0.0;
  double rotation_deg = 0.0; ///< rotation angle of delta
  double yaw_deg = 0.0;      ///< its rotation about world z
};

/// The pass-to-pass residual of one world x/y cell (the drift heatmap).
struct DriftCell {
  double x = 0.0, y = 0.0; ///< cell centre, world [m]
  double residual_m = 0.0; ///< median
  std::size_t points = 0;
};

/// One camera step that crosses a reference-plan wall axis.
struct WallCrossing {
  std::int64_t from_node = 0, to_node = 0;
  std::string wall_guid;
  int storey = 0;          ///< plan storey idx
  double s_m = 0.0;        ///< position along the wall axis from (x0, y0)
  double x = 0.0, y = 0.0; ///< crossing point in the storey's level frame
  bool at_opening = false; ///< within crossing_opening_tol_m of an opening
};

/// trajectory_wall_crossings() result.
struct CrossingSummary {
  std::size_t steps = 0; ///< steps inside a storey and <= crossing_max_step_m
  std::vector<WallCrossing> crossings;
  [[nodiscard]] std::size_t through_wall() const;
};

/// Doubled-wall patch statistics of one storey's wall band.
struct DoubledWallStats {
  int storey = 0;
  std::size_t patches = 0; ///< patches with >= doubled_min_patch_points
  std::size_t bimodal = 0; ///< of which two layers
  double bimodal_share = 0.0;
  std::optional<double> separation_median_m; ///< of the bimodal patches
};

/// Half-vs-half summary of the segment matrix: segment pairs within the
/// first or within the second half of the capture against pairs across.
struct HalvesSummary {
  std::optional<double> within_m; ///< median of the within pairs' medians
  std::optional<double> across_m; ///< median of the across pairs' medians
  std::size_t within_pairs = 0, across_pairs = 0;
};

struct DriftReport {
  std::string pose_source;      ///< "current" | "initial"
  bool timestamps = false;      ///< passes split by time (else by frame index)
  std::size_t frames_total = 0; ///< sensor frames in the project
  std::size_t frames_sampled = 0; ///< after frame_step
  std::size_t frames_used = 0;    ///< sampled frames with depth and a pose
  std::size_t points = 0;         ///< back-projected points
  std::size_t revisit_points = 0; ///< points matched to another pass

  std::optional<double> noise_floor_m, noise_floor_p90_m;
  std::optional<double> residual_median_m, residual_p90_m;
  /// Median and p90 over the frames' median residuals.
  std::optional<double> frame_residual_median_m, frame_residual_p90_m;
  std::size_t revisit_frames = 0; ///< frames with a residual
  std::size_t frames_flagged = 0; ///< residual > frame_flag_m

  std::vector<FrameDrift> frames; ///< node order
  std::vector<DriftSegment> segments;
  std::vector<SegmentPair> pairs; ///< sorted by (a, b)
  HalvesSummary halves;
  std::vector<SegmentSuggestion> suggestions; ///< by segment
  std::vector<DriftCell> cells;               ///< by (x, y)
  /// Present when a plan was given.
  std::optional<CrossingSummary> crossings;
  std::vector<DoubledWallStats> doubled_walls; ///< by storey
  /// Each STANDARDS §5 warning also logged (e.g. no pass-to-pass overlap).
  std::vector<std::string> warnings;
};

/// Measure where the stored poses of @p db disagree (see the file comment).
/// Read-only.
///
/// @param plan optional reference plan (solid-model document) for the
///        through-wall crossings; its storeys also band the doubled-wall
///        patches (otherwise the project's storeys are used).
/// @param poses_override optional world poses replacing the stored ones,
///        index-aligned with db.sensor_frame_ids() (all frames, before
///        frame_step); e.g. a candidate solve that is not written yet.
/// @throws std::invalid_argument for a non-empty @p poses_override of the
///         wrong size, or options out of range (segments < 1, step < 1...).
DriftReport
diagnose_drift(const ProjectDB &db, const DriftDiagnosticsOptions &opts = {},
               const core::SolidModel *plan = nullptr,
               std::span<const Eigen::Matrix4d> poses_override = {});

/// Steps of a camera path (consecutive @p poses, world pose-frame origins)
/// that cross a wall axis of @p plan, and whether each lies at one of the
/// wall's openings. Shared with the door-evidence work (reusex-rqn.3).
/// @throws std::invalid_argument when the spans differ in length.
CrossingSummary
trajectory_wall_crossings(std::span<const std::int64_t> node_ids,
                          std::span<const Eigen::Matrix4d> poses,
                          const core::SolidModel &plan,
                          const DriftDiagnosticsOptions &opts = {});

/// The report as a JSON document (every field; nullopt as null).
std::string drift_report_to_json(const DriftReport &report, int indent = 2);

} // namespace reusex::slam
