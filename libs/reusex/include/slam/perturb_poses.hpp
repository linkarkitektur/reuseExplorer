// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Issue #338: seeded, deterministic drift synthesis on a seed trajectory.
//
// WHY THIS EXISTS. Issue #221 Tier 2 asks for a capture that BOTH drifts and
// has absolute ground truth, because only such a capture can adjudicate a pose
// refinement stage. The benchmark suite has neither half in one scan:
//
//   - the office / NewOffice captures drift heavily (~80% of the trajectory's
//     extent, docs/research/registration-improvements.md §9.5) but have no
//     ground-truth geometry, only the GT-FREE surface-consistency metrics; and
//   - the three ARKitScenes scans carry an absolute `3dod_mesh` GT but their
//     seeds are already accurate to 15.7-18.6 mm median with 2-9% edge
//     disagreement — they do not drift.
//
// On a scan that does not drift, the correct behaviour of every pose stage is
// to do nothing, so every solver-side experiment since #298 (#310, #311, #316,
// #337) has measured the benchmark rather than the solver. This module closes
// that gap the cheap way: take the ARKitScenes seeds, inject a realistic and
// exactly reproducible amount of drift, and keep the GT mesh — which lives in
// the world frame and is untouched — as the adjudicator. A pose stage that
// removes the synthetic drift now has somewhere to show it.
//
// WHAT "REALISTIC" MEANS HERE. Drift from a SLAM front-end is not a random
// displacement of each pose; it is the accumulation of small errors in the
// RELATIVE (odometry) chain, which is exactly why §8.3 found no outlier edge
// for a robust kernel to reject. So the model perturbs each consecutive
// relative pose and integrates:
//
//     P'[0]   = P[0]                                   (gauge, see below)
//     P'[i+1] = P'[i] * (P[i]^-1 * P[i+1]) * Exp(xi_i)
//
// with the per-step twist xi_i built from two components, both scaled by the
// step length so that drift grows with distance travelled rather than with
// frame count:
//
//   - a BIAS term, an Ornstein-Uhlenbeck process with a metric correlation
//     length. This is the systematic part — gyro bias, a scale error, a
//     miscalibrated depth unit — and it is what makes accumulated drift smooth
//     and curved rather than a jitter. Its rotational channel is the dominant
//     one in practice: a small constant yaw error integrates into a position
//     error that grows super-linearly with path length via the lever arm.
//   - a RANDOM-WALK term scaled by sqrt(step) so the relative errors compose
//     as Brownian motion, i.e. the classic "odometry error grows with the
//     square root of distance" behaviour.
//
// GAUGE. Frame 0 is left exactly on the seed. This is not cosmetic: the pose
// graph's only absolute anchor is its tight prior on frame 0 (§8.1), so a
// rigid displacement of the whole trajectory would be drift that no pose stage
// could possibly recover, and the benchmark would measure an impossibility
// rather than a capability. Anchoring frame 0 makes the injected error exactly
// the DEFORMATION the solver is asked to undo.
//
// DETERMINISM (STANDARDS §6). Everything is a pure function of (poses, seed,
// options). The noise sequence is drawn ONCE from a seeded std::mt19937 and
// then rescaled by the calibration search, so the amplitude solve cannot
// perturb the realisation it is calibrating.

#pragma once

#include <Eigen/Core>

#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// Parameters for synthetic trajectory drift (#338).
///
/// The primary knob is `drift_scale`; the base gains below set the CHARACTER of
/// the drift (how much is systematic vs random, rotational vs translational)
/// and the calibration solves for the overall amplitude that hits the requested
/// magnitude, so changing a base gain changes the shape of the deformation
/// without changing how large it is.
struct PoseDriftOptions {
  /// RNG seed (STANDARDS §6). Different seeds give different realisations of
  /// the same drift statistics — the intended way to get an ensemble.
  unsigned seed = 42;

  /// Multiplier on `target_drift_ratio`. 0 disables drift entirely (the poses
  /// come back bit-identical), which is the no-op guard.
  double drift_scale = 1.0;

  /// Target median relative-pose disagreement between the drifted and the seed
  /// trajectory over temporally distant frame pairs, as a FRACTION of the
  /// trajectory's extent. The amplitude of the perturbation is solved so the
  /// realised ratio matches `target_drift_ratio * drift_scale`.
  ///
  /// Default 0.20. The office scan's own measured value is 0.80 (14 455 mm of
  /// median disagreement over an 18.01 m extent, §9.5) and is reachable with
  /// `drift_scale = 4`, but it should not be the default: office walks 69 m of
  /// path inside an 18 m box, whereas an ARKitScenes room scan orbits 8-15 m of
  /// path inside a 1.8-2.6 m box. Demanding 80% of THAT extent needs a
  /// perturbation ~38x the base gains and tumbles the worst frame by 180 deg —
  /// which is no longer drift, it is destruction, and a benchmark built on it
  /// would measure nothing a solver could ever recover.
  ///
  /// 0.20 is chosen against the two thresholds that decide whether the
  /// benchmark can answer anything (§9.5): it puts 0.36-0.51 m of relative
  /// drift on these scans, i.e. 2-9x the measured XFeat edge error (58-169 mm),
  /// so drift >> edge error and loop closure has something to win; and it is
  /// 7-10x the 50 mm F-score threshold, so the no-pose-stage baseline is
  /// unambiguously bad against GT instead of marginally so.
  double target_drift_ratio = 0.20;

  /// Frame gap defining "temporally distant" for the metric above, mirroring
  /// LoopClosureOptions::min_frame_gap: pairs closer than this in index are
  /// handled by the odometry factors, not by drift-correcting constraints.
  /// Clamped down on short sequences so the measurement always has samples.
  int min_frame_gap = 50;

  // --- Character of the drift (base gains, rescaled by the calibration) ----
  /// Systematic rotational error per metre travelled (rad/m). The dominant
  /// real-world drift mode: integrated through the lever arm it produces the
  /// characteristic curved, super-linear divergence.
  double rot_bias_gain = 0.020;
  /// Rotational random walk (rad per sqrt(m)).
  double rot_walk_gain = 0.010;
  /// Systematic translational (scale) error, as a fraction of each step.
  double trans_bias_gain = 0.020;
  /// Translational random walk (m per sqrt(m)).
  double trans_walk_gain = 0.010;
  /// Correlation length of the bias process (m). Shorter = the systematic
  /// component wanders more often; longer = a more constant bias.
  double bias_correlation_length = 5.0;

  // --- Calibration search --------------------------------------------------
  /// Bisection steps used to solve for the amplitude. 60 is far more than
  /// needed for double precision and costs microseconds.
  int calibration_iterations = 60;
  /// Relative tolerance on the realised drift ratio.
  double calibration_tolerance = 1e-4;
};

/// What a drift-synthesis run did, in the units the benchmark reports.
struct PoseDriftResult {
  int frames = 0;                 ///< poses perturbed
  double trajectory_extent = 0.0; ///< m, bbox diagonal of the SEED centres
  double path_length = 0.0;       ///< m, cumulative seed travel
  double amplitude = 0.0;         ///< solved multiplier on the base gains
  bool calibrated = false;        ///< the search reached the requested ratio

  /// Realised median relative-pose disagreement over temporally distant pairs,
  /// as a fraction of `trajectory_extent` — the quantity `target_drift_ratio`
  /// asks for, and directly comparable to §9.5's per-scan table.
  double drift_ratio = 0.0;
  double median_pair_disagreement = 0.0; ///< m, numerator of the above

  // Camera-centre displacement between the seed and drifted trajectories.
  double median_position_error = 0.0; ///< m
  double max_position_error = 0.0;    ///< m
  double final_position_error = 0.0;  ///< m, last frame
  double max_rotation_error = 0.0;    ///< deg
};

/// Apply synthetic drift to a trajectory in place (the pure, database-free
/// core — this is what the unit tests exercise).
///
/// @param poses  optical->world poses in temporal order; perturbed in place.
///               Fewer than 2 poses is a no-op.
/// @param opt    drift parameters
/// @returns      realised drift statistics
PoseDriftResult perturb_trajectory(std::vector<Eigen::Matrix4d> &poses,
                                   const PoseDriftOptions &opt);

/// Measure the median relative-pose disagreement between two trajectories over
/// temporally distant frame pairs (m). This is the §9.5 "edge <-> seed
/// disagreement" statistic, computed against a known reference instead of
/// against a matcher, and is exported so the benchmark can report the same
/// number for a drifted scan that #337 reported for a real one.
///
/// Both vectors must be the same length and in the same temporal order.
double median_pair_disagreement(const std::vector<Eigen::Matrix4d> &reference,
                                const std::vector<Eigen::Matrix4d> &other,
                                int min_frame_gap);

/// Apply synthetic drift to every sensor-frame pose stored in @p db (#338).
///
/// Frames are ordered by node id (the same ordering optimize_sensor_poses uses)
/// and only frames that actually carry a pose participate. The stored poses are
/// OVERWRITTEN — this is a destructive operation intended for a disposable COPY
/// of a project, never for an original capture; `rux edit perturb-poses` says
/// so in its help and the bench harness copies first.
///
/// Also writes the drifted poses to `initial_transform` and, when the
/// project has `odometry_init` edges, re-derives them
/// (`slam::init_pose_graph(force)`), so `rux optimize` sees the drift.
///
/// @param db       project to perturb, opened read-write
/// @param opt      drift parameters
/// @param dry_run  measure and report without writing anything back (how the
///                 harness reads a trajectory's extent before committing)
/// @returns        realised drift statistics
/// @throws std::runtime_error if fewer than two frames carry a pose
PoseDriftResult perturb_sensor_poses(ProjectDB &db, const PoseDriftOptions &opt,
                                     bool dry_run = false);

} // namespace reusex::geometry
