// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Pose-reference scoring (reusex-rcv.1.6): ATE/RPE between an estimated and a
// reference trajectory, the pose analogue of
// `reconstruction/solid_metrics.hpp`. Backs `rux analyze poses` and
// `scripts/bench-poses.sh`.
//
// Matching. Two trajectories are matched by nearest `timestamp`
// (`core/trajectory.hpp` explains why: a trajectory read back from a TUM file
// carries no other key), within a fixed absolute tolerance
// (`kMatchToleranceS`) chosen for the module's two real uses -- comparing two
// exports of the SAME project (identical timestamps, or identical node ids
// written into the timestamp column) and comparing a drifted ARKitScenes
// trajectory against its undrifted seed (same, since `perturb_sensor_poses`
// never touches timestamps). A looser association (TUM's `associate.py`
// style nearest-neighbour over a wide window) is not needed for either case
// and would risk silently pairing unrelated frames.
//
// RPE (Sturm et al., 2012) needs no alignment: it compares relative motion
// within each trajectory over a fixed "delta" (a path-length target or a
// fixed matched-index gap), so a shared global offset cancels. ATE does need
// alignment to be meaningful when the two trajectories do not already share a
// gauge; `PoseMetricsOptions::align` controls it (Umeyama, `Eigen::umeyama`).

#include <reusex/core/trajectory.hpp>

#include <cstddef>
#include <span>
#include <vector>

namespace reusex::slam {

/// Options for `compare_trajectories`.
struct PoseMetricsOptions {
  enum class Align { none, se3, sim3 };

  /// Rigid (se3) or similarity (sim3) Umeyama alignment of the estimate onto
  /// the reference before computing ATE. `none` (the default) is right when
  /// both trajectories already share a gauge, e.g. the same project's current
  /// poses against its own frame-0-anchored reference (docs/research/
  /// visual-drift-loop.md §4.7: "no alignment by default because the gauge is
  /// shared").
  Align align = Align::none;

  /// RPE delta as a path length travelled along the reference trajectory [m].
  double rpe_distance_m = 1.0;

  /// RPE delta as a matched-index gap (not a raw node-id gap: the gap is
  /// counted over the matched, time-sorted pairs).
  int rpe_frames = 10;

  /// Number of contiguous chunks `segment_ate_m` splits the matched sequence
  /// into (equal size, the remainder in the last chunk).
  int segments = 12;
};

/// Result of `compare_trajectories`. A metric is 0 when `matched < 2` (too
/// few pairs to compute anything); the caller is told via `matched`.
struct PoseMetrics {
  double ate_rmse_m = 0.0;
  double ate_median_m = 0.0;
  double ate_max_m = 0.0;
  /// RMS rotation error between matched, aligned poses [deg].
  double rot_rmse_deg = 0.0;

  /// RPE at `rpe_distance_m` metres travelled (RMS over all valid deltas).
  double rpe_trans_m = 0.0;
  double rpe_rot_deg = 0.0;
  /// RPE at `rpe_frames` matched-index steps (RMS over all valid deltas).
  double rpe_trans_frames_m = 0.0;
  double rpe_rot_frames_deg = 0.0;

  /// Per-segment ATE RMSE [m], under the one alignment computed over the
  /// whole trajectory. Empty when `matched < 2`.
  std::vector<double> segment_ate_m;

  std::size_t matched = 0; ///< number of (est, ref) pairs the comparison used

  /// Recovered Umeyama scale (1.0 for `Align::none` / `Align::se3`).
  double scale = 1.0;
};

/// Compare an estimated trajectory against a reference.
///
/// @throws std::invalid_argument if any option is out of range
///         (`rpe_distance_m <= 0`, `rpe_frames <= 0`, `segments <= 0`).
PoseMetrics compare_trajectories(std::span<const core::StampedPose> est,
                                 std::span<const core::StampedPose> ref,
                                 const PoseMetricsOptions &opt = {});

} // namespace reusex::slam
