// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

// Included for the sole purpose of MIRRORING the library defaults below
// (docs/STANDARDS.md §4): a CLI default that references the library's own
// value cannot silently drift away from it.
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/drift_diagnostics.hpp>

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

/// Options for the `rux optimize` subcommand (global plane-landmark pose-graph
/// optimization). Defaults mirror reusex::geometry::PlaneGraphOptions so the
/// library remains the single source of truth (docs/STANDARDS.md §4).
struct SubcommandOptimizeOptions {
  // Per-frame plane detection. Defaults are the "mid-density" sweet spot
  // measured on the office + MuSHRoom-honka scans: with per-observation inlier
  // weighting on, this beats the conservative (max=4/200/obs=5) defaults on
  // BOTH GT-free flatness and laser GT F-score, and is the first pose stage to
  // improve flatness without degrading GT (see docs/research/…). More/denser
  // planes than this only help when combined with inlier weighting.
  int max_planes_per_frame = 6;
  int min_plane_inliers = 120;
  float ransac_distance = 0.02f;
  float ransac_normal_angle = 20.0f;
  int ransac_iterations = 200;

  // Cross-frame association.
  float assoc_normal_angle = 10.0f;
  float assoc_distance = 0.10f;
  int min_observations = 4;
  float assoc_overlap_margin = 0.30f;
  float min_landmark_spread_ratio = 0.05f;

  // Alternating rounds.
  int assoc_rounds = 2;
  float assoc_round_tol = 0.02f;

  // Factor-graph noise.
  float odometry_sigma_rot = 0.005f;
  float odometry_sigma_trans = 0.01f;
  float underconstrained_odom_scale = 0.25f;
  std::string odometry_noise = "fixed"; // fixed | motion
  float odometry_weight_min = 0.5f;
  float odometry_weight_max = 3.0f;
  bool odometry_robust = false;
  float odometry_gnc_inlier_cost = 8.41f;
  float plane_sigma_normal = 0.24f;
  float plane_sigma_distance = 0.19f;
  std::string plane_noise = "inliers"; // uniform | inliers | fit
  bool no_plane_inlier_weight = false;
  float plane_weight_min = 0.5f;
  float plane_weight_max = 3.0f;
  float plane_sigma_scale = 1.0f;
  bool no_plane_factors = false;
  float prior_sigma_rot = 0.001f;
  float prior_sigma_trans = 0.001f;

  // Solver.
  bool no_gnc = false; ///< disable Graduated Non-Convexity (plain LM)
  float gnc_inlier_cost = 5.67f;
  int iterations = 100;
  unsigned seed = 42;

  // Loop-edge solver handling (the edges themselves come from the stored
  // graph — `rux graph discover`). Defaults mirror PlaneGraphOptions.
  bool loop_trust = false;
  float loop_trust_inlier_cost =
      reusex::geometry::PlaneGraphOptions{}.loop_trust_inlier_cost;
  float loop_default_sigma_rot =
      reusex::geometry::PlaneGraphOptions{}.loop_default_sigma_rot;
  float loop_default_sigma_trans =
      reusex::geometry::PlaneGraphOptions{}.loop_default_sigma_trans;

  // Manhattan-world prior (reusex-rqn.1). Defaults mirror PlaneGraphOptions.
  bool manhattan = reusex::geometry::PlaneGraphOptions{}.manhattan;
  float manhattan_sigma_deg =
      reusex::geometry::PlaneGraphOptions{}.manhattan_sigma_deg;
  float manhattan_max_angle_deg =
      reusex::geometry::PlaneGraphOptions{}.manhattan_max_angle_deg;
  float manhattan_level_max_angle_deg =
      reusex::geometry::PlaneGraphOptions{}.manhattan_level_max_angle_deg;
  std::string manhattan_up = "z"; ///< auto | x | y | z

  // Pose edits (reusex-rcv.1.3): a reusex.pose_edits.v1 file
  // (docs/design/pose-edits.md); empty = none. Default mirrors
  // PlaneGraphOptions.
  std::string edits;
  float edit_inlier_cost =
      reusex::geometry::PlaneGraphOptions{}.edit_inlier_cost;

  // Surfel extraction (shared with `rux register`).
  float surfel_voxel = 0.03f;
  float min_distance = 0.0f;
  float max_distance = 4.0f;
  // Mirrors the plane-graph override of the shared surfel default (8 -> 6).
  int sampling_factor =
      reusex::geometry::PlaneGraphOptions{}.surfel.sampling_factor;
  int confidence_threshold = 2;

  bool dry_run = false;

  // The pose loop (reusex-rcv.1.5, optimize_watch.hpp): `--edits e.yaml
  // --watch` re-solves on every save of the edits file. In memory unless
  // --apply; defaults mirror slam::DriftDiagnosticsOptions and
  // visualize::PreviewCloudOptions.
  bool watch = false;
  int watch_interval_ms = 500;
  int watch_cycles = -1; ///< hidden: stop after N re-solves (-1 = never)
  bool apply = false;    ///< write `transform` on every good solve
  /// No plane factors: the edits bend the no-edit full solve (reusex-rcv.1.21)
  bool fast = false;
  /// Image path stem: `<stem>.<panel>.png` per panel; empty = no images.
  fs::path render;
  int render_width = 1280;
  int render_height = 960;
  fs::path plan; ///< reference plan: crossings, wall support, overlay
  fs::path json; ///< the metrics block as JSON, rewritten every cycle
  /// The drift numbers depend on the sampling (reusex-74q), so the loop pins
  /// them and prints them with every block.
  int drift_frame_step = reusex::slam::DriftDiagnosticsOptions{}.frame_step;
  int drift_depth_stride = reusex::slam::DriftDiagnosticsOptions{}.depth_stride;
  int drift_segments = reusex::slam::DriftDiagnosticsOptions{}.segments;
  int suggestions = 3; ///< ICP suggestions printed as YAML
  /// An untouched segment moving more than this vs the no-edit full solve,
  /// and fitting worse there, counts as shear (reusex-sb2, rcv.1.22).
  double shear_flag_m = 0.05;
};

/// Every flag -> the optimizer's options (what `rux optimize` runs with).
reusex::geometry::PlaneGraphOptions
plane_graph_options(SubcommandOptimizeOptions const &opt);

/// The flags the in-process optimize stage reads, as its JSON.
std::string optimize_stage_parameters(SubcommandOptimizeOptions const &opt);

void setup_subcommand_optimize(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_optimize(SubcommandOptimizeOptions const &opt,
                            const RuxOptions &global_opt);

/// Flags that moved to `rux graph discover` on 2026-10-13, with whether they
/// take a value. `rux optimize` registers them hidden and refuses them.
const std::vector<std::pair<std::string, bool>> &optimize_moved_flags();

/// The refusal `rux optimize` prints for a moved @p flag: names the flag and
/// points at `rux graph discover`.
std::string optimize_moved_flag_message(const std::string &flag);
