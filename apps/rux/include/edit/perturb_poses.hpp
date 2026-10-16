// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>
#include <string>

/// Options for `rux edit perturb-poses` (issue #338). Defaults mirror
/// reusex::geometry::PoseDriftOptions exactly (docs/STANDARDS.md §4) — the CLI
/// never redefines a library default.
struct SubcommandEditPerturbPosesOptions {
  unsigned seed = 42;
  double drift_scale = 1.0;
  double target_drift_ratio = 0.20;
  int min_frame_gap = 50;
  double rot_bias_gain = 0.020;
  double rot_walk_gain = 0.010;
  double trans_bias_gain = 0.020;
  double trans_walk_gain = 0.010;
  double bias_correlation_length = 5.0;
  /// "full" or "yaw" (reusex::geometry::DriftMode).
  std::string mode = "full";
  bool dry_run = false;
  /// Required acknowledgement that the target project is a disposable copy.
  bool yes = false;
};

void setup_subcommand_edit_perturb_poses(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_edit_perturb_poses(
    SubcommandEditPerturbPosesOptions const &opt, const RuxOptions &global_opt);
