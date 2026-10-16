// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/slam/pose_metrics.hpp>

#include <CLI/CLI.hpp>

#include <memory>
#include <string>

/// Options for `rux analyze poses` (reusex-rcv.1.6).
struct SubcommandAnalyzePosesOptions {
  std::string reference_path; ///< reference trajectory, TUM (required)
  /// Estimated trajectory, TUM; "" scores the project's own current poses
  /// (reusex::io::trajectory_from_project) instead.
  std::string estimate_path;
  bool initial = false;    ///< with no --estimate, score initial_transform
  bool json = false;       ///< print the JSON report to stdout
  std::string output_path; ///< also write the JSON report here
  /// --align: none | se3 | sim3 (see reusex::slam::PoseMetricsOptions::Align).
  /// Kept as a string at the CLI boundary, like --solver in create mesh.hpp
  /// (STANDARDS §4); translated to the enum in run_subcommand_analyze_poses.
  std::string align = "none";
  reusex::slam::PoseMetricsOptions metrics;
};

/// The human-readable summary `rux analyze poses` prints.
std::string format_pose_metrics_summary(const reusex::slam::PoseMetrics &m);

void setup_subcommand_analyze_poses(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_analyze_poses(SubcommandAnalyzePosesOptions const &opt,
                                 const RuxOptions &global_opt);
