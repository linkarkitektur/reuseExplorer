// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/reconstruction/solid_metrics.hpp>

#include <CLI/CLI.hpp>

#include <memory>
#include <string>

/// Options for `rux analyze solid`.
struct SubcommandAnalyzeSolidOptions {
  std::string reference_path; ///< reference plan JSON (required)
  /// Detected model JSON to score instead of the project's own model ("" =
  /// io::load_solid_model on the project).
  std::string model_path;
  std::string cloud_name = "cloud"; ///< scan cloud for the fit metrics
  std::string mesh_name = "mesh";   ///< solid mesh for the fit metrics
  /// --cloud or --mesh was given explicitly: fit even when --model is used.
  bool fit_requested = false;
  bool json = false;       ///< print the JSON report to stdout
  std::string output_path; ///< also write the JSON report here
  reusex::geometry::SolidMetricsOptions metrics; ///< matching tolerances
};

/// The human-readable summary `rux analyze solid` prints (one metric per
/// line, n/a for a missing one); also printed by `rux render --watch`.
std::string format_solid_summary(const reusex::geometry::SolidModelReport &r);

/// Register `rux analyze solid` under @p app.
void setup_subcommand_analyze_solid(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);

/// Run `rux analyze solid`; returns a RuxError status.
int run_subcommand_analyze_solid(SubcommandAnalyzeSolidOptions const &opt,
                                 const RuxOptions &global_opt);
