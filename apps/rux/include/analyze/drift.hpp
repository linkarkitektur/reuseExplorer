// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/slam/drift_diagnostics.hpp>

#include <CLI/CLI.hpp>

#include <memory>
#include <string>

/// Options for `rux analyze drift`.
struct SubcommandAnalyzeDriftOptions {
  std::string plan_path;   ///< reference plan for the crossings ("" = none)
  bool initial = false;    ///< use initial_transform instead of transform
  bool json = false;       ///< print the JSON report to stdout
  std::string output_path; ///< also write the JSON report here
  int top = 5;             ///< rows of the summary's worst-pairs table
  reusex::slam::DriftDiagnosticsOptions diag; ///< the library's knobs
};

/// The human-readable summary `rux analyze drift` prints.
std::string format_drift_summary(const reusex::slam::DriftReport &r, int top);

/// Register `rux analyze drift` under @p app.
void setup_subcommand_analyze_drift(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);

/// Run `rux analyze drift`; returns a RuxError status.
int run_subcommand_analyze_drift(SubcommandAnalyzeDriftOptions const &opt,
                                 const RuxOptions &global_opt);
