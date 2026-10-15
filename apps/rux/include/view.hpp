// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"
#include <CLI/CLI.hpp>
#include <cstddef>
#include <memory>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandViewOptions {
  // View command now exclusively uses global project database
  std::string filter_expr; ///< Filter expression for limiting visualization

  // ── Ground-truth workbench (reusex-yf4.2) ──────────────────────────────
  /// Solid-model plan (JSON or YAML) to draw over the scene; switches the
  /// viewer to the workbench (the scene from visualize::populate_scene).
  fs::path plan;
  /// Reload the plan whenever the file changes.
  bool watch = false;
  /// Workbench layers, comma-separated (render's vocabulary).
  std::string layers = "cloud";
  /// Named geometry cloud of the workbench's point layers.
  std::string cloud_name = "cloud";
  /// Initial plan cut above the floor [m]; <= 0 = the library default.
  double cut_height = 0.0;
  /// Poll interval of --watch [ms].
  int watch_interval_ms = 300;
  /// Draw at most this many points per point layer (0 = all).
  std::size_t max_points = 3'000'000;
};

// Function declarations.
void setup_subcommand_view(CLI::App &app,
                           std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_view(SubcommandViewOptions const &opt,
                        const RuxOptions &global_opt);

/// `rux view --plan`: the ground-truth workbench (src/view/plan_view.cpp).
int run_view_plan(SubcommandViewOptions const &opt,
                  const RuxOptions &global_opt);
