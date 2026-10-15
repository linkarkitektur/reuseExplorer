// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <reusex/core/solid_model.hpp>

#include <CLI/CLI.hpp>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace fs = std::filesystem;

/// Options for the render subcommand.
///
/// Every default here is read from `reusex::visualize::RenderOptions` at setup
/// time rather than written out again (STANDARDS §4); the fields below only
/// hold the string forms the CLI parses.
struct SubcommandRenderOptions {
  fs::path output = "render.png";
  /// `top`, `plan[:height]`, `front`, `orbit[:N]` or `frame:<node_id>`.
  std::string view;
  /// Comma-separated layer names.
  std::string layers;
  /// `WxH`.
  std::string size;
  std::string cloud_name;
  std::string mesh_name;
  double point_size = 0.0;
  double orbit_elevation_deg = 0.0;
  /// Cut height in metres above the floor; <= 0 means "let the library derive
  /// it", which is how `--view plan` with no suffix behaves (#306).
  double cut_height = 0.0;
  /// Solid-model plan (JSON or YAML, by extension) to draw over the layers;
  /// empty = none.
  fs::path plan;
  /// Leave the plan overlay's space-name and wall-guid labels out.
  bool no_plan_labels = false;
  /// View yaw in degrees; unset = the project's `view.yaw_deg` setting.
  std::optional<double> yaw_deg;
  /// `--section` specs (`x=<m>`, `y=<m>`, `<x0>,<y0>:<x1>,<y1>`); each one a
  /// section view, or the sections of `--qa`.
  std::vector<std::string> sections;
  double section_thickness = 0.0;
  /// `auto`, `solid`, `overlay` or `edges`.
  std::string mesh_style;
  double deviation_within_m = 0.0;
  double deviation_uncovered_m = 0.0;
  bool no_legend = false;
  /// Frame on the [q, 1-q] quantiles of the cloud; < 0 = the default (0 for
  /// a single view, QaSheetOptions::frame_quantile for --qa).
  double frame_quantile = -1.0;
  /// Render the mesh-QA contact sheet instead of one view.
  bool qa = false;

  // ── --watch (reusex-yf4.2) ────────────────────────────────────────────
  /// Re-render (and re-score the --plan) whenever the plan file changes,
  /// until Ctrl-C / SIGTERM.
  bool watch = false;
  /// Also re-render when the project file changes (another rux writes it).
  bool watch_project = false;
  /// Poll interval of the watch [ms].
  int watch_interval_ms = 500;
  /// Stop after this many re-renders; < 0 = never, 0 = only the initial
  /// render and scores (a `--once` for tests and scripts).
  int watch_cycles = -1;
};

// Function declarations
void setup_subcommand_render(CLI::App &app,
                             std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_render(SubcommandRenderOptions const &opt,
                          const RuxOptions &global_opt);

/// Render what @p opt asks for with @p plan as the overlay (opt.plan is not
/// read: the caller has parsed it, or --watch holds its last good version).
/// @return every image written, in order.
/// @throws std::exception on any render or argument error.
std::vector<fs::path>
render_to_files(SubcommandRenderOptions const &opt,
                const RuxOptions &global_opt,
                const std::optional<reusex::core::SolidModel> &plan);
