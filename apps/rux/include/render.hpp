// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>
#include <string>

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
  /// Solid-model plan JSON to draw over the layers; empty = none.
  fs::path plan;
  /// Leave the plan overlay's space-name and wall-guid labels out.
  bool no_plan_labels = false;
};

// Function declarations
void setup_subcommand_render(CLI::App &app,
                             std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_render(SubcommandRenderOptions const &opt,
                          const RuxOptions &global_opt);
