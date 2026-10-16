// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

#include <reusex/reconstruction/walls.hpp>

/// Options of the hidden dev driver `rux create walls-dev` (reusex-z1v.5):
/// runs the wall sub-steps on a project, read-only, and writes what they
/// found as a solid-model JSON for `rux analyze solid --model` and
/// `rux render --plan`. Replaced by the `walls` stage (W3.5, reusex-z1v.5.5).
struct SubcommandCreateWallsDevOptions {
  reusex::geometry::WallOptions walls{};
  std::filesystem::path output;    ///< solid-model JSON; empty = none
  std::filesystem::path reference; ///< reference plan to compare families to
  bool faces_only = false;         ///< write the W3.2 faces, not walls
  bool no_graph = false;           ///< write the W3.3 walls, no joins
};

void setup_subcommand_create_walls_dev(CLI::App &app,
                                       std::shared_ptr<RuxOptions> global_opt);

int run_subcommand_create_walls_dev(SubcommandCreateWallsDevOptions const &opt,
                                    const RuxOptions &global_opt);
