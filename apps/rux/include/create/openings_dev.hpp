// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

#include <reusex/reconstruction/openings.hpp>

/// Options of the hidden dev driver `rux create openings-dev` (reusex-z1v.7):
/// runs the opening sub-steps on a project's walls, read-only, and writes
/// the storeys, walls and the openings found as a solid-model JSON for
/// `rux analyze solid --model` and `rux render --plan`. Replaced by the
/// `openings` stage (W5.3, reusex-z1v.7.3).
struct SubcommandCreateOpeningsDevOptions {
  reusex::geometry::OpeningOptions openings{};
  std::filesystem::path output;     ///< solid-model JSON; empty = none
  std::filesystem::path raster_dir; ///< one PNG per wall face raster
};

void setup_subcommand_create_openings_dev(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt);

int run_subcommand_create_openings_dev(
    SubcommandCreateOpeningsDevOptions const &opt,
    const RuxOptions &global_opt);
