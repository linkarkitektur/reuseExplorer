// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

#include <reusex/pipeline/wall_parameters.hpp>

/// Options of `rux create walls`: reusex::pipeline::WallStageOptions
/// (geometry::WallOptions + core::WallIdentityOptions). The flags, their
/// defaults and bounds come from pipeline::wall_parameter_table(), so they
/// are the library's (docs/STANDARDS.md §4). `use_normal_sign` is not an
/// option: the stage reads it from the clouds' provenance (reusex-0se).
struct SubcommandCreateWallsOptions {
  reusex::pipeline::WallStageOptions walls{};
};

void setup_subcommand_create_walls(CLI::App &app,
                                   std::shared_ptr<RuxOptions> global_opt);

/// The stage parameters (pipeline_log / run_stage JSON) these options mean:
/// every key of pipeline::wall_parameter_table().
std::string walls_stage_parameters(SubcommandCreateWallsOptions const &opt);

int run_subcommand_create_walls(SubcommandCreateWallsOptions const &opt,
                                const RuxOptions &global_opt);
