// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

#include <reusex/pipeline/solid_parameters.hpp>

/// Options of `rux create solids`: reusex::pipeline::SolidStageOptions
/// (geometry::SolidsOptions). The flags, their
/// defaults and bounds come from pipeline::solid_parameter_table(), so they
/// are the library's (docs/STANDARDS.md §4).
struct SubcommandCreateSolidsOptions {
  reusex::pipeline::SolidStageOptions solids{};
};

void setup_subcommand_create_solids(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);

/// The stage parameters (pipeline_log / run_stage JSON) these options mean:
/// every key of pipeline::solid_parameter_table().
std::string solids_stage_parameters(SubcommandCreateSolidsOptions const &opt);

int run_subcommand_create_solids(SubcommandCreateSolidsOptions const &opt,
                                 const RuxOptions &global_opt);
