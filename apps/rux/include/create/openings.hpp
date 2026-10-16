// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

#include <reusex/pipeline/opening_parameters.hpp>

/// Options of `rux create openings`: reusex::pipeline::OpeningStageOptions
/// (geometry::OpeningOptions + core::OpeningIdentityOptions). The flags,
/// their defaults and bounds come from pipeline::opening_parameter_table(),
/// so they are the library's (docs/STANDARDS.md §4).
struct SubcommandCreateOpeningsOptions {
  reusex::pipeline::OpeningStageOptions openings{};
};

void setup_subcommand_create_openings(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt);

/// The stage parameters (pipeline_log / run_stage JSON) these options mean:
/// every key of pipeline::opening_parameter_table().
std::string
openings_stage_parameters(SubcommandCreateOpeningsOptions const &opt);

int run_subcommand_create_openings(SubcommandCreateOpeningsOptions const &opt,
                                   const RuxOptions &global_opt);
