// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

#include <reusex/pipeline/space_parameters.hpp>

/// Options of `rux create spaces`: reusex::pipeline::SpaceStageOptions
/// (geometry::SpacesOptions + core::SpaceIdentityOptions). The flags, their
/// defaults and bounds come from pipeline::space_parameter_table(), so they
/// are the library's (docs/STANDARDS.md §4).
struct SubcommandCreateSpacesOptions {
  reusex::pipeline::SpaceStageOptions spaces{};
};

void setup_subcommand_create_spaces(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);

/// The stage parameters (pipeline_log / run_stage JSON) these options mean:
/// every key of pipeline::space_parameter_table().
std::string spaces_stage_parameters(SubcommandCreateSpacesOptions const &opt);

int run_subcommand_create_spaces(SubcommandCreateSpacesOptions const &opt,
                                 const RuxOptions &global_opt);
