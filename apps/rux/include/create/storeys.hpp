// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

#include <reusex/core/storey_identity.hpp>
#include <reusex/segmentation/level.hpp>

/// Options of `rux create storeys`: reusex::geometry::LevelOptions plus the
/// identity tolerance, so the CLI defaults are the library's
/// (docs/STANDARDS.md §4).
struct SubcommandCreateStoreysOptions {
  reusex::geometry::LevelOptions level{};
  reusex::core::StoreyIdentityOptions identity{};
};

void setup_subcommand_create_storeys(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt);

/// The stage parameters (pipeline_log / run_stage JSON) these options mean.
/// Keys are the LevelOptions field names.
std::string storeys_stage_parameters(SubcommandCreateStoreysOptions const &opt);

int run_subcommand_create_storeys(SubcommandCreateStoreysOptions const &opt,
                                  const RuxOptions &global_opt);
