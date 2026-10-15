// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

#include <reusex/segmentation/classify_planes.hpp>

/// Options of `rux create classify`: reusex::geometry::ClassifyPlanesOptions
/// plus the class -> role map file, so the CLI defaults are the library's
/// (docs/STANDARDS.md §4). `oriented_normals` is not an option: the stage
/// reads it from the clouds' provenance (reusex-0se).
struct SubcommandCreateClassifyOptions {
  reusex::geometry::ClassifyPlanesOptions classify{};
  /// core/class_role_map.hpp JSON file; "" = the built-in map.
  std::string role_map;
};

void setup_subcommand_create_classify(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt);

/// The stage parameters (pipeline_log / run_stage JSON) these options mean.
/// Keys are the ClassifyPlanesOptions field names, plus "role_map".
std::string
classify_stage_parameters(SubcommandCreateClassifyOptions const &opt);

int run_subcommand_create_classify(SubcommandCreateClassifyOptions const &opt,
                                   const RuxOptions &global_opt);
