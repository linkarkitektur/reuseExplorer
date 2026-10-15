// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>

#include <filesystem>
#include <memory>

/// Options for `rux edit plan` (reusex-z1v.2.6).
struct SubcommandEditPlanOptions {
  /// Solid-model plan (JSON or YAML, by extension) to import ("" = none).
  std::filesystem::path import_path;
  /// Where to write the solid-model plan (JSON or YAML, by extension);
  /// "" = none.
  std::filesystem::path export_path;
  /// Validate --import without opening the project; --export then writes
  /// the imported model instead of the project's.
  bool dry_run = false;
};

void setup_subcommand_edit_plan(CLI::App &app,
                                std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_edit_plan(SubcommandEditPlanOptions const &opt,
                             const RuxOptions &global_opt);
