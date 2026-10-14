// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>
#include <optional>
#include <string>

struct SubcommandGraphPruneOptions {
  std::optional<double> residual_above;
  std::optional<double> deviation_above;
  std::optional<double> deviation_deg_above;
  std::string type;   ///< "" = any
  std::string source; ///< "" = any
  std::string pair;   ///< "A,B" or ""
  bool include_confirmed = false;
  bool dry_run = false;
};

void setup_subcommand_graph_prune(CLI::App &parent,
                                  std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_graph_prune(const SubcommandGraphPruneOptions &opt,
                               const RuxOptions &global_opt);
