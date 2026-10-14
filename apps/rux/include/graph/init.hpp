// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/slam/pose_graph.hpp>

#include <CLI/CLI.hpp>
#include <memory>

struct SubcommandGraphInitOptions {
  bool force = reusex::slam::InitPoseGraphOptions{}.force;
};

void setup_subcommand_graph_init(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_graph_init(const SubcommandGraphInitOptions &opt,
                              const RuxOptions &global_opt);
