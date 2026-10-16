// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>

#include <filesystem>
#include <memory>

/// Options for `rux export trajectory` (reusex-rcv.1.6).
struct SubcommandExportTrajectoryOptions {
  std::filesystem::path output_path; ///< TUM output file (-o, required)
  /// Export `sensor_frames.initial_transform` (the pose `rux optimize` seeds
  /// from) instead of the current stored pose.
  bool initial = false;
};

void setup_subcommand_export_trajectory(CLI::App &parent,
                                        std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_export_trajectory(
    SubcommandExportTrajectoryOptions const &opt, const RuxOptions &global_opt);
