// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

#include <reusex/segmentation/segment_rooms.hpp>

namespace fs = std::filesystem;

/// Collection of all options for room segmentation subcommand.
///
/// Defaults are derived from reusex::geometry::SegmentRoomsOptions so the CLI
/// and library never disagree on a parameter's default (docs/STANDARDS.md §4).
struct SubcommandSegRoomsOptions {
  /// Leiden resolution parameter (cluster granularity).
  float resolution = reusex::geometry::SegmentRoomsOptions{}.resolution;
  /// Leiden beta (refinement randomness).
  float beta = reusex::geometry::SegmentRoomsOptions{}.beta;
  /// Maximum Leiden iterations (finite bound, see library default).
  int max_iter = reusex::geometry::SegmentRoomsOptions{}.max_iter;

  float grid_size = reusex::geometry::SegmentRoomsOptions{}.grid_size;

  /// Distance-bounded k-NN label propagation to non-sampled points.
  float propagate_max_radius =
      reusex::geometry::SegmentRoomsOptions{}.propagate_max_radius;

  /// Minimum room size [m^2 of sampled plane surface].
  float min_room_area = reusex::geometry::SegmentRoomsOptions{}.min_room_area;

  /// Merge communities that see each other about as well as themselves.
  float merge_affinity = reusex::geometry::SegmentRoomsOptions{}.merge_affinity;

  std::string filter_expr; ///< Filter expression to limit processing
};

/**
 * @brief Setup the segment rooms subcommand in the CLI application.
 * @param app CLI application to add the subcommand to.
 */
/// The stage parameters (pipeline_log / run_stage JSON) these options mean.
std::string rooms_stage_parameters(SubcommandSegRoomsOptions const &opt);

void setup_subcommand_create_rooms(CLI::App &app,
                                   std::shared_ptr<RuxOptions> global_opt);

/**
 * @brief Run the segment rooms subcommand with given options.
 * @param opt Options for room segmentation.
 * @return Exit code (0 for success).
 */
int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt,
                                 const RuxOptions &global_opt);
