// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/rooms.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>

#include <CLI/CLI.hpp>

#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

/**
 * @brief Setup CLI options for the room segmentation subcommand.
 *
 * Configures command-line arguments including paths, Leiden parameters,
 * and clustering options for room segmentation.
 *
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_create_rooms(CLI::App &app,
                                   std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandSegRoomsOptions>();
  auto *sub =
      app.add_subcommand("rooms", "Segment rooms using Leiden clustering");

  sub->footer(R"(
DESCRIPTION:
  Performs room segmentation using the Leiden community detection algorithm
  based on spatial relationships and visual connections between planar surfaces.
  Groups planes into distinct room volumes using graph-based clustering.

EXAMPLES:
  rux create rooms                     # Default settings (resolution=1.0)
  rux create rooms -r 1.5              # More clusters (higher resolution)
  rux create rooms -r 0.5 -g 0.1       # Fewer clusters, finer grid
  rux create rooms -f 'planes >= 5'    # Filter to specific planes

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct point clouds
  3. rux create planes                 # Segment planar surfaces
  4. rux create rooms                  # Segment rooms from planes
  5. rux create mesh                   # Generate 3D mesh

NOTES:
  - Requires 'cloud', 'planes', 'plane_centroids', 'plane_normals' in project
  - Resolution parameter controls cluster granularity (higher = more rooms)
  - Grid size affects spatial discretization (default: 0.2m)
  - Filter syntax supports boolean expressions: 'planes in [1,2] || rooms == 5'
  - Output saved as 'rooms' label cloud in project database
)");

  sub->add_option("-r, --resolution", opt->resolution,
                  "Leiden resolution parameter. "
                  "Higher values lead to more clusters. "
                  "Common values: 0.5, 1.0, 1.5, 2.0")
      ->default_val(opt->resolution)
      ->check(CLI::Range(0.0, 10.0));

  sub->add_option("-b, --beta", opt->beta,
                  "Leiden beta parameter for refinement phase randomness. "
                  "Lower values = more deterministic.")
      ->default_val(opt->beta)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-m, --max-iter", opt->max_iter,
                  "Maximum number of Leiden iterations (finite bound). "
                  "Negative value = iterate until convergence.")
      ->default_val(opt->max_iter)
      ->check(CLI::Range(-1, 1000));

  sub->add_option("-g, --grid-size", opt->grid_size,
                  "Grid size for spatial discretization.")
      ->default_val(opt->grid_size)
      ->check(CLI::Range(0.01, 10.0));

  sub->add_option("--propagate-radius", opt->propagate_max_radius,
                  "Max search radius (meters) for propagating room labels to "
                  "non-sampled points. Points with no room label within this "
                  "radius stay unlabeled.")
      ->default_val(opt->propagate_max_radius)
      ->check(CLI::Range(0.0, 10.0));

  sub->add_option("--min-room-area", opt->min_room_area,
                  "Minimum room size, as the plane surface [m^2] its sample "
                  "points cover. Smaller communities (points that see little "
                  "or nothing) join the room they see most instead of "
                  "becoming rooms. 0 keeps every community.")
      ->default_val(opt->min_room_area)
      ->check(CLI::Range(0.0, 1000.0));

  sub->add_option(
         "-f, --filter", opt->filter_expr,
         "Filter expression to limit processing to specific labeled points.\n"
         "Syntax: <cloud_name> <op> <value(s)>\n"
         "Examples:\n"
         "  -f 'planes in [1, 2, 5]'        # Filter to labels 1, 2, 5 from "
         "planes cloud\n"
         "  -f 'rooms == 3'                 # Only process room 3\n"
         "  -f 'planes in [1,2] || rooms == 5'  # Combine multiple clouds\n"
         "  -f 'planes >= 10 && planes <= 20'   # Range filter")
      ->default_val("");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling seg-rooms subcommand");
    if (rux::capture_stage_params("rooms", rooms_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_segment_rooms(*opt, *global_opt));
  });
}

/**
 * @brief Execute room segmentation using Leiden community detection.
 *
 * Validates the CLI's prerequisites, then hands the run to
 * reusex::pipeline::run_stage, which owns loading, segmenting, saving and the
 * pipeline_log record (#284).
 *
 * @param opt Options containing project path and Leiden parameters.
 * @return Exit code (RuxError::SUCCESS on success).
 */
std::string rooms_stage_parameters(SubcommandSegRoomsOptions const &opt) {
  return rux::StageParams()
      .set("grid_size", opt.grid_size)
      .set("resolution", opt.resolution)
      .set("beta", opt.beta)
      .set("max_iter", opt.max_iter)
      .set("propagate_max_radius", opt.propagate_max_radius)
      .set("min_room_area", opt.min_room_area)
      .set_if(!opt.filter_expr.empty(), "filter", opt.filter_expr)
      .dump();
}

int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt,
                                 const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Segmenting rooms in project: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Pre-flight validation: check for planes prerequisites
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::rooms);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::rooms;
    ctx.parameters = rooms_stage_parameters(opt);

    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Room segmentation complete: {}", result.message);
    return rux::exit_code_for(result);

  } catch (const std::exception &e) {
    spdlog::error("Room segmentation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
