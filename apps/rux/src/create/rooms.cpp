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
      app.add_subcommand("rooms", "Segment rooms (Leiden clustering of the "
                                  "plane graph, or the 2D band watershed)");

  sub->footer(R"(
DESCRIPTION:
  Performs room segmentation using the Leiden community detection algorithm
  based on spatial relationships and visual connections between planar surfaces.
  Groups planes into distinct room volumes using graph-based clustering.

  --method band instead finds rooms per storey in 2D: wall evidence from
  vertical points in a clutter-free height band (default 1.2-2.3 m above the
  storey floor), footprint from floor and ceiling points, exterior flooded
  from outside, rooms = watershed of the free-space distance transform, cut
  where the free space is narrower than 2 x --core-radius (doors, gaps
  between glass-partition mullions). Needs `rux create storeys`. Rooms
  narrower than about 1-1.5 m (WC stalls) join their neighbour.

EXAMPLES:
  rux create rooms                     # Default settings (resolution=1.0)
  rux create storeys && rux create rooms --method band
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

  sub->add_option("--merge-affinity", opt->merge_affinity,
                  "Merge rooms that see each other about as well as they see "
                  "themselves: two communities merge while the density of "
                  "sight lines between them is at least this fraction of the "
                  "internal density of the sparser one. Keeps an open office "
                  "split by desk screens one room; rooms joined by a door "
                  "score far lower. 0 disables the merge.")
      ->default_val(opt->merge_affinity)
      ->check(CLI::Range(0.0, 10.0));

  sub->add_option("--method", opt->method,
                  "Room method: 'leiden' (plane visibility graph) or 'band' "
                  "(2D free-space watershed from a clutter-free wall band; "
                  "needs `rux create storeys`).")
      ->default_val(opt->method)
      ->check(CLI::IsMember({std::string(reusex::geometry::kRoomsMethodLeiden),
                             std::string(reusex::geometry::kRoomsMethodBand)}));

  sub->add_option("--band-bottom", opt->band_bottom,
                  "band: wall evidence starts this far above the storey "
                  "floor [m] (above desks and screens).")
      ->default_val(opt->band_bottom)
      ->check(CLI::Range(0.0, 10.0));

  sub->add_option("--band-top", opt->band_top,
                  "band: wall evidence ends this far above the storey floor "
                  "[m] (clamped below the ceiling).")
      ->default_val(opt->band_top)
      ->check(CLI::Range(0.0, 10.0));

  sub->add_option("--band-cell", opt->band_cell_size,
                  "band: occupancy grid cell edge [m].")
      ->default_val(opt->band_cell_size)
      ->check(CLI::Range(0.01, 1.0));

  sub->add_option("--core-radius", opt->band_core_radius,
                  "band: rooms are cut where the free space is narrower than "
                  "twice this [m].")
      ->default_val(opt->band_core_radius)
      ->check(CLI::Range(0.05, 5.0));

  sub->add_option("--band-min-room-area", opt->band_min_room_area,
                  "band: regions smaller than this [m^2] are dropped.")
      ->default_val(opt->band_min_room_area)
      ->check(CLI::Range(0.0, 1000.0));

  sub->add_option("--band-wall-points", opt->band_wall_min_points,
                  "band: points a cell needs in the band to count as wall.")
      ->default_val(opt->band_wall_min_points)
      ->check(CLI::Range(1, 1000));

  sub->add_option("--footprint-close", opt->band_footprint_close,
                  "band: closing radius of the floor/ceiling footprint [m].")
      ->default_val(opt->band_footprint_close)
      ->check(CLI::Range(0.0, 5.0));

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
      .set("merge_affinity", opt.merge_affinity)
      .set_if(!opt.filter_expr.empty(), "filter", opt.filter_expr)
      .set("method", opt.method)
      .set("band_bottom", opt.band_bottom)
      .set("band_top", opt.band_top)
      .set("band_cell_size", opt.band_cell_size)
      .set("band_core_radius", opt.band_core_radius)
      .set("band_min_room_area", opt.band_min_room_area)
      .set("band_wall_min_points", opt.band_wall_min_points)
      .set("band_footprint_close", opt.band_footprint_close)
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
