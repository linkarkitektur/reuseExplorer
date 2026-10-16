// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/mesh.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "filter_utils.hpp"
#include "processing_observer.hpp"
#include "stage_prerequisites.hpp"
#include <reusex/pipeline/mesh_parameters.hpp>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <fmt/format.h>
#include <fmt/std.h>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/reusex.hpp>
#include <reusex/reconstruction/mesh.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <range/v3/to_container.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>

#include <algorithm>
#include <filesystem>
#include <latch>
#include <mutex>
#include <queue>
#include <thread>

namespace fs = std::filesystem;

void setup_subcommand_create_mesh(CLI::App &app,
                                  std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandMeshOptions>();
  auto *sub =
      app.add_subcommand("mesh", "Generate watertight mesh from planes");

  sub->footer(R"(
DESCRIPTION:
  Generates a watertight 3D polygon mesh by computing best-fit volumes
  from segmented planar surfaces. Uses mixed-integer programming (MIP)
  to solve for optimal plane pairings and creates enclosed room geometries
  suitable for CAD export and building information modeling.

EXAMPLES:
  rux create mesh                      # Default thresholds
  rux create mesh -f 'rooms == 2'      # Mesh only room 2
  rux create mesh --output-name room1  # Custom mesh name

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct point clouds
  3. rux create planes                 # Segment planar surfaces
  4. rux create rooms                  # Segment rooms (optional)
  5. rux create mesh                   # Generate watertight mesh
  6. rux export rhino mesh.3dm         # Export to Rhino

NOTES:
  - Requires 'cloud', 'normals', 'planes', 'rooms', plane centroids/normals
  - Filter syntax: 'planes in [1,2,5]' or 'rooms == 3'
  - Output saved to ProjectDB with name specified by --output-name
  - Uses HiGHS MIP solver (CPU-based optimization)
)");

  sub->add_option("--output-name", opt->output_mesh_name,
                  "Name for the output mesh in ProjectDB")
      ->default_val(opt->output_mesh_name);

  sub->add_option("-t, --threshold", opt->search_threshold,
                  "Search distance threshold for finding nearby planes.")
      ->default_val(opt->search_threshold)
      ->check(CLI::Range(0.01, 10.0));

  sub->add_option("-o, --offset", opt->new_plane_offset,
                  "Offset distance for new plane creation.")
      ->default_val(opt->new_plane_offset)
      ->check(CLI::Range(0.01, 1.0));

  sub->add_option("--time-limit", opt->time_limit_seconds,
                  "MIP solver wall-clock time limit in seconds. Guards against "
                  "the solver hanging on complex buildings (issue #212).")
      ->default_val(opt->time_limit_seconds)
      ->check(CLI::Range(1.0, 36000.0));

  sub->add_option("--alpha", opt->alpha,
                  "Weight on the wall/complexity term of the MIP objective "
                  "(issue #212).")
      ->default_val(opt->alpha)
      ->check(CLI::Range(0.0, 100.0));

  sub->add_option("--max-cells", opt->max_cells,
                  "Maximum number of cell-complex cells before failing fast "
                  "with a diagnostic (issue #213).")
      ->default_val(opt->max_cells)
      ->check(CLI::Range(size_t{1}, size_t{1000000}));

  sub->add_flag("!--no-sectioned", opt->sectioned,
                "Disable the per-section (per-storey) MIP solve and force the "
                "monolithic one-shot solve, even on large multi-room buildings "
                "(issue #226). Sectioned solve is on by default.");

  sub->add_option("--sectioned-threshold", opt->sectioned_threshold,
                  "Engage the sectioned solve only when the cell complex has "
                  "at least this many cells (issue #226).")
      ->default_val(opt->sectioned_threshold)
      ->check(CLI::Range(size_t{1}, size_t{1000000}));

  sub->add_flag("!--no-structural-filter", opt->structural.enabled,
                "Build the cell complex from every plane instead of only wall "
                "lines and storey floors/ceilings (reusex-8l3). The filter is "
                "on by default.");

  sub->add_option("--structural-min-area", opt->structural.min_wall_area_m2,
                  "Surface [m^2] a wall line needs, counting the opposite "
                  "face of the same wall, to become an arrangement line.")
      ->default_val(opt->structural.min_wall_area_m2)
      ->check(CLI::Range(0.0, 100.0));

  sub->add_option("--structural-collinear", opt->structural.collinear_offset_m,
                  "Same-facing wall planes closer than this [m] become one "
                  "arrangement line.")
      ->default_val(opt->structural.collinear_offset_m)
      ->check(CLI::Range(0.0, 2.0));

  sub->add_option("--structural-clearance",
                  opt->structural.min_storey_clearance_m,
                  "A horizontal plane less than this [m] above a larger floor "
                  "under it is furniture, not a storey section.")
      ->default_val(opt->structural.min_storey_clearance_m)
      ->check(CLI::Range(0.0, 10.0));

  sub->add_option("--structural-overlap", opt->structural.min_footprint_overlap,
                  "Fraction of a horizontal plane's footprint over the floor "
                  "below needed for the clearance rule to apply.")
      ->default_val(opt->structural.min_footprint_overlap)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_flag("--plane-classes", opt->use_plane_classes,
                "Leave the planes `rux create classify` called furniture or "
                "clutter out of the cell complex (reusex-z1v.4.3). Off by "
                "default; no effect before `rux create classify` has run.");

  sub->add_flag("--storey-sections", opt->use_storey_sections,
                "Use only the storey floors and ceilings of the storeys table "
                "as section planes of the cell complex, instead of every "
                "horizontal plane (reusex-z1v.3.4). Off by default: it also "
                "drops split levels and attics the storeys table does not "
                "describe. No effect before `rux create storeys` has run.");

  sub->add_flag("!--no-storey-local", opt->storey_local,
                "Extend every wall through every section instead of giving "
                "each section only the walls whose height reaches into it "
                "(reusex-sba). Storey-local is on by default.");

  sub->add_option("--solver", opt->solver,
                  "MIP backend: 'auto' (primary backend with HiGHS CPU "
                  "fallback on GPU error/OOM), 'cuopt' (force GPU), or 'highs' "
                  "(force CPU) (issue #226).")
      ->default_val(opt->solver)
      ->check(CLI::IsMember({"auto", "cuopt", "highs"}));

  sub->add_option(
         "--cuopt-max-variables", opt->cuopt_max_variables,
         "With --solver auto, run a (sub)problem over this many MIP "
         "variables on HiGHS instead of cuOpt, pre-emptively (reusex-81y: "
         "cuOpt SIGSEGVs on sections around ~3.3M variables). Ignored for "
         "--solver highs; an explicit --solver cuopt past this size is "
         "still honored (just logged loudly).")
      ->default_val(opt->cuopt_max_variables)
      ->check(CLI::Range(size_t{1000}, size_t{10000000}));

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
    spdlog::trace("calling run_subcommand_mesh");
    if (rux::capture_stage_params("mesh", mesh_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_mesh(*opt, *global_opt));
  });
};

std::string mesh_stage_parameters(SubcommandMeshOptions const &opt) {
  // `rux create mesh` still drives the solver itself (it predates
  // pipeline::run_stage); these are its flags under the keys run_stage's mesh
  // stage reads, which is what the Qt client's runs carry.
  return rux::StageParams()
      .set("solver", opt.solver)
      .set("time_limit_seconds", opt.time_limit_seconds)
      .set("output_name", opt.output_mesh_name)
      .set("search_threshold", opt.search_threshold)
      .set("new_plane_offset", opt.new_plane_offset)
      .set("alpha", opt.alpha)
      .set("max_cells", opt.max_cells)
      .set("sectioned", opt.sectioned)
      .set("sectioned_threshold", opt.sectioned_threshold)
      .set("cuopt_max_variables", opt.cuopt_max_variables)
      .set("structural_filter", opt.structural.enabled)
      .set("structural_min_wall_area", opt.structural.min_wall_area_m2)
      .set("structural_collinear_offset", opt.structural.collinear_offset_m)
      .set("structural_storey_clearance", opt.structural.min_storey_clearance_m)
      .set("structural_footprint_overlap", opt.structural.min_footprint_overlap)
      .set("plane_classes", opt.use_plane_classes)
      .set("storey_sections", opt.use_storey_sections)
      .set("storey_local", opt.storey_local)
      .set_if(!opt.filter_expr.empty(), "filter", opt.filter_expr)
      .dump();
}

int run_subcommand_mesh(SubcommandMeshOptions const &opt,
                        const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Generating mesh from project: {}", project_path.string());

  // rux::VizualizationObserver
  auto &observer = rux::get_processing_observer();
  observer.viewer_request_viewports(4);

  try {
    reusex::ProjectDB db(project_path);

    // Pre-flight validation: comprehensive check for all prerequisites
    // This resolves the TODO comment from lines 101-107
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::mesh);
        rc != RuxError::SUCCESS)
      return rc;

    // The same parameters, under the same keys, the GUI's mesh stage reads —
    // so this row's "Kopiér som rux-kommando" reproduces the run.
    const std::string params = mesh_stage_parameters(opt);
    int logId = db.log_pipeline_start("mesh_generation", params);

    spdlog::trace("Loading point clouds from ProjectDB");
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto normals = db.point_cloud_normal("normals");
    auto rooms = db.point_cloud_label("rooms");
    auto plane_labels = db.point_cloud_label("planes");
    auto plane_centroids = db.point_cloud_xyz("plane_centroids");
    auto plane_normals = db.point_cloud_normal("plane_normals");

    spdlog::trace("Processing plane data");
    auto [planes, centroids, inliers] =
        reusex::io::getPlanes(plane_labels, plane_normals, plane_centroids);

    // Options from the shared parameter path (pipeline/mesh_parameters.hpp),
    // exactly as the GUI's run of the mesh stage reads them.
    reusex::geometry::MeshOptions options;
    try {
      const auto stage_opts =
          reusex::pipeline::mesh_options_from_parameters(params);
      options = reusex::pipeline::mesh_options_for(db, stage_opts, inliers);
    } catch (const std::invalid_argument &e) {
      spdlog::error("{}", e.what());
      db.log_pipeline_end(logId, false, e.what());
      return RuxError::INVALID_ARGUMENT;
    }

    // Evaluate filter if provided and set in options
    if (!opt.filter_expr.empty()) {
      try {
        options.filter =
            rux::filters::evaluate_filter(opt.filter_expr, db, cloud->size());
      } catch (const std::exception &e) {
        spdlog::error("Filter evaluation failed: {}", e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    spdlog::trace("Generating mesh geometry");
    pcl::PolygonMeshPtr mesh = reusex::geometry::mesh(
        cloud, normals, planes, centroids, inliers, rooms, options);

    const size_t vertex_count =
        static_cast<size_t>(mesh->cloud.width) * mesh->cloud.height;
    const size_t face_count = mesh->polygons.size();
    if (vertex_count == 0 || face_count == 0) {
      spdlog::error("Mesh generation produced an empty mesh ({} vertices, {} "
                    "faces); not saving.",
                    vertex_count, face_count);
      spdlog::info(
          "Resolution: the MIP solver reported optimal but selected zero "
          "cells. This usually means the filter excluded too many planes or "
          "the cell complex had no viable interior. Try widening --filter, "
          "lowering --threshold, or rerunning 'rux create planes/rooms'.");
      db.log_pipeline_end(logId, false, "Empty mesh, save skipped");
      return RuxError::GENERIC;
    }

    spdlog::trace("Saving mesh to ProjectDB");
    db.save_mesh(opt.output_mesh_name, *mesh, "mesh_generation");

    spdlog::info("Mesh '{}' saved to ProjectDB ({} vertices, {} faces)",
                 opt.output_mesh_name, vertex_count, face_count);

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Mesh generation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
