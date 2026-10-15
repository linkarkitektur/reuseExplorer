// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/storeys.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>

#include <spdlog/spdlog.h>

void setup_subcommand_create_storeys(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateStoreysOptions>();
  auto &l = opt->level;
  auto *sub = app.add_subcommand(
      "storeys", "Detect storeys and the measured vertical (level frame)");

  sub->footer(R"(
DESCRIPTION:
  Fits the floor and ceiling of each storey to the horizontal planes of the
  plane segmentation (iterated least squares), measures the vertical as the
  bisector of their normals, and finds storeys as peaks of a height
  histogram. Writes one row per storey to the 'storeys' table: floor and
  ceiling height in the level frame (storey-0 floor at z = 0) and the
  world -> level transform. The scan itself is never rewritten.

EXAMPLES:
  rux create storeys                        # Default settings
  rux create storeys --min-storey-height 2.2
  rux get storeys                           # Show the result

WORKFLOW:
  1. rux create clouds
  2. rux create planes
  3. rux create storeys

NOTES:
  - Requires 'cloud', 'planes', 'plane_centroids', 'plane_normals'
  - Always writes at least one storey; a single-storey or ceiling-less scan
    is one row whose metadata says why
  - A re-detected storey keeps the GUID of the stored storey whose floor
    it matches (--guid-match), even when its idx changes; locked rows are
    kept on a re-run (warned about when they no longer fit the new frame)
)");

  sub->get_formatter()->column_width(40);

  sub->add_option("--horizontal-cos", l.horizontal_cos,
                  "A plane is horizontal when |n.z| exceeds this cosine")
      ->default_val(l.horizontal_cos)
      ->check(CLI::Range(0.5, 1.0));
  sub->add_option("--inlier", l.inlier_m,
                  "Inlier band of the iterated floor/ceiling fit [m]")
      ->default_val(l.inlier_m)
      ->check(CLI::Range(0.001, 0.5));
  sub->add_option("--max-fit-iterations", l.max_fit_iterations,
                  "Refits per floor/ceiling surface")
      ->default_val(l.max_fit_iterations)
      ->check(CLI::Range(1, 100));
  sub->add_option("--hist-bin", l.hist_bin_m, "Height histogram bin [m]")
      ->default_val(l.hist_bin_m)
      ->check(CLI::Range(0.005, 1.0));
  sub->add_option("--peak-min-share", l.peak_min_share,
                  "A histogram bin is a level when it holds at least this "
                  "share of the fullest bin")
      ->default_val(l.peak_min_share)
      ->check(CLI::Range(0.01, 1.0));
  sub->add_option("--slab-merge", l.slab_merge_m,
                  "Surfaces closer than this along the vertical are one "
                  "level (a slab) [m]")
      ->default_val(l.slab_merge_m)
      ->check(CLI::Range(0.0, 3.0));
  sub->add_option("--min-storey-height", l.min_storey_height_m,
                  "Consecutive levels closer than this are not a storey [m]")
      ->default_val(l.min_storey_height_m)
      ->check(CLI::Range(0.5, 20.0));
  sub->add_option("--default-height", l.default_height_m,
                  "Storey height used when no ceiling level exists [m]")
      ->default_val(l.default_height_m)
      ->check(CLI::Range(0.5, 20.0));

  sub->add_option("--guid-match", opt->identity.match_m,
                  "A re-detected storey keeps the GUID of the stored storey "
                  "whose floor is within this distance [m]")
      ->default_val(opt->identity.match_m)
      ->check(CLI::Range(0.0, 5.0));

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling create storeys subcommand");
    if (rux::capture_stage_params("storeys", storeys_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_create_storeys(*opt, *global_opt));
  });
}

std::string
storeys_stage_parameters(SubcommandCreateStoreysOptions const &opt) {
  const auto &l = opt.level;
  return rux::StageParams()
      .set("horizontal_cos", l.horizontal_cos)
      .set("inlier_m", l.inlier_m)
      .set("max_fit_iterations", l.max_fit_iterations)
      .set("hist_bin_m", l.hist_bin_m)
      .set("peak_min_share", l.peak_min_share)
      .set("slab_merge_m", l.slab_merge_m)
      .set("min_storey_height_m", l.min_storey_height_m)
      .set("default_height_m", l.default_height_m)
      .set("guid_match_m", opt.identity.match_m)
      .dump();
}

int run_subcommand_create_storeys(SubcommandCreateStoreysOptions const &opt,
                                  const RuxOptions &global_opt) {
  const auto &project_path = global_opt.project_db;
  spdlog::info("Detecting storeys in project: {}", project_path.string());
  try {
    reusex::ProjectDB db(project_path);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::storeys);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::storeys;
    ctx.parameters = storeys_stage_parameters(opt);

    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Storey detection complete: {}", result.message);
    return rux::exit_code_for(result);
  } catch (const std::exception &e) {
    spdlog::error("Storey detection failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
