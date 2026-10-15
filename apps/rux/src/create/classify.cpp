// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/classify.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/class_role_map.hpp>
#include <reusex/pipeline/stages.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

void setup_subcommand_create_classify(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateClassifyOptions>();
  auto &c = opt->classify;
  auto *sub = app.add_subcommand(
      "classify", "Classify each plane: wall, floor, ceiling, furniture, ...");

  sub->footer(R"(
DESCRIPTION:
  For every plane of the plane segmentation: its orientation against the
  measured vertical, the majority semantic class of its points (when a
  'labels' cloud exists) with the vote share, its role (wall, floor,
  ceiling, slab, furniture, clutter, unknown), its storey, area and height
  range. The role comes from the class through the class -> role map when
  the class is trusted and the geometry agrees, otherwise from geometric
  rules. Writes one row per plane to the 'plane_classes' table.

EXAMPLES:
  rux create classify                         # Built-in role map
  rux create classify --print-role-map > map.json
  rux create classify --role-map map.json     # Edited role map
  rux get plane_classes                       # Show the result

WORKFLOW:
  1. rux create clouds
  2. rux create planes
  3. rux create storeys
  4. rux create classify

NOTES:
  - Requires 'cloud', 'planes', 'plane_normals' and the 'storeys' table;
    uses the 'labels' cloud and its class names when present
  - Which way a plane faces (floor up, ceiling down) is used only when the
    clouds record that their normals face the sensor (clouds made since
    reusex-0se); older clouds are classified without it, with a warning
  - Rows with locked = 1 (edited by hand) are kept on a re-run
)");

  sub->get_formatter()->column_width(40);

  sub->add_option("--horizontal-cos", c.horizontal_cos,
                  "A plane is horizontal when |n.up| exceeds this cosine")
      ->default_val(c.horizontal_cos)
      ->check(CLI::Range(0.5, 1.0));
  sub->add_option("--vertical-cos", c.vertical_cos,
                  "A plane is vertical when |n.up| is below this cosine")
      ->default_val(c.vertical_cos)
      ->check(CLI::Range(0.0, 0.5));
  sub->add_option("--min-large-area", c.min_large_area_m2,
                  "A floor/ceiling candidate covers at least this [m^2]")
      ->default_val(c.min_large_area_m2)
      ->check(CLI::Range(0.0, 1000.0));
  sub->add_option("--band", c.band_m,
                  "Floor and ceiling band around the storey's heights [m]")
      ->default_val(c.band_m)
      ->check(CLI::Range(0.0, 2.0));
  sub->add_option("--min-wall-height", c.min_wall_height_m,
                  "A wall is at least this tall [m] ...")
      ->default_val(c.min_wall_height_m)
      ->check(CLI::Range(0.0, 20.0));
  sub->add_option("--min-wall-height-share", c.min_wall_height_share,
                  "... or this share of its storey's height, if less")
      ->default_val(c.min_wall_height_share)
      ->check(CLI::Range(0.0, 1.0));
  sub->add_option("--max-furniture-area", c.max_furniture_area_m2,
                  "A horizontal plane larger than this is never furniture "
                  "[m^2]")
      ->default_val(c.max_furniture_area_m2)
      ->check(CLI::Range(0.0, 1000.0));
  sub->add_option("--table-min", c.table_min_m,
                  "Lowest table top above the floor [m]")
      ->default_val(c.table_min_m)
      ->check(CLI::Range(0.0, 3.0));
  sub->add_option("--table-max", c.table_max_m,
                  "Highest table top above the floor [m]")
      ->default_val(c.table_max_m)
      ->check(CLI::Range(0.0, 3.0));
  sub->add_option("--min-storey-height", c.min_storey_height_m,
                  "Without storeys: the ceiling is at least this far above "
                  "the floor [m]")
      ->default_val(c.min_storey_height_m)
      ->check(CLI::Range(0.5, 20.0));
  sub->add_option("--area-cell", c.area_cell_m,
                  "Grid cell measuring a plane's area [m]")
      ->default_val(c.area_cell_m)
      ->check(CLI::Range(0.005, 1.0));
  sub->add_option("--extent-quantile", c.extent_quantile,
                  "Robust height range = these quantiles of the inliers")
      ->default_val(c.extent_quantile)
      ->check(CLI::Range(0.0, 0.49));
  sub->add_option("--min-labelled-share", c.min_labelled_share,
                  "The class decides only if this share of the inliers is "
                  "labelled ...")
      ->default_val(c.min_labelled_share)
      ->check(CLI::Range(0.0, 1.0));
  sub->add_option("--min-class-share", c.min_class_share,
                  "... and the majority class holds this share of the votes")
      ->default_val(c.min_class_share)
      ->check(CLI::Range(0.0, 1.0));
  sub->add_option("--role-map", opt->role_map,
                  "Class -> role map JSON file (default: the built-in map)");
  auto *print_map = sub->add_flag(
      "--print-role-map", "Print the built-in class -> role map and exit");

  sub->callback([opt, global_opt, print_map]() {
    spdlog::trace("calling create classify subcommand");
    if (print_map->count() > 0) {
      fmt::print("{}", reusex::core::default_class_role_map_json());
      rux::finish(RuxError::SUCCESS);
      return;
    }
    if (rux::capture_stage_params("classify", classify_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_create_classify(*opt, *global_opt));
  });
}

std::string
classify_stage_parameters(SubcommandCreateClassifyOptions const &opt) {
  const auto &c = opt.classify;
  return rux::StageParams()
      .set("horizontal_cos", c.horizontal_cos)
      .set("vertical_cos", c.vertical_cos)
      .set("min_large_area_m2", c.min_large_area_m2)
      .set("band_m", c.band_m)
      .set("min_wall_height_m", c.min_wall_height_m)
      .set("min_wall_height_share", c.min_wall_height_share)
      .set("max_furniture_area_m2", c.max_furniture_area_m2)
      .set("table_min_m", c.table_min_m)
      .set("table_max_m", c.table_max_m)
      .set("min_storey_height_m", c.min_storey_height_m)
      .set("area_cell_m", c.area_cell_m)
      .set("extent_quantile", c.extent_quantile)
      .set("min_labelled_share", c.min_labelled_share)
      .set("min_class_share", c.min_class_share)
      .set("role_map", opt.role_map)
      .dump();
}

int run_subcommand_create_classify(SubcommandCreateClassifyOptions const &opt,
                                   const RuxOptions &global_opt) {
  const auto &project_path = global_opt.project_db;
  spdlog::info("Classifying planes in project: {}", project_path.string());
  try {
    reusex::ProjectDB db(project_path);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::classify);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::classify;
    ctx.parameters = classify_stage_parameters(opt);

    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Plane classification complete: {}", result.message);
    return rux::exit_code_for(result);
  } catch (const std::exception &e) {
    spdlog::error("Plane classification failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
