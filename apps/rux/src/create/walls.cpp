// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/walls.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>

#include <spdlog/spdlog.h>

#include <cmath>
#include <limits>
#include <string>
#include <type_traits>

void setup_subcommand_create_walls(CLI::App &app,
                                   std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateWallsOptions>();
  auto *sub = app.add_subcommand(
      "walls", "Walls with a measured thickness and the joins between them");

  sub->footer(R"(
DESCRIPTION:
  Per storey, in its level frame: the directions walls run in (axis
  families), the faces of the wall-role planes from their offset profile,
  which two faces are the sides of one wall (gap, empty valley between,
  co-visibility), its thickness at mid-height with a bootstrap CI and its
  lean, whether it is external, and the wall graph (snapping, bridging over
  doorways, corners and tees). A wall seen from one side only has its
  axis on the seen face and thickness -1 (unknown); --default-thickness is
  the body assumed behind that face. Writes the 'walls' and 'wall_joins'
  tables.

EXAMPLES:
  rux create walls                          # Default settings
  rux create walls --max-gap 0.6            # Allow thicker walls
  rux get walls                             # Show the result
  rux get storeys.0.walls                   # The walls of storey 0
  rux render -o plan.png --view plan --layers cloud,walls

WORKFLOW:
  1. rux create clouds
  2. rux create planes
  3. rux create storeys
  4. rux create classify
  5. rux create walls

NOTES:
  - Requires 'cloud', 'planes', 'plane_normals' and the 'storeys' and
    'plane_classes' tables; refuses classes made for another plane
    segmentation (re-run classify after planes)
  - Which side a face was seen from is used only when the clouds record
    that their normals face the sensor (clouds made since reusex-0se)
  - A re-detected wall keeps the GUID of the stored wall it lies on
    (--guid-match); rows with locked = 1 (edited by hand) are kept
)");

  sub->get_formatter()->column_width(40);

  for (const auto &p : reusex::pipeline::wall_parameter_table()) {
    const std::string flag(p.flag);
    const std::string help(p.description);
    std::visit(
        [&](auto get) {
          auto &field = get(opt->walls);
          using T = std::remove_reference_t<decltype(field)>;
          if constexpr (std::is_same_v<T, bool>) {
            // Every boolean defaults to true; its flag turns it off.
            sub->add_flag_callback(
                flag, [&field] { field = false; },
                help + " (this flag turns it off)");
          } else {
            auto *o = sub->add_option(flag, field, help)->default_val(field);
            const double lo =
                p.minimum.value_or(-std::numeric_limits<double>::max());
            const double hi = p.maximum && std::isfinite(*p.maximum)
                                  ? *p.maximum
                                  : std::numeric_limits<double>::max();
            o->check(CLI::Range(lo, hi));
          }
        },
        p.field);
  }

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling create walls subcommand");
    if (rux::capture_stage_params("walls", walls_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_create_walls(*opt, *global_opt));
  });
}

std::string walls_stage_parameters(SubcommandCreateWallsOptions const &opt) {
  return reusex::pipeline::wall_parameters_json(opt.walls);
}

int run_subcommand_create_walls(SubcommandCreateWallsOptions const &opt,
                                const RuxOptions &global_opt) {
  const auto &project_path = global_opt.project_db;
  spdlog::info("Detecting walls in project: {}", project_path.string());
  try {
    reusex::ProjectDB db(project_path);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::walls);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::walls;
    ctx.parameters = walls_stage_parameters(opt);

    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Wall detection complete: {}", result.message);
    return rux::exit_code_for(result);
  } catch (const std::exception &e) {
    spdlog::error("Wall detection failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
