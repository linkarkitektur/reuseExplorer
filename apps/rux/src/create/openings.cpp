// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/openings.hpp"
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

void setup_subcommand_create_openings(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateOpeningsOptions>();
  auto *sub = app.add_subcommand(
      "openings", "Doors, windows and openings in the walls' faces");

  sub->footer(R"(
DESCRIPTION:
  Per wall of the 'walls' table, in its storey's level frame: a 10 cm
  (s, z) raster over the face; a cell is open when no point lies in the
  wall's slab there and nothing stands within --front in front of either
  face (a wall hidden behind a cabinet is not an opening). The open cells
  are opened morphologically, split into components and fitted with
  rectangles, kept when big and full enough and framed by wall on both
  sides. Each is classified: sill below --door-max-sill and a door's height
  -> door; low sill and head at the ceiling -> opening; else window.
  Evidence raises the confidence: semantic 'door' / 'window' / glass labels
  and instances on it, glass confidence images, and the camera path walking
  through it. An open door leaf gives the door's state and swing. Writes the
  'openings' table.

EXAMPLES:
  rux create openings                       # Default settings
  rux create openings --front 0.8           # Shorter occluder check
  rux get openings                          # Show the result
  rux get walls.<id>.openings               # The openings of one wall
  rux render -o plan.png --view plan --layers cloud,walls,openings

WORKFLOW:
  1. rux create storeys
  2. rux create classify
  3. rux create walls
  4. rux create openings

NOTES:
  - Requires 'cloud' and the 'storeys' and 'walls' tables; uses 'labels',
    'instances', glass confidence images and the sensor poses when present
  - Replaces `rux create windows` (deprecated, removed in WP6)
  - A re-detected opening keeps the GUID of the stored one in the same wall
    (--guid-match); rows with locked = 1 (edited by hand) are kept
)");

  sub->get_formatter()->column_width(40);

  for (const auto &p : reusex::pipeline::opening_parameter_table()) {
    const std::string flag(p.flag);
    const std::string help(p.description);
    std::visit(
        [&](auto get) {
          auto &field = get(opt->openings);
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
    spdlog::trace("calling create openings subcommand");
    if (rux::capture_stage_params("openings", openings_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_create_openings(*opt, *global_opt));
  });
}

std::string
openings_stage_parameters(SubcommandCreateOpeningsOptions const &opt) {
  return reusex::pipeline::opening_parameters_json(opt.openings);
}

int run_subcommand_create_openings(SubcommandCreateOpeningsOptions const &opt,
                                   const RuxOptions &global_opt) {
  const auto &project_path = global_opt.project_db;
  spdlog::info("Detecting openings in project: {}", project_path.string());
  try {
    reusex::ProjectDB db(project_path);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::openings);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::openings;
    ctx.parameters = openings_stage_parameters(opt);

    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Opening detection complete: {}", result.message);
    return rux::exit_code_for(result);
  } catch (const std::exception &e) {
    spdlog::error("Opening detection failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
