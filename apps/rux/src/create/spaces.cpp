// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/spaces.hpp"
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

void setup_subcommand_create_spaces(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateSpacesOptions>();
  auto *sub = app.add_subcommand(
      "spaces", "Spaces: the band rooms as polygons against the walls");

  sub->footer(R"(
DESCRIPTION:
  Per storey, in its level frame: a CGAL arrangement of every wall's two
  faces (run --join-extension past their ends) and the storey's interior
  outline; each bounded face goes to the band room covering most of it,
  faces of one room merge into one polygon with holes, and where the walls
  do not close a room its raster outline fills in. Per space: area,
  perimeter, volume, compactness, kind (room | corridor | open_plan) and
  the stretches of wall that bound it. Writes the 'spaces' and
  'space_boundaries' tables.

  The band grid is rebuilt from the parameters the 'rooms' cloud was made
  with and must give back its labels; when storeys, walls or openings
  changed since `rux create rooms`, the stage refuses: re-run rooms first.

EXAMPLES:
  rux create spaces                        # Default settings
  rux create spaces --no-walls             # Raster outlines only
  rux get spaces                           # Show the result
  rux get spaces.<id>.boundaries           # The walls around one space
  rux render -o plan.png --view plan --layers cloud,spaces,walls

WORKFLOW:
  1. rux create storeys
  2. rux create classify
  3. rux create walls
  4. rux create openings
  5. rux create rooms                      # --method band (the default)
  6. rux create spaces

NOTES:
  - Requires 'cloud', 'normals', 'rooms' (band) and the 'storeys' table;
    uses the 'walls' and 'openings' tables when present
  - A re-detected space keeps the GUID of the stored one of its storey
    (--guid-match, --guid-min-area-ratio); rows with locked = 1 (edited by
    hand) are kept with their boundaries
  - `rux analyze solid --reference plan.yaml` scores the stored spaces
)");

  sub->get_formatter()->column_width(40);

  const reusex::pipeline::SpaceStageOptions defaults{};
  for (const auto &p : reusex::pipeline::space_parameter_table()) {
    const std::string flag(p.flag);
    const std::string help(p.description);
    std::visit(
        [&](auto get) {
          auto &field = get(opt->spaces);
          using T = std::remove_reference_t<decltype(field)>;
          if constexpr (std::is_same_v<T, bool>) {
            // The flag sets the opposite of the default.
            auto copy = defaults;
            const bool def = get(copy);
            sub->add_flag_callback(
                flag, [&field, def] { field = !def; },
                help + (def ? " (this flag turns it off)"
                            : " (this flag turns it on)"));
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
    spdlog::trace("calling create spaces subcommand");
    if (rux::capture_stage_params("spaces", spaces_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_create_spaces(*opt, *global_opt));
  });
}

std::string spaces_stage_parameters(SubcommandCreateSpacesOptions const &opt) {
  return reusex::pipeline::space_parameters_json(opt.spaces);
}

int run_subcommand_create_spaces(SubcommandCreateSpacesOptions const &opt,
                                 const RuxOptions &global_opt) {
  const auto &project_path = global_opt.project_db;
  spdlog::info("Polygonising spaces in project: {}", project_path.string());
  try {
    reusex::ProjectDB db(project_path);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::spaces);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::spaces;
    ctx.parameters = spaces_stage_parameters(opt);

    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Spaces complete: {}", result.message);
    return rux::exit_code_for(result);
  } catch (const std::exception &e) {
    spdlog::error("Spaces failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
