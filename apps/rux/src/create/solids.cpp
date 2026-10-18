// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/solids.hpp"
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

void setup_subcommand_create_solids(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateSolidsOptions>();
  auto *sub = app.add_subcommand(
      "solids", "Typed solids: walls minus openings, slabs, space volumes");

  sub->footer(R"(
DESCRIPTION:
  The typed building model as closed solids, one per element:
    - wall:  a box along its axis (thickness, z_base, height, in its
             storey's level frame) minus a box per opening, so a door
             leaves a gap and a window a hole (CGAL corefinement)
    - slab:  each storey's floor polygon (the union of its spaces and wall
             footprints; the hull of its walls without spaces), from the
             storey below's ceiling up to the floor, else --slab-thickness;
             plus a roof over the top storey (--no-roof)
    - space: its polygon (with holes) from floor to ceiling
  Writes the meshes 'solid_walls', 'solid_slabs' and 'solid_spaces'. Each
  mesh's parameters carry the element index: per element its type, source
  GUID (wall / storey / space), face and vertex range, closed flag, volume
  and the IFC-ready extrusion profile (2D profile, depth, placement), plus a
  wall's opening voids.

EXAMPLES:
  rux create solids                        # Default settings
  rux create solids --unknown-thickness 0.2 --no-roof
  rux get meshes.solid_walls               # Metadata + element index
  rux render -o axo.png --view orbit:8 --layers cloud,solids
  rux render -o plan.png --view plan --layers cloud,solids

WORKFLOW:
  1. rux create storeys
  2. rux create classify
  3. rux create walls
  4. rux create openings
  5. rux create rooms
  6. rux create spaces
  7. rux create solids

NOTES:
  - Requires the 'walls' and 'storeys' tables; uses 'openings' and
    'spaces' when present
  - A wall whose thickness is unknown (seen from one side; its axis is the
    seen face) grows --unknown-thickness away from the space it faces
  - Wall ends at a corner or tee run on to the far face of the other wall
    (--no-join-extension turns it off)
  - In the Qt client: 3D workspace, layer 'Solider (typede)'
)");

  sub->get_formatter()->column_width(40);

  const reusex::pipeline::SolidStageOptions defaults{};
  for (const auto &p : reusex::pipeline::solid_parameter_table()) {
    const std::string flag(p.flag);
    const std::string help(p.description);
    std::visit(
        [&](auto get) {
          auto &field = get(opt->solids);
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
    spdlog::trace("calling create solids subcommand");
    if (rux::capture_stage_params("solids", solids_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_create_solids(*opt, *global_opt));
  });
}

std::string solids_stage_parameters(SubcommandCreateSolidsOptions const &opt) {
  return reusex::pipeline::solid_parameters_json(opt.solids);
}

int run_subcommand_create_solids(SubcommandCreateSolidsOptions const &opt,
                                 const RuxOptions &global_opt) {
  const auto &project_path = global_opt.project_db;
  spdlog::info("Building typed solids in project: {}", project_path.string());
  try {
    reusex::ProjectDB db(project_path);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::solids);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::solids;
    ctx.parameters = solids_stage_parameters(opt);

    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Solids complete: {}", result.message);
    return rux::exit_code_for(result);
  } catch (const std::exception &e) {
    spdlog::error("Solids failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
