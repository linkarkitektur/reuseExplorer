// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "edit/plan.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>

#include <stdexcept>

namespace io = reusex::io;

namespace {

std::string summary(const reusex::core::SolidModel &m) {
  return fmt::format("{} storeys, {} walls, {} spaces, {} openings",
                     m.storeys.size(), m.walls.size(), m.spaces.size(),
                     m.openings.size());
}

} // namespace

void setup_subcommand_edit_plan(CLI::App &app,
                                std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandEditPlanOptions>();
  auto *sub = app.add_subcommand(
      "plan", "Import or export the solid-model plan (walls, spaces, openings) "
              "as JSON or YAML");

  sub->footer(R"(
DESCRIPTION:
  Moves a solid-model plan between a document (docs/design/solid-model-json.md,
  JSON or YAML -- picked by extension: .yaml/.yml is YAML, anything else
  JSON) and the project. --import reads a plan strictly (an unknown key, a
  dangling reference or an invalid polygon is an error naming the element),
  marks every element source "imported" and locked, and stores it; --export
  writes the project's model, in the format --export's own extension picks.

  The project's solid-model tables (storeys, walls, spaces, openings) do not
  exist yet, so an import into a project stores nothing and fails. Use
  --dry-run to validate a plan and write it back normalised without touching
  the project; the authoring loop that uses it is
  docs/guides/solid-model-reference-authoring.md.

EXAMPLES:
  # Validate an authored plan and write it back in canonical form
  rux edit plan --import ref.plan.json --dry-run --export ref.plan.json

  # Import a reference into the project
  rux -p scan.rux edit plan --import ref.plan.yaml

  # Export the project's model
  rux -p scan.rux edit plan --export model.json
)");

  sub->add_option("--import", opt->import_path,
                  "Solid-model plan (JSON or YAML) to import")
      ->check(CLI::ExistingFile);
  sub->add_option("--export", opt->export_path,
                  "Write the solid-model plan here (JSON or YAML by "
                  "extension; the project's model, or the imported one with "
                  "--dry-run)");
  sub->add_flag("--dry-run", opt->dry_run,
                "Validate --import without opening the project");

  sub->callback([opt, global_opt]() {
    spdlog::trace("Running edit plan subcommand");
    rux::finish(run_subcommand_edit_plan(*opt, *global_opt));
  });
}

int run_subcommand_edit_plan(SubcommandEditPlanOptions const &opt,
                             const RuxOptions &global_opt) {
  if (opt.import_path.empty() && opt.export_path.empty()) {
    spdlog::error("edit plan: pass --import <plan.json>, --export "
                  "<plan.json>, or both");
    return RuxError::INVALID_ARGUMENT;
  }
  if (opt.dry_run && opt.import_path.empty()) {
    spdlog::error("edit plan: --dry-run validates an --import; none given");
    return RuxError::INVALID_ARGUMENT;
  }

  try {
    if (!opt.import_path.empty()) {
      reusex::core::SolidModel model;
      try {
        model = io::solid_model_from_file(opt.import_path);
      } catch (const std::invalid_argument &e) {
        spdlog::error("edit plan: invalid plan '{}': {}",
                      opt.import_path.string(), e.what());
        return RuxError::INVALID_ARGUMENT;
      }
      io::mark_imported(model);
      spdlog::info("edit plan: '{}' is valid: {}", opt.import_path.string(),
                   summary(model));

      if (opt.dry_run) {
        if (!opt.export_path.empty()) {
          io::solid_model_to_file(model, opt.export_path);
          spdlog::info("edit plan: wrote the imported plan to '{}'",
                       opt.export_path.string());
        }
        return RuxError::SUCCESS;
      }

      reusex::ProjectDB db(global_opt.project_db);
      const auto stored = io::store_solid_model(db, model);
      if (!stored.not_stored.empty())
        spdlog::warn("edit plan: not stored — the project has no table for {} "
                     "yet",
                     fmt::join(stored.not_stored, ", "));
      if (stored.stored() == 0) {
        spdlog::error("edit plan: nothing was imported into '{}': the "
                      "solid-model tables do not exist yet. Use --dry-run to "
                      "validate the plan, and `rux render --plan` to check it "
                      "against the scan",
                      global_opt.project_db.string());
        return RuxError::NOT_IMPLEMENTED;
      }
      spdlog::info("edit plan: imported {} storeys, {} walls, {} spaces, {} "
                   "openings",
                   stored.storeys, stored.walls, stored.spaces,
                   stored.openings);
    }

    if (!opt.export_path.empty()) {
      const reusex::ProjectDB db(global_opt.project_db, /* readOnly */ true);
      const auto model = io::load_solid_model(db);
      if (!model.has_storeys() && !model.has_walls() && !model.has_spaces() &&
          !model.has_openings())
        spdlog::warn("edit plan: '{}' has no solid-model element yet (no "
                     "storeys, walls, spaces or openings table); the export "
                     "is an empty document",
                     global_opt.project_db.string());
      io::solid_model_to_file(model, opt.export_path);
      spdlog::info("edit plan: wrote {} to '{}'", summary(model),
                   opt.export_path.string());
    }
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("edit plan failed: {}", e.what());
    return RuxError::IO;
  }
}
