// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/survey.hpp"
#include "exit_status.hpp"
#include "global-params.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/survey_service.hpp>

#include <spdlog/spdlog.h>

using reusex::core::SurveySyncOptions;

void setup_subcommand_create_survey(CLI::App &parent,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateSurveyOptions>();
  auto *sub = parent.add_subcommand(
      "survey", "Fill the Ressourcekortlægning from the instances table");

  sub->footer(R"(
DESCRIPTION:
  Fills the Ressourcekortlægning (Kortlægning in the web GUI): one survey type
  per semantic class and one bygningsdel (RX-###) per instance, each placed in
  the room most of its points fall in. Idempotent — re-running only adds
  instances that have no part yet and never overwrites edits made in the GUI.
  Each part without a resource image gets the sensor frame that sees it best.

EXAMPLES:
  rux create survey
  rux -p scan.rux create survey --rooms rooms

WORKFLOW:
  1. rux create instances
  2. rux create rooms          # optional, for room assignment
  3. rux create survey
  4. ruxd --local scan.rux     # review in Kortlægning
)");

  sub->add_option("-i,--instances", opt->sync.instances_cloud,
                  "Instance-label cloud name")
      ->default_val(SurveySyncOptions{}.instances_cloud);
  sub->add_option("--semantic", opt->sync.semantic_cloud,
                  "Semantic-label cloud name (for type naming)")
      ->default_val(SurveySyncOptions{}.semantic_cloud);
  sub->add_option("--rooms", opt->sync.rooms_cloud,
                  "Rooms-label cloud name (for room assignment)")
      ->default_val(SurveySyncOptions{}.rooms_cloud);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling create survey subcommand");
    rux::finish(run_subcommand_create_survey(*opt, *global_opt));
  });
}

int run_subcommand_create_survey(SubcommandCreateSurveyOptions const &opt,
                                 const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project database: {}", project_path.string());
    reusex::ProjectDB db(project_path, /*readOnly=*/false);

    const auto report = reusex::core::sync_survey(db, opt.sync);

    const std::string orphan_note =
        report.parts_orphaned > 0
            ? ", " + std::to_string(report.parts_orphaned) +
                  " part(s) orphaned (linked instance gone)"
            : "";
    const std::string dismissed_note =
        report.parts_dismissed > 0
            ? ", " + std::to_string(report.parts_dismissed) +
                  " deleted part(s) not re-created"
            : "";
    const std::string views_note =
        report.views_attached + report.views_unavailable > 0
            ? ", " + std::to_string(report.views_attached) +
                  " resource image(s) attached, " +
                  std::to_string(report.views_unavailable) +
                  " part(s) without one"
            : "";
    spdlog::info(
        "Survey: {} type(s) and {} part(s) created, {} part(s) already "
        "present{}{}{}{}",
        report.types_created, report.parts_created, report.parts_existing,
        report.rooms_assigned ? "" : " (no rooms assigned)", orphan_note,
        dismissed_note, views_note);
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("create survey failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
