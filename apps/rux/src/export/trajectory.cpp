// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/trajectory.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/trajectory.hpp>

#include <spdlog/spdlog.h>

void setup_subcommand_export_trajectory(
    CLI::App &parent, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandExportTrajectoryOptions>();
  auto *sub = parent.add_subcommand(
      "trajectory", "Export the stored sensor-frame poses as a TUM trajectory");

  sub->footer(R"(
DESCRIPTION:
  Writes every sensor frame's stored world pose to a TUM-format trajectory
  file (one line per frame: timestamp tx ty tz qx qy qz qw), ordered by node
  id. A frame with no usable stored pose is skipped and reported.

  This is the file format a pose reference is frozen into
  (tests/fixtures/pose_reference/<scan>.trajectory.tum,
  docs/guides/pose-reference-authoring.md) and that `rux analyze poses`
  scores an estimate against.

EXAMPLES:
  rux export trajectory -o poses.tum
  rux export trajectory -o seed.tum --initial     # the optimize-seed poses

NOTES:
  - A frame with no real recorded timestamp writes its node id (as a double)
    in the timestamp column instead, so the file still sorts and matches
    correctly against another export of the same project.
)");

  sub->add_option("-o,--output", opt->output_path, "Output TUM file")
      ->required();
  sub->add_flag("--initial", opt->initial,
                "Export initial_transform (the optimize-seed pose) instead of "
                "the current pose");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_export_trajectory");
    rux::finish(run_subcommand_export_trajectory(*opt, *global_opt));
  });
}

int run_subcommand_export_trajectory(
    SubcommandExportTrajectoryOptions const &opt,
    const RuxOptions &global_opt) {
  try {
    spdlog::info("Exporting trajectory from project: {}",
                 global_opt.project_db.string());

    reusex::ProjectDB db(global_opt.project_db, /*readOnly=*/true);
    const auto trajectory =
        reusex::io::trajectory_from_project(db, opt.initial);
    reusex::io::write_tum(trajectory, opt.output_path);

    spdlog::info("Exported {} poses to {}", trajectory.size(),
                 opt.output_path.string());
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Trajectory export failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
