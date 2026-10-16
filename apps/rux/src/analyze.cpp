// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "analyze.hpp"
#include "analyze/accuracy.hpp"
#include "analyze/poses.hpp"
#include "analyze/quality.hpp"
#include "analyze/solid.hpp"

void setup_subcommand_analyze(CLI::App &app,
                              std::shared_ptr<RuxOptions> global_opt) {
  auto *sub =
      app.add_subcommand("analyze", "Analyze reconstruction quality metrics");

  sub->footer(R"(
DESCRIPTION:
  Parent command for computing quality metrics on reconstructed data.
  Ground-truth-free metrics measure internal-consistency properties (plane
  flatness, surface thickness) that pose error destroys; ground-truth metrics
  score the reconstruction against an external reference cloud. Both enable
  before/after comparison of SLAM and registration changes.

SUBCOMMANDS:
  quality      Plane flatness / surface thickness report (JSON)
  accuracy     Ground-truth accuracy / completeness / F-score (JSON)
  solid        Typed building model vs a reference plan (walls, spaces,
               openings, point-to-solid fit)
  poses        Trajectory vs a reference trajectory (ATE/RPE, TUM format)

TYPICAL WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct geometry
  3. rux create planes                 # Segment planar surfaces
  4. rux analyze quality -o before.json
  5. <change SLAM parameters / refinement, re-run 2-3>
  6. rux analyze quality -o after.json # Compare against before.json

NOTES:
  - Use 'rux analyze <subcommand> --help' for detailed options
)");

  setup_subcommand_analyze_quality(*sub, global_opt);
  setup_subcommand_analyze_accuracy(*sub, global_opt);
  setup_subcommand_analyze_solid(*sub, global_opt);
  setup_subcommand_analyze_poses(*sub, global_opt);

  sub->require_subcommand(1);
}
