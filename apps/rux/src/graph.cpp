// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "graph.hpp"
#include "graph/init.hpp"

void setup_subcommand_graph(CLI::App &app,
                            std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand(
      "graph", "Edit the stored pose graph that 'rux optimize' solves");
  sub->footer(R"(
DESCRIPTION:
  The project's pose graph (pose_graph_edges) is the source of truth for
  'rux optimize': odometry edges (from 'rux import rtabmap' or 'graph init'),
  loop edges, and rejected edges kept as tombstones.

SUBCOMMANDS:
  init      Snapshot initial poses and create odometry edges

NOTES:
  - Use 'rux graph <subcommand> --help' for detailed options
)");
  sub->require_subcommand(1);
  setup_subcommand_graph_init(*sub, global_opt);
}
