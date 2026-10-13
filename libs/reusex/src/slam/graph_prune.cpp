// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/graph_prune.hpp"

#include "core/logging.hpp"
#include "core/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace reusex::slam {

PruneResult prune_pose_graph(ProjectDB &db, const PruneFilter &f) {
  namespace pg = core::pose_graph;
  if (!f.residual_above && !f.deviation_above_m && !f.deviation_above_deg &&
      !f.edge_type && !f.source && !f.pair)
    throw std::invalid_argument(
        "graph prune: give at least one criterion (--residual-above, "
        "--deviation-above, --deviation-deg-above, --type, --source, --pair)");

  PruneResult r;
  for (const auto &e : db.list_pose_graph_edges()) {
    if (e.status == pg::kStatusRejected)
      continue;
    if (e.status == pg::kStatusConfirmed && !f.include_confirmed)
      continue;
    if (f.residual_above &&
        !(std::isfinite(e.residual) && e.residual > *f.residual_above))
      continue;
    if (f.deviation_above_m &&
        !(e.deviation_m && *e.deviation_m > *f.deviation_above_m))
      continue;
    if (f.deviation_above_deg &&
        !(e.deviation_deg && *e.deviation_deg > *f.deviation_above_deg))
      continue;
    if (f.edge_type && e.edge_type != *f.edge_type)
      continue;
    if (f.source && e.source != *f.source)
      continue;
    if (f.pair && !(std::min(f.pair->first, f.pair->second) ==
                        std::min(e.from_node_id, e.to_node_id) &&
                    std::max(f.pair->first, f.pair->second) ==
                        std::max(e.from_node_id, e.to_node_id)))
      continue;
    r.matched.push_back(e);
  }

  const auto odometry =
      std::count_if(r.matched.begin(), r.matched.end(), [](const auto &e) {
        return e.edge_type == pg::kTypeOdometry;
      });
  if (odometry > 0)
    core::warn("GraphPrune: {} of the {} matched edges are odometry; "
               "rejecting them splits the chain, which optimize bridges from "
               "the initial poses",
               odometry, r.matched.size());
  if (f.dry_run) {
    core::info("GraphPrune dry-run: {} edges would be rejected",
               r.matched.size());
    return r;
  }
  ProjectDB::Transaction tx(db);
  for (const auto &e : r.matched)
    r.rejected +=
        db.set_pose_graph_edge_status(e.from_node_id, e.to_node_id, e.edge_type,
                                      pg::kStatusRejected)
            ? 1
            : 0;
  tx.commit();
  core::info("GraphPrune: rejected {} edges", r.rejected);
  return r;
}

} // namespace reusex::slam
