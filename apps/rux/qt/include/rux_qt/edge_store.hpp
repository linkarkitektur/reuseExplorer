// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pose-graph store <-> the staged edit model. Header-only so the save
// path EdgeEditor runs on its thread and the save test
// (tests/unit/rux_qt/test_edge_save_locked.cpp) are one and the same code.

#include <rux_qt/database_logic.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace rux::qt {

inline EdgeRecord
edge_record_from_row(const reusex::ProjectDB::PoseGraphEdge &e) {
  EdgeRecord r;
  r.key = {e.from_node_id, e.to_node_id, e.edge_type};
  r.residual = e.residual;
  r.weight = e.weight;
  r.transform = e.transform;
  r.information = e.information;
  r.source = e.source;
  r.status = e.status.empty()
                 ? std::string(reusex::core::pose_graph::kStatusAuto)
                 : e.status;
  r.seed_disagreement_m = e.seed_disagreement_m;
  r.seed_disagreement_deg = e.seed_disagreement_deg;
  r.deviation_m = e.deviation_m;
  r.deviation_deg = e.deviation_deg;
  return r;
}

/// created_at is left empty: the store stamps it. A record without a source
/// (a user edge staged without ICP) is written as "manual": the store
/// refuses an empty source.
inline reusex::ProjectDB::PoseGraphEdge
edge_row_from_record(const EdgeRecord &r) {
  reusex::ProjectDB::PoseGraphEdge e;
  e.from_node_id = r.key.from;
  e.to_node_id = r.key.to;
  e.edge_type = r.key.type;
  e.residual = r.residual;
  e.weight = r.weight;
  e.transform = r.transform;
  e.information = r.information;
  e.source = r.source.empty()
                 ? std::string(reusex::core::pose_graph::kSourceManual)
                 : r.source;
  e.status = r.status;
  e.seed_disagreement_m = r.seed_disagreement_m;
  e.seed_disagreement_deg = r.seed_disagreement_deg;
  e.deviation_m = r.deviation_m;
  e.deviation_deg = r.deviation_deg;
  return e;
}

/// Apply @p ops in ONE transaction. Throws (and rolls back) when the store
/// refuses an upsert: the key was rejected in the file after it was loaded.
inline void apply_edge_ops(reusex::ProjectDB &db,
                           const std::vector<PendingEdgeEdits::Op> &ops) {
  reusex::ProjectDB::Transaction tx(db);
  for (const auto &op : ops) {
    const auto &k = op.edge.key;
    if (op.kind == PendingEdgeEdits::Op::Kind::set_status) {
      db.set_pose_graph_edge_status(k.from, k.to, k.type, op.edge.status);
    } else if (!db.upsert_pose_graph_edge(edge_row_from_record(op.edge),
                                          op.overwrite_rejected)) {
      throw std::runtime_error(
          "edge " + std::to_string(k.from) + " -> " + std::to_string(k.to) +
          " (" + k.type +
          ") was rejected in the project after it was loaded; reload and "
          "add it again");
    }
  }
  tx.commit();
}

} // namespace rux::qt
