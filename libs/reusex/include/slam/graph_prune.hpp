// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `rux graph prune` (spec 2026-10-13 §4.3): mark edges "rejected" by filter.
// Rejected rows stay as tombstones so `graph discover` never re-adds them.

#include <reusex/core/ProjectDB.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace reusex::slam {

/// Criteria AND together; at least one of the optional criteria is required.
struct PruneFilter {
  std::optional<double> residual_above; ///< residual > x (NULL never matches)
  std::optional<double> deviation_above_m;   ///< deviation_m > x
  std::optional<double> deviation_above_deg; ///< deviation_deg > x
  std::optional<std::string> edge_type;      ///< exact edge_type
  std::optional<std::string> source;         ///< exact source
  std::optional<std::pair<int, int>> pair;   ///< node pair, either direction
  bool include_confirmed = false;            ///< confirmed edges match too
  bool dry_run = false;                      ///< report only
};

struct PruneResult {
  std::vector<ProjectDB::PoseGraphEdge> matched; ///< as stored before pruning
  int rejected = 0;                              ///< rows marked (0 on dry run)
};

/// Opens its own ProjectDB::Transaction: do not call inside one.
/// @throws std::invalid_argument when @p filter has no criterion.
PruneResult prune_pose_graph(ProjectDB &db, const PruneFilter &filter);

} // namespace reusex::slam
