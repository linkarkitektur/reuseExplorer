// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `rux graph init` (spec 2026-10-13 §4.3, D3): snapshot the initial poses
// `rux optimize` seeds from and freeze odometry edges between consecutive
// frames of each scan. RTABMap imports get both from the importer instead.

#include <reusex/core/ProjectDB.hpp>

#include <stdexcept>
#include <vector>

namespace reusex::slam {

/// init_pose_graph() without `force` on a graph that already has usable
/// odometry. The CLI maps it to INVALID_ARGUMENT.
class PoseGraphExistsError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

struct InitPoseGraphOptions {
  /// Replace the `odometry_init` edges and re-snapshot every initial pose
  /// from the current transforms. Loop edges, other odometry sources and
  /// tombstones are left alone.
  bool force = false;
};

struct InitPoseGraphResult {
  int frames = 0;                 ///< posed frames considered
  int snapshots_written = 0;      ///< initial_transform rows written
  int odometry_edges_written = 0; ///< odometry_init edges upserted
  int odometry_edges_removed = 0; ///< odometry_init rows deleted (force)
  int pairs_with_existing_odometry =
      0;         ///< pairs left to a stored edge/tombstone
  int scans = 0; ///< scan groups chained independently
};

/// Odometry edges that can enter a solve: type "odometry", not rejected,
/// with a measurement. Legacy NULL-transform rows do not count.
int count_usable_odometry_edges(
    const std::vector<ProjectDB::PoseGraphEdge> &edges);

/// The `odometry_init` edges init_pose_graph() would write — one per
/// consecutive pair of posed frames within a scan — computed from the
/// initial poses (falling back to `transform` per frame), or from the
/// current transforms when @p from_current_transform. Writes nothing.
std::vector<ProjectDB::PoseGraphEdge>
odometry_init_edges(const ProjectDB &db, bool from_current_transform);

/// Opens its own ProjectDB::Transaction: do not call inside one.
/// @throws PoseGraphExistsError when usable odometry exists and !force.
InitPoseGraphResult init_pose_graph(ProjectDB &db,
                                    const InitPoseGraphOptions &options = {});

} // namespace reusex::slam
