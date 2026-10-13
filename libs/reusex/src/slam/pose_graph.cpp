// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/pose_graph.hpp"

#include "core/logging.hpp"
#include "core/pose_graph.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <map>
#include <set>
#include <utility>

namespace reusex::slam {
namespace {

namespace pg = core::pose_graph;

bool has_rigid_current_pose(const ProjectDB &db, int id) {
  return db.has_sensor_frame_pose(id) &&
         pg::is_rigid_transform(db.sensor_frame_pose(id));
}

/// Posed frames grouped by scan, each group in node-id order (the order
/// sensor_frame_ids_with_scan() returns).
std::map<int, std::vector<int>> posed_frames_by_scan(const ProjectDB &db,
                                                     bool from_current,
                                                     int *skipped = nullptr) {
  std::map<int, std::vector<int>> groups;
  int skip = 0;
  for (const auto &[id, scan] : db.sensor_frame_ids_with_scan()) {
    const bool posed = from_current ? has_rigid_current_pose(db, id)
                                    : (db.has_initial_pose(id) ||
                                       has_rigid_current_pose(db, id));
    if (!posed) {
      ++skip;
      continue;
    }
    groups[scan].push_back(id);
  }
  if (skipped)
    *skipped = skip;
  return groups;
}

Eigen::Matrix4d seed_pose(const ProjectDB &db, int id, bool from_current) {
  if (!from_current)
    if (auto p = db.initial_pose(id))
      return pg::matrix_from_row_major(*p);
  return pg::matrix_from_row_major(db.sensor_frame_pose(id));
}

} // namespace

int count_usable_odometry_edges(
    const std::vector<ProjectDB::PoseGraphEdge> &edges) {
  return static_cast<int>(std::count_if(
      edges.begin(), edges.end(), [](const ProjectDB::PoseGraphEdge &e) {
        return e.edge_type == pg::kTypeOdometry &&
               e.status != pg::kStatusRejected && e.transform.has_value();
      }));
}

std::vector<ProjectDB::PoseGraphEdge>
odometry_init_edges(const ProjectDB &db, bool from_current_transform) {
  std::vector<ProjectDB::PoseGraphEdge> out;
  for (const auto &[scan, ids] :
       posed_frames_by_scan(db, from_current_transform)) {
    for (size_t k = 0; k + 1 < ids.size(); ++k) {
      const Eigen::Matrix4d A = seed_pose(db, ids[k], from_current_transform);
      const Eigen::Matrix4d B =
          seed_pose(db, ids[k + 1], from_current_transform);
      ProjectDB::PoseGraphEdge e;
      e.from_node_id = ids[k];
      e.to_node_id = ids[k + 1];
      e.edge_type = std::string(pg::kTypeOdometry);
      e.transform = pg::row_major_from_matrix(A.inverse() * B);
      e.source = std::string(pg::kSourceOdometryInit);
      e.status = std::string(pg::kStatusAuto);
      e.seed_disagreement_m = 0.0;
      e.seed_disagreement_deg = 0.0;
      out.push_back(std::move(e));
    }
  }
  return out;
}

InitPoseGraphResult init_pose_graph(ProjectDB &db,
                                    const InitPoseGraphOptions &options) {
  InitPoseGraphResult r;
  const auto stored = db.list_pose_graph_edges();
  const int usable = count_usable_odometry_edges(stored);

  // A pair already covered by an odometry measurement (rtabmap, or a
  // confirmed odometry_init edge that force keeps) or an odometry tombstone
  // keeps it; legacy NULL-transform rows are overwritten. Under force the
  // "auto" odometry_init rows are about to be purged, so they do not count.
  std::set<std::pair<int, int>> taken;
  std::set<std::pair<int, int>> confirmed_unmeasured; // keep "confirmed"
  for (const auto &e : stored)
    if (e.edge_type == pg::kTypeOdometry && !e.transform &&
        e.status == pg::kStatusConfirmed)
      confirmed_unmeasured.emplace(e.from_node_id, e.to_node_id);
  for (const auto &e : stored)
    if (e.edge_type == pg::kTypeOdometry &&
        (e.transform.has_value() || e.status == pg::kStatusRejected) &&
        !(options.force && e.source == pg::kSourceOdometryInit &&
          e.status == pg::kStatusAuto))
      taken.emplace(e.from_node_id, e.to_node_id);

  int skipped = 0;
  const auto groups =
      posed_frames_by_scan(db, /*from_current=*/false, &skipped);

  // Without force, init only fills gaps (final review I5): frames without a
  // snapshot and consecutive in-scan pairs without usable odometry. It
  // refuses only when the graph already has odometry and nothing is missing.
  // The planned edges are measured from the initial poses, falling back to
  // the current pose of a frame not snapshotted yet -- the pose the snapshot
  // below copies, so planning before writing gives the same measurements.
  const auto planned = odometry_init_edges(db, /*from_current=*/false);
  if (!options.force && usable > 0) {
    int missing_snapshots = 0;
    for (const auto &[scan, ids] : groups)
      for (int id : ids)
        missing_snapshots += db.has_initial_pose(id) ? 0 : 1;
    const auto missing_pairs =
        std::count_if(planned.begin(), planned.end(), [&](const auto &e) {
          return taken.count({e.from_node_id, e.to_node_id}) == 0;
        });
    if (missing_snapshots == 0 && missing_pairs == 0)
      throw PoseGraphExistsError(fmt::format(
          "the stored pose graph already has {} odometry edges and every "
          "posed frame has an initial pose, so there is nothing to add; pass "
          "--force ('rux graph init --force') to re-derive the odometry_init "
          "edges and re-snapshot the initial poses from the current poses",
          usable));
  }

  ProjectDB::Transaction tx(db);
  if (options.force)
    r.odometry_edges_removed =
        db.delete_pose_graph_edges_from_source(pg::kSourceOdometryInit);

  r.scans = static_cast<int>(groups.size());
  int stale_snapshots = 0; // force, but the current pose is unusable
  for (const auto &[scan, ids] : groups) {
    r.frames += static_cast<int>(ids.size());
    for (int id : ids) {
      if (!options.force && db.has_initial_pose(id))
        continue;
      if (!has_rigid_current_pose(db, id)) {
        ++stale_snapshots; // only reachable under force
        continue;
      }
      db.set_initial_pose(id, db.sensor_frame_pose(id));
      ++r.snapshots_written;
    }
  }

  // Under force the snapshots were just rewritten: measure from them.
  for (auto e : options.force ? odometry_init_edges(db, /*from_current=*/false)
                              : planned) {
    if (confirmed_unmeasured.count({e.from_node_id, e.to_node_id}) > 0)
      e.status = std::string(pg::kStatusConfirmed);
    if (taken.count({e.from_node_id, e.to_node_id}) > 0 ||
        !db.upsert_pose_graph_edge(e)) {
      ++r.pairs_with_existing_odometry;
      continue;
    }
    ++r.odometry_edges_written;
  }
  tx.commit();

  if (stale_snapshots > 0)
    core::warn("GraphInit: {} frames have no usable current pose (missing or "
               "non-rigid transform) and kept their previous initial pose; "
               "fix their poses (rux get/rux set, or re-import) and re-run "
               "'rux graph init --force' to re-snapshot them",
               stale_snapshots);
  if (skipped > 0)
    core::warn("GraphInit: {} sensor frames carry no usable pose (missing or "
               "non-rigid transform) and were left out of the odometry chain; "
               "fix their poses (rux get/rux set, or re-import) and re-run "
               "'rux graph init --force'",
               skipped);
  if (r.odometry_edges_written == 0 && r.pairs_with_existing_odometry == 0)
    core::warn("GraphInit: no odometry edges written: {} posed frames in {} "
               "scan(s) form no consecutive pair within a scan (need at least "
               "two posed frames in one scan); import or re-pose frames and "
               "re-run 'rux graph init'",
               r.frames, r.scans);
  core::info("GraphInit: {} frames in {} scan(s): {} initial poses "
             "snapshotted, {} odometry edges written ({} pairs kept an "
             "existing edge), {} odometry_init edges replaced",
             r.frames, r.scans, r.snapshots_written, r.odometry_edges_written,
             r.pairs_with_existing_odometry, r.odometry_edges_removed);
  return r;
}

} // namespace reusex::slam
