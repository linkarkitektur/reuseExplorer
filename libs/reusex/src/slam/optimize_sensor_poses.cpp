// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Database glue for the plane-landmark pose-graph back-end: extract surfels
// from a ProjectDB, run the pure in-memory optimizer, and write optimized
// poses back. Kept separate from PlaneGraphOptimizer.cpp so the solver core
// stays free of any database dependency (mirrors refine_sensor_poses.cpp).

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "geometry/transform_utils.hpp"
#include "slam/PlaneGraphOptimizer.hpp"

#include <Eigen/Geometry>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

namespace reusex::geometry {

PlaneGraphResult optimize_sensor_poses(ProjectDB &db,
                                       const PlaneGraphOptions &options,
                                       bool dry_run) {
  auto frameIds = db.sensor_frame_ids();
  std::sort(frameIds.begin(), frameIds.end()); // ensure temporal ordering

  std::vector<FrameSurfels> frames;
  std::vector<core::SensorIntrinsics> intrinsics; // parallel to `frames`
  frames.reserve(frameIds.size());
  for (int id : frameIds) {
    auto fs = extract_frame_surfels(db, id, options.surfel);
    if (!fs)
      continue;
    intrinsics.push_back(db.sensor_frame_intrinsics(id));
    frames.push_back(std::move(*fs));
  }

  // extract_frame_surfels() drops a frame silently (missing colour/depth,
  // invalid intrinsics, no usable stored pose, too few points). Report the
  // shortfall rather than letting the graph quietly shrink (STANDARDS §5).
  if (frames.size() < frameIds.size())
    core::warn("PlaneGraph: {} of {} sensor frames yielded no surfels "
               "(missing colour/depth, invalid intrinsics, no usable stored "
               "pose, or too few points); optimizing the remaining {}",
               frameIds.size() - frames.size(), frameIds.size(), frames.size());

  if (frames.size() < 2)
    throw std::runtime_error(fmt::format(
        "PlaneGraph: fewer than 2 usable sensor frames with depth/pose ({} of "
        "{} frames yielded surfels)",
        frames.size(), frameIds.size()));

  PlaneGraphOptimizer optimizer(options);
  PlaneGraphResult result = optimizer.optimize(frames);

  if (dry_run) {
    core::info("PlaneGraph dry-run: poses NOT written back");
    return result;
  }

  if (result.landmarks == 0 && result.loop_edges == 0) {
    core::warn(
        "PlaneGraph: no landmarks or loop edges — poses NOT written back");
    return result;
  }

  if (!result.converged) {
    core::error("PlaneGraph: optimization did not converge — poses NOT "
                "written back (docs/STANDARDS.md §5: no silent failure)");
    return result;
  }

  // Write optimized poses back. The stored `transform` column is worldTf, so we
  // remove the constant local (optical->sensor) transform that was folded into
  // world_pose during extraction:  worldTf = world_pose * localTf^-1.
  int written = 0;
  for (size_t k = 0; k < frames.size(); ++k) {
    const Eigen::Affine3d localTf = to_affine(intrinsics[k].local_transform);
    const Eigen::Affine3d worldTf = frames[k].world_pose * localTf.inverse();
    db.update_sensor_frame_pose(frames[k].node_id, to_array16(worldTf));
    ++written;
  }
  core::info("PlaneGraph: wrote {} optimized poses back to database", written);

  // Persist the pose graph: translate frame-vector indices → DB node_ids via
  // FrameSurfels::node_id (always set; no separate node_ids lookup needed).
  std::vector<reusex::ProjectDB::PoseGraphEdge> pg_edges;
  pg_edges.reserve(result.edges.size());
  const int N_frames = static_cast<int>(frames.size());
  for (const auto &e : result.edges) {
    if (e.i < 0 || e.j < 0 || e.i >= N_frames || e.j >= N_frames)
      continue;
    std::string type = e.type;
    pg_edges.push_back({frames[e.i].node_id, frames[e.j].node_id,
                        std::move(type), e.residual, e.weight});
  }
  db.save_pose_graph_edges(pg_edges);
  core::info("PlaneGraph: saved {} pose-graph edges to database",
             pg_edges.size());
  return result;
}

} // namespace reusex::geometry
