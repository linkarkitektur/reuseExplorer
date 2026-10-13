// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Database glue for the plane-landmark pose-graph back-end: solve the STORED
// pose graph (docs/superpowers/specs/2026-10-13-pose-graph-editor-design.md
// §4.3). Seeds come from sensor_frames.initial_transform, so re-running gives
// the same answer; every active stored edge (auto + confirmed, with a
// measurement) becomes one BetweenFactor; poses are written to `transform`
// and per-edge metrics back to pose_graph_edges. The edge set itself is never
// replaced. Kept separate from PlaneGraphOptimizer.cpp so the solver core
// stays free of any database dependency.

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "slam/PlaneGraphOptimizer.hpp"
#include "slam/pose_graph.hpp"
#include "slam/pose_graph_conventions.hpp"

#include <Eigen/Geometry>

#include <fmt/format.h>

#include <algorithm>
#include <initializer_list>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace reusex::geometry {
namespace {

namespace pg = core::pose_graph;

/// Union-find over frame indices, for the connectivity bridge.
class Components {
    public:
  explicit Components(int n) : parent_(static_cast<size_t>(n)) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }
  int find(int x) {
    while (parent_[static_cast<size_t>(x)] != x) {
      parent_[static_cast<size_t>(x)] =
          parent_[static_cast<size_t>(parent_[static_cast<size_t>(x)])];
      x = parent_[static_cast<size_t>(x)];
    }
    return x;
  }
  void unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b)
      parent_[static_cast<size_t>(std::max(a, b))] = std::min(a, b);
  }

    private:
  std::vector<int> parent_;
};

/// Newest successful pipeline_log id among @p stages, 0 when none.
int newest_success(const std::vector<ProjectDB::PipelineLogEntry> &log,
                   std::initializer_list<std::string_view> stages) {
  int newest = 0;
  for (const auto &e : log)
    if (e.status == "success" &&
        std::find(stages.begin(), stages.end(), e.stage) != stages.end())
      newest = std::max(newest, e.id);
  return newest;
}

} // namespace

PlaneGraphResult optimize_sensor_poses(ProjectDB &db,
                                       const PlaneGraphOptions &options,
                                       bool dry_run) {
  // --- 0. The stored graph; initialise it when it has no usable odometry --
  auto stored = db.list_pose_graph_edges();
  bool initialised = false;
  if (slam::count_usable_odometry_edges(stored) == 0) {
    if (dry_run) {
      auto planned = slam::odometry_init_edges(db, /*from_current=*/false);
      core::info("PlaneGraph dry-run: the stored pose graph has no odometry "
                 "edges; using {} derived from the current poses in memory "
                 "(nothing written)",
                 planned.size());
      stored.insert(stored.end(), planned.begin(), planned.end());
    } else {
      const auto init = slam::init_pose_graph(db);
      core::info("PlaneGraph: the stored pose graph had no odometry edges; "
                 "initialised {} from the current poses and snapshotted {} "
                 "initial poses (as 'rux graph init')",
                 init.odometry_edges_written, init.snapshots_written);
      stored = db.list_pose_graph_edges();
      initialised = true;
    }
  }

  // Review focus 5: a pose refiner that ran after the graph was set up.
  if (!initialised) {
    const auto log = db.pipeline_log();
    if (newest_success(log, {"pose_refinement_jpr"}) >
        newest_success(
            log, {"import", "graph_init", "pose_optimization_plane_graph"}))
      core::warn("PlaneGraph: 'rux register' ran after the pose graph was "
                 "initialised; optimize seeds from the stored initial poses "
                 "and ignores its refinement. Run 'rux graph init --force' "
                 "first to optimise on top of it.");
  }

  // --- 1. Frames: surfels (optical frame, pose-independent) + seeds -------
  auto frameIds = db.sensor_frame_ids();
  std::sort(frameIds.begin(), frameIds.end()); // temporal ordering

  std::vector<FrameSurfels> frames;
  std::vector<Eigen::Matrix4d> local;       // parallel to frames
  std::vector<Eigen::Matrix4d> seed_stored; // initial pose, worldTf convention
  int fallback = 0;
  for (int id : frameIds) {
    auto fs = extract_frame_surfels(db, id, options.surfel);
    if (!fs)
      continue;
    std::array<double, 16> seed{};
    if (auto p = db.initial_pose(id)) {
      seed = *p;
    } else {
      seed = db.sensor_frame_pose(id);
      ++fallback;
    }
    const Eigen::Matrix4d L = pg::matrix_from_row_major(
        db.sensor_frame_intrinsics(id).local_transform);
    const Eigen::Matrix4d S = pg::matrix_from_row_major(seed);
    fs->world_pose.matrix() = S * L; // optical -> world, as extraction does
    local.push_back(L);
    seed_stored.push_back(S);
    frames.push_back(std::move(*fs));
  }

  if (frames.size() < frameIds.size())
    core::warn("PlaneGraph: {} of {} sensor frames yielded no surfels "
               "(missing colour/depth, invalid intrinsics, no usable stored "
               "pose, or too few points); optimizing the remaining {}",
               frameIds.size() - frames.size(), frameIds.size(), frames.size());
  if (fallback > 0)
    core::warn("PlaneGraph: {} of {} frames have no initial pose "
               "(sensor_frames.initial_transform); seeded from their current "
               "transform. Run 'rux graph init --force' to snapshot them.",
               fallback, frames.size());
  if (frames.size() < 2)
    throw std::runtime_error(fmt::format(
        "PlaneGraph: fewer than 2 usable sensor frames with depth/pose ({} of "
        "{} frames yielded surfels)",
        frames.size(), frameIds.size()));

  // --- 2. Active stored edges -> solver edges (optical convention) --------
  std::map<int, int> index_of;
  for (size_t k = 0; k < frames.size(); ++k)
    index_of[frames[k].node_id] = static_cast<int>(k);

  std::vector<GraphEdge> edges;
  int rejected = 0, no_measurement = 0, missing_frame = 0, panorama = 0;
  for (size_t k = 0; k < stored.size(); ++k) {
    const auto &s = stored[k];
    if (s.status == pg::kStatusRejected) {
      ++rejected;
      continue;
    }
    if (!s.transform) {
      ++no_measurement;
      continue;
    }
    const auto a = index_of.find(s.from_node_id);
    const auto b = index_of.find(s.to_node_id);
    if (a == index_of.end() || b == index_of.end()) {
      ++missing_frame;
      continue;
    }
    GraphEdge e;
    e.i = a->second;
    e.j = b->second;
    e.T_ij = slam::stored_to_optical(pg::matrix_from_row_major(*s.transform),
                                     local[static_cast<size_t>(e.i)],
                                     local[static_cast<size_t>(e.j)]);
    if (s.information)
      e.information = slam::information_stored_to_optical(
          pg::information_from_row_major(*s.information),
          local[static_cast<size_t>(e.j)]);
    e.kind = s.edge_type == pg::kTypeOdometry ? GraphEdge::Kind::odometry
                                              : GraphEdge::Kind::loop;
    e.known_inlier =
        e.kind == GraphEdge::Kind::loop && s.status == pg::kStatusConfirmed;
    e.tag = static_cast<int>(k);
    if (s.edge_type == pg::kTypePanorama)
      ++panorama;
    edges.push_back(std::move(e));
  }
  if (no_measurement > 0)
    core::warn("PlaneGraph: {} stored edges carry no measurement (pre-v27 "
               "'legacy' rows) and were skipped; re-create them with 'rux "
               "graph discover' or 'rux graph init --force'",
               no_measurement);
  if (missing_frame > 0)
    core::warn("PlaneGraph: {} stored edges reference a frame that yielded "
               "no surfels or has no pose and were skipped",
               missing_frame);
  if (rejected > 0)
    core::info("PlaneGraph: {} rejected edges left out of the solve", rejected);
  const int graph_edges = static_cast<int>(edges.size());

  // --- 3. Connectivity bridge (spec review focus 3) ------------------------
  Components comp(static_cast<int>(frames.size()));
  for (const auto &e : edges)
    comp.unite(e.i, e.j);
  int bridged = 0;
  for (int k = 0; k + 1 < static_cast<int>(frames.size()); ++k) {
    if (comp.find(k) == comp.find(k + 1))
      continue;
    GraphEdge e;
    e.i = k;
    e.j = k + 1;
    e.T_ij = frames[static_cast<size_t>(k)].world_pose.matrix().inverse() *
             frames[static_cast<size_t>(k + 1)].world_pose.matrix();
    edges.push_back(e);
    comp.unite(k, k + 1);
    ++bridged;
  }
  if (bridged > 0)
    core::warn("PlaneGraph: the stored graph splits into {} disconnected "
               "parts over the usable frames (frames without surfels, "
               "rejected odometry, or scan boundaries); bridged them with {} "
               "odometry edges from the initial poses (not stored)",
               bridged + 1, bridged);

  // --- 4. Solve -------------------------------------------------------------
  PlaneGraphOptimizer optimizer(options);
  PlaneGraphResult result = optimizer.optimize_graph(frames, edges);
  result.graph_edges = graph_edges;
  result.bridge_edges = bridged;
  result.graph_initialised = initialised;
  result.panorama_loop_edges = panorama;

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

  // --- 5. Poses (transform only) and per-edge metrics, one transaction ----
  // worldTf = world_pose * localTf^-1 (the stored `transform` convention).
  std::vector<Eigen::Matrix4d> solved(frames.size());
  for (size_t k = 0; k < frames.size(); ++k)
    solved[k] = frames[k].world_pose.matrix() * local[k].inverse();

  std::map<int, double> residual_of; // stored index -> residual
  for (const auto &rec : result.edges)
    if (rec.tag >= 0)
      residual_of[rec.tag] = rec.residual;

  std::vector<ProjectDB::PoseGraphEdgeMetrics> metrics;
  metrics.reserve(stored.size());
  for (size_t k = 0; k < stored.size(); ++k) {
    const auto &s = stored[k];
    ProjectDB::PoseGraphEdgeMetrics m;
    m.from_node_id = s.from_node_id;
    m.to_node_id = s.to_node_id;
    m.edge_type = s.edge_type;
    if (auto it = residual_of.find(static_cast<int>(k));
        it != residual_of.end())
      m.residual = it->second;
    const auto a = index_of.find(s.from_node_id);
    const auto b = index_of.find(s.to_node_id);
    if (s.transform && a != index_of.end() && b != index_of.end()) {
      const Eigen::Matrix4d T = pg::matrix_from_row_major(*s.transform);
      const auto i = static_cast<size_t>(a->second);
      const auto j = static_cast<size_t>(b->second);
      const auto dev = pg::transform_error(T, solved[i], solved[j]);
      const auto seed = pg::transform_error(T, seed_stored[i], seed_stored[j]);
      m.deviation_m = dev.translation_m;
      m.deviation_deg = dev.rotation_deg;
      m.seed_disagreement_m = seed.translation_m;
      m.seed_disagreement_deg = seed.rotation_deg;
    }
    metrics.push_back(std::move(m));
  }

  ProjectDB::Transaction tx(db);
  for (size_t k = 0; k < frames.size(); ++k)
    db.update_sensor_frame_pose(frames[k].node_id,
                                pg::row_major_from_matrix(solved[k]));
  result.edges_with_metrics = db.update_pose_graph_edge_metrics(metrics);
  tx.commit();
  core::info("PlaneGraph: wrote {} optimized poses and metrics for {} stored "
             "pose-graph edges",
             frames.size(), result.edges_with_metrics);
  return result;
}

} // namespace reusex::geometry
