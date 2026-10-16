// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Database glue for the plane-landmark pose-graph back-end: solve the STORED
// pose graph (docs/design/2026-10-13-pose-graph-editor-design.md
// §4.3). Seeds come from sensor_frames.initial_transform, so re-running gives
// the same answer; every active stored edge (auto + confirmed, with a
// measurement) becomes one BetweenFactor; poses are written to `transform`
// and per-edge metrics back to pose_graph_edges. The edge set itself is never
// replaced. Kept separate from PlaneGraphOptimizer.cpp so the solver core
// stays free of any database dependency.

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/pose_edits.hpp"
#include "core/pose_graph.hpp"
#include "slam/PlaneGraphOptimizer.hpp"
#include "slam/pose_edit_lowering.hpp"
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

namespace {

StoredPoseGraph load_graph(const ProjectDB &db,
                           const PlaneGraphOptions &options,
                           bool warn_refiner) {
  StoredPoseGraph g;
  // --- 0. The stored graph; derive odometry in memory when it has none ----
  auto stored = db.list_pose_graph_edges();
  if (slam::count_usable_odometry_edges(stored) == 0) {
    auto planned = slam::odometry_init_edges(db, /*from_current=*/false);
    core::info("PlaneGraph: the stored pose graph has no odometry edges; "
               "using {} derived from the current poses in memory (nothing "
               "written)",
               planned.size());
    stored.insert(stored.end(), planned.begin(), planned.end());
    g.odometry_in_memory = true;
  } else if (warn_refiner) {
    // Review focus 5: a pose refiner that ran after the graph was set up.
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
  g.frames_total = frameIds.size();

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
    g.local.push_back(L);
    g.seed_stored.push_back(S);
    g.frames.push_back(std::move(*fs));
  }

  if (g.frames.size() < frameIds.size())
    core::warn("PlaneGraph: {} of {} sensor frames yielded no surfels "
               "(missing colour/depth, invalid intrinsics, no usable stored "
               "pose, or too few points); optimizing the remaining {}",
               frameIds.size() - g.frames.size(), frameIds.size(),
               g.frames.size());
  if (fallback > 0)
    core::warn("PlaneGraph: {} of {} frames have no initial pose "
               "(sensor_frames.initial_transform); seeded from their current "
               "transform. Run 'rux graph init' to snapshot them.",
               fallback, g.frames.size());
  if (g.frames.size() < 2)
    throw std::runtime_error(fmt::format(
        "PlaneGraph: fewer than 2 usable sensor frames with depth/pose ({} of "
        "{} frames yielded surfels)",
        g.frames.size(), frameIds.size()));

  // --- 2. Stored edges -> solver edges (optical convention) ---------------
  std::map<int, int> index_of;
  for (size_t k = 0; k < g.frames.size(); ++k)
    index_of[g.frames[k].node_id] = static_cast<int>(k);

  int rejected = 0, no_measurement_odometry = 0, no_measurement_loop = 0,
      missing_frame = 0;
  g.edges.reserve(stored.size());
  for (size_t k = 0; k < stored.size(); ++k) {
    const auto &s = stored[k];
    StoredPoseGraph::Edge out;
    out.from_node_id = s.from_node_id;
    out.to_node_id = s.to_node_id;
    out.edge_type = s.edge_type;
    out.panorama = s.edge_type == pg::kTypePanorama;
    if (s.transform)
      out.measurement = pg::matrix_from_row_major(*s.transform);
    const auto a = index_of.find(s.from_node_id);
    const auto b = index_of.find(s.to_node_id);
    if (s.status == pg::kStatusRejected) {
      ++rejected;
    } else if (!s.transform) {
      ++(s.edge_type == pg::kTypeOdometry ? no_measurement_odometry
                                          : no_measurement_loop);
    } else if (a == index_of.end() || b == index_of.end()) {
      ++missing_frame;
    } else {
      GraphEdge e;
      e.i = a->second;
      e.j = b->second;
      e.T_ij = slam::stored_to_optical(*out.measurement,
                                       g.local[static_cast<size_t>(e.i)],
                                       g.local[static_cast<size_t>(e.j)]);
      if (s.information)
        e.information = slam::information_stored_to_optical(
            pg::information_from_row_major(*s.information),
            g.local[static_cast<size_t>(e.j)]);
      e.kind = s.edge_type == pg::kTypeOdometry ? GraphEdge::Kind::odometry
                                                : GraphEdge::Kind::loop;
      e.known_inlier =
          e.kind == GraphEdge::Kind::loop && s.status == pg::kStatusConfirmed;
      e.tag = static_cast<int>(k);
      out.solver = std::move(e);
    }
    g.edges.push_back(std::move(out));
  }
  if (no_measurement_odometry + no_measurement_loop > 0)
    core::warn("PlaneGraph: {} stored edges carry no measurement (pre-v28 "
               "'legacy' or GUI-added rows) and were skipped: {} odometry "
               "('rux graph init' fills them) and {} loop/panorama ('rux "
               "graph discover' fills the pairs its front-ends re-find); "
               "reject the rest in Rediger / 'rux graph prune --pair'",
               no_measurement_odometry + no_measurement_loop,
               no_measurement_odometry, no_measurement_loop);
  if (missing_frame > 0)
    core::warn("PlaneGraph: {} stored edges reference a frame that yielded "
               "no surfels or has no pose and were skipped",
               missing_frame);
  if (rejected > 0)
    core::info("PlaneGraph: {} rejected edges left out of the solve", rejected);
  return g;
}

} // namespace

StoredPoseGraph load_stored_pose_graph(const ProjectDB &db,
                                       const PlaneGraphOptions &options) {
  return load_graph(db, options, /*warn_refiner=*/true);
}

StoredGraphSolve solve_stored_pose_graph(const ProjectDB &db,
                                         const StoredPoseGraph &graph,
                                         const PlaneGraphOptions &options,
                                         const core::PoseEdits *edits) {
  // Every solve starts from the seeds: the frames are copied (their surfel
  // clouds are shared, the solver only moves world_pose).
  std::vector<FrameSurfels> frames = graph.frames;

  // Pose edits (reusex-rcv.1.3): lowered over the frames that are solved.
  slam::LoweredEdits lowered;
  if (edits != nullptr && !edits->edits.empty()) {
    std::vector<int> node_order;
    node_order.reserve(frames.size());
    for (const auto &f : frames)
      node_order.push_back(f.node_id);
    lowered = slam::lower_pose_edits(db, *edits, node_order);
    core::info("PlaneGraph: {} pose edits -> {} priors, {} relative / "
               "surface constraints, {} edge rejections, {} skipped",
               edits->edits.size(), lowered.priors.size(),
               lowered.surfaces.size(), lowered.tombstones.size(),
               lowered.skipped.size());
  }
  auto tombstoned = [&](const StoredPoseGraph::Edge &s) {
    const int a = std::min(s.from_node_id, s.to_node_id);
    const int b = std::max(s.from_node_id, s.to_node_id);
    return std::any_of(lowered.tombstones.begin(), lowered.tombstones.end(),
                       [&](const slam::EdgeTombstone &t) {
                         return t.from == a && t.to == b &&
                                t.edge_type == s.edge_type;
                       });
  };

  int rejected_by_edits = 0, cut_by_moves = 0, panorama = 0;
  std::vector<GraphEdge> edges;
  edges.reserve(graph.edges.size());
  for (const auto &s : graph.edges) {
    if (!s.solver)
      continue;
    if (tombstoned(s)) {
      ++rejected_by_edits;
      continue;
    }
    // reusex-9fx: a move replaces the odometry into its block with its
    // junction factor.
    if (s.solver->kind == GraphEdge::Kind::odometry &&
        slam::cuts_odometry_edge(lowered, s.solver->i, s.solver->j)) {
      ++cut_by_moves;
      continue;
    }
    if (s.panorama)
      ++panorama;
    edges.push_back(*s.solver);
  }
  if (lowered.tombstones.size() > static_cast<size_t>(rejected_by_edits))
    core::warn("PlaneGraph: {} reject_edge edits name no active stored edge "
               "({} matched)",
               lowered.tombstones.size() - rejected_by_edits,
               rejected_by_edits);
  const int graph_edges = static_cast<int>(edges.size());
  if (!lowered.cuts.empty()) {
    core::info("PlaneGraph: {} move junction(s) replace {} odometry edges "
               "into the moved blocks",
               lowered.cuts.size(), cut_by_moves);
    if (cut_by_moves == 0)
      core::warn("PlaneGraph: the {} move junction{} cut no stored odometry "
                 "edge (the stored graph has none into the block); the "
                 "junction is added on top of what holds the block",
                 lowered.cuts.size(), lowered.cuts.size() == 1 ? "" : "s");
  }

  // --- 3. Connectivity bridge (spec review focus 3) ------------------------
  Components comp(static_cast<int>(frames.size()));
  for (const auto &e : edges)
    comp.unite(e.i, e.j);
  for (const auto &c : lowered.cuts) // the junction holds the cut together
    comp.unite(c.from, c.to);
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
  StoredGraphSolve out;
  out.result =
      optimizer.optimize_graph(frames, edges, lowered.priors, lowered.surfaces);
  out.result.edits_skipped = lowered.skipped;
  out.result.edges_rejected_by_edits = rejected_by_edits;
  out.result.graph_edges = graph_edges;
  out.result.bridge_edges = bridged;
  out.result.panorama_loop_edges = panorama;

  // worldTf = world_pose * localTf^-1 (the stored `transform` convention).
  out.poses.resize(frames.size());
  for (size_t k = 0; k < frames.size(); ++k)
    out.poses[k] = frames[k].world_pose.matrix() * graph.local[k].inverse();
  return out;
}

PlaneGraphResult optimize_sensor_poses(ProjectDB &db,
                                       const PlaneGraphOptions &options,
                                       bool dry_run,
                                       const core::PoseEdits *edits) {
  // A graph without usable odometry is initialised for real unless this is
  // a dry run (then load_stored_pose_graph() derives it in memory).
  bool initialised = false;
  if (!dry_run &&
      slam::count_usable_odometry_edges(db.list_pose_graph_edges()) == 0) {
    const auto init = slam::init_pose_graph(db);
    core::info("PlaneGraph: the stored pose graph had no odometry edges; "
               "initialised {} from the current poses and snapshotted {} "
               "initial poses (as 'rux graph init')",
               init.odometry_edges_written, init.snapshots_written);
    initialised = true;
  }

  const StoredPoseGraph graph =
      load_graph(db, options, /*warn_refiner=*/!initialised);
  StoredGraphSolve solve = solve_stored_pose_graph(db, graph, options, edits);
  PlaneGraphResult &result = solve.result;
  result.graph_initialised = initialised;

  if (dry_run) {
    core::info("PlaneGraph dry-run: poses NOT written back");
    return result;
  }
  if (result.landmarks == 0 && result.loop_edges == 0 &&
      result.edit_factors == 0) {
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
  const std::vector<Eigen::Matrix4d> &solved = solve.poses;
  std::map<int, int> index_of;
  for (size_t k = 0; k < graph.frames.size(); ++k)
    index_of[graph.frames[k].node_id] = static_cast<int>(k);

  std::map<int, double> residual_of; // stored index -> residual
  for (const auto &rec : result.edges)
    if (rec.tag >= 0)
      residual_of[rec.tag] = rec.residual;

  std::vector<ProjectDB::PoseGraphEdgeMetrics> metrics;
  metrics.reserve(graph.edges.size());
  for (size_t k = 0; k < graph.edges.size(); ++k) {
    const auto &s = graph.edges[k];
    ProjectDB::PoseGraphEdgeMetrics m;
    m.from_node_id = s.from_node_id;
    m.to_node_id = s.to_node_id;
    m.edge_type = s.edge_type;
    if (auto it = residual_of.find(static_cast<int>(k));
        it != residual_of.end())
      m.residual = it->second;
    const auto a = index_of.find(s.from_node_id);
    const auto b = index_of.find(s.to_node_id);
    if (s.measurement && a != index_of.end() && b != index_of.end()) {
      const Eigen::Matrix4d &T = *s.measurement;
      const auto i = static_cast<size_t>(a->second);
      const auto j = static_cast<size_t>(b->second);
      const auto dev = pg::transform_error(T, solved[i], solved[j]);
      const auto seed =
          pg::transform_error(T, graph.seed_stored[i], graph.seed_stored[j]);
      m.deviation_m = dev.translation_m;
      m.deviation_deg = dev.rotation_deg;
      m.seed_disagreement_m = seed.translation_m;
      m.seed_disagreement_deg = seed.rotation_deg;
    }
    metrics.push_back(std::move(m));
  }

  ProjectDB::Transaction tx(db);
  for (size_t k = 0; k < graph.frames.size(); ++k)
    db.update_sensor_frame_pose(graph.frames[k].node_id,
                                pg::row_major_from_matrix(solved[k]));
  result.edges_with_metrics = db.update_pose_graph_edge_metrics(metrics);
  tx.commit();
  core::info("PlaneGraph: wrote {} optimized poses and metrics for {} stored "
             "pose-graph edges",
             graph.frames.size(), result.edges_with_metrics);
  return result;
}

} // namespace reusex::geometry
