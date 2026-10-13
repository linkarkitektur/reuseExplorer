// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/graph_discover.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "slam/pose_graph.hpp"
#include "slam/pose_graph_conventions.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace reusex::slam {
namespace {

namespace pg = core::pose_graph;

/// Canonical stored-row key: (min node, max node, edge_type).
using EdgeKey = std::tuple<int, int, std::string>;

EdgeKey edge_key(int a, int b, std::string type) {
  return {std::min(a, b), std::max(a, b), std::move(type)};
}

/// The edge_type a candidate is stored as.
std::string target_type(const LoopCandidate &c) {
  return std::string(c.source == "panorama" ? pg::kTypePanorama
                                            : pg::kTypeLoopClosure);
}

bool is_loop_type(const std::string &t) {
  return t == pg::kTypeLoopClosure || t == pg::kTypePanorama;
}

} // namespace

GraphDiscoverResult discover_loop_edges(ProjectDB &db,
                                        const GraphDiscoverOptions &options) {
  const auto &c = options.candidates;
  if (!c.loop_closure.enable && !c.panorama_loops.enable &&
      c.loop_edges_file.empty())
    throw std::invalid_argument(
        "graph discover: no edge source selected (ORB loop closure is off, "
        "no --use-panoramas, no --loop-edges)");

  GraphDiscoverResult res;
  if (!options.dry_run &&
      count_usable_odometry_edges(db.list_pose_graph_edges()) == 0) {
    const auto init = init_pose_graph(db);
    res.initialised_graph = true;
    core::info("GraphDiscover: the stored pose graph had no odometry; "
               "initialised {} odometry edges from the current poses "
               "(as 'rux graph init')",
               init.odometry_edges_written);
  }

  // --- Seeds: initial poses (stored convention) and optical seeds ---------
  auto ids = db.sensor_frame_ids();
  std::sort(ids.begin(), ids.end());
  std::vector<int> node_ids;
  std::vector<Eigen::Matrix4d> seed_stored, seed_optical, local;
  int fallback = 0;
  for (int id : ids) {
    std::optional<std::array<double, 16>> p = db.initial_pose(id);
    if (!p) {
      if (!db.has_sensor_frame_pose(id) ||
          !pg::is_rigid_transform(db.sensor_frame_pose(id)))
        continue;
      p = db.sensor_frame_pose(id);
      ++fallback;
    }
    const Eigen::Matrix4d L = pg::matrix_from_row_major(
        db.sensor_frame_intrinsics(id).local_transform);
    const Eigen::Matrix4d S = pg::matrix_from_row_major(*p);
    node_ids.push_back(id);
    seed_stored.push_back(S);
    seed_optical.push_back(S * L);
    local.push_back(L);
  }
  res.frames = static_cast<int>(node_ids.size());
  if (fallback > 0)
    core::warn("GraphDiscover: {} of {} frames have no initial pose; seeded "
               "from their current transform (run 'rux graph init --force' "
               "to snapshot them)",
               fallback, node_ids.size());
  if (node_ids.size() < 2)
    throw std::runtime_error(
        fmt::format("graph discover: fewer than 2 posed sensor frames ({})",
                    node_ids.size()));
  std::map<int, int> index_of;
  for (size_t k = 0; k < node_ids.size(); ++k)
    index_of[node_ids[k]] = static_cast<int>(k);

  // --- Candidates ----------------------------------------------------------
  LoopCandidateReport rep;
  auto cands = collect_loop_candidates(db, node_ids, seed_optical, c, &rep);
  res.proposed = rep.proposed;
  res.candidates = static_cast<int>(cands.size());

  // --- Drop keys the stored graph already holds (any status) -------------
  // Keyed by (pair, edge_type) (spec §4.3 "skips any key that already
  // exists", ruling R10): a stored panorama edge does not block a
  // loop_closure on the same pair, and vice versa.
  const auto stored = db.list_pose_graph_edges();
  std::map<EdgeKey, bool> decided; // key -> tombstone
  for (const auto &s : stored)
    if (is_loop_type(s.edge_type)) {
      bool &tomb = decided[edge_key(s.from_node_id, s.to_node_id, s.edge_type)];
      tomb = tomb || s.status == pg::kStatusRejected;
    }
  std::vector<LoopCandidate> fresh;
  for (auto &cand : cands) {
    const auto key =
        edge_key(node_ids[static_cast<size_t>(cand.edge.i)],
                 node_ids[static_cast<size_t>(cand.edge.j)], target_type(cand));
    if (auto it = decided.find(key); it != decided.end()) {
      (it->second ? res.skipped_rejected : res.skipped_existing)++;
      continue;
    }
    fresh.push_back(std::move(cand));
  }
  if (res.skipped_existing + res.skipped_rejected > 0)
    core::info("GraphDiscover: skipped {} candidates already stored and {} "
               "rejected before (tombstones; un-reject one in Rediger to let "
               "discover propose it again)",
               res.skipped_existing, res.skipped_rejected);

  // --- PCM over stored active loop edges ∪ fresh candidates ---------------
  std::vector<geometry::LoopEdge> pcm_set;
  std::vector<bool> fixed;
  std::vector<int> fresh_index; // -1 for a stored edge
  int stored_unseeded = 0, stored_no_measurement = 0;
  for (const auto &s : stored) {
    if (!is_loop_type(s.edge_type) || s.status == pg::kStatusRejected)
      continue;
    if (!s.transform) {
      ++stored_no_measurement;
      continue;
    }
    const auto a = index_of.find(s.from_node_id);
    const auto b = index_of.find(s.to_node_id);
    if (a == index_of.end() || b == index_of.end()) {
      ++stored_unseeded;
      continue;
    }
    geometry::LoopEdge e;
    e.i = a->second;
    e.j = b->second;
    e.T_ij = stored_to_optical(pg::matrix_from_row_major(*s.transform),
                               local[static_cast<size_t>(e.i)],
                               local[static_cast<size_t>(e.j)]);
    pcm_set.push_back(e);
    fixed.push_back(s.status == pg::kStatusConfirmed);
    fresh_index.push_back(-1);
  }
  if (stored_no_measurement > 0)
    core::warn("GraphDiscover: {} active stored loop edges carry no "
               "measurement (legacy rows) and are left out of the "
               "consistency check; reject them in Rediger / `rux graph prune "
               "--pair`, or re-import the scan to restore their transforms",
               stored_no_measurement);
  if (stored_unseeded > 0)
    core::warn("GraphDiscover: {} stored active loop edges reference a frame "
               "with no usable pose and are left out of the consistency "
               "check; give the frame a pose or reject the edge in Rediger",
               stored_unseeded);
  // pcm_max_edges caps the NEW candidates only (the O(M^2) bound); stored
  // edges always take part, so a large imported graph cannot crowd the
  // candidates out. Capped candidates are never reported as inconsistent.
  geometry::LoopClosureOptions pcm_opt = c.loop_closure;
  if (pcm_opt.pcm && pcm_opt.pcm_max_edges > 0 &&
      fresh.size() > static_cast<size_t>(pcm_opt.pcm_max_edges)) {
    std::vector<size_t> by_support(fresh.size());
    std::iota(by_support.begin(), by_support.end(), size_t{0});
    std::stable_sort(by_support.begin(), by_support.end(),
                     [&](size_t a, size_t b) {
                       return fresh[a].edge.inliers > fresh[b].edge.inliers;
                     });
    by_support.resize(static_cast<size_t>(pcm_opt.pcm_max_edges));
    std::sort(by_support.begin(), by_support.end()); // keep input order
    std::vector<LoopCandidate> capped;
    capped.reserve(by_support.size());
    for (size_t k : by_support)
      capped.push_back(std::move(fresh[k]));
    res.pcm_capped = static_cast<int>(fresh.size() - capped.size());
    fresh = std::move(capped);
    core::warn("GraphDiscover: {} new candidates over pcm_max_edges = {} "
               "(lowest inlier count) were dropped without a consistency "
               "test; raise loop_closure.pcm_max_edges or run discover again "
               "to propose them",
               res.pcm_capped, pcm_opt.pcm_max_edges);
  }
  pcm_opt.pcm_max_edges = 0; // already applied to the candidates above
  for (size_t k = 0; k < fresh.size(); ++k) {
    pcm_set.push_back(fresh[k].edge);
    fixed.push_back(false);
    fresh_index.push_back(static_cast<int>(k));
  }
  std::vector<bool> keep(fresh.size(), true);
  // Ruling R9: a confirmed (fixed) edge makes even a 2-edge set worth a PCM
  // pass — one candidate contradicting it must be rejected.
  const bool any_fixed =
      std::find(fixed.begin(), fixed.end(), true) != fixed.end();
  if (c.loop_closure.pcm && (pcm_set.size() > 2 || any_fixed)) {
    const auto kept_idx = geometry::consistent_loop_edge_indices(
        pcm_set, seed_optical, pcm_opt, fixed);
    std::vector<char> kept(pcm_set.size(), 0);
    for (size_t k : kept_idx)
      kept[k] = 1;
    std::map<std::string, std::pair<int, int>> per_source; // rejected, total
    for (size_t n = 0; n < pcm_set.size(); ++n) {
      const int f = fresh_index[n];
      if (f >= 0) {
        auto &[rej, total] = per_source[fresh[static_cast<size_t>(f)].source];
        ++total;
        if (!kept[n]) {
          keep[static_cast<size_t>(f)] = false;
          ++res.pcm_rejected;
          ++rej;
        }
      } else if (!kept[n] && !fixed[n]) {
        ++res.stored_inconsistent;
      }
    }
    // Same line shape the pre-v27 optimize logged; scripts/bench-loop-edges.sh
    // parses it.
    for (const auto &[source, counts] : per_source)
      if (source != "orb")
        core::info("PlaneGraph: PCM kept {} of {} unioned loop edges; "
                   "{} of {} {} edges rejected as inconsistent",
                   kept_idx.size(), pcm_set.size(), counts.first, counts.second,
                   source == "file" ? "external" : "panorama");
    if (res.stored_inconsistent > 0)
      core::warn("GraphDiscover: PCM finds {} stored auto loop edges "
                 "inconsistent with the rest; they are left unchanged — "
                 "review them in Rediger or 'rux graph prune' them",
                 res.stored_inconsistent);
  } else if (!c.loop_closure.pcm && !fresh.empty()) {
    core::warn("GraphDiscover: PCM disabled (--loop-no-pcm) — {} new loop "
               "edges are stored WITHOUT a consistency filter",
               fresh.size());
  }

  const int survivors =
      static_cast<int>(std::count(keep.begin(), keep.end(), true));
  if (options.dry_run) {
    core::info("GraphDiscover dry-run: {} proposed, {} matched, {} "
               "PCM-rejected, {} over the PCM cap, {} skipped (already "
               "stored), {} skipped (rejected before); {} would be added",
               res.proposed, res.candidates, res.pcm_rejected, res.pcm_capped,
               res.skipped_existing, res.skipped_rejected, survivors);
    return res;
  }

  // --- Store survivors ------------------------------------------------------
  ProjectDB::Transaction tx(db);
  int not_written = 0; // a tombstone appeared under the key (cannot normally)
  for (size_t k = 0; k < fresh.size(); ++k) {
    if (!keep[k])
      continue;
    const auto &f = fresh[k];
    const auto i = static_cast<size_t>(f.edge.i);
    const auto j = static_cast<size_t>(f.edge.j);
    const Eigen::Matrix4d T =
        optical_to_stored(f.edge.T_ij, local[i], local[j]);
    ProjectDB::PoseGraphEdge row;
    row.from_node_id = node_ids[i];
    row.to_node_id = node_ids[j];
    row.edge_type = target_type(f);
    row.transform = pg::row_major_from_matrix(T);
    row.information =
        pg::row_major_from_information(information_optical_to_stored(
            pg::information_from_sigmas(f.edge.sigma_rot, f.edge.sigma_trans),
            local[j]));
    row.source = f.source;
    row.status = std::string(pg::kStatusAuto);
    const auto err = pg::transform_error(T, seed_stored[i], seed_stored[j]);
    row.seed_disagreement_m = err.translation_m;
    row.seed_disagreement_deg = err.rotation_deg;
    if (!db.upsert_pose_graph_edge(row)) {
      ++not_written;
      continue;
    }
    ++res.added;
    if (f.source == "orb")
      ++res.added_orb;
    else if (f.source == "panorama")
      ++res.added_panorama;
    else
      ++res.added_file;
  }
  tx.commit();
  if (not_written > 0)
    core::warn("GraphDiscover: {} surviving candidates were not written "
               "because their key is a tombstone (rejected edge); un-reject "
               "it in Rediger to store it",
               not_written);
  core::info("GraphDiscover: {} proposed, {} matched, {} PCM-rejected, {} "
             "over the PCM cap, {} skipped (already stored), {} skipped "
             "(rejected before), {} added ({} ORB, {} panorama, {} file)",
             res.proposed, res.candidates, res.pcm_rejected, res.pcm_capped,
             res.skipped_existing, res.skipped_rejected, res.added,
             res.added_orb, res.added_panorama, res.added_file);
  return res;
}

} // namespace reusex::slam
