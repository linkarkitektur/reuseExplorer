// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/loop_candidates.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace reusex::slam {
namespace {

std::pair<int, int> pair_key(const geometry::LoopEdge &e) {
  return {std::min(e.i, e.j), std::max(e.i, e.j)};
}

} // namespace

std::vector<LoopCandidate>
collect_loop_candidates(ProjectDB &db, const std::vector<int> &node_ids,
                        const std::vector<Eigen::Matrix4d> &seed_poses,
                        const LoopCandidateOptions &options,
                        LoopCandidateReport *report) {
  using namespace reusex::geometry;
  LoopCandidateReport rep;
  std::vector<LoopCandidate> out;
  // One factor per pair (two factors would multiply the information): the
  // first source to propose a pair wins, in ORB > panorama > file order —
  // ORB is a direct RANSAC on this scan's own depth.
  std::set<std::pair<int, int>> taken;

  if (options.loop_closure.enable) {
    LoopClosureResult lc;
    auto edges =
        detect_loop_edges(db, node_ids, seed_poses, options.loop_closure, &lc);
    core::info("PlaneGraph: loop closure proposed {} edges from {} matched "
               "candidates",
               lc.edges, lc.candidates);
    rep.proposed += lc.candidates;
    for (auto &e : edges) {
      taken.insert(pair_key(e));
      out.push_back({std::move(e), "orb"});
      ++rep.orb;
    }
  }

  if (options.panorama_loops.enable) {
    PanoramaLoopResult pl;
    // #339: resolve the scale-relative gate here so every source is judged
    // against one gate computed from one extent.
    LoopClosureOptions pano_gates = options.loop_closure;
    pano_gates.min_seed_disagreement = static_cast<float>(
        seed_disagreement_gate(trajectory_extent(seed_poses),
                               pano_gates.min_seed_disagreement_fraction,
                               pano_gates.min_seed_disagreement));
    pano_gates.min_seed_disagreement_fraction = 0.0f;
    auto pano = detect_panorama_loop_edges(
        db, node_ids, seed_poses, options.panorama_loops, pano_gates, &pl);
    if (pl.panoramas == 0)
      core::warn("PlaneGraph: --use-panoramas requested but this project "
                 "contains 0 panoramic images ({} sensor frames); no "
                 "panorama loop edges will be added. Import 360 imagery "
                 "with 'rux import 360' first.",
                 node_ids.size());
    rep.proposed += pl.proposed;
    int dropped_dup = 0;
    for (auto &e : pano) {
      if (!taken.insert(pair_key(e)).second) {
        ++dropped_dup;
        continue;
      }
      out.push_back({std::move(e), "panorama"});
      ++rep.panorama;
    }
    rep.dropped_duplicate += dropped_dup;
    if (dropped_dup > 0)
      core::info("PlaneGraph: dropped {} panorama loop edges duplicating an "
                 "internally-detected pair (avoids double factors)",
                 dropped_dup);
    core::info("PlaneGraph: {} panorama loop edges kept from {}/{} panoramas "
               "({} frame resections, {} pairs proposed; dropped {} gap / "
               "{} seed-gate / {} per-panorama cap)",
               rep.panorama, pl.panoramas_matched, pl.panoramas,
               pl.frames_resected, pl.proposed, pl.dropped_gap,
               pl.dropped_seed_gate, pl.dropped_cap);
  }

  if (!options.loop_edges_file.empty()) {
    LoopClosureResult ext;
    auto file_edges = load_loop_edges(options.loop_edges_file, node_ids, &ext);
    core::info("PlaneGraph: {} external loop edges loaded from '{}'", ext.edges,
               options.loop_edges_file);
    rep.file_loaded = ext.edges;
    rep.proposed += ext.edges;

    // An external edge agreeing with the seed carries no drift information
    // and only injects matcher noise (honka laser-GT 0.7958 -> 0.62); #339
    // scales the gate with the capture.
    const double extent = trajectory_extent(seed_poses);
    const double gate = seed_disagreement_gate(
        extent, options.loop_edges_min_seed_disagreement_fraction,
        options.loop_edges_min_seed_disagreement);
    core::info("PlaneGraph: trajectory extent {:.2f} m -> external "
               "seed-disagreement gate {:.3f} m (fraction {:.5f}, floor "
               "{:.3f} m)",
               extent, gate, options.loop_edges_min_seed_disagreement_fraction,
               options.loop_edges_min_seed_disagreement);

    int dropped = 0, dropped_dup = 0;
    for (auto &e : file_edges) {
      if (gate > 0.0 && e.i >= 0 && e.j >= 0) {
        const Eigen::Matrix4d rel_seed =
            seed_poses[e.i].inverse() * seed_poses[e.j];
        const double disagreement =
            (rel_seed.block<3, 1>(0, 3) - e.T_ij.block<3, 1>(0, 3)).norm();
        if (disagreement < gate) {
          ++dropped;
          continue;
        }
      }
      if (!taken.insert(pair_key(e)).second) {
        ++dropped_dup;
        continue;
      }
      out.push_back({std::move(e), "file"});
      ++rep.file;
    }
    rep.dropped_gate += dropped;
    rep.dropped_duplicate += dropped_dup;
    if (dropped > 0)
      core::info("PlaneGraph: dropped {} external loop edges that agree with "
                 "the seed within {:.3f} m (non-informative)",
                 dropped, gate);
    if (dropped_dup > 0)
      core::info("PlaneGraph: dropped {} external loop edges duplicating an "
                 "internally-detected pair (avoids double factors)",
                 dropped_dup);
    core::info("PlaneGraph: {} external loop edges kept after gating",
               rep.file);
  }

  if (report)
    *report = rep;
  return out;
}

} // namespace reusex::slam
