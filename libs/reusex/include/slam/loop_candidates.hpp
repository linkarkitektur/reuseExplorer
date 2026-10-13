// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The wide-baseline loop-edge FRONT-ENDS (ORB frame pairs #225 P2, 360
// panoramas #236, external matcher files), moved out of
// optimize_sensor_poses when the stored pose graph became the source of truth
// (spec 2026-10-13 §4.3). `rux graph discover` calls this, filters against
// the stored graph and runs PCM over stored ∪ new edges.

#include <reusex/slam/LoopClosure.hpp>
#include <reusex/slam/PanoramaLoopEdges.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::slam {

struct LoopCandidateOptions {
  /// ORB frame-pair detection (`enable` defaults to false here; the
  /// discover options turn it on by default).
  geometry::LoopClosureOptions loop_closure;
  /// Panorama-derived edges (#236), gated by `loop_closure`'s
  /// min_frame_gap / seed-disagreement knobs.
  geometry::PanoramaLoopOptions panorama_loops;
  /// "reusex.loop_edges.v1" JSON from an out-of-process matcher; "" = none.
  std::string loop_edges_file;
  /// Absolute floor (m) of the seed-disagreement gate for file edges; the
  /// gate is max(fraction * trajectory_extent, floor) (#339).
  double loop_edges_min_seed_disagreement = 0.0;
  /// Scale-relative part of that gate; 0.0278 == 0.50 m on the 18.01 m
  /// office scan.
  double loop_edges_min_seed_disagreement_fraction = 0.0278;

  /// Memberwise through the nested option structs (their own `= default`).
  bool operator==(const LoopCandidateOptions &) const = default;
};

struct LoopCandidate {
  geometry::LoopEdge edge; ///< i/j index node_ids; T_ij optical convention
  std::string source;      ///< "orb" | "panorama" | "file"
};

struct LoopCandidateReport {
  int proposed = 0;          ///< pairs the front-ends tried / file edges loaded
  int orb = 0;               ///< ORB edges returned (after detect's own PCM)
  int panorama = 0;          ///< panorama edges kept after pair dedup
  int file_loaded = 0;       ///< valid edges in the file
  int file = 0;              ///< file edges kept after gate + dedup
  int dropped_duplicate = 0; ///< pair already proposed by an earlier source
  int dropped_gate = 0;      ///< file edges agreeing with the seed
};

/// Run the enabled front-ends over @p node_ids (DB node ids, index order)
/// with @p seed_poses (OPTICAL convention: initial pose * local transform).
/// One edge per frame pair: ORB wins over panorama wins over file. No PCM
/// across sources here.
std::vector<LoopCandidate>
collect_loop_candidates(ProjectDB &db, const std::vector<int> &node_ids,
                        const std::vector<Eigen::Matrix4d> &seed_poses,
                        const LoopCandidateOptions &options,
                        LoopCandidateReport *report = nullptr);

} // namespace reusex::slam
