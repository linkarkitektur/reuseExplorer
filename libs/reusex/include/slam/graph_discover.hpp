// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `rux graph discover` (spec 2026-10-13 §4.3): run the loop-edge front-ends
// seeded from the INITIAL poses, drop candidates whose key (pair, edge type)
// the stored graph already holds (including tombstones), PCM over stored
// active loop edges ∪ new candidates with confirmed edges as fixed inliers,
// and store the survivors with status "auto".

#include <reusex/slam/loop_candidates.hpp>

namespace reusex {
class ProjectDB;
}

namespace reusex::slam {

struct GraphDiscoverOptions {
  /// Front-ends; ORB frame-pair detection is ON by default here.
  LoopCandidateOptions candidates = [] {
    LoopCandidateOptions o;
    o.loop_closure.enable = true;
    return o;
  }();
  /// Report what would be added; write nothing (no graph init either).
  bool dry_run = false;

  bool operator==(const GraphDiscoverOptions &) const = default;
};

struct GraphDiscoverResult {
  int frames = 0;       ///< posed frames searched
  int proposed = 0;     ///< pairs the front-ends tried / edges loaded
  int candidates = 0;   ///< edges the front-ends returned
  int pcm_rejected = 0; ///< candidates inconsistent with the rest
  int pcm_capped = 0; ///< lowest-inlier candidates over pcm_max_edges, untested
  int stored_inconsistent = 0; ///< stored auto loop edges PCM disagrees with
  int skipped_existing = 0;    ///< key (pair, type) already stored, active
  int skipped_rejected = 0;    ///< key (pair, type) is a tombstone
  int added = 0;               ///< rows written (0 under dry_run)
  int added_orb = 0;
  int added_panorama = 0;
  int added_file = 0;
  bool initialised_graph = false; ///< odometry was created first
};

/// PCM cost is O(M^2) pairwise cycle tests with M = stored active loop edges
/// + min(new candidates, loop_closure.pcm_max_edges): `pcm_max_edges` caps
/// the NEW candidates only (strongest by inliers kept, the rest reported in
/// `pcm_capped`, never as inconsistent); every stored active loop edge with a
/// measurement always takes part.
///
/// Opens its own ProjectDB::Transaction: do not call inside one.
/// @throws std::invalid_argument when no source is enabled;
///         std::runtime_error when fewer than 2 frames carry a pose.
GraphDiscoverResult discover_loop_edges(ProjectDB &db,
                                        const GraphDiscoverOptions &options);

} // namespace reusex::slam
