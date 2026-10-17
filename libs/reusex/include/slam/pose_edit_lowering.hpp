// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Pose edits -> solver factors (reusex-rcv.1.3). The format is
// docs/design/pose-edits.md (records core/pose_edits.hpp, reader
// io/pose_edits.hpp); this turns a document into the PosePrior /
// SurfaceConstraint factors PlaneGraphOptimizer::optimize_graph() takes and
// the stored edges a solve leaves out.
//
//   anchor        -> one PosePrior, DoF-masked in the document frame's axes
//   move          -> one PosePrior per node at delta * initial pose, each
//                    with its information divided by n (sigma * sqrt(n)), so
//                    the block carries one anchor's authority
//   relative      -> one SurfaceConstraint with full (DoF-masked) information,
//                    plus a tombstone per edge type for the stored edge
//                    between a and b (either direction), as if reject_edge
//                    had named it
//   same_surface  -> SurfaceConstraints between the frames of a and of b
//                    (up to 12 each, evenly spread), from a point-to-plane
//                    fit of b's region points onto a's plane; information
//                    from each b frame's own points, J^T J / (|A| K sigma^2)
//                    in total one constraint's authority, rank <= 3 (normal
//                    offset + two tilts), so both blocks act as wholes
//   on_plan_wall  -> not lowered yet (reusex-rcv.1.12): warned and skipped
//   reject_edge   -> a tombstone the solve applies to the stored edges
//
// Every factor is in the optimizer's optical convention
// (slam/pose_graph_conventions.hpp). Header is GTSAM-free (STANDARDS §2).

#include <reusex/core/pose_edits.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>

#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::slam {

/// A stored edge an edit leaves out of the solve, canonical (from < to).
struct EdgeTombstone {
  int from = -1;
  int to = -1;
  std::string edge_type;
  std::string tag; ///< the edit's id
};

/// Knobs of the same_surface lowering.
struct SameSurfaceOptions {
  int pixel_step = 4;               ///< depth pixel stride per frame
  int max_points_per_side = 20000;  ///< deterministic stride above this
  int min_points = 100;             ///< plane inliers each side needs
  int min_points_per_frame = 20;    ///< a b frame needs this many to count
  int max_frames_per_side = 12;     ///< frames per side (evenly spread)
  double ransac_distance_m = 0.02;  ///< RANSAC inlier band
  int ransac_iterations = 300;      ///< RANSAC hypotheses (seeded, §6)
  double max_normal_angle_deg = 15; ///< sides must agree this well
  unsigned seed = 42;
};

struct LoweredEdits {
  std::vector<geometry::PosePrior> priors;
  std::vector<geometry::SurfaceConstraint> surfaces;
  std::vector<EdgeTombstone> tombstones;
  /// Ids of edits that were read but not lowered (on_plan_wall), each
  /// warned about with the reason.
  std::vector<std::string> skipped;
  /// Per same_surface edit: the correction its fit found (b onto a), for the
  /// report: offset along the normal (m) and tilt (deg).
  struct SurfaceFit {
    std::string tag;
    double offset_m = 0.0;
    double tilt_deg = 0.0;
    int points_a = 0, points_b = 0;
  };
  std::vector<SurfaceFit> surface_fits;
};

/// Lower @p edits for a solve over the frames @p node_order (node ids, in the
/// solver's index order; PosePrior::i etc. index into it).
///
/// Poses: `current` is the node's `transform`, `initial` its
/// `initial_transform` (falling back to `transform`), and a move is applied
/// to the initial poses (the solve's seed). `frame: storey:<guid>` maps
/// poses, deltas, pivots, regions and DoF axes through that storey's level
/// frame.
///
/// @throws std::runtime_error naming the edit when an edit cannot be lowered:
///         none of its nodes is in @p node_order, an unknown storey, a
///         same_surface side with too few plane points or two sides that are
///         not parallel.
LoweredEdits lower_pose_edits(const ProjectDB &db, const core::PoseEdits &edits,
                              const std::vector<int> &node_order,
                              const SameSurfaceOptions &surface_options = {});

} // namespace reusex::slam
