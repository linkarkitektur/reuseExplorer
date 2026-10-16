// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A trajectory is a sequence of timestamped poses (reusex-rcv.1.6). It is the
// shared vocabulary between `io` (TUM file reading/writing, reading a
// project's own poses) and `slam` (ATE/RPE scoring), so, like
// `core/pose_graph.hpp`, it lives in `core` and is Eigen-only (STANDARDS §1:
// slam and io are peers and must not link each other).
//
// Convention: `pose` is a world pose in the `sensor_frames.transform`
// convention (row-major 4x4, column 0 the viewing direction). `timestamp` is
// whatever a TUM file's first column holds: a real epoch time when the
// capture recorded one, or the node id cast to `double` when it did not (as
// on house1, which has no timestamps) -- callers that build a Trajectory from
// a project decide which; `io::read_tum` always takes the file's column
// verbatim. `node_id` is a convenience label (the project's sensor-frame node
// id when known, otherwise the position in the sequence) and is NOT used to
// match two trajectories against each other -- only `timestamp` is, since a
// trajectory round-tripped through a TUM file has no other matching key
// (`slam/pose_metrics.hpp`).

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace reusex::core {

/// One pose sample of a trajectory.
struct StampedPose {
  std::int64_t node_id = 0;
  double timestamp = 0.0;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
};

using Trajectory = std::vector<StampedPose>;

} // namespace reusex::core
