// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <filesystem>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

/// Import raw sensor data from an RTABMap database into a ProjectDB.
///
/// For each node the function extracts color, depth, confidence, the optimized
/// world pose, and camera intrinsics in their original RTABMap format and
/// stores everything in the project database. Images are stored in their native
/// orientation without rotation.
///
/// Also stores RTABMap's graph links in `pose_graph_edges`
/// (Neighbor/NeighborMerged → odometry, Global/LocalSpace/LocalTime/User
/// closures → loop_closure, source `rtabmap`) and each imported pose in
/// `sensor_frames.initial_transform`.
///
/// No heavy processing (voxelization, normal estimation, etc.) is performed.
/// Point cloud generation is handled separately by
/// reusex::geometry::reconstruct_point_clouds().
///
/// @param db  Target project database (must be open in write mode).
/// @param rtabmap_db_path  Path to the source RTABMap .db file.
void import_rtabmap(ProjectDB &db,
                    const std::filesystem::path &rtabmap_db_path);
} // namespace reusex::io
