// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>
#include <string_view>

/// @file pose_fingerprint.hpp
/// @brief Detecting a stored cloud reconstructed from poses the project no
/// longer has (reusex-rcv.1.15).
///
/// `rux create clouds` stamps a cheap fingerprint of every sensor frame's
/// CURRENT pose into the reconstructed cloud's `parameters` JSON
/// (`"pose_fingerprint"`). A later pose change — `rux optimize`, `rux
/// register`, a manual pose edit — leaves the stored cloud built from a pose
/// set the project no longer has, and nothing about the cloud itself looks
/// wrong: it is just baked from the wrong frame placement (NewOffice: median
/// back-projection distance 20 mm against the initial poses it was built
/// from, 51 mm against the current ones). Comparing the stamped fingerprint
/// against the current one (compute_pose_fingerprint()) detects this without
/// re-reading any depth or color data.

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// The `parameters` JSON key `create clouds` stamps the fingerprint under.
inline constexpr std::string_view kPoseFingerprintKey = "pose_fingerprint";

/// A cheap (FNV-1a, not cryptographic), deterministic fingerprint of every
/// sensor frame that currently has a usable stored pose
/// (`ProjectDB::has_sensor_frame_pose()`): the frame id and its
/// `sensor_frame_pose()` matrix, folded in `sensor_frame_ids()`-sorted order
/// so the result does not depend on iteration order. Changes whenever any
/// frame's pose changes, a frame gains or loses a pose, or a frame is added
/// or removed — exactly the input set `reconstruct_point_clouds()` reads.
std::string compute_pose_fingerprint(const ProjectDB &db);

/// Whether a stored cloud was reconstructed from the project's CURRENT poses.
enum class PoseFreshness {
  fresh,   ///< the stamped fingerprint matches the current one
  stale,   ///< the stamped fingerprint differs — poses changed since
  unknown, ///< no such cloud, no stamp (legacy, predates this check), or the
           ///< stamp is unparsable
};

/// Compare @p cloud_name's stamped pose fingerprint (if any) against the
/// project's current one.
PoseFreshness cloud_pose_freshness(const ProjectDB &db,
                                   std::string_view cloud_name);

/// Call wherever @p cloud_name is read as the basis for further geometry (a
/// plane/room/mesh stage, a render, `rux analyze solid`, ...): logs a warning
/// naming the cloud and the fix when `stale`, an info-level note (never a
/// warning — a legacy cloud is not a false alarm) when `unknown`, and logs
/// nothing for `fresh` or when @p cloud_name does not exist.
/// @return cloud_pose_freshness(db, cloud_name).
PoseFreshness warn_if_cloud_poses_stale(const ProjectDB &db,
                                        std::string_view cloud_name);

} // namespace reusex::core
