// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// TUM-format trajectory I/O (reusex-rcv.1.6), the file format a pose
// reference is frozen into
// (`tests/fixtures/pose_reference/<scan>.trajectory.tum`,
// docs/guides/pose-reference-authoring.md) and that `rux export trajectory`
// writes / `rux analyze poses` reads.
//
// One line per pose: "<timestamp> tx ty tz qx qy qz qw" (space separated,
// the de facto TUM RGB-D benchmark convention), written with full double
// precision (`std::numeric_limits<double>::max_digits10`) so a round trip is
// exact, not merely close. Blank lines and lines starting with '#' are
// skipped on read. `node_id` is not a TUM column -- `read_tum` sets it to the
// 0-based row index; see `core/trajectory.hpp` for why that is fine.

#include <reusex/core/trajectory.hpp>

#include <filesystem>

namespace reusex {
class ProjectDB;
} // namespace reusex

namespace reusex::io {

/// Write a trajectory as TUM text. Throws std::runtime_error if the file
/// cannot be opened for writing.
void write_tum(const core::Trajectory &trajectory,
               const std::filesystem::path &path);

/// Parse a TUM trajectory file.
///
/// @throws std::runtime_error naming the line number and the problem (wrong
///         column count, a non-finite value, a quaternion with ~zero norm)
///         for the first malformed line.
core::Trajectory read_tum(const std::filesystem::path &path);

/// Build a Trajectory from a project's own stored sensor-frame poses, ordered
/// by node id. A frame with no usable stored pose is skipped (and warned
/// about, STANDARDS §5) rather than contributing an identity pose that would
/// silently corrupt a comparison.
///
/// `timestamp` is `db.sensor_frame_timestamp(id)` when set (>= 0), otherwise
/// the node id cast to `double` -- the house1 case the module comment
/// describes.
///
/// @param use_initial read `initial_transform` (the pose `rux optimize` seeds
///                     from) instead of the current `transform`. A frame
///                     without an initial pose is skipped the same way.
/// @throws std::runtime_error if the project has no sensor frames at all.
core::Trajectory trajectory_from_project(const ProjectDB &db,
                                         bool use_initial = false);

} // namespace reusex::io
