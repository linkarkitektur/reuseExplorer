// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Stable identity of walls across re-runs of the walls stage (solid-model
/// WP3, reusex-z1v.5.5).
///
/// A wall's natural key is its axis endpoints rounded to 5 cm
/// ("walls/<storey idx>/<x0>,<y0>,<x1>,<y1>", reconstruction::wall_records),
/// so an end that moves by a few millimetres across the rounding boundary
/// would mint another GUID. A re-detected wall therefore takes the GUID of
/// the stored wall of the same storey that it lies on: parallel within
/// `match_angle_deg`, each axis within `match_m` of the other's midpoint, and
/// overlapping by at least `min_overlap` of the shorter of the two. Pairs are
/// taken greedily, nearest first. Only an unmatched wall gets a fresh GUID,
/// uuid5(project_guid, key), suffixed "#n" until it collides with no stored
/// or assigned GUID.
///
/// Pure functions over core::WallRecord; no ProjectDB.

#include "reusex/core/solid_model.hpp"

#include <string_view>
#include <vector>

namespace reusex::core {

struct WallIdentityOptions {
  /// A stored wall is the same wall as a new one when each axis passes
  /// within this distance of the other's midpoint [m]...
  double match_m = 0.25;
  /// ...their axes are parallel within this angle [deg]...
  double match_angle_deg = 10.0;
  /// ...and they overlap along the axis by at least this share of the
  /// shorter one.
  double min_overlap = 0.5;
};

struct WallIdentityResult {
  int matched = 0;        ///< new walls that took a stored wall's GUID
  int minted = 0;         ///< new walls that got a fresh GUID
  int locked_matched = 0; ///< of the matched, those whose stored row is locked
};

/// Set the `guid` of every wall in @p fresh. On input each guid is the
/// wall's natural key (reconstruction::wall_records); on output a UUID: the
/// GUID of the stored wall it matches (same storey idx), else
/// uuid5(@p project_guid, key). Each wall_a / wall_b of @p joins that names
/// a key of @p fresh is rewritten to that wall's GUID. Deterministic: the
/// same inputs give the same GUIDs.
/// @throws std::invalid_argument when @p project_guid is not a UUID, or two
///         walls of @p fresh share a key.
WallIdentityResult assign_wall_guids(std::vector<WallRecord> &fresh,
                                     std::vector<WallJoinRecord> &joins,
                                     const std::vector<WallRecord> &stored,
                                     std::string_view project_guid,
                                     const WallIdentityOptions &opt = {});

} // namespace reusex::core
