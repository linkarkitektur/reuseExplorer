// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Stable identity of spaces across re-runs of the spaces stage (solid-model
/// WP4, reusex-z1v.6.3).
///
/// A space's natural key is its storey and the centroid of its polygon
/// rounded to 5 cm ("spaces/<storey idx>/<cx>,<cy>", space_natural_key()),
/// so an outline that moves by a few millimetres across the rounding
/// boundary would mint another GUID. A re-detected space therefore takes the
/// GUID of the stored space of the same storey that it lies on: centroids
/// within `match_m` of each other and areas within `min_area_ratio` (smaller
/// / larger). Pairs are taken greedily, nearest first. Only an unmatched
/// space gets a fresh GUID, uuid5(project_guid, key), suffixed "#n" until it
/// collides with no stored or assigned GUID.
///
/// Pure functions over core::SpaceRecord; no ProjectDB.

#include "reusex/core/solid_model.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

struct SpaceIdentityOptions {
  /// A stored space is the same space as a new one of the same storey when
  /// their centroids are within this distance [m]...
  double match_m = 1.0;
  /// ...and the smaller area is at least this share of the larger one.
  double min_area_ratio = 0.5;
};

struct SpaceIdentityResult {
  int matched = 0;        ///< new spaces that took a stored space's GUID
  int minted = 0;         ///< new spaces that got a fresh GUID
  int locked_matched = 0; ///< of the matched, those whose stored row is locked
};

/// "spaces/<storey>/<cx>,<cy>": the polygon's area centroid rounded to 5 cm,
/// two decimals.
std::string space_natural_key(const SpaceRecord &space);

/// Set the `guid` of every space in @p fresh. On input each guid is a
/// placeholder unique within @p fresh (polygonise_spaces' "space-<storey>-
/// <room>"); on output a UUID: the GUID of the stored space it matches
/// (same storey idx), else uuid5(@p project_guid, space_natural_key()). Each
/// SpaceBoundaryRecord::space of @p boundaries that names a placeholder of
/// @p fresh is rewritten to that space's GUID. Deterministic: the same
/// inputs give the same GUIDs.
/// @throws std::invalid_argument when @p project_guid is not a UUID, or two
///         spaces of @p fresh share a placeholder.
SpaceIdentityResult
assign_space_guids(std::vector<SpaceRecord> &fresh,
                   std::vector<SpaceBoundaryRecord> &boundaries,
                   const std::vector<SpaceRecord> &stored,
                   std::string_view project_guid,
                   const SpaceIdentityOptions &opt = {});

} // namespace reusex::core
