// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Stable identity of openings across re-runs of the openings stage
/// (solid-model WP5, reusex-z1v.7.3).
///
/// An opening's natural key is its host wall's GUID and its s_start rounded
/// to 5 cm ("openings/<wall guid>/<s>", reconstruction::detect_openings), so
/// a jamb that moves a few millimetres across the rounding boundary would
/// mint another GUID. A re-detected opening therefore takes the GUID of the
/// stored opening in the same host wall that it overlaps: starts within
/// `match_m` of each other and overlapping along the wall by at least
/// `min_overlap` of the narrower one. Pairs are taken greedily, nearest
/// first. Only an unmatched opening gets a fresh GUID, uuid5(project_guid,
/// key), suffixed "#n" until it collides with no stored or assigned GUID.
///
/// Walls keep their GUIDs across a walls re-run (core/wall_identity.hpp), so
/// an opening keeps its own across both stages.
///
/// Pure functions over core::OpeningRecord; no ProjectDB.

#include "reusex/core/solid_model.hpp"

#include <string_view>
#include <vector>

namespace reusex::core {

struct OpeningIdentityOptions {
  /// A stored opening is the same opening as a new one in the same wall when
  /// their starts are within this of each other [m]...
  double match_m = 0.25;
  /// ...and they overlap along the wall by at least this share of the
  /// narrower one.
  double min_overlap = 0.5;
};

struct OpeningIdentityResult {
  int matched = 0;        ///< new openings that took a stored GUID
  int minted = 0;         ///< new openings that got a fresh GUID
  int locked_matched = 0; ///< of the matched, those whose stored row is locked
};

/// Set the `guid` of every opening in @p fresh. On input each guid is the
/// opening's natural key; on output a UUID: the GUID of the stored opening
/// it matches (same host wall), else uuid5(@p project_guid, key).
/// Deterministic: the same inputs give the same GUIDs.
/// @throws std::invalid_argument when @p project_guid is not a UUID, or two
///         openings of @p fresh share a key.
OpeningIdentityResult assign_opening_guids(
    std::vector<OpeningRecord> &fresh, const std::vector<OpeningRecord> &stored,
    std::string_view project_guid, const OpeningIdentityOptions &opt = {});

} // namespace reusex::core
