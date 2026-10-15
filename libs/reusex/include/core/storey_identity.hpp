// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Stable identity of storeys across re-runs of the storeys stage
/// (solid-model WP1, reusex-z1v.3.3).
///
/// A storey's idx is its rank from the bottom, so it moves when a re-run
/// finds a storey below the old lowest one; its GUID must not. A re-detected
/// storey therefore takes the GUID of the stored storey whose floor it
/// matches, compared in the new run's level frame. Only an unmatched storey
/// gets a fresh GUID: uuid5(project_guid, "storeys/z<floor height in cm>"),
/// suffixed until it collides with no stored or assigned GUID.
///
/// Pure functions over core::StoreyRecord; no ProjectDB.

#include "reusex/core/solid_model.hpp"

#include <array>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

struct StoreyIdentityOptions {
  /// A stored storey is the same storey as a new one when their floors are
  /// within this distance in the new level frame [m].
  double match_m = 0.5;
};

/// Floor height of @p s, expressed in the level frame @p world_to_level
/// (4x4 row-major), measured on the vertical through @p s's frame origin.
double floor_in_frame(const StoreyRecord &s,
                      const std::array<double, 16> &world_to_level);

struct StoreyIdentityResult {
  int matched = 0; ///< new storeys that took a stored storey's GUID
  int minted = 0;  ///< new storeys that got a fresh GUID
};

/// Set the `guid` of every storey in @p fresh: reuse the GUID of the
/// nearest unused stored storey within `match_m` (pairs taken greedily by
/// floor distance, ties by idx), else mint one. Deterministic.
/// @throws std::invalid_argument when @p project_guid is not a UUID.
StoreyIdentityResult assign_storey_guids(
    std::vector<StoreyRecord> &fresh, const std::vector<StoreyRecord> &stored,
    std::string_view project_guid, const StoreyIdentityOptions &opt = {});

/// A locked stored storey that will stand next to a new run's rows.
struct LockedStoreyConflict {
  std::string guid;
  int idx = 0;
  /// its level_transform != that of the new row it stands for (the one
  /// with its GUID, else the one whose floor is nearest)
  bool frame_differs = false;
  double frame_angle_deg = 0.0; ///< rotation between the two frames
  double frame_shift_m = 0.0;   ///< translation between the two frames
  /// idx of new storeys whose [floor, ceiling] overlaps the locked storey's
  /// range (in that new storey's frame); the new storey it is matched to
  /// (same GUID) is not counted.
  std::vector<int> overlapping_idx;
};

/// Locked rows of @p stored whose frame differs from that of the new row
/// they stand for (beyond
/// @p angle_tol_deg / @p shift_tol_m) or whose height range overlaps a row
/// of @p fresh. Empty when @p fresh is empty.
std::vector<LockedStoreyConflict>
locked_storey_conflicts(const std::vector<StoreyRecord> &stored,
                        const std::vector<StoreyRecord> &fresh,
                        double angle_tol_deg = 0.05, double shift_tol_m = 0.01);

} // namespace reusex::core
