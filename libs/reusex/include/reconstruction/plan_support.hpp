// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// How well a solid-model plan's walls follow the scan cloud (reusex-yf4.2).
//
// The self-check of an authored reference plan: `rux analyze solid` scores a
// detected model against the reference, but while the reference itself is
// being drawn the question is "does this wall sit on the cloud?". For every
// wall, the cloud is taken into the wall's storey level frame and looked at in
// a vertical band around the wall's footprint:
//
//  - support share: the fraction of the wall's length (in stations of
//    PlanSupportOptions::station_m) where at least
//    PlanSupportOptions::min_points points lie within the footprint widened
//    by PlanSupportOptions::tolerance_m -- the length the scan confirms;
//  - face offset: where the densest sheet of points near the wall lies
//    relative to the drawn wall -- the peak of a histogram (bins of
//    PlanSupportOptions::offset_bin_m) of the points' distances within
//    PlanSupportOptions::search_m. For a wall of known thickness the distance
//    is measured outward from the nearer face (+ = the evidence lies outside
//    the footprint: the wall is drawn too thin; - = inside: too thick); for
//    one of unknown thickness it is the signed distance from the axis along
//    the wall's left normal (+ = left of the axis seen from (x0, y0) toward
//    (x1, y1)), i.e. how far to move the axis.
//
// Points near the floor and ceiling (PlanSupportOptions::z_margin_m) are left
// out, so the floor slab under a wall drawn in the middle of a room does not
// count as evidence for it. Deterministic: a pure function of the inputs.

#include "reusex/core/solid_model.hpp"
#include "reusex/types/point_types.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace reusex::geometry {

struct PlanSupportOptions {
  double station_m = 0.10;    ///< length of one station along a wall [m]
  int min_points = 3;         ///< points a supported station needs
  double tolerance_m = 0.05;  ///< footprint widening for support [m]
  double search_m = 0.25;     ///< reach of the face-offset measurement [m]
  double offset_bin_m = 0.01; ///< histogram bin of the face offset [m]
  /// Height above the wall's base and below its top that is ignored [m].
  double z_margin_m = 0.30;
  /// A wall whose support share is below this counts as weak.
  double weak_below = 0.5;
};

struct WallSupport {
  std::string guid;
  double support_share = 0.0; ///< 0..1 of the wall's length
  /// Signed offset of the evidence from the drawn wall [m] (see the file
  /// comment); nullopt when no point is within PlanSupportOptions::search_m.
  std::optional<double> face_offset_m;
  std::size_t points = 0; ///< points within the support band
};

struct PlanSupportReport {
  std::vector<WallSupport> walls; ///< in plan order
  double support_mean = 0.0;      ///< mean support share; 0 without walls
  /// Median of the walls' |face offset|; nullopt when no wall has one.
  std::optional<double> face_offset_abs_median_m;
  std::size_t weak_walls = 0; ///< support below PlanSupportOptions::weak_below
};

/// Measure @p plan's walls against @p cloud (world coordinates; each wall is
/// in the level frame of its storey, see core/solid_model.hpp).
/// @throws std::invalid_argument when a storey's level_transform is not
///         finite.
PlanSupportReport plan_support(const Cloud &cloud, const core::SolidModel &plan,
                               const PlanSupportOptions &opt = {});

} // namespace reusex::geometry
