// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// The stored form of a space's footprint (spaces.polygon, schema v35,
/// reusex-z1v.6.3) and the plain metrics core needs of it.
///
/// Blob layout, little-endian: uint32 ring count (>= 1), then per ring a
/// uint32 point count (>= 3) and that many (double x, double y) pairs. The
/// first ring is the outer ring, the rest are holes; rings are open (the
/// first point is not repeated), as in core::SpacePolygon.

#include "reusex/core/solid_model.hpp"

#include <cstdint>
#include <vector>

namespace reusex::core {

/// @throws std::invalid_argument for an outer ring or hole with fewer than
///         3 points, or a coordinate that is not finite.
std::vector<uint8_t> encode_space_polygon(const SpacePolygon &polygon);

/// @throws std::runtime_error for a blob that is empty, truncated, has
///         trailing bytes, or a ring with fewer than 3 points.
SpacePolygon decode_space_polygon(const std::vector<uint8_t> &blob);

/// Area of the outer ring minus the holes [m^2], whatever their orientation.
double space_polygon_area(const SpacePolygon &polygon);

/// Area centroid of the outer ring minus the holes. The vertex mean when the
/// area is degenerate (zero).
PlanPoint space_polygon_centroid(const SpacePolygon &polygon);

} // namespace reusex::core
