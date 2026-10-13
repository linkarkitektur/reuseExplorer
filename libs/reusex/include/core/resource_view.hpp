// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// The view record of a resource (docs/superpowers/specs/
/// 2026-10-13-resource-view-record-design.md §2): the one chosen image a
/// survey part is shown by — a sensor frame or a 360° panorama — plus the
/// region of it that shows the part. Pure value type: no ProjectDB, so
/// ProjectDB.hpp, resource keys and ruxd can all share the validation.

#include <array>
#include <optional>
#include <string>
#include <string_view>

namespace reusex::core {

enum class ViewKind { frame, panorama };
std::string_view to_string(ViewKind kind); ///< "frame" | "panorama"
std::optional<ViewKind> view_kind_from_string(std::string_view s);

/// [x, y, w, h], normalised 0..1 of the image (the equirect for a panorama).
using ViewRegion = std::array<double, 4>;

/// Slack on `y + h <= 1` (and a frame's `x + w <= 1`), so a region computed
/// by clamping never fails on rounding. The schema CHECK uses the same value.
inline constexpr double kViewRegionEpsilon = 1e-9;
/// Padding on every side of an automatically chosen region, as a fraction
/// of the region's own width/height (spec §2: "padded 15 % per side").
inline constexpr double kViewRegionPadding = 0.15;

struct ResourceView {
  ViewKind kind = ViewKind::frame;
  int id = 0; ///< sensor_frames.node_id or panoramic_images.id
  std::optional<ViewRegion> region; ///< nullopt: the whole image
  bool operator==(const ResourceView &) const = default;
};

/// @throws std::invalid_argument naming the broken rule: id < 0; a
///         non-finite coordinate; w or h <= 0; x or y < 0; x >= 1; w > 1;
///         y + h > 1; for a frame, x + w > 1 (only a panorama wraps across
///         its seam). Existence of the frame/panorama is not checked here.
void validate_view_shape(const ResourceView &view);

/// Compact JSON, keys in the order kind, id, region (region null for the
/// whole image) — the canonical `sys:view` value.
std::string view_to_json(const ResourceView &view);

/// Parse a `sys:view` value: an object with "kind" and "id" and an optional
/// "region" (null or four numbers); any other field is refused. Validated
/// with validate_view_shape. @throws std::invalid_argument.
ResourceView parse_view(std::string_view json);

/// "Foto #123" for a frame, "360° #4" for a panorama (CSV display).
std::string view_display(const ResourceView &view);

/// @p region grown by @p padding of its own width/height on every side,
/// clamped to [0, 1].
ViewRegion pad_region(const ViewRegion &region,
                      double padding = kViewRegionPadding);

} // namespace reusex::core
