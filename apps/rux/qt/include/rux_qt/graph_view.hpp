// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Qt-free geometry of the Rediger graph canvas (spec §5.2): how far the view
// may zoom, how a 3D pose lands on the chosen view plane after the view yaw,
// the heading tick, the "Auto" yaw, the yaw's text in project_settings, and
// which of A/B a node click fills. Standard library only (rux_qt_core).

namespace rux::qt {

// ----------------------------------------------------------------- zoom --

/// The closest zoom: one millimetre per screen pixel.
inline constexpr double kMinUnitsPerPx = 0.001;

struct ZoomLimits {
  double min_units_per_px = kMinUnitsPerPx;
  double max_units_per_px = 1.0;
};

/// Limits for a graph @p content_extent metres across shown in a viewport
/// @p viewport_px pixels across: in to kMinUnitsPerPx, out to four graphs
/// (at least 4 m) across the viewport.
ZoomLimits zoom_limits(double content_extent, int viewport_px);

/// The scale factor to apply for a requested @p factor (> 1 zooms in) at
/// @p units_per_px, clamped so the result stays within @p limits. 1 when
/// either input is not positive.
double clamp_zoom_factor(double units_per_px, double factor,
                         const ZoomLimits &limits);

} // namespace rux::qt
