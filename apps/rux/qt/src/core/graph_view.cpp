// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/graph_view.hpp>

#include <algorithm>
#include <cmath>

namespace rux::qt {

ZoomLimits zoom_limits(double content_extent, int viewport_px) {
  ZoomLimits z;
  const double px = static_cast<double>(std::max(viewport_px, 1));
  z.max_units_per_px =
      std::max(std::max(content_extent, 1.0) * 4.0 / px, z.min_units_per_px);
  return z;
}

double clamp_zoom_factor(double units_per_px, double factor,
                         const ZoomLimits &limits) {
  if (!(units_per_px > 0.0) || !(factor > 0.0))
    return 1.0;
  const double target = std::clamp(
      units_per_px / factor, limits.min_units_per_px, limits.max_units_per_px);
  return units_per_px / target;
}

} // namespace rux::qt
