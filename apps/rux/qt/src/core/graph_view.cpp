// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/graph_view.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <numbers>
#include <system_error>

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
  // A view already past a limit (a very wide viewport fits the graph by its
  // height) widens the range to include it: it is never pushed further out,
  // moves back by the step asked, and never against the wheel.
  const double lo = std::min(limits.min_units_per_px, units_per_px);
  const double hi = std::max(limits.max_units_per_px, units_per_px);
  return units_per_px / std::clamp(units_per_px / factor, lo, hi);
}

FitFrame fit_avoiding_corner(const FitRect &c, double vw, double vh, double rw,
                             double rh) {
  const auto fit = [&c](double w, double h) {
    return w > 0.0 && h > 0.0 && c.w > 0.0 && c.h > 0.0
               ? std::min(w / c.w, h / c.h)
               : 0.0;
  };
  const double plain = fit(vw, vh);
  // A viewport not laid out yet, or content with no extent: nothing to fit
  // (and every fit below would be 0, a divisor).
  if (!(plain > 0.0))
    return {1.0, c.cx, c.cy};
  const double above = rh > 0.0 ? fit(vw, vh - rh) : 0.0;
  const double right = rw > 0.0 ? fit(vw - rw, vh) : 0.0;
  // A corner that would cost more than half the scale (a large card in a
  // small pane) is ignored: a graph partly under it beats a speck.
  if (std::max(above, right) < kMinCornerFitRatio * plain)
    return {plain, c.cx, c.cy};
  // Centred in what is left: the viewport's centre is half the corner away
  // from the content's.
  if (above >= right)
    return {above, c.cx, c.cy + rh / 2.0 / above};
  return {right, c.cx - rw / 2.0 / right, c.cy};
}

namespace {
constexpr double kDeg = std::numbers::pi / 180.0;

Vec2 pick(ViewPlane plane, double x, double y, double z) {
  switch (plane) {
  case ViewPlane::xy:
    return {x, y};
  case ViewPlane::xz:
    return {x, z};
  case ViewPlane::yz:
    return {y, z};
  }
  return {x, y};
}
} // namespace

std::string_view view_plane_key(ViewPlane plane) {
  switch (plane) {
  case ViewPlane::xy:
    return "xy";
  case ViewPlane::xz:
    return "xz";
  case ViewPlane::yz:
    return "yz";
  }
  return "xy";
}

std::optional<ViewPlane> parse_view_plane(std::string_view key) {
  for (ViewPlane p : {ViewPlane::xy, ViewPlane::xz, ViewPlane::yz})
    if (view_plane_key(p) == key)
      return p;
  return std::nullopt;
}

Vec2 project_point(const ViewProjection &view, double x, double y, double z) {
  const double a = view.yaw_deg * kDeg;
  const double c = std::cos(a), s = std::sin(a);
  return pick(view.plane, x * c - y * s, x * s + y * c, z);
}

double view_depth(const ViewProjection &view, double x, double y, double z) {
  const double a = view.yaw_deg * kDeg;
  const double c = std::cos(a), s = std::sin(a);
  switch (view.plane) {
  case ViewPlane::xy:
    return z;
  case ViewPlane::xz:
    return x * s + y * c;
  case ViewPlane::yz:
    return x * c - y * s;
  }
  return z;
}

Vec2 reproject_after_yaw(const ViewProjection &view, double new_yaw_deg, Vec2 p,
                         double depth) {
  // Lift into the old view's rotated frame, undo its yaw, project anew.
  double rx = 0, ry = 0, z = 0;
  switch (view.plane) {
  case ViewPlane::xy:
    rx = p.x, ry = p.y, z = depth;
    break;
  case ViewPlane::xz:
    rx = p.x, ry = depth, z = p.y;
    break;
  case ViewPlane::yz:
    rx = depth, ry = p.x, z = p.y;
    break;
  }
  const double a = view.yaw_deg * kDeg;
  const double c = std::cos(a), s = std::sin(a);
  return project_point({view.plane, new_yaw_deg}, rx * c + ry * s,
                       -rx * s + ry * c, z);
}

double median_step(const std::vector<Vec2> &points,
                   const std::vector<int> &scans) {
  std::vector<double> steps;
  steps.reserve(points.size());
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (i < scans.size() && scans[i] != scans[i - 1])
      continue;
    steps.push_back(std::hypot(points[i].x - points[i - 1].x,
                               points[i].y - points[i - 1].y));
  }
  if (steps.empty())
    return 0.0;
  const std::size_t mid = steps.size() / 2;
  std::nth_element(steps.begin(), steps.begin() + mid, steps.end());
  const double upper = steps[mid];
  if (steps.size() % 2)
    return upper;
  const double lower = *std::max_element(steps.begin(), steps.begin() + mid);
  return 0.5 * (lower + upper);
}

bool heading_ticks_visible(double median_step_units, double units_per_px,
                           double tick_px) {
  if (!(median_step_units > 0.0) || !(units_per_px > 0.0))
    return true;
  return median_step_units / units_per_px >= tick_px;
}

std::array<double, 3> camera_forward(const std::array<double, 16> &w,
                                     const std::array<double, 16> &l) {
  // Third column of the local rotation, then the world rotation.
  const double v0 = l[2], v1 = l[6], v2 = l[10];
  return {w[0] * v0 + w[1] * v1 + w[2] * v2, w[4] * v0 + w[5] * v1 + w[6] * v2,
          w[8] * v0 + w[9] * v1 + w[10] * v2};
}

Heading heading_in_view(const ViewProjection &view,
                        const std::array<double, 3> &f) {
  const double n = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  if (!(n > 0.0))
    return {};
  const Vec2 p = project_point(view, f[0] / n, f[1] / n, f[2] / n);
  const double len = std::hypot(p.x, p.y);
  if (len < 0.25)
    return {};
  return {{p.x / len, p.y / len}, true};
}

double auto_yaw_deg(const std::vector<std::array<double, 3>> &forwards) {
  double c = 0.0, s = 0.0;
  for (const auto &f : forwards) {
    const double h = std::hypot(f[0], f[1]);
    const double n = std::hypot(h, f[2]);
    if (!(n > 0.0) || h < 0.5 * n)
      continue; // looks mostly up or down
    const double a = 4.0 * std::atan2(f[1], f[0]);
    c += h / n * std::cos(a);
    s += h / n * std::sin(a);
  }
  if (std::hypot(c, s) < 1e-9)
    return 0.0;
  double yaw = -std::atan2(s, c) / 4.0 / kDeg;
  if (yaw <= -45.0)
    yaw += 90.0;
  if (yaw > 45.0)
    yaw -= 90.0;
  return yaw + 0.0;
}

double normalize_yaw_deg(double deg) {
  if (!std::isfinite(deg))
    return 0.0;
  double r = std::fmod(deg, 360.0);
  if (r <= -180.0)
    r += 360.0;
  else if (r > 180.0)
    r -= 360.0;
  return r + 0.0;
}

std::optional<double> parse_yaw_setting(std::string_view t) {
  while (!t.empty() && t.front() == ' ')
    t.remove_prefix(1);
  while (!t.empty() && t.back() == ' ')
    t.remove_suffix(1);
  double v = 0.0;
  const auto r = std::from_chars(t.data(), t.data() + t.size(), v);
  if (t.empty() || r.ec != std::errc{} || r.ptr != t.data() + t.size() ||
      !std::isfinite(v))
    return std::nullopt;
  return normalize_yaw_deg(v);
}

std::string format_yaw_setting(double deg) {
  const double v =
      normalize_yaw_deg(std::round(normalize_yaw_deg(deg) * 10.0) / 10.0);
  char buf[32];
  const auto r = std::to_chars(buf, buf + sizeof buf, v + 0.0,
                               std::chars_format::fixed, 1);
  return std::string(buf, r.ptr);
}

std::optional<Slot> SlotPicker::click(int id, bool force_b) {
  const Slot target = force_b ? Slot::b : next_;
  if (target == Slot::a) {
    if (id == b_)
      return std::nullopt;
    a_ = id;
    next_ = Slot::b;
    return Slot::a;
  }
  if (id == a_)
    return std::nullopt;
  b_ = id;
  next_ = Slot::a;
  return Slot::b;
}

void SlotPicker::set_pair(int a, int b) {
  a_ = a;
  b_ = b;
  next_ = Slot::a;
}

void SlotPicker::clear() { set_pair(-1, -1); }

bool SlotPicker::forget_missing(const std::vector<int> &frame_ids) {
  const auto gone = [&frame_ids](int id) {
    return id >= 0 &&
           std::find(frame_ids.begin(), frame_ids.end(), id) == frame_ids.end();
  };
  if (!gone(a_) && !gone(b_))
    return true;
  clear();
  return false;
}

} // namespace rux::qt
