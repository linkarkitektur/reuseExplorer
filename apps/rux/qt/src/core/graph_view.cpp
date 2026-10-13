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
  const double target = std::clamp(
      units_per_px / factor, limits.min_units_per_px, limits.max_units_per_px);
  const double f = units_per_px / target;
  // Already past a limit (a very wide viewport fits the graph by its
  // height): a wheel step never moves the opposite way to the one asked.
  if ((factor > 1.0 && f < 1.0) || (factor < 1.0 && f > 1.0))
    return 1.0;
  return f;
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

} // namespace rux::qt
