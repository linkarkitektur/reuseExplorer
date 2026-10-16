// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/wall_identity.hpp"
#include "core/guid.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace reusex::core {
namespace {

struct Axis {
  double ax, ay, bx, by; // endpoints
  double dx, dy, len;    // unit direction, length
  double mx, my;         // midpoint
};

Axis axis_of(const WallRecord &w) {
  Axis a{w.x0, w.y0, w.x1, w.y1, w.x1 - w.x0, w.y1 - w.y0, 0.0, 0.0, 0.0};
  a.len = std::hypot(a.dx, a.dy);
  if (a.len > 0.0) {
    a.dx /= a.len;
    a.dy /= a.len;
  }
  a.mx = 0.5 * (w.x0 + w.x1);
  a.my = 0.5 * (w.y0 + w.y1);
  return a;
}

/// Distance of (px, py) from the infinite line through @p a.
double line_distance(const Axis &a, double px, double py) {
  if (a.len <= 0.0)
    return std::hypot(px - a.ax, py - a.ay);
  return std::abs((px - a.ax) * a.dy - (py - a.ay) * a.dx);
}

/// How alike @p f and @p s are; < 0 when they are not the same wall.
double match_distance(const Axis &f, const Axis &s,
                      const WallIdentityOptions &opt) {
  if (f.len <= 0.0 || s.len <= 0.0)
    return -1.0;
  const double cos_angle = std::abs(f.dx * s.dx + f.dy * s.dy);
  const double max_angle = opt.match_angle_deg * 3.14159265358979323846 / 180.0;
  if (cos_angle < std::cos(max_angle))
    return -1.0;
  const double d_fs = line_distance(f, s.mx, s.my);
  const double d_sf = line_distance(s, f.mx, f.my);
  if (d_fs > opt.match_m || d_sf > opt.match_m)
    return -1.0;
  // Overlap along f's direction.
  const auto proj = [&](double x, double y) {
    return (x - f.ax) * f.dx + (y - f.ay) * f.dy;
  };
  const double s0 = std::min(proj(s.ax, s.ay), proj(s.bx, s.by));
  const double s1 = std::max(proj(s.ax, s.ay), proj(s.bx, s.by));
  const double overlap = std::min(f.len, s1) - std::max(0.0, s0);
  if (overlap < opt.min_overlap * std::min(f.len, s.len))
    return -1.0;
  // Offset between the axes plus how far the ends moved.
  const double ends = std::min(std::hypot(f.ax - s.ax, f.ay - s.ay) +
                                   std::hypot(f.bx - s.bx, f.by - s.by),
                               std::hypot(f.ax - s.bx, f.ay - s.by) +
                                   std::hypot(f.bx - s.ax, f.by - s.ay));
  return 0.5 * (d_fs + d_sf) + 0.5 * ends;
}

} // namespace

WallIdentityResult assign_wall_guids(std::vector<WallRecord> &fresh,
                                     std::vector<WallJoinRecord> &joins,
                                     const std::vector<WallRecord> &stored,
                                     std::string_view project_guid,
                                     const WallIdentityOptions &opt) {
  // Validates the namespace even when there is nothing to mint.
  (void)deterministic_guid(project_guid, "walls");
  WallIdentityResult result;
  std::set<std::string> keys;
  for (const auto &w : fresh)
    if (!keys.insert(w.guid).second)
      throw std::invalid_argument(
          "assign_wall_guids: two walls share the key " + w.guid);

  std::vector<Axis> fa, sa;
  for (const auto &w : fresh)
    fa.push_back(axis_of(w));
  for (const auto &w : stored)
    sa.push_back(axis_of(w));
  // (distance, fresh index, stored index): greedy by distance, ties by
  // position (both inputs come in a deterministic order).
  std::vector<std::tuple<double, size_t, size_t>> pairs;
  for (size_t i = 0; i < fresh.size(); ++i)
    for (size_t j = 0; j < stored.size(); ++j) {
      if (fresh[i].storey != stored[j].storey)
        continue;
      const double d = match_distance(fa[i], sa[j], opt);
      if (d >= 0.0)
        pairs.emplace_back(d, i, j);
    }
  std::sort(pairs.begin(), pairs.end());

  std::map<std::string, std::string> renamed; // key -> GUID
  std::vector<bool> fresh_done(fresh.size(), false);
  std::vector<bool> stored_used(stored.size(), false);
  std::set<std::string> taken;
  for (const auto &s : stored)
    taken.insert(s.guid);
  for (const auto &[d, i, j] : pairs) {
    (void)d;
    if (fresh_done[i] || stored_used[j])
      continue;
    renamed[fresh[i].guid] = stored[j].guid;
    fresh[i].guid = stored[j].guid;
    fresh_done[i] = stored_used[j] = true;
    ++result.matched;
    if (stored[j].locked)
      ++result.locked_matched;
  }

  for (size_t i = 0; i < fresh.size(); ++i) {
    if (fresh_done[i])
      continue;
    const std::string key = fresh[i].guid;
    std::string guid = deterministic_guid(project_guid, key);
    for (int k = 2; taken.count(guid); ++k)
      guid = deterministic_guid(project_guid, fmt::format("{}#{}", key, k));
    renamed[key] = guid;
    fresh[i].guid = guid;
    taken.insert(guid);
    ++result.minted;
  }

  for (auto &j : joins) {
    if (auto it = renamed.find(j.wall_a); it != renamed.end())
      j.wall_a = it->second;
    if (auto it = renamed.find(j.wall_b); it != renamed.end())
      j.wall_b = it->second;
  }
  return result;
}

} // namespace reusex::core
