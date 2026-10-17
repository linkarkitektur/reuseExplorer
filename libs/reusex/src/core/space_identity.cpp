// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/space_identity.hpp"

#include "reusex/core/guid.hpp"
#include "reusex/core/space_polygon.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace reusex::core {
namespace {

/// Rounded to 5 cm, printed with two decimals; -0.00 prints as 0.00.
std::string coord(double v) {
  double r = std::round(v / 0.05) * 0.05;
  if (std::abs(r) < 1e-9)
    r = 0.0;
  return fmt::format("{:.2f}", r);
}

double area_of(const SpaceRecord &s) {
  return s.area > 0.0 ? s.area : space_polygon_area(s.polygon);
}

} // namespace

std::string space_natural_key(const SpaceRecord &space) {
  const auto c = space_polygon_centroid(space.polygon);
  return fmt::format("spaces/{}/{},{}", space.storey, coord(c[0]), coord(c[1]));
}

SpaceIdentityResult
assign_space_guids(std::vector<SpaceRecord> &fresh,
                   std::vector<SpaceBoundaryRecord> &boundaries,
                   const std::vector<SpaceRecord> &stored,
                   std::string_view project_guid,
                   const SpaceIdentityOptions &opt) {
  // Validates the namespace even when there is nothing to mint.
  (void)deterministic_guid(project_guid, "spaces");
  SpaceIdentityResult result;
  std::set<std::string> placeholders;
  for (const auto &s : fresh)
    if (!placeholders.insert(s.guid).second)
      throw std::invalid_argument(
          "assign_space_guids: two spaces share the placeholder " + s.guid);

  std::vector<PlanPoint> fc(fresh.size()), sc(stored.size());
  for (size_t i = 0; i < fresh.size(); ++i)
    fc[i] = space_polygon_centroid(fresh[i].polygon);
  for (size_t j = 0; j < stored.size(); ++j)
    sc[j] = space_polygon_centroid(stored[j].polygon);

  // (distance, fresh index, stored index), greedy by distance then index.
  std::vector<std::tuple<double, size_t, size_t>> pairs;
  for (size_t i = 0; i < fresh.size(); ++i)
    for (size_t j = 0; j < stored.size(); ++j) {
      if (fresh[i].storey != stored[j].storey)
        continue;
      const double d = std::hypot(fc[i][0] - sc[j][0], fc[i][1] - sc[j][1]);
      if (d > opt.match_m)
        continue;
      const double a = area_of(fresh[i]), b = area_of(stored[j]);
      const double hi = std::max(a, b);
      if (!(hi > 0.0) || std::min(a, b) < opt.min_area_ratio * hi)
        continue;
      pairs.emplace_back(d, i, j);
    }
  std::sort(pairs.begin(), pairs.end());

  std::map<std::string, std::string> renamed; // placeholder -> GUID
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
    const std::string key = space_natural_key(fresh[i]);
    std::string guid = deterministic_guid(project_guid, key);
    for (int k = 2; taken.count(guid); ++k)
      guid = deterministic_guid(project_guid, fmt::format("{}#{}", key, k));
    renamed[fresh[i].guid] = guid;
    fresh[i].guid = guid;
    taken.insert(guid);
    ++result.minted;
  }
  for (auto &b : boundaries)
    if (const auto it = renamed.find(b.space); it != renamed.end())
      b.space = it->second;
  return result;
}

} // namespace reusex::core
