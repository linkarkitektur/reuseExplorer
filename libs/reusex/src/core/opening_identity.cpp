// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/opening_identity.hpp"

#include "reusex/core/guid.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>

namespace reusex::core {

OpeningIdentityResult assign_opening_guids(
    std::vector<OpeningRecord> &fresh, const std::vector<OpeningRecord> &stored,
    std::string_view project_guid, const OpeningIdentityOptions &opt) {
  // Validates the namespace even when there is nothing to mint.
  (void)deterministic_guid(project_guid, "openings");
  OpeningIdentityResult result;
  std::set<std::string> keys;
  for (const auto &o : fresh)
    if (!keys.insert(o.guid).second)
      throw std::invalid_argument(
          "assign_opening_guids: two openings share the key " + o.guid);

  // (distance, fresh index, stored index), greedy by distance then index.
  std::vector<std::tuple<double, size_t, size_t>> pairs;
  for (size_t i = 0; i < fresh.size(); ++i)
    for (size_t j = 0; j < stored.size(); ++j) {
      const auto &f = fresh[i], &s = stored[j];
      if (f.wall != s.wall)
        continue;
      const double d = std::abs(f.s_start - s.s_start);
      if (d > opt.match_m)
        continue;
      const double overlap =
          std::min(f.s_end, s.s_end) - std::max(f.s_start, s.s_start);
      const double narrower =
          std::min(f.s_end - f.s_start, s.s_end - s.s_start);
      if (!(narrower > 0.0) || overlap < opt.min_overlap * narrower)
        continue;
      pairs.emplace_back(d + std::abs(f.s_end - s.s_end), i, j);
    }
  std::sort(pairs.begin(), pairs.end());

  std::vector<bool> fresh_done(fresh.size(), false);
  std::vector<bool> stored_used(stored.size(), false);
  std::set<std::string> taken;
  for (const auto &s : stored)
    taken.insert(s.guid);
  for (const auto &[d, i, j] : pairs) {
    (void)d;
    if (fresh_done[i] || stored_used[j])
      continue;
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
    fresh[i].guid = guid;
    taken.insert(guid);
    ++result.minted;
  }
  return result;
}

} // namespace reusex::core
