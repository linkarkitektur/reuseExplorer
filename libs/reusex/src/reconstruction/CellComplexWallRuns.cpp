// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// CellComplex::mark_wall_runs(): which slab cells lie on their wall's
// measured run (reusex-9a8).

#include "core/logging.hpp"
#include "reconstruction/CellComplex.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace reusex::geometry {

auto CellComplex::mark_wall_runs(const std::vector<WallRun> &runs,
                                 double end_margin_m) -> WallRunStats {
  if (runs.size() > n_walls)
    throw std::invalid_argument(
        fmt::format("CellComplex::mark_wall_runs: {} runs for {} wall pairs",
                    runs.size(), n_walls));
  WallRunStats stats;
  std::vector<bool> limited(runs.size(), false);
  for (size_t i = 0; i < runs.size(); ++i) {
    limited[i] = !runs[i].axis.isZero() && !runs[i].spans.empty();
    stats.pairs_measured += limited[i] ? 1 : 0;
  }
  if (stats.pairs_measured == 0)
    return stats;

  auto volume = this->property_map<Vertex, double>("c:volume");
  for (auto cit = cells_begin(); cit != cells_end(); ++cit) {
    auto &ids = std::get<CellData>((*this)[*cit].data).run_wall_ids;
    bool off = false;
    for (auto it = ids.begin(); it != ids.end();) {
      const auto w = static_cast<size_t>(*it);
      if (w >= runs.size() || !limited[w]) {
        ++it;
        continue;
      }
      const auto &run = runs[w];
      // The cell's extent along the axis: over the corners of its faces.
      double lo = std::numeric_limits<double>::infinity();
      double hi = -lo;
      for (auto f = faces_of_cell_begin(*cit); f != faces_of_cell_end(*cit);
           ++f)
        for (auto v = vertices_of_face_begin(*f); v != vertices_of_face_end(*f);
             ++v) {
          const double s = run.axis.dot((*this)[*v].pos.head<2>());
          lo = std::min(lo, s);
          hi = std::max(hi, s);
        }
      const bool on_run =
          std::any_of(run.spans.begin(), run.spans.end(), [&](const auto &sp) {
            return lo < sp[1] + end_margin_m && hi > sp[0] - end_margin_m;
          });
      if (on_run) {
        ++it;
        continue;
      }
      it = ids.erase(it);
      off = true;
    }
    if (off) {
      ++stats.cells_off_run;
      stats.volume_off_run += volume[*cit];
    }
  }
  reusex::debug("CellComplex: {} of {} wall pair(s) measured (runs +-{:.2f} "
                "m); {} slab cell(s), {:.1f} m^3 beyond a run",
                stats.pairs_measured, n_walls, end_margin_m,
                stats.cells_off_run, stats.volume_off_run);
  return stats;
}

} // namespace reusex::geometry
