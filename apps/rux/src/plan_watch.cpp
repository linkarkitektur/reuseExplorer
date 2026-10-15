// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "plan_watch.hpp"

#include <reusex/io/solid_model_json.hpp>

#include <fmt/format.h>

#include <cmath>
#include <exception>

namespace rux {

bool PlanFile::reload() {
  try {
    model_ = reusex::io::solid_model_from_file(path_);
    error_.clear();
    ++generation_;
    return true;
  } catch (const std::exception &e) {
    error_ = e.what();
    return false;
  }
}

namespace {

/// A supported wall whose evidence peaks this far from it is listed [m].
constexpr double kOffWallM = 0.03;

std::string count(std::size_t n, const char *one, const char *many) {
  return fmt::format("{} {}", n, n == 1 ? one : many);
}

std::string mm(const std::optional<double> &m, bool sign = false) {
  if (!m)
    return "n/a";
  return sign ? fmt::format("{:+.0f} mm", *m * 1000.0)
              : fmt::format("{:.0f} mm", *m * 1000.0);
}

} // namespace

std::string plan_counts(const reusex::core::SolidModel &plan) {
  return fmt::format("{}, {}, {}, {}",
                     count(plan.storeys.size(), "storey", "storeys"),
                     count(plan.walls.size(), "wall", "walls"),
                     count(plan.spaces.size(), "space", "spaces"),
                     count(plan.openings.size(), "opening", "openings"));
}

std::string
format_plan_support(const reusex::geometry::PlanSupportReport &report,
                    const reusex::geometry::PlanSupportOptions &opt) {
  if (report.walls.empty())
    return "wall support          n/a (the plan has no walls)\n";
  std::string out = fmt::format(
      "wall support          mean {:.2f}, |face offset| median {}, {} of {} "
      "walls weak (support < {:g})\n",
      report.support_mean, mm(report.face_offset_abs_median_m),
      report.weak_walls, report.walls.size(), opt.weak_below);
  for (const auto &w : report.walls)
    if (w.support_share < opt.weak_below)
      out += fmt::format("  weak wall {:<20} support {:.2f}  offset {}\n",
                         w.guid, w.support_share, mm(w.face_offset_m, true));
  for (const auto &w : report.walls)
    if (w.support_share >= opt.weak_below && w.face_offset_m &&
        std::abs(*w.face_offset_m) >= kOffWallM)
      out += fmt::format("  off  wall {:<20} support {:.2f}  offset {}\n",
                         w.guid, w.support_share, mm(w.face_offset_m, true));
  return out;
}

} // namespace rux
