// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/plan_support.hpp"

#include "reusex/geometry/transform_utils.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace reusex::geometry {

namespace {

double median(std::vector<double> v) {
  if (v.empty())
    return 0.0;
  const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
  std::nth_element(v.begin(), mid, v.end());
  if (v.size() % 2 == 1)
    return *mid;
  const double hi = *mid;
  const double lo = *std::max_element(v.begin(), mid);
  return 0.5 * (lo + hi);
}

/// The mean of the values in the fullest histogram bin of width @p bin and
/// its two neighbours (ties: the lowest bin) -- a sub-bin estimate of where
/// the values pile up.
double histogram_peak(const std::vector<double> &values, double bin) {
  if (!(bin > 0.0))
    bin = 0.01;
  std::map<long, std::size_t> counts;
  for (double v : values)
    ++counts[static_cast<long>(std::floor(v / bin))];
  long best = counts.begin()->first;
  std::size_t best_n = 0;
  for (const auto &[k, n] : counts)
    if (n > best_n) {
      best = k;
      best_n = n;
    }
  double sum = 0.0;
  std::size_t n = 0;
  for (double v : values) {
    const long k = static_cast<long>(std::floor(v / bin));
    if (k >= best - 1 && k <= best + 1) {
      sum += v;
      ++n;
    }
  }
  return sum / static_cast<double>(n);
}

} // namespace

PlanSupportReport plan_support(const Cloud &cloud, const core::SolidModel &plan,
                               const PlanSupportOptions &opt) {
  PlanSupportReport report;

  // The cloud in each storey's level frame, built on first use.
  std::map<int, std::vector<Eigen::Vector3d>> level_points;
  const auto points_of =
      [&](int storey) -> const std::vector<Eigen::Vector3d> & {
    auto it = level_points.find(storey);
    if (it != level_points.end())
      return it->second;
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    for (const auto &s : plan.storeys)
      if (s.idx == storey) {
        for (double v : s.level_transform)
          if (!std::isfinite(v))
            throw std::invalid_argument(
                "plan_support: storey " + std::to_string(s.idx) +
                " has a level_transform that is not finite");
        t = to_affine(s.level_transform);
      }
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(cloud.size());
    for (const auto &p : cloud.points)
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
        pts.push_back(t * Eigen::Vector3d(p.x, p.y, p.z));
    return level_points.emplace(storey, std::move(pts)).first->second;
  };

  const double station = opt.station_m > 0.0 ? opt.station_m : 0.10;
  std::vector<double> offsets_of_walls;
  for (const auto &w : plan.walls) {
    WallSupport ws;
    ws.guid = w.guid;
    const Eigen::Vector2d p0(w.x0, w.y0), p1(w.x1, w.y1);
    const double len = (p1 - p0).norm();
    if (!(len > 0.0)) {
      report.walls.push_back(ws);
      continue;
    }
    const Eigen::Vector2d d = (p1 - p0) / len;
    const Eigen::Vector2d n(-d.y(), d.x());
    const double half = w.thickness > 0.0 ? 0.5 * w.thickness : 0.0;
    const double band = half + opt.tolerance_m;
    const double reach = half + opt.search_m;
    const double z_lo = w.z_base + opt.z_margin_m;
    const double z_hi = w.height > 0.0
                            ? w.z_base + w.height - opt.z_margin_m
                            : std::numeric_limits<double>::infinity();

    const auto stations =
        static_cast<std::size_t>(std::max(1.0, std::ceil(len / station)));
    std::vector<int> hits(stations, 0);
    std::vector<double> residuals; // signed, see the header
    for (const auto &p : points_of(w.storey)) {
      if (p.z() < z_lo || p.z() > z_hi)
        continue;
      const Eigen::Vector2d q = p.head<2>() - p0;
      const double t = q.dot(d);
      if (t < 0.0 || t > len)
        continue;
      const double signed_perp = q.dot(n);
      const double perp = std::abs(signed_perp);
      if (perp > reach)
        continue;
      residuals.push_back(half > 0.0 ? perp - half : signed_perp);
      if (perp <= band) {
        ++ws.points;
        const auto i =
            std::min(stations - 1, static_cast<std::size_t>(t / station));
        ++hits[i];
      }
    }
    const auto supported = std::count_if(
        hits.begin(), hits.end(), [&](int h) { return h >= opt.min_points; });
    ws.support_share =
        static_cast<double>(supported) / static_cast<double>(stations);
    if (!residuals.empty()) {
      ws.face_offset_m = histogram_peak(residuals, opt.offset_bin_m);
      offsets_of_walls.push_back(std::abs(*ws.face_offset_m));
    }
    report.walls.push_back(std::move(ws));
  }

  if (!report.walls.empty()) {
    double sum = 0.0;
    for (const auto &ws : report.walls) {
      sum += ws.support_share;
      if (ws.support_share < opt.weak_below)
        ++report.weak_walls;
    }
    report.support_mean = sum / static_cast<double>(report.walls.size());
  }
  if (!offsets_of_walls.empty())
    report.face_offset_abs_median_m = median(std::move(offsets_of_walls));
  return report;
}

} // namespace reusex::geometry
