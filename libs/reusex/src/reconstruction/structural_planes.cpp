// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reconstruction/structural_planes.hpp"
#include "core/logging.hpp"
#include "utils/tolerances.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace reusex::geometry {

namespace {

/// |n . up| below this is a wall, above 1 - this a floor or ceiling. Same
/// split as separate_planes(), so this step sees the planes the cell complex
/// will see.
constexpr double kOrientationEps = 0.1;

using CellSet = std::unordered_set<std::uint64_t>;

std::uint64_t cell_key(double a, double b, double cell) {
  const auto i = static_cast<std::int64_t>(std::floor(a / cell));
  const auto j = static_cast<std::int64_t>(std::floor(b / cell));
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(i)) << 32) |
         static_cast<std::uint32_t>(j);
}

/// Unit vector along a vertical plane `n`, horizontal.
Eigen::Vector3d along_wall(const Eigen::Vector3d &n) {
  Eigen::Vector3d u = Eigen::Vector3d::UnitZ().cross(n);
  if (u.norm() < kEpsilonRelative)
    u = Eigen::Vector3d::UnitX();
  return u.normalized();
}

/// Points per occupied cell of a plane's inliers in its own 2D frame:
/// (along-wall, z) for a wall, (x, y) for a floor or ceiling.
void cell_counts(const Cloud &cloud, const Indices &idx,
                 const Eigen::Vector3d &n, bool vertical, double cell,
                 std::vector<int> &out) {
  std::unordered_map<std::uint64_t, int> counts;
  const Eigen::Vector3d u = along_wall(n);
  for (int i : idx) {
    const Eigen::Vector3d p = cloud.points[i].getVector3fMap().cast<double>();
    ++counts[vertical ? cell_key(p.dot(u), p.z(), cell)
                      : cell_key(p.x(), p.y(), cell)];
  }
  for (const auto &[k, c] : counts)
    out.push_back(c);
}

/// Occupied xy cells of a plane's inliers.
CellSet xy_cells(const Cloud &cloud, const Indices &idx, double cell) {
  CellSet out;
  for (int i : idx) {
    const auto &p = cloud.points[i];
    out.insert(cell_key(p.x, p.y, cell));
  }
  return out;
}

/// Footprint cells dilated by one cell in x and y.
CellSet dilate(const CellSet &cells) {
  CellSet out;
  for (auto k : cells) {
    const auto i = static_cast<std::int32_t>(k >> 32);
    const auto j = static_cast<std::int32_t>(k & 0xffffffffU);
    for (int di = -1; di <= 1; ++di)
      for (int dj = -1; dj <= 1; ++dj)
        out.insert(
            (static_cast<std::uint64_t>(static_cast<std::uint32_t>(i + di))
             << 32) |
            static_cast<std::uint32_t>(j + dj));
  }
  return out;
}

} // namespace

auto select_structural_planes(EigenVectorContainer<double, 4> &planes,
                              std::vector<IndicesPtr> &inliers,
                              EigenVectorContainer<double, 3> &centroids,
                              CloudConstPtr cloud,
                              const StructuralPlaneOptions &opt)
    -> StructuralPlaneReport {
  if (planes.size() != inliers.size() || planes.size() != centroids.size())
    throw std::runtime_error(fmt::format(
        "select_structural_planes: input sizes disagree (planes={}, "
        "inliers={}, centroids={})",
        planes.size(), inliers.size(), centroids.size()));
  if (!cloud)
    throw std::runtime_error("select_structural_planes: cloud is null");

  StructuralPlaneReport report;
  const std::size_t n = planes.size();
  if (!opt.enabled) {
    report.planes_out = n;
    return report;
  }

  std::vector<std::size_t> verticals, horizontals;
  std::vector<bool> keep(n, true);
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d nrm = planes[i].head<3>().normalized();
    const double up = std::abs(nrm.z());
    if (up < kOrientationEps)
      verticals.push_back(i);
    else if (up > 1.0 - kOrientationEps)
      horizontals.push_back(i);
    else
      ++report.other;
  }
  report.vertical_in = verticals.size();
  report.horizontal_in = horizontals.size();

  // ── Surface area per inlier point ───────────────────────────────────────
  // A plane's surface area is its inlier count times the area one point
  // stands for, cell^2 / (median points per occupied cell) over all walls,
  // floors and ceilings. Counting points rather than occupied cells keeps a
  // sparse patch (a few hundred stray points strung along a line) from
  // reading as a large surface, and the median ties the measure to this
  // scan's own density.
  double area_per_point = opt.area_cell_m * opt.area_cell_m;
  {
    std::vector<int> counts;
    for (auto i : verticals)
      cell_counts(*cloud, *inliers[i], planes[i].head<3>().normalized(), true,
                  opt.area_cell_m, counts);
    for (auto i : horizontals)
      cell_counts(*cloud, *inliers[i], planes[i].head<3>().normalized(), false,
                  opt.area_cell_m, counts);
    if (!counts.empty()) {
      auto mid =
          counts.begin() + static_cast<std::ptrdiff_t>(counts.size() / 2);
      std::nth_element(counts.begin(), mid, counts.end());
      area_per_point /= static_cast<double>(std::max(*mid, 1));
    }
  }
  report.area_per_point_m2 = area_per_point;
  auto area_of = [&](std::size_t i) {
    return static_cast<double>(inliers[i]->size()) * area_per_point;
  };

  // ── Vertical planes: one arrangement line per wall line ─────────────────
  // Visit planes by area (largest first, index breaks ties) so each line is
  // represented by its best-supported piece.
  {
    std::vector<std::size_t> order = verticals;
    std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
      return inliers[a]->size() > inliers[b]->size();
    });

    const double cos_tol =
        std::cos(opt.collinear_angle_deg * std::numbers::pi / 180.0);
    std::vector<std::size_t> leaders;
    for (auto i : order) {
      const Eigen::Vector3d ni = planes[i].head<3>().normalized();
      bool absorbed = false;
      for (auto g : leaders) {
        const Eigen::Vector3d ng = planes[g].head<3>().normalized();
        if (ng.dot(ni) < cos_tol)
          continue; // not parallel, or facing the other way
        const double off = std::abs(ng.dot(centroids[i]) +
                                    planes[g][3] / planes[g].head<3>().norm());
        if (off >= opt.collinear_offset_m)
          continue;
        // Same wall line: the leader keeps its plane and takes the inliers.
        auto merged = std::make_shared<Indices>(*inliers[g]);
        merged->insert(merged->end(), inliers[i]->begin(), inliers[i]->end());
        inliers[g] = merged;
        keep[i] = false;
        ++report.vertical_merged;
        absorbed = true;
        break;
      }
      if (!absorbed)
        leaders.push_back(i);
    }
    // A wall's support is its own face plus the opposite face of the same
    // wall, when there is one within wall_pair_max_m behind it: a short thin
    // wall seen from both sides is a wall even if neither face alone reaches
    // the threshold. A lone jamb has no such partner.
    auto support = [&](std::size_t g) {
      const Eigen::Vector3d ng = planes[g].head<3>().normalized();
      const double dg = planes[g][3] / planes[g].head<3>().norm();
      double best = 0.0;
      for (auto h : leaders) {
        const Eigen::Vector3d nh = planes[h].head<3>().normalized();
        if (-ng.dot(nh) < cos_tol)
          continue; // not the opposite face
        const double gap = std::abs(ng.dot(centroids[h]) + dg);
        if (gap < opt.wall_pair_max_m)
          best = std::max(best, area_of(h));
      }
      return area_of(g) + best;
    };
    std::vector<std::size_t> small;
    for (auto g : leaders)
      if (support(g) < opt.min_wall_area_m2)
        small.push_back(g);
    for (auto g : small) {
      reusex::debug("structural planes: wall line {} ({:.2f} m^2 with its "
                    "opposite face) below {:.2f} m^2, not an arrangement line",
                    g, support(g), opt.min_wall_area_m2);
      keep[g] = false;
      ++report.vertical_small;
    }
  }

  // ── Horizontal planes: storey floors and ceilings only ──────────────────
  {
    struct H {
      std::size_t id;
      double height;
      bool up; // faces up: a floor or a table top
      double area;
      CellSet footprint;
    };
    std::vector<H> hs;
    for (auto i : horizontals) {
      const Eigen::Vector3d nrm = planes[i].head<3>();
      H h;
      h.id = i;
      h.up = nrm.z() > 0.0;
      // n . x + d = 0 with n ~ (0, 0, +-1): height = -d / n_z.
      h.height = -planes[i][3] / nrm.z();
      h.area = area_of(i);
      h.footprint = xy_cells(*cloud, *inliers[i], opt.footprint_cell_m);
      hs.push_back(std::move(h));
    }
    std::vector<CellSet> dilated(hs.size());
    for (std::size_t q = 0; q < hs.size(); ++q)
      if (hs[q].up)
        dilated[q] = dilate(hs[q].footprint);

    for (std::size_t p = 0; p < hs.size(); ++p) {
      const auto &P = hs[p];
      if (P.footprint.empty())
        continue;
      for (std::size_t q = 0; q < hs.size(); ++q) {
        const auto &Q = hs[q];
        if (q == p || !Q.up || Q.area < P.area)
          continue;
        const double rise = P.height - Q.height;
        if (rise <= kToleranceCoplanarM || rise >= opt.min_storey_clearance_m)
          continue;
        std::size_t over = 0;
        for (auto k : P.footprint)
          over += dilated[q].count(k);
        const double frac =
            static_cast<double>(over) / static_cast<double>(P.footprint.size());
        if (frac < opt.min_footprint_overlap)
          continue;
        reusex::debug("structural planes: horizontal plane {} ({} at {:.2f} "
                      "m, {:.1f} m^2) is {:.2f} m above plane {} ({:.1f} m^2, "
                      "{:.0f}% under it): furniture, not a storey section",
                      P.id, P.up ? "up" : "down", P.height, P.area, rise, Q.id,
                      Q.area, 100.0 * frac);
        keep[P.id] = false;
        ++report.horizontal_furniture;
        break;
      }
    }
  }

  // ── Compact, keeping input order ────────────────────────────────────────
  EigenVectorContainer<double, 4> out_planes;
  std::vector<IndicesPtr> out_inliers;
  EigenVectorContainer<double, 3> out_centroids;
  for (std::size_t i = 0; i < n; ++i) {
    if (!keep[i])
      continue;
    out_planes.push_back(planes[i]);
    out_inliers.push_back(inliers[i]);
    out_centroids.push_back(centroids[i]);
  }
  planes = std::move(out_planes);
  inliers = std::move(out_inliers);
  centroids = std::move(out_centroids);
  report.planes_out = planes.size();

  reusex::info(
      "structural planes: {} vertical -> {} wall lines ({} merged onto a "
      "collinear line, {} below {:.1f} m^2 dropped); {} horizontal -> {} "
      "section planes ({} furniture-level dropped, < {:.1f} m over a larger "
      "floor); {} oblique passed through; {:.5f} m^2 per point",
      report.vertical_in,
      report.vertical_in - report.vertical_merged - report.vertical_small,
      report.vertical_merged, report.vertical_small, opt.min_wall_area_m2,
      report.horizontal_in, report.horizontal_in - report.horizontal_furniture,
      report.horizontal_furniture, opt.min_storey_clearance_m, report.other,
      area_per_point);
  return report;
}

} // namespace reusex::geometry
