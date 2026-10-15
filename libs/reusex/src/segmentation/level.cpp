// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segmentation/level.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"
#include "geometry/level_frame.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace reusex::geometry {
namespace {

/// Passes of (histogram -> fit -> re-estimate the vertical). The second pass
/// re-bins along the measured vertical; a third only runs if that moved it.
constexpr int kMaxPasses = 3;
constexpr double kVerticalConvergedRad = 1e-7;

/// One fitted horizontal surface (a floor, a ceiling, a slab face).
struct Surface {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(); ///< oriented with +v
  double height = 0.0; ///< centroid . v at fit time [m]
  size_t points = 0;   ///< inliers of the final fit
  double rms = 0.0;    ///< RMS distance of the inliers to the plane [m]
  bool fitted = false; ///< false: < 3 inliers, plane = peak height along v
};

/// Surfaces closer than slab_merge_m, ascending.
struct Level {
  std::vector<Surface> surfaces;
  [[nodiscard]] const Surface &bottom() const { return surfaces.front(); }
  [[nodiscard]] const Surface &top() const { return surfaces.back(); }
  [[nodiscard]] const Surface &strongest() const {
    return *std::max_element(
        surfaces.begin(), surfaces.end(),
        [](const Surface &a, const Surface &b) { return a.points < b.points; });
  }
};

struct StoreySpan {
  Surface floor;
  Surface ceiling;
  bool ceiling_assumed = false;
  std::string reason;
};

/// Least-squares plane through @p pts[idx]: centroid and smallest-eigenvalue
/// eigenvector of the covariance, oriented so that normal . v > 0.
void fit_plane(const std::vector<Eigen::Vector3d> &pts,
               const std::vector<size_t> &idx, const Eigen::Vector3d &v,
               Eigen::Vector3d &centroid, Eigen::Vector3d &normal) {
  centroid.setZero();
  for (size_t i : idx)
    centroid += pts[i];
  centroid /= static_cast<double>(idx.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (size_t i : idx) {
    const Eigen::Vector3d d = pts[i] - centroid;
    cov += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  normal = es.eigenvectors().col(0).normalized();
  if (normal.dot(v) < 0.0)
    normal = -normal;
}

/// Iterated LSQ: start from the points within one bin of @p h0 along @p v,
/// then refit on the points within inlier_m of the plane until the inlier set
/// stops changing or max_fit_iterations is reached.
Surface fit_surface(const std::vector<Eigen::Vector3d> &pts,
                    const std::vector<double> &heights,
                    const Eigen::Vector3d &v, double h0,
                    const LevelOptions &opt) {
  std::vector<size_t> sel;
  for (size_t i = 0; i < pts.size(); ++i)
    if (std::abs(heights[i] - h0) <= opt.hist_bin_m)
      sel.push_back(i);

  Surface s;
  s.height = h0;
  s.normal = v;
  s.centroid = v * h0;
  if (sel.size() < 3)
    return s;

  const int iters = std::max(1, opt.max_fit_iterations);
  for (int it = 0; it < iters; ++it) {
    Eigen::Vector3d c, n;
    fit_plane(pts, sel, v, c, n);
    s.centroid = c;
    s.normal = n;
    s.fitted = true;
    std::vector<size_t> next;
    for (size_t i = 0; i < pts.size(); ++i)
      if (std::abs(n.dot(pts[i] - c)) <= opt.inlier_m)
        next.push_back(i);
    if (next.size() < 3 || next == sel)
      break;
    sel = std::move(next);
  }
  double ss = 0.0;
  for (size_t i : sel) {
    const double d = s.normal.dot(pts[i] - s.centroid);
    ss += d * d;
  }
  s.points = sel.size();
  s.rms = std::sqrt(ss / static_cast<double>(sel.size()));
  s.height = s.centroid.dot(v);
  return s;
}

/// Histogram peaks of @p heights: bins (anchored at height 0, so the grid
/// does not move with an outlier) that are local maxima and hold at least
/// peak_min_share of the largest bin; peaks within two bins of a stronger
/// one are the same surface. Returns refined peak heights, ascending.
std::vector<double> histogram_peaks(const std::vector<double> &heights,
                                    const LevelOptions &opt) {
  std::map<int64_t, size_t> bins;
  for (double h : heights)
    ++bins[static_cast<int64_t>(std::floor(h / opt.hist_bin_m))];
  size_t max_count = 0;
  for (const auto &[b, c] : bins)
    max_count = std::max(max_count, c);
  const auto count = [&](int64_t b) -> size_t {
    const auto it = bins.find(b);
    return it == bins.end() ? 0 : it->second;
  };

  std::vector<std::pair<size_t, int64_t>> candidates; // (count, bin)
  for (const auto &[b, c] : bins)
    if (static_cast<double>(c) >=
            opt.peak_min_share * static_cast<double>(max_count) &&
        c > count(b - 1) && c >= count(b + 1))
      candidates.emplace_back(c, b);
  std::sort(
      candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
        return a.first != b.first ? a.first > b.first : a.second < b.second;
      });
  std::vector<int64_t> accepted;
  for (const auto &[c, b] : candidates) {
    const bool near =
        std::any_of(accepted.begin(), accepted.end(),
                    [&](int64_t a) { return std::abs(a - b) <= 2; });
    if (!near)
      accepted.push_back(b);
  }
  std::sort(accepted.begin(), accepted.end());

  // Refine each peak to the mean height of its bin and both neighbours.
  std::vector<double> peaks;
  peaks.reserve(accepted.size());
  for (int64_t b : accepted) {
    const double lo = static_cast<double>(b - 1) * opt.hist_bin_m;
    const double hi = static_cast<double>(b + 2) * opt.hist_bin_m;
    double sum = 0.0;
    size_t n = 0;
    for (double h : heights)
      if (h >= lo && h < hi) {
        sum += h;
        ++n;
      }
    peaks.push_back(n ? sum / static_cast<double>(n)
                      : (static_cast<double>(b) + 0.5) * opt.hist_bin_m);
  }
  return peaks;
}

/// Group surfaces (ascending) into levels: a gap below slab_merge_m keeps
/// the next surface in the current level.
std::vector<Level> group_levels(std::vector<Surface> surfaces,
                                const LevelOptions &opt) {
  std::sort(
      surfaces.begin(), surfaces.end(),
      [](const Surface &a, const Surface &b) { return a.height < b.height; });
  std::vector<Level> levels;
  for (auto &s : surfaces) {
    if (!levels.empty() &&
        s.height - levels.back().top().height < opt.slab_merge_m)
      levels.back().surfaces.push_back(s);
    else
      levels.push_back(Level{{s}});
  }
  return levels;
}

/// Walk the levels bottom-up. The lowest level's strongest surface is the
/// storey-0 floor; a storey's ceiling is the bottom of the first level at
/// least min_storey_height_m above its floor (levels in between — tables,
/// shelves — are skipped); the next floor is the top of that level, unless
/// it is the last level and a single surface (the roof ceiling). A floor with
/// no ceiling above gets default_height_m.
std::vector<StoreySpan> walk_storeys(const std::vector<Level> &levels,
                                     const LevelOptions &opt) {
  std::vector<StoreySpan> spans;
  if (levels.empty())
    return spans;
  Surface floor = levels.front().strongest();
  size_t at = 0;
  for (;;) {
    size_t j = at + 1;
    while (j < levels.size() &&
           levels[j].bottom().height - floor.height < opt.min_storey_height_m)
      ++j;
    StoreySpan span;
    span.floor = floor;
    if (j >= levels.size()) {
      span.ceiling_assumed = true;
      span.reason =
          fmt::format("no level at least {:.2f} m above the floor; ceiling "
                      "height defaulted to {:.2f} m",
                      opt.min_storey_height_m, opt.default_height_m);
      spans.push_back(std::move(span));
      break;
    }
    span.ceiling = levels[j].bottom();
    spans.push_back(std::move(span));
    const bool last = j + 1 == levels.size();
    if (last && levels[j].surfaces.size() < 2)
      break; // the roof ceiling
    floor = levels[j].top();
    at = j;
  }
  return spans;
}

/// Height of plane (c, n) where it crosses the line o + t v.
double height_on_axis(const Surface &s, const Eigen::Vector3d &o,
                      const Eigen::Vector3d &v) {
  const double nv = s.normal.dot(v);
  if (std::abs(nv) < 1e-9)
    return (s.centroid - o).dot(v);
  return s.normal.dot(s.centroid - o) / nv;
}

/// Evidence for one surface; tilt_deg is the angle between its fitted normal
/// and the scan's measured vertical @p v (how far this surface disagrees with
/// the one level frame every storey shares).
nlohmann::json surface_json(const Surface &s, const Eigen::Vector3d &v) {
  const double tilt =
      std::acos(std::clamp(s.normal.dot(v), -1.0, 1.0)) * 180.0 / M_PI;
  return {{"points", s.points},
          {"rms_m", s.rms},
          {"fitted", s.fitted},
          {"tilt_deg", tilt}};
}

} // namespace

StoreysResult detect_storeys(const Cloud &cloud, const CloudL &plane_labels,
                             const CloudLoc &plane_centroids,
                             const CloudN &plane_normals,
                             const LevelOptions &opt) {
  if (cloud.size() != plane_labels.size())
    throw std::invalid_argument(fmt::format(
        "detect_storeys: cloud has {} points but the planes cloud has {}",
        cloud.size(), plane_labels.size()));
  if (plane_centroids.size() != plane_normals.size())
    throw std::invalid_argument(fmt::format(
        "detect_storeys: plane_centroids has {} entries but plane_normals has "
        "{}",
        plane_centroids.size(), plane_normals.size()));
  if (plane_normals.empty())
    throw std::invalid_argument(
        "detect_storeys: no plane (plane_centroids / plane_normals are empty)");
  if (opt.hist_bin_m <= 0.0 || opt.inlier_m <= 0.0)
    throw std::invalid_argument(
        "detect_storeys: hist_bin_m and inlier_m must be positive");

  // Horizontal planes, normals oriented to world +z.
  const size_t n_planes = plane_normals.size();
  std::vector<char> horizontal(n_planes, 0);
  std::vector<Eigen::Vector3d> normal(n_planes);
  StoreysResult result;
  for (size_t k = 0; k < n_planes; ++k) {
    const auto &pn = plane_normals.points[k];
    Eigen::Vector3d n(pn.normal_x, pn.normal_y, pn.normal_z);
    const double len = n.norm();
    if (!(len > 0.0) || !std::isfinite(len))
      continue;
    n /= len;
    if (n.z() < 0.0)
      n = -n;
    normal[k] = n;
    if (n.z() > opt.horizontal_cos) {
      horizontal[k] = 1;
      ++result.horizontal_planes;
    }
  }

  // Points of horizontal planes; initial vertical = point-weighted mean of
  // their plane normals.
  std::vector<Eigen::Vector3d> pts;
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < cloud.size(); ++i) {
    const uint32_t label = plane_labels.points[i].label;
    if (!core::is_valid_label(label))
      continue;
    const size_t k = core::label_to_index(label);
    if (k >= n_planes)
      throw std::invalid_argument(fmt::format(
          "detect_storeys: point {} has plane label {} but only {} planes "
          "exist",
          i, label, n_planes));
    if (!horizontal[k])
      continue;
    const auto &p = cloud.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    pts.emplace_back(p.x, p.y, p.z);
    v += normal[k];
  }

  const auto one_row = [&](core::StoreyRecord s, const std::string &reason,
                           const nlohmann::json &meta) {
    reusex::warn("detect_storeys: one storey ({})", reason);
    s.metadata = meta.dump();
    result.storeys.push_back(std::move(s));
    return result;
  };

  if (pts.empty()) {
    // Nothing to measure: world z up, floor at a low percentile of the cloud
    // (not the minimum, which one outlier would move).
    std::vector<float> zs;
    zs.reserve(cloud.size());
    for (const auto &p : cloud.points)
      if (std::isfinite(p.z))
        zs.push_back(p.z);
    double z0 = 0.0;
    if (!zs.empty()) {
      const size_t q = zs.size() / 100;
      std::nth_element(zs.begin(), zs.begin() + static_cast<long>(q), zs.end());
      z0 = zs[q];
    }
    const std::string reason = fmt::format(
        "no horizontal plane among {} planes; vertical assumed +z, floor at "
        "the 1st z percentile {:.3f} m, height defaulted to {:.2f} m",
        n_planes, z0, opt.default_height_m);
    core::StoreyRecord s;
    s.z_floor = 0.0;
    s.z_ceiling = opt.default_height_m;
    s.level_transform =
        make_level_frame(Eigen::Vector3d::UnitZ(), {0.0, 0.0, z0}).to_array();
    return one_row(s, reason,
                   {{"reason", reason},
                    {"vertical_assumed", true},
                    {"ceiling_assumed", true}});
  }

  v.normalize();
  std::vector<Level> levels;
  std::vector<StoreySpan> spans;
  std::vector<double> heights(pts.size());
  for (int pass = 0; pass < kMaxPasses; ++pass) {
    for (size_t i = 0; i < pts.size(); ++i)
      heights[i] = pts[i].dot(v);
    std::vector<Surface> surfaces;
    for (double h : histogram_peaks(heights, opt))
      surfaces.push_back(fit_surface(pts, heights, v, h, opt));
    levels = group_levels(std::move(surfaces), opt);
    spans = walk_storeys(levels, opt);

    // The vertical: normalised sum of every fitted floor/ceiling normal —
    // for one storey, the bisector of its floor and ceiling.
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto &sp : spans) {
      if (sp.floor.fitted)
        sum += sp.floor.normal;
      if (!sp.ceiling_assumed && sp.ceiling.fitted)
        sum += sp.ceiling.normal;
    }
    if (sum.norm() < 1e-12)
      break;
    const Eigen::Vector3d next = sum.normalized();
    const double moved = std::acos(std::clamp(next.dot(v), -1.0, 1.0));
    v = next;
    if (moved < kVerticalConvergedRad)
      break;
  }

  result.levels = static_cast<int>(levels.size());
  result.vertical = {v.x(), v.y(), v.z()};

  // One level frame for the scan: origin on the storey-0 floor.
  const Eigen::Vector3d origin = spans.front().floor.centroid;
  const LevelFrame frame = make_level_frame(v, origin);
  const auto transform = frame.to_array();

  for (size_t i = 0; i < spans.size(); ++i) {
    const auto &sp = spans[i];
    core::StoreyRecord s;
    s.idx = static_cast<int>(i);
    s.z_floor = i == 0 ? 0.0 : height_on_axis(sp.floor, origin, v);
    s.z_ceiling = sp.ceiling_assumed ? s.z_floor + opt.default_height_m
                                     : height_on_axis(sp.ceiling, origin, v);
    s.level_transform = transform;
    nlohmann::json meta = {{"floor", surface_json(sp.floor, v)},
                           {"ceiling_assumed", sp.ceiling_assumed}};
    if (!sp.ceiling_assumed)
      meta["ceiling"] = surface_json(sp.ceiling, v);
    if (!sp.reason.empty())
      meta["reason"] = sp.reason;
    s.metadata = meta.dump();
    result.storeys.push_back(std::move(s));
  }

  if (result.storeys.size() == 1) {
    reusex::info("detect_storeys: one storey from {} levels of {} horizontal "
                 "planes ({} points): {}",
                 result.levels, result.horizontal_planes, pts.size(),
                 spans.front().reason.empty()
                     ? fmt::format("no floor above the ceiling at {:.2f} m",
                                   result.storeys.front().z_ceiling)
                     : spans.front().reason);
  } else {
    reusex::info("detect_storeys: {} storeys from {} levels of {} horizontal "
                 "planes ({} points)",
                 result.storeys.size(), result.levels, result.horizontal_planes,
                 pts.size());
  }
  for (size_t i = 0; i < spans.size(); ++i)
    if (spans[i].ceiling_assumed)
      reusex::warn("detect_storeys: storey {} has no ceiling level; height "
                   "defaulted to {:.2f} m",
                   i, opt.default_height_m);
  return result;
}

} // namespace reusex::geometry
