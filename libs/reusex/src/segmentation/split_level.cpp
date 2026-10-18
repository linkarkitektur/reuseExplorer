// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/segmentation/split_level.hpp"

#include <fmt/format.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>

namespace reusex::geometry {
namespace {

constexpr float kNone = std::numeric_limits<float>::quiet_NaN();

int radius_cells(double metres, double cell) {
  return std::max(1, static_cast<int>(std::lround(metres / cell)));
}

/// The lowest height with at least @p min_points of @p z (sorted) within
/// @p bin of it: the median of those points; NaN when there is none.
float lowest_surface(std::vector<float> &z, int min_points, double bin) {
  if (static_cast<int>(z.size()) < min_points)
    return kNone;
  std::sort(z.begin(), z.end());
  const auto n = static_cast<int>(z.size());
  for (int i = 0; i + min_points - 1 < n; ++i) {
    int j = i;
    while (j + 1 < n &&
           z[static_cast<size_t>(j + 1)] - z[static_cast<size_t>(i)] <= bin)
      ++j;
    if (j - i + 1 >= min_points)
      return z[static_cast<size_t>((i + j) / 2)];
  }
  return kNone;
}

float median_of(std::vector<float> v) {
  const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
  std::nth_element(v.begin(), mid, v.end());
  return *mid;
}

} // namespace

FloorLevelMap floor_level_map(const std::vector<Eigen::Vector3d> &points,
                              const std::vector<double> &abs_nz,
                              const std::vector<uint8_t> &use, double z_floor,
                              int col0, int row0, int width, int height,
                              double cell_size, const SplitLevelOptions &o) {
  if (points.size() != abs_nz.size() ||
      (!use.empty() && use.size() != points.size()))
    throw std::invalid_argument(fmt::format(
        "floor_level_map: {} points but {} normals and {} mask entries",
        points.size(), abs_nz.size(), use.size()));
  if (width <= 0 || height <= 0 || !(cell_size > 0.0))
    throw std::invalid_argument(
        fmt::format("floor_level_map: empty grid ({} x {} cells of {} m)",
                    width, height, cell_size));

  const size_t n_grid =
      static_cast<size_t>(width) * static_cast<size_t>(height);
  FloorLevelMap out;
  out.floor_z.assign(n_grid, static_cast<float>(z_floor));
  // The floor is measured on blocks of k x k grid cells (block_m), so a
  // block holds enough points at scan density; the result is per grid cell.
  const int k =
      std::max(1, static_cast<int>(std::lround(o.block_m / cell_size)));
  const int full_width = width;
  const int full_height = height;
  const int bw = (width + k - 1) / k;
  const int bh = (height + k - 1) / k;
  // From here on, "cells" are blocks.
  const int grid_col0 = col0, grid_row0 = row0;
  width = bw;
  height = bh;
  const size_t n_cells = static_cast<size_t>(bw) * static_cast<size_t>(bh);
  const double block = cell_size * k;
  const double cell_area = block * block;

  // 1. Per-cell floor: the lowest horizontal surface in the step range.
  std::vector<std::vector<float>> samples(n_cells);
  const double lo = z_floor - o.cell_bin_m;
  const double hi = z_floor + o.max_step_m;
  for (size_t pi = 0; pi < points.size(); ++pi) {
    if (!use.empty() && !use[pi])
      continue;
    if (!(abs_nz[pi] > o.horizontal_min_abs_nz))
      continue;
    const Eigen::Vector3d &q = points[pi];
    if (!(q.z() >= lo && q.z() <= hi))
      continue;
    const int gc = static_cast<int>(std::floor(q.x() / cell_size)) - grid_col0;
    const int gr = static_cast<int>(std::floor(q.y() / cell_size)) - grid_row0;
    if (gc < 0 || gr < 0 || gc >= full_width || gr >= full_height)
      continue;
    const int c = gc / k, r = gr / k;
    samples[static_cast<size_t>(r) * static_cast<size_t>(width) +
            static_cast<size_t>(c)]
        .push_back(static_cast<float>(q.z()));
  }
  std::vector<float> cell_floor(n_cells, kNone);
  for (size_t i = 0; i < n_cells; ++i) {
    cell_floor[i] = lowest_surface(samples[i], std::max(1, o.cell_min_points),
                                   o.cell_bin_m);
    if (std::isfinite(cell_floor[i]))
      ++out.evidence_cells;
  }
  samples.clear();
  samples.shrink_to_fit();

  // 2. Clusters of raised cells, strongest height bin first.
  // level per block: -1 = no evidence, 0 = base, l >= 1 = sub-level l.
  std::vector<int> level(n_cells, -1);
  for (size_t i = 0; i < n_cells; ++i)
    if (std::isfinite(cell_floor[i]))
      level[i] = 0;

  std::vector<uint8_t> taken(n_cells, 0);
  struct Found {
    double z;
    std::vector<size_t> cells;
  };
  std::vector<Found> found;
  const int open_r = radius_cells(o.open_radius_m, block);
  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(2 * open_r + 1, 2 * open_r + 1));
  const auto min_cells =
      static_cast<size_t>(std::ceil(o.min_area_m2 / cell_area));
  for (int guard = 0; guard < 16; ++guard) {
    std::map<long, size_t> bins;
    for (size_t i = 0; i < n_cells; ++i)
      if (!taken[i] && std::isfinite(cell_floor[i]) &&
          cell_floor[i] >= z_floor + o.min_step_m)
        ++bins[std::lround(std::floor(cell_floor[i] / o.cell_bin_m))];
    if (bins.empty())
      break;
    // Strongest bin, ties by the lower height (map order).
    auto best = bins.begin();
    for (auto it = bins.begin(); it != bins.end(); ++it)
      if (it->second > best->second)
        best = it;
    if (best->second < min_cells / 4)
      break; // too few cells left in any bin to hold a level
    const double centre =
        (static_cast<double>(best->first) + 0.5) * o.cell_bin_m;
    std::vector<float> near;
    for (size_t i = 0; i < n_cells; ++i)
      if (!taken[i] && std::isfinite(cell_floor[i]) &&
          std::abs(cell_floor[i] - centre) <= o.level_tol_m)
        near.push_back(cell_floor[i]);
    const double z = median_of(near);
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8U);
    for (size_t i = 0; i < n_cells; ++i)
      if (!taken[i] && std::isfinite(cell_floor[i]) &&
          std::abs(cell_floor[i] - z) <= o.level_tol_m)
        mask.data[i] = 1;
    // Whatever the cluster is, its cells are not looked at again.
    for (size_t i = 0; i < n_cells; ++i)
      if (mask.data[i])
        taken[i] = 1;
    cv::Mat seed;
    cv::morphologyEx(mask, seed, cv::MORPH_OPEN, kernel);
    cv::Mat comp;
    const int n_comp = cv::connectedComponents(mask, comp, 8, CV_32S);
    std::vector<size_t> seed_cells(static_cast<size_t>(n_comp), 0);
    for (size_t i = 0; i < n_cells; ++i)
      if (seed.data[i])
        ++seed_cells[static_cast<size_t>(comp.ptr<int>()[i])];
    Found f{z, {}};
    for (size_t i = 0; i < n_cells; ++i) {
      const int l = comp.ptr<int>()[i];
      if (l > 0 && seed_cells[static_cast<size_t>(l)] >= min_cells)
        f.cells.push_back(i);
    }
    if (!f.cells.empty())
      found.push_back(std::move(f));
  }
  std::stable_sort(found.begin(), found.end(),
                   [](const Found &a, const Found &b) { return a.z < b.z; });
  out.levels.push_back({z_floor, 0.0, true});
  for (size_t f = 0; f < found.size(); ++f) {
    out.levels.push_back({found[f].z, 0.0, false});
    for (const size_t i : found[f].cells)
      level[i] = static_cast<int>(f + 1);
  }

  // 3. Cells without evidence take the nearest evidence cell's level within
  // fill_radius_m (breadth-first in raster order: deterministic).
  if (out.split()) {
    const int fill_r = radius_cells(o.fill_radius_m, block);
    std::vector<int> dist(n_cells, -1);
    std::deque<size_t> queue;
    for (size_t i = 0; i < n_cells; ++i)
      if (level[i] >= 0) {
        dist[i] = 0;
        queue.push_back(i);
      }
    while (!queue.empty()) {
      const size_t i = queue.front();
      queue.pop_front();
      if (dist[i] >= fill_r)
        continue;
      const int c = static_cast<int>(i % static_cast<size_t>(width));
      const int r = static_cast<int>(i / static_cast<size_t>(width));
      static constexpr int dc[4] = {1, -1, 0, 0};
      static constexpr int dr[4] = {0, 0, 1, -1};
      for (int d = 0; d < 4; ++d) {
        const int cc = c + dc[d], rr = r + dr[d];
        if (cc < 0 || rr < 0 || cc >= width || rr >= height)
          continue;
        const size_t j = static_cast<size_t>(rr) * static_cast<size_t>(width) +
                         static_cast<size_t>(cc);
        if (dist[j] >= 0)
          continue;
        dist[j] = dist[i] + 1;
        level[j] = level[i];
        ++out.filled_cells;
        queue.push_back(j);
      }
    }
    for (int gr = 0; gr < full_height; ++gr)
      for (int gc = 0; gc < full_width; ++gc) {
        const int l =
            level[static_cast<size_t>(gr / k) * static_cast<size_t>(bw) +
                  static_cast<size_t>(gc / k)];
        if (l > 0)
          out.floor_z[static_cast<size_t>(gr) *
                          static_cast<size_t>(full_width) +
                      static_cast<size_t>(gc)] =
              static_cast<float>(out.levels[static_cast<size_t>(l)].z);
      }
  }
  for (size_t i = 0; i < n_cells; ++i)
    if (level[i] >= 0 && std::isfinite(cell_floor[i]))
      out.levels[static_cast<size_t>(level[i])].area_m2 += cell_area;
  return out;
}

} // namespace reusex::geometry
