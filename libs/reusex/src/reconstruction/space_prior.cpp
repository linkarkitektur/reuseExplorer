// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reconstruction/space_prior.hpp"

#include "core/logging.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <map>
#include <stdexcept>

namespace reusex::geometry {

namespace {

/// Even-odd point-in-ring test (open ring).
bool in_ring(const std::vector<Eigen::Vector2d> &ring,
             const Eigen::Vector2d &q) {
  bool inside = false;
  const size_t n = ring.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const Eigen::Vector2d &a = ring[i];
    const Eigen::Vector2d &b = ring[j];
    if ((a.y() > q.y()) != (b.y() > q.y())) {
      const double x =
          a.x() + (q.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
      if (q.x() < x)
        inside = !inside;
    }
  }
  return inside;
}

/// The cell's centroid and the points halfway from it to each of its
/// corners: the ray origins of compute_room_probabilities(), all inside the
/// (convex) cell.
std::vector<Eigen::Vector3d> cell_samples(const CellComplex &cc,
                                          CellComplex::Vertex cell) {
  const Eigen::Vector3d centre = cc[cell].pos;
  std::vector<CellComplex::Vertex> corners;
  for (auto [f, f_end] = boost::adjacent_vertices(cell, cc); f != f_end; ++f) {
    if (cc[*f].type != CellNodeType::face)
      continue;
    for (auto [v, v_end] = boost::adjacent_vertices(*f, cc); v != v_end; ++v)
      if (cc[*v].type == CellNodeType::vertex)
        corners.push_back(*v);
  }
  std::sort(corners.begin(), corners.end());
  corners.erase(std::unique(corners.begin(), corners.end()), corners.end());
  std::vector<Eigen::Vector3d> out{centre};
  out.reserve(corners.size() + 1);
  for (auto v : corners)
    out.push_back(centre + 0.5 * (cc[v].pos - centre));
  return out;
}

} // namespace

bool SpaceFootprint::contains(const Eigen::Vector3d &p) const {
  if (outer.size() < 3)
    return false;
  const Eigen::Vector3d l = world_to_level * p;
  if (l.z() < z_floor || l.z() > z_ceiling)
    return false;
  const Eigen::Vector2d q = l.head<2>();
  if (!in_ring(outer, q))
    return false;
  for (const auto &h : holes)
    if (h.size() >= 3 && in_ring(h, q))
      return false;
  return true;
}

auto apply_space_prior(CellComplex &cc,
                       const std::vector<SpaceFootprint> &spaces, double weight)
    -> SpacePriorStats {
  if (weight > 1.0)
    throw std::invalid_argument(
        fmt::format("apply_space_prior: weight {} is above 1", weight));
  SpacePriorStats stats;
  if (weight <= 0.0 || spaces.empty())
    return stats;

  using Cd = CellComplex::Vertex;
  auto c_rp = cc.property_map<Cd, std::vector<double>>("c:room_probabilities");
  auto volume = cc.property_map<Cd, double>("c:volume");
  // Absent (probabilities set by hand): no ray escaped, every outside vote
  // is a back face.
  auto c_esc = cc.add_property_map<Cd, double>("c:escape_probability").first;

  // Space -> index into the probability vector (1 + position in
  // room_labels); a space whose room the rooms cloud no longer has is left
  // out.
  std::vector<size_t> index(spaces.size(), 0);
  for (size_t s = 0; s < spaces.size(); ++s) {
    const auto it = std::find(cc.room_labels.begin(), cc.room_labels.end(),
                              spaces[s].room_label);
    if (it == cc.room_labels.end()) {
      ++stats.spaces_skipped;
      continue;
    }
    index[s] = 1 + static_cast<size_t>(it - cc.room_labels.begin());
    ++stats.spaces;
  }
  if (stats.spaces_skipped > 0)
    reusex::warn("mesh: {} of {} space(s) name a room the 'rooms' cloud does "
                 "not have; they give no room prior. Re-run `rux create "
                 "spaces`",
                 stats.spaces_skipped, spaces.size());
  if (stats.spaces == 0)
    return stats;

  for (auto cit = cc.cells_begin(); cit != cc.cells_end(); ++cit) {
    // A cell inside a wall's slab (between a pair's two faces, on the
    // wall's measured run) is never pulled into a room: a space drawn
    // across a wall (raster fill where the wall graph is open, a drifted
    // wall) would otherwise merge two rooms through it and drop the wall
    // from the envelope. (Measured, reusex-6t4: letting every slab cell
    // take the prior merges rooms -- house1 +7 % unsupported mesh,
    // NewOffice fit +6 %.) A strip of the wall's line through open floor
    // beyond its run is no wall (reusex-9a8, CellComplex::mark_wall_runs()):
    // it takes the prior, so house2's stair pocket, which such strips
    // cross, fills.
    if (!std::get<CellData>(cc[*cit].data).run_wall_ids.empty())
      continue;
    auto &p = c_rp[*cit];
    const auto samples = cell_samples(cc, *cit);
    std::map<size_t, size_t> hits; // probability index -> samples inside
    for (const auto &x : samples)
      for (size_t s = 0; s < spaces.size(); ++s)
        if (index[s] != 0 && index[s] < p.size() && spaces[s].contains(x)) {
          ++hits[index[s]];
          break; // spaces of one storey do not overlap
        }
    if (hits.empty())
      continue;
    // The room holding most samples; ties go to the lower index.
    auto best = hits.begin();
    for (auto it = hits.begin(); it != hits.end(); ++it)
      if (it->second > best->second)
        best = it;
    const double c =
        static_cast<double>(best->second) / static_cast<double>(samples.size());
    const double wc = weight * c;

    const auto argmax = [](const std::vector<double> &v) {
      return static_cast<size_t>(std::max_element(v.begin(), v.end()) -
                                 v.begin());
    };
    const bool was_out = argmax(p) == 0;
    const double p_out_before = p[0];
    // Only the rays that hit something are re-weighed: the share that
    // escaped the scan stays outside.
    const double e = std::clamp(c_esc[*cit], 0.0, p[0]);
    for (auto &v : p)
      v *= 1.0 - wc;
    p[0] += wc * e;
    p[best->first] += wc * (1.0 - e);

    ++stats.cells;
    stats.volume += c * volume[*cit];
    if (was_out && argmax(p) != 0) {
      const auto &pos = cc[*cit].pos;
      reusex::debug("space prior: cell {} at ({:.2f}, {:.2f}, {:.2f}), "
                    "{:.2f} m^3, cover {:.2f}: outside -> room {} (p_out "
                    "{:.2f} -> {:.2f})",
                    cc[*cit].id, pos.x(), pos.y(), pos.z(), volume[*cit], c,
                    cc.room_labels[best->first - 1], p_out_before, p[0]);
      ++stats.cells_flipped;
      stats.volume_flipped += volume[*cit];
    }
  }
  return stats;
}

} // namespace reusex::geometry
