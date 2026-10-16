// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/openings.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace reusex::geometry {

namespace {

/// The wall's axis frame in the level frame: start, unit direction d along
/// the axis, unit normal n = d turned +90 deg, length.
struct AxisFrame {
  Eigen::Vector2d a, d, n;
  double length = 0.0;
};

AxisFrame axis_frame(const core::WallRecord &w) {
  AxisFrame f;
  f.a = Eigen::Vector2d(w.x0, w.y0);
  const Eigen::Vector2d b(w.x1, w.y1);
  f.length = (b - f.a).norm();
  if (!(f.length > 0.0) || !std::isfinite(f.length))
    throw std::invalid_argument(fmt::format(
        "openings: wall {} has a zero-length or non-finite axis", w.guid));
  f.d = (b - f.a) / f.length;
  f.n = Eigen::Vector2d(-f.d.y(), f.d.x());
  return f;
}

/// A boolean mask over the raster, row-major like FaceRaster::cells.
using Mask = std::vector<char>;

/// Morphological erosion (@p erode) or dilation of @p m by a square of
/// radius @p r. Cells outside the raster count as set for erosion (a shape
/// touching the border is not worn away by it) and as unset for dilation.
Mask morph(const Mask &m, int ns, int nz, int r, bool erode) {
  Mask out(m.size(), 0);
  for (int iz = 0; iz < nz; ++iz)
    for (int is = 0; is < ns; ++is) {
      bool v = erode;
      for (int dz = -r; dz <= r && v == erode; ++dz)
        for (int ds = -r; ds <= r; ++ds) {
          const int z = iz + dz, s = is + ds;
          const bool inside = z >= 0 && z < nz && s >= 0 && s < ns;
          const bool set =
              inside ? m[static_cast<std::size_t>(z * ns + s)] != 0 : erode;
          if (erode && !set) {
            v = false;
            break;
          }
          if (!erode && set) {
            v = true;
            break;
          }
        }
      out[static_cast<std::size_t>(iz * ns + is)] = v ? 1 : 0;
    }
  return out;
}

} // namespace

FaceRaster face_raster(const std::vector<Eigen::Vector3d> &points,
                       const core::WallRecord &wall, double z_floor,
                       double z_ceiling, const OpeningOptions &opt) {
  if (!(opt.cell_m > 0.0))
    throw std::invalid_argument("face_raster: cell_m must be positive");
  const AxisFrame f = axis_frame(wall);
  const double z_lo = z_floor + opt.floor_clear_m;
  const double z_hi = z_ceiling - opt.ceiling_clear_m;
  if (!(z_hi > z_lo))
    throw std::invalid_argument(fmt::format(
        "face_raster: wall {}: the storey has no height between its floor "
        "and ceiling clearances ({:.3f} .. {:.3f} m)",
        wall.guid, z_lo, z_hi));

  FaceRaster r;
  r.cell = opt.cell_m;
  r.z0 = z_lo;
  r.length = f.length;
  r.half_thickness = wall.thickness > 0.0 ? 0.5 * wall.thickness
                                          : opt.unknown_half_thickness_m;
  r.ns = std::max(1, static_cast<int>(std::ceil(f.length / opt.cell_m - 1e-9)));
  r.nz = std::max(
      1, static_cast<int>(std::ceil((z_hi - z_lo) / opt.cell_m - 1e-9)));
  const auto n =
      static_cast<std::size_t>(r.ns) * static_cast<std::size_t>(r.nz);
  r.wall_points.assign(n, 0);
  r.occluder_points.assign(n, 0);

  const double slab = r.half_thickness + opt.slab_margin_m;
  // The sides the wall was seen from (+t = side a, face_a_points): a wall
  // seen from one side only is hidden only by what stands on that side.
  // With no face counts at all (a hand-made wall) both sides count.
  const bool known = wall.face_a_points > 0 || wall.face_b_points > 0;
  const bool side_a = !known || wall.face_a_points > 0;
  const bool side_b = !known || wall.face_b_points > 0;
  const double front = r.half_thickness + opt.front_m;
  for (const auto &p : points) {
    if (!(p.z() >= z_lo && p.z() < z_hi))
      continue;
    const Eigen::Vector2d q = p.head<2>() - f.a;
    const double s = q.dot(f.d);
    if (!(s >= 0.0 && s < f.length))
      continue;
    const double ts = q.dot(f.n);
    const double t = std::abs(ts);
    if (!(t <= front))
      continue;
    // Beyond the slab on a side the wall was never seen from: nothing there
    // hid the face from a sensor (a window's reveal and glazing, the
    // outside), so it is no occluder.
    if (t > slab && ((ts > 0.0 && !side_a) || (ts < 0.0 && !side_b)))
      continue;
    const int is = std::min(r.ns - 1, static_cast<int>(s / opt.cell_m));
    const int iz =
        std::min(r.nz - 1, static_cast<int>((p.z() - z_lo) / opt.cell_m));
    const auto k =
        static_cast<std::size_t>(iz) * static_cast<std::size_t>(r.ns) +
        static_cast<std::size_t>(is);
    if (t <= slab)
      ++r.wall_points[k];
    else
      ++r.occluder_points[k];
  }

  r.cells.resize(n);
  for (std::size_t k = 0; k < n; ++k)
    r.cells[k] = r.wall_points[k] >= opt.min_wall_points ? FaceCellState::solid
                 : r.occluder_points[k] >= opt.min_occluder_points
                     ? FaceCellState::occluded
                     : FaceCellState::open;
  return r;
}

std::vector<OpeningRect> fit_openings(const FaceRaster &raster, double z_floor,
                                      double z_ceiling,
                                      const OpeningOptions &opt) {
  const int ns = raster.ns, nz = raster.nz;
  std::vector<OpeningRect> out;
  if (ns <= 0 || nz <= 0)
    return out;
  const auto idx = [ns](int is, int iz) {
    return static_cast<std::size_t>(iz) * static_cast<std::size_t>(ns) +
           static_cast<std::size_t>(is);
  };

  Mask open(raster.cells.size(), 0);
  for (std::size_t k = 0; k < open.size(); ++k)
    open[k] = raster.cells[k] != FaceCellState::solid ? 1 : 0;
  Mask opened = open;
  if (opt.morph_radius > 0)
    opened = morph(morph(open, ns, nz, opt.morph_radius, true), ns, nz,
                   opt.morph_radius, false);
  // Opening never adds a cell: dilation can reach beyond the eroded shape's
  // source only where the source was set.
  for (std::size_t k = 0; k < opened.size(); ++k)
    opened[k] = opened[k] && open[k];

  // 4-connected components, labelled in index order.
  std::vector<int> comp(opened.size(), -1);
  int ncomp = 0;
  std::vector<std::pair<int, int>> stack;
  for (int iz = 0; iz < nz; ++iz)
    for (int is = 0; is < ns; ++is) {
      if (!opened[idx(is, iz)] || comp[idx(is, iz)] >= 0)
        continue;
      const int c = ncomp++;
      int s0 = is, s1 = is, z0 = iz, z1 = iz;
      stack.assign(1, {is, iz});
      comp[idx(is, iz)] = c;
      while (!stack.empty()) {
        const auto [s, z] = stack.back();
        stack.pop_back();
        s0 = std::min(s0, s);
        s1 = std::max(s1, s);
        z0 = std::min(z0, z);
        z1 = std::max(z1, z);
        for (const auto &[ds, dz] : {std::pair{1, 0}, std::pair{-1, 0},
                                     std::pair{0, 1}, std::pair{0, -1}}) {
          const int a = s + ds, b = z + dz;
          if (a < 0 || a >= ns || b < 0 || b >= nz)
            continue;
          if (!opened[idx(a, b)] || comp[idx(a, b)] >= 0)
            continue;
          comp[idx(a, b)] = c;
          stack.emplace_back(a, b);
        }
      }

      // Trim the bounding box edge by edge while an edge row or column is
      // less than half open (of this component); the worst edge goes first.
      const auto mine = [&](int a, int b) { return comp[idx(a, b)] == c; };
      const auto col_share = [&](int a) {
        int k = 0;
        for (int b = z0; b <= z1; ++b)
          k += mine(a, b) ? 1 : 0;
        return static_cast<double>(k) / (z1 - z0 + 1);
      };
      const auto row_share = [&](int b) {
        int k = 0;
        for (int a = s0; a <= s1; ++a)
          k += mine(a, b) ? 1 : 0;
        return static_cast<double>(k) / (s1 - s0 + 1);
      };
      for (int guard = 0; guard < ns + nz && s0 <= s1 && z0 <= z1; ++guard) {
        const std::array<double, 4> share{col_share(s0), col_share(s1),
                                          row_share(z0), row_share(z1)};
        const auto worst = static_cast<int>(
            std::min_element(share.begin(), share.end()) - share.begin());
        if (share[static_cast<std::size_t>(worst)] >= 0.5)
          break;
        switch (worst) {
        case 0:
          ++s0;
          break;
        case 1:
          --s1;
          break;
        case 2:
          ++z0;
          break;
        default:
          --z1;
          break;
        }
      }
      if (s0 > s1 || z0 > z1)
        continue;

      OpeningRect rect;
      int cells = 0, gaps = 0;
      for (int b = z0; b <= z1; ++b)
        for (int a = s0; a <= s1; ++a) {
          ++cells;
          gaps += mine(a, b) ? 1 : 0;
          rect.open_cells +=
              mine(a, b) && raster.at(a, b) == FaceCellState::open ? 1 : 0;
        }
      rect.fill = static_cast<double>(gaps) / cells;
      rect.open_share = static_cast<double>(rect.open_cells) / cells;
      rect.at_floor = z0 == 0;
      rect.at_ceiling = z1 == nz - 1;
      rect.s_start = s0 * raster.cell;
      rect.s_end = std::min(raster.length, (s1 + 1) * raster.cell);
      rect.z_sill = rect.at_floor ? z_floor : raster.z0 + z0 * raster.cell;
      rect.z_head =
          rect.at_ceiling ? z_ceiling : raster.z0 + (z1 + 1) * raster.cell;
      const auto jamb = [&](int a) {
        if (a < 0 || a >= ns)
          return 0.0;
        // Solid counts fully, occluded half: an open door leaf or a shelf
        // standing at the jamb hides it, but it is not open face.
        double k = 0.0;
        for (int b = z0; b <= z1; ++b)
          k += raster.at(a, b) == FaceCellState::solid      ? 1.0
               : raster.at(a, b) == FaceCellState::occluded ? 0.5
                                                            : 0.0;
        return k / (z1 - z0 + 1);
      };
      rect.jamb_start = jamb(s0 - 1);
      rect.jamb_end = jamb(s1 + 1);

      if (rect.s_end - rect.s_start < opt.min_width_m - 1e-9 ||
          rect.z_head - rect.z_sill < opt.min_height_m - 1e-9 ||
          rect.fill < opt.min_fill || rect.open_share < opt.min_open_share ||
          std::min(rect.jamb_start, rect.jamb_end) < opt.min_jamb_share)
        continue;
      if (opt.drop_top_bands && rect.at_ceiling && !rect.at_floor &&
          rect.z_head - rect.z_sill < opt.top_band_max_m)
        continue;
      if (rect.at_floor && !rect.at_ceiling &&
          rect.z_head - rect.z_sill < opt.min_floor_height_m - 1e-9)
        continue;
      out.push_back(rect);
    }

  std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
    return std::tie(a.s_start, a.z_sill) < std::tie(b.s_start, b.z_sill);
  });
  return out;
}

} // namespace reusex::geometry
