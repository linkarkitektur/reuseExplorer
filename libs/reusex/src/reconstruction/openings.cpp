// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/openings.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numbers>
#include <stdexcept>
#include <tuple>

namespace reusex::geometry {

namespace {

constexpr double kPi = std::numbers::pi;

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

FaceLevels face_levels(const std::vector<Eigen::Vector3d> &points,
                       const core::WallRecord &wall, double z_floor,
                       double z_ceiling, const OpeningOptions &opt) {
  FaceLevels out;
  out.z_floor = z_floor;
  out.z_ceiling = z_ceiling;
  if (!opt.local_levels || !(opt.level_search_m > 0.0))
    return out;
  const AxisFrame f = axis_frame(wall);
  const double half = wall.thickness > 0.0 ? 0.5 * wall.thickness
                                           : opt.unknown_half_thickness_m;
  const double slab = half + opt.slab_margin_m;
  const double front = half + opt.front_m;
  constexpr double kBin = 0.02;
  const double below = 0.5 * opt.level_search_m;
  // Bins anchored at the storey's levels: [level - below, level + search].
  const int nb =
      static_cast<int>(std::ceil((below + opt.level_search_m) / kBin));
  std::vector<int> lo(static_cast<std::size_t>(nb), 0),
      hi(static_cast<std::size_t>(nb), 0);
  for (const auto &p : points) {
    const Eigen::Vector2d q = p.head<2>() - f.a;
    const double s = q.dot(f.d);
    if (!(s >= 0.0 && s < f.length))
      continue;
    const double t = std::abs(q.dot(f.n));
    if (!(t > slab && t <= front))
      continue;
    const int bf =
        static_cast<int>(std::floor((p.z() - z_floor + below) / kBin));
    if (bf >= 0 && bf < nb)
      ++lo[static_cast<std::size_t>(bf)];
    const int bc = static_cast<int>(
        std::floor((z_ceiling + below - p.z()) / kBin)); // 0 = highest
    if (bc >= 0 && bc < nb)
      ++hi[static_cast<std::size_t>(bc)];
  }
  // The lowest (highest) layer holding at least 30 % of the densest one:
  // the floor, not a sofa seat or a desk top above it.
  const auto peak = [&](const std::vector<int> &h) {
    const int top = *std::max_element(h.begin(), h.end());
    const int need = std::max(opt.min_level_points, (3 * top + 9) / 10);
    for (int b = 0; b < nb; ++b)
      if (h[static_cast<std::size_t>(b)] >= need)
        return b;
    return -1;
  };
  if (const int b = peak(lo); b >= 0) {
    out.z_floor = z_floor - below + (b + 0.5) * kBin;
    out.floor_measured = true;
  }
  if (const int b = peak(hi); b >= 0) {
    out.z_ceiling = z_ceiling + below - (b + 0.5) * kBin;
    out.ceiling_measured = true;
  }
  // Never a storey shorter than the clearances leave room for.
  if (out.z_ceiling - out.z_floor <
      opt.floor_clear_m + opt.ceiling_clear_m + 2.0 * opt.cell_m) {
    out.z_floor = z_floor;
    out.z_ceiling = z_ceiling;
    out.floor_measured = out.ceiling_measured = false;
  }
  return out;
}

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
      // Only a threshold (or floor noise) below it: it stands on the floor.
      if (!rect.at_floor && rect.z_sill - z_floor <= opt.sill_snap_m + 1e-9) {
        rect.z_sill = z_floor;
        rect.at_floor = true;
      }
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

// ===== W5.2 ==================================================================

std::vector<OpeningRect> merge_mullions(std::vector<OpeningRect> rects,
                                        const OpeningOptions &opt) {
  std::sort(rects.begin(), rects.end(), [](const auto &a, const auto &b) {
    return std::tie(a.s_start, a.z_sill) < std::tie(b.s_start, b.z_sill);
  });
  std::vector<OpeningRect> out;
  for (const auto &r : rects) {
    if (!out.empty()) {
      OpeningRect &m = out.back();
      if (r.s_start - m.s_end <= opt.mullion_max_gap_m + 1e-9 &&
          std::abs(r.z_sill - m.z_sill) <= opt.mullion_max_dz_m + 1e-9 &&
          std::abs(r.z_head - m.z_head) <= opt.mullion_max_dz_m + 1e-9) {
        const double area_m = (m.s_end - m.s_start) * (m.z_head - m.z_sill);
        const double area_r = (r.s_end - r.s_start) * (r.z_head - r.z_sill);
        OpeningRect u = m;
        u.s_end = std::max(m.s_end, r.s_end);
        u.z_sill = std::min(m.z_sill, r.z_sill);
        u.z_head = std::max(m.z_head, r.z_head);
        const double area_u = (u.s_end - u.s_start) * (u.z_head - u.z_sill);
        u.fill =
            area_u > 0.0 ? (m.fill * area_m + r.fill * area_r) / area_u : 0.0;
        u.open_share =
            area_u > 0.0
                ? (m.open_share * area_m + r.open_share * area_r) / area_u
                : 0.0;
        u.open_cells = m.open_cells + r.open_cells;
        u.jamb_end = r.jamb_end;
        u.at_floor = m.at_floor || r.at_floor;
        u.at_ceiling = m.at_ceiling || r.at_ceiling;
        m = u;
        continue;
      }
    }
    out.push_back(r);
  }
  // A transom: rectangles stacked over the same stretch of wall, at most
  // mullion_max_gap_m apart vertically, are one opening too.
  const auto area = [](const OpeningRect &r) {
    return (r.s_end - r.s_start) * (r.z_head - r.z_sill);
  };
  for (bool merged = true; merged;) {
    merged = false;
    for (std::size_t a = 0; a < out.size() && !merged; ++a)
      for (std::size_t b = 0; b < out.size() && !merged; ++b) {
        if (a == b)
          continue;
        const OpeningRect &lo = out[a], &up = out[b];
        if (up.z_sill < lo.z_head - 1e-9 ||
            up.z_sill - lo.z_head > opt.mullion_max_gap_m + 1e-9 ||
            std::abs(up.s_start - lo.s_start) > opt.mullion_max_dz_m + 1e-9 ||
            std::abs(up.s_end - lo.s_end) > opt.mullion_max_dz_m + 1e-9)
          continue;
        OpeningRect u = lo;
        u.s_start = std::min(lo.s_start, up.s_start);
        u.s_end = std::max(lo.s_end, up.s_end);
        u.z_head = up.z_head;
        u.at_ceiling = up.at_ceiling;
        const double au = area(u);
        u.fill =
            au > 0.0 ? (lo.fill * area(lo) + up.fill * area(up)) / au : 0.0;
        u.open_share =
            au > 0.0
                ? (lo.open_share * area(lo) + up.open_share * area(up)) / au
                : 0.0;
        u.open_cells = lo.open_cells + up.open_cells;
        u.jamb_start = std::min(lo.jamb_start, up.jamb_start);
        u.jamb_end = std::min(lo.jamb_end, up.jamb_end);
        out[a] = u;
        out.erase(out.begin() + static_cast<std::ptrdiff_t>(b));
        merged = true;
      }
  }
  // A rectangle lying mostly (over half its area) inside a bigger one is
  // part of it (the trimmed box of one component reaching over another).
  std::vector<OpeningRect> kept;
  for (std::size_t a = 0; a < out.size(); ++a) {
    bool inside = false;
    for (std::size_t b = 0; b < out.size() && !inside; ++b) {
      if (a == b || area(out[b]) < area(out[a]) ||
          (area(out[b]) == area(out[a]) && b > a))
        continue;
      const double w = std::min(out[a].s_end, out[b].s_end) -
                       std::max(out[a].s_start, out[b].s_start);
      const double h = std::min(out[a].z_head, out[b].z_head) -
                       std::max(out[a].z_sill, out[b].z_sill);
      inside = w > 0.0 && h > 0.0 && w * h > 0.5 * area(out[a]);
    }
    if (!inside)
      kept.push_back(out[a]);
  }
  std::sort(kept.begin(), kept.end(), [](const auto &x, const auto &y) {
    return std::tie(x.s_start, x.z_sill) < std::tie(y.s_start, y.z_sill);
  });
  return kept;
}

std::string classify_opening(const OpeningRect &rect, double z_floor,
                             double z_ceiling, const OpeningOptions &opt) {
  const double sill = rect.z_sill - z_floor;
  const double height = rect.z_head - rect.z_sill;
  if (sill <= opt.door_max_sill_m) {
    if (rect.z_head >= z_ceiling - opt.full_height_tol_m)
      return "opening";
    if (height >= opt.door_min_height_m && height <= opt.door_max_height_m)
      return "door";
  }
  return "window";
}

std::vector<double>
trajectory_crossings(const std::vector<Eigen::Vector3d> &trajectory,
                     const core::WallRecord &wall, double z_floor,
                     double z_ceiling, const OpeningOptions &opt) {
  std::vector<double> out;
  const AxisFrame f = axis_frame(wall);
  const Eigen::Vector2d b = f.a + f.d * f.length;
  const auto inside = [&](const Eigen::Vector3d &p) {
    return p.z() > z_floor - opt.crossing_storey_below_m &&
           p.z() < z_ceiling + opt.crossing_storey_above_m;
  };
  for (std::size_t k = 0; k + 1 < trajectory.size(); ++k) {
    const auto &c0 = trajectory[k], &c1 = trajectory[k + 1];
    if (!inside(c0) || !inside(c1))
      continue;
    const Eigen::Vector2d p = c0.head<2>(), q = c1.head<2>();
    if ((q - p).norm() > opt.crossing_max_step_m)
      continue;
    Eigen::Matrix2d m;
    m.col(0) = q - p;
    m.col(1) = f.a - b;
    if (std::abs(m.determinant()) < 1e-12) // parallel: no crossing
      continue;
    const Eigen::Vector2d tu = m.partialPivLu().solve(f.a - p);
    if (tu.x() < 0.0 || tu.x() > 1.0 || tu.y() < 0.0 || tu.y() > 1.0)
      continue;
    out.push_back(tu.y() * f.length);
  }
  return out;
}

DoorLeaf door_leaf(const std::vector<Eigen::Vector3d> &points,
                   const core::WallRecord &wall, const OpeningRect &rect,
                   const OpeningOptions &opt) {
  DoorLeaf best;
  const AxisFrame f = axis_frame(wall);
  const double half = wall.thickness > 0.0 ? 0.5 * wall.thickness
                                           : opt.unknown_half_thickness_m;
  const double width = rect.s_end - rect.s_start;
  const double zlo = rect.z_sill + opt.leaf_margin_m;
  const double zhi = rect.z_head - opt.leaf_margin_m;
  if (!(width > 0.0) || !(zhi > zlo) || !(opt.leaf_bin_deg > 0.0))
    return best;
  const int nbins = static_cast<int>(std::ceil(90.0 / opt.leaf_bin_deg));
  // [hinge 0 = start, 1 = end][side 0 = +t, 1 = -t][bin]: points of the leaf
  // (r within the width) and of what continues beyond it at that angle (a
  // wall running away from the jamb, not a leaf).
  std::vector<int> leaf(static_cast<std::size_t>(4 * nbins), 0);
  std::vector<int> beyond(leaf.size(), 0);
  // Radial extent of each bin's leaf points: a leaf runs from the hinge out
  // to its width, a stray stretch of another surface does not.
  std::vector<double> r_lo(leaf.size(), 1e9), r_hi(leaf.size(), -1e9);
  for (const auto &p : points) {
    if (!(p.z() >= zlo && p.z() <= zhi))
      continue;
    const Eigen::Vector2d q = p.head<2>() - f.a;
    const double s = q.dot(f.d), t = q.dot(f.n);
    for (int side = 0; side < 2; ++side) {
      const double v = (side == 0 ? t : -t) - half; // out from the face
      if (!(v > opt.slab_margin_m))
        continue;
      for (int hinge = 0; hinge < 2; ++hinge) {
        const double u =
            hinge == 0 ? s - rect.s_start : rect.s_end - s; // into the door
        if (u < -opt.slab_margin_m)
          continue;
        const double r = std::hypot(u, v);
        if (r < opt.leaf_min_radius_share * width)
          continue;
        const double a = std::atan2(v, std::max(u, 0.0)) * 180.0 / kPi;
        const int bin =
            std::clamp(static_cast<int>(a / opt.leaf_bin_deg), 0, nbins - 1);
        const auto k =
            static_cast<std::size_t>((hinge * 2 + side) * nbins + bin);
        if (r <= 1.05 * width) {
          ++leaf[k];
          r_lo[k] = std::min(r_lo[k], r);
          r_hi[k] = std::max(r_hi[k], r);
        } else if (r <= 1.6 * width)
          ++beyond[k];
      }
    }
  }
  for (int hinge = 0; hinge < 2; ++hinge)
    for (int side = 0; side < 2; ++side)
      for (int bin = 0; bin < nbins; ++bin) {
        const auto k =
            static_cast<std::size_t>((hinge * 2 + side) * nbins + bin);
        if (leaf[k] < opt.min_leaf_points || leaf[k] <= best.points)
          continue;
        if (beyond[k] * 2 >= leaf[k]) // runs on past the leaf: a wall
          continue;
        if (r_hi[k] - r_lo[k] < 0.5 * (1.0 - opt.leaf_min_radius_share) * width)
          continue; // not a sheet running out from the hinge
        best.found = true;
        best.hinge_at_start = hinge == 0;
        best.side = side == 0 ? 1 : -1;
        best.angle_deg = (bin + 0.5) * opt.leaf_bin_deg;
        best.points = leaf[k];
      }
  return best;
}

namespace {

/// Natural key of an opening: its host wall and s_start rounded to 5 cm.
std::string opening_key(const std::string &wall_guid, double s_start) {
  const double r = std::round(s_start / 0.05) * 0.05;
  return fmt::format("openings/{}/{:.2f}", wall_guid, r == 0.0 ? 0.0 : r);
}

} // namespace

std::vector<DetectedOpening>
detect_openings(const OpeningEvidence &ev,
                const std::vector<core::WallRecord> &walls,
                const core::StoreyRecord &storey, const OpeningOptions &opt) {
  if (!ev.classes.empty() && ev.classes.size() != ev.points.size())
    throw std::invalid_argument(
        fmt::format("detect_openings: {} semantic classes for {} points",
                    ev.classes.size(), ev.points.size()));
  if (!ev.instance.empty() && ev.instance.size() != ev.points.size())
    throw std::invalid_argument(
        fmt::format("detect_openings: {} instance ids for {} points",
                    ev.instance.size(), ev.points.size()));
  std::vector<DetectedOpening> out;
  std::map<std::string, int> used;
  for (const auto &wall : walls) {
    if (wall.storey != storey.idx)
      continue;
    const AxisFrame f = axis_frame(wall);
    const double half = wall.thickness > 0.0 ? 0.5 * wall.thickness
                                             : opt.unknown_half_thickness_m;
    // The points near this wall: what the raster, the evidence and the leaf
    // search look at (a leaf sweeps up to a door's width out).
    const double reach = half + std::max(opt.front_m, 2.5);
    std::vector<std::size_t> near;
    std::vector<Eigen::Vector3d> local;
    for (std::size_t i = 0; i < ev.points.size(); ++i) {
      const Eigen::Vector2d q = ev.points[i].head<2>() - f.a;
      const double s = q.dot(f.d);
      if (s < -reach || s > f.length + reach || std::abs(q.dot(f.n)) > reach)
        continue;
      near.push_back(i);
      local.push_back(ev.points[i]);
    }
    // The floor and ceiling in front of this wall (drift between rooms
    // moves them off the storey's).
    const FaceLevels lv =
        face_levels(local, wall, storey.z_floor, storey.z_ceiling, opt);
    const double z_floor = lv.z_floor, z_ceiling = lv.z_ceiling;
    const auto raster = face_raster(local, wall, z_floor, z_ceiling, opt);
    const auto rects =
        merge_mullions(fit_openings(raster, z_floor, z_ceiling, opt), opt);
    if (rects.empty())
      continue;
    const auto crossings = trajectory_crossings(
        ev.trajectory, wall, storey.z_floor, storey.z_ceiling, opt);

    for (const auto &rect : rects) {
      DetectedOpening d;
      d.rect = rect;
      for (const double s : crossings)
        if (s >= rect.s_start - opt.crossing_tol_m &&
            s <= rect.s_end + opt.crossing_tol_m)
          ++d.crossings;
      // Nobody walks through a window: a low-silled rectangle the camera
      // path crosses is a door (or a passage) whose lower part was hidden
      // (a leaf, a chair) -- it reaches the floor.
      bool sill_from_path = false;
      if (d.crossings > 0 && !d.rect.at_floor &&
          d.rect.z_sill - z_floor <= opt.crossing_door_max_sill_m &&
          classify_opening(d.rect, z_floor, z_ceiling, opt) == "window") {
        d.rect.z_sill = z_floor;
        d.rect.at_floor = true;
        sill_from_path = true;
      }
      auto &o = d.record;
      o.kind = classify_opening(d.rect, z_floor, z_ceiling, opt);
      o.wall = wall.guid;
      o.s_start = rect.s_start;
      o.s_end = rect.s_end;
      o.z_sill = d.rect.z_sill;
      o.z_head = rect.z_head;
      o.is_external = wall.is_external;
      o.source = "detected";

      // Semantic evidence: the points on (and just in front of) the face
      // over the rectangle, grown by a cell to catch the frame.
      const double grow = opt.cell_m;
      const double band = half + opt.slab_margin_m + opt.evidence_band_m;
      int total = 0, n_door = 0, n_window = 0, n_glass = 0;
      std::map<int, int> per_instance;
      for (std::size_t k = 0; k < near.size(); ++k) {
        const auto &p = local[k];
        const Eigen::Vector2d q = p.head<2>() - f.a;
        const double s = q.dot(f.d);
        if (s < rect.s_start - grow || s > rect.s_end + grow ||
            p.z() < rect.z_sill - grow || p.z() > rect.z_head + grow ||
            std::abs(q.dot(f.n)) > band)
          continue;
        ++total;
        const std::size_t i = near[k];
        if (!ev.classes.empty()) {
          switch (ev.classes[i]) {
          case OpeningClass::door:
            ++n_door;
            break;
          case OpeningClass::window:
            ++n_window;
            break;
          case OpeningClass::glass:
            ++n_glass;
            break;
          case OpeningClass::none:
            break;
          }
        }
        if (!ev.instance.empty() && ev.instance[i] >= 0)
          ++per_instance[ev.instance[i]];
      }
      if (total > 0) {
        d.door_share = static_cast<double>(n_door) / total;
        d.window_share = static_cast<double>(n_window) / total;
        d.glass_label_share = static_cast<double>(n_glass) / total;
      }
      // The instance of an opening class with most points there.
      int best_n = 0;
      OpeningClass best_kind = OpeningClass::none;
      for (const auto &[id, n] : per_instance) {
        if (id < 0 || id >= static_cast<int>(ev.instances.size()))
          continue;
        const auto &inst = ev.instances[static_cast<std::size_t>(id)];
        if (inst.kind == OpeningClass::none || n < opt.min_instance_points ||
            static_cast<double>(n) < opt.min_class_share * total || n <= best_n)
          continue;
        best_n = n;
        best_kind = inst.kind;
        o.source_instance_guid = inst.guid;
      }
      d.glass_share = ev.glass ? ev.glass(wall, rect) : -1.0;
      if (o.kind == "door") {
        d.leaf = door_leaf(local, wall, d.rect, opt);
        o.state = d.leaf.found ? "open" : "unknown";
      }

      // Confidence: the geometry alone, at most 0.7, raised by evidence that
      // agrees with the kind.
      d.geometry_confidence =
          std::min(0.7, 0.15 + 0.45 * rect.fill +
                            0.1 * std::min(rect.jamb_start, rect.jamb_end));
      double c = d.geometry_confidence;
      const bool is_door = o.kind == "door", is_window = o.kind == "window";
      const bool label_support =
          is_door     ? d.door_share >= opt.min_class_share
          : is_window ? std::max(d.window_share, d.glass_label_share) >=
                            opt.min_class_share
                      : std::max({d.door_share, d.glass_label_share}) >=
                            opt.min_class_share;
      const bool instance_support =
          !o.source_instance_guid.empty() &&
          (is_door     ? best_kind == OpeningClass::door
           : is_window ? best_kind != OpeningClass::door
                       : true);
      if (label_support)
        c += 0.15;
      if (instance_support)
        c += 0.1;
      if (!is_window)
        c += std::min(0.2, 0.1 * d.crossings);
      if (is_window && d.glass_share >= opt.min_glass_share)
        c += 0.1;
      o.confidence = std::min(1.0, c);

      nlohmann::json meta = {{"fill", rect.fill},
                             {"open_share", rect.open_share},
                             {"z_floor_local", z_floor},
                             {"z_ceiling_local", z_ceiling},
                             {"jamb_start", rect.jamb_start},
                             {"jamb_end", rect.jamb_end},
                             {"geometry_confidence", d.geometry_confidence},
                             {"door_share", d.door_share},
                             {"window_share", d.window_share},
                             {"glass_label_share", d.glass_label_share},
                             {"crossings", d.crossings}};
      if (sill_from_path)
        meta["sill_from_camera_path"] = true;
      if (d.glass_share >= 0.0)
        meta["glass_share"] = d.glass_share;
      if (d.leaf.found) {
        // Seen from the side it swings to, the hinge is on the right when it
        // is at s_start from side a (+t) or at s_end from side b.
        const bool right = (d.leaf.side > 0) == d.leaf.hinge_at_start;
        meta["leaf"] = {{"hinge", d.leaf.hinge_at_start ? "start" : "end"},
                        {"side", d.leaf.side > 0 ? "a" : "b"},
                        {"angle_deg", d.leaf.angle_deg},
                        {"points", d.leaf.points}};
        meta["swing"] = right ? "right" : "left";
      }
      o.metadata = meta.dump();

      std::string key = opening_key(wall.guid, rect.s_start);
      if (const int n = used[key]++; n > 0)
        key += fmt::format("#{}", n);
      o.guid = key;
      out.push_back(std::move(d));
    }
  }
  return out;
}

} // namespace reusex::geometry
