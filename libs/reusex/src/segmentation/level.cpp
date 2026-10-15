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
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace reusex::geometry {
namespace {

/// Passes of (surfaces -> fit -> re-estimate the vertical). The second pass
/// re-measures heights along the measured vertical; a third only runs if
/// that moved it.
constexpr int kMaxPasses = 3;
constexpr double kVerticalConvergedRad = 1e-7;
constexpr double kRadToDeg = 180.0 / M_PI;

/// Which way a horizontal plane was seen from, by its stored normal's sign.
enum class Facing : int { any = 0, up = 1, down = -1 };

/// One horizontal plane of the segmentation.
struct HPlane {
  std::vector<size_t> idx; ///< into the horizontal points
  Facing facing = Facing::any;
  size_t index = 0;    ///< plane index (tie-break)
  double height = 0.0; ///< mean of its points . v
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(); ///< oriented to +z
};

/// One fitted horizontal surface (a floor, a ceiling, a slab face): planes
/// at one height (and facing), fitted on their own points.
struct Surface {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(); ///< oriented with +v
  double height = 0.0; ///< centroid . v at fit time [m]
  size_t support = 0;  ///< points of its planes (its strength)
  size_t points = 0;   ///< inliers of the final fit
  double rms = 0.0;    ///< RMS distance of the inliers to the plane [m]
  bool fitted = false; ///< false: < 3 inliers, plane = seed height along v
  Facing facing = Facing::any;
};

/// Surfaces closer than slab_merge_m, ascending.
struct Level {
  std::vector<const Surface *> surfaces;
  [[nodiscard]] const Surface &bottom() const { return *surfaces.front(); }
  [[nodiscard]] const Surface &top() const { return *surfaces.back(); }
  [[nodiscard]] const Surface &strongest() const {
    return **std::max_element(surfaces.begin(), surfaces.end(),
                              [](const Surface *a, const Surface *b) {
                                return a->support < b->support;
                              });
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

/// Iterated LSQ over the points @p pool of one surface: start from @p seed
/// (its strongest plane), then refit on the pool points within inlier_m of
/// the plane until the inlier set stops changing or max_fit_iterations is
/// reached. Only the surface's own planes take part, so on a drifted scan a
/// tilted fit cannot pick up another surface's points at the far end.
Surface fit_surface(const std::vector<Eigen::Vector3d> &pts,
                    const std::vector<size_t> &seed,
                    const std::vector<size_t> &pool, const Eigen::Vector3d &v,
                    double h0, const LevelOptions &opt) {
  Surface s;
  s.height = h0;
  s.normal = v;
  s.centroid = v * h0;
  s.support = pool.size();
  std::vector<size_t> sel = seed;
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
    for (size_t i : pool)
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

/// Group the planes into surfaces: strongest plane first (ties by index),
/// each joins the first surface of the same facing whose seed height is
/// within two bins, else seeds a new one. Seeds do not move, so the groups
/// cannot chain. Returns fitted surfaces, ascending by height.
std::vector<Surface> build_surfaces(const std::vector<Eigen::Vector3d> &pts,
                                    const std::vector<HPlane> &planes,
                                    const Eigen::Vector3d &v,
                                    const LevelOptions &opt) {
  std::vector<size_t> order(planes.size());
  for (size_t k = 0; k < order.size(); ++k)
    order[k] = k;
  std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    if (planes[a].idx.size() != planes[b].idx.size())
      return planes[a].idx.size() > planes[b].idx.size();
    return planes[a].index < planes[b].index;
  });
  struct Group {
    size_t seed;
    std::vector<size_t> pool;
  };
  std::vector<Group> groups;
  const double tol = 2.0 * opt.hist_bin_m;
  for (size_t k : order) {
    const HPlane &p = planes[k];
    auto it = std::find_if(groups.begin(), groups.end(), [&](const Group &g) {
      const HPlane &seed = planes[g.seed];
      return seed.facing == p.facing && std::abs(seed.height - p.height) <= tol;
    });
    if (it == groups.end()) {
      groups.push_back(Group{k, {}});
      it = std::prev(groups.end());
    }
    it->pool.insert(it->pool.end(), p.idx.begin(), p.idx.end());
  }
  std::vector<Surface> surfaces;
  surfaces.reserve(groups.size());
  for (auto &g : groups) {
    std::sort(g.pool.begin(), g.pool.end());
    Surface s = fit_surface(pts, planes[g.seed].idx, g.pool, v,
                            planes[g.seed].height, opt);
    s.facing = planes[g.seed].facing;
    surfaces.push_back(std::move(s));
  }
  std::sort(surfaces.begin(), surfaces.end(),
            [](const Surface &a, const Surface &b) {
              return a.height != b.height ? a.height < b.height
                                          : a.support > b.support;
            });
  return surfaces;
}

/// Surfaces of @p facing (any = all) with at least peak_min_share of the
/// strongest such surface's support, ascending.
std::vector<const Surface *> significant(const std::vector<Surface> &surfaces,
                                         Facing facing,
                                         const LevelOptions &opt) {
  size_t max_support = 0;
  for (const auto &s : surfaces)
    if (facing == Facing::any || s.facing == facing)
      max_support = std::max(max_support, s.support);
  std::vector<const Surface *> out;
  for (const auto &s : surfaces)
    if ((facing == Facing::any || s.facing == facing) &&
        static_cast<double>(s.support) >=
            opt.peak_min_share * static_cast<double>(max_support))
      out.push_back(&s);
  return out;
}

/// Group surfaces (ascending) into levels: a gap below slab_merge_m keeps
/// the next surface in the current level.
std::vector<Level> group_levels(const std::vector<const Surface *> &surfaces,
                                const LevelOptions &opt) {
  std::vector<Level> levels;
  for (const Surface *s : surfaces) {
    if (!levels.empty() &&
        s->height - levels.back().top().height < opt.slab_merge_m)
      levels.back().surfaces.push_back(s);
    else
      levels.push_back(Level{{s}});
  }
  return levels;
}

/// The first level (of @p cand, ascending) that starts at or above @p from,
/// grown upward while the gaps stay below slab_merge_m; empty when none.
Level first_level_from(const std::vector<const Surface *> &cand, double from,
                       const LevelOptions &opt) {
  Level l;
  for (const Surface *s : cand) {
    if (s->height < from)
      continue;
    if (!l.surfaces.empty() && s->height - l.top().height >= opt.slab_merge_m)
      break;
    l.surfaces.push_back(s);
  }
  return l;
}

std::string no_ceiling_reason(const LevelOptions &opt) {
  return fmt::format("no level at least {:.2f} m above the floor; ceiling "
                     "height defaulted to {:.2f} m",
                     opt.min_storey_height_m, opt.default_height_m);
}

/// Walk the levels bottom-up, by height alone. The lowest level's strongest
/// surface is the storey-0 floor; a storey's ceiling is the bottom of the
/// first level at least min_storey_height_m above its floor (levels in
/// between — tables, shelves — are skipped); the next floor is the top of
/// that level, unless it is the last level and a single surface (the roof
/// ceiling). A floor with no ceiling above gets default_height_m.
std::vector<StoreySpan> walk_by_height(const std::vector<Level> &levels,
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
      span.reason = no_ceiling_reason(opt);
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

/// Walk the storeys bottom-up using which way each surface faces. The
/// storey-0 floor is the strongest up-facing surface of the lowest
/// up-facing level. A storey's ceiling is the strongest down-facing surface
/// of the first down-facing level at least min_storey_height_m above its
/// floor, among surfaces with peak_min_share of the floor's support. The
/// next floor is the strongest up-facing surface above that ceiling level
/// and within slab_merge_m of its top (the top of the slab) — no share
/// rule, so a small or drift-smeared upper floor is not lost against the
/// ground floor; none means the ceiling is the roof. With no ceiling, the
/// next floor is the next significant up-facing level, if any.
std::vector<StoreySpan> walk_by_facing(const std::vector<Surface> &surfaces,
                                       const LevelOptions &opt) {
  std::vector<StoreySpan> spans;
  const auto floors = significant(surfaces, Facing::up, opt);
  if (floors.empty())
    return spans;
  Surface floor = group_levels(floors, opt).front().strongest();
  for (;;) {
    StoreySpan span;
    span.floor = floor;
    std::vector<const Surface *> ceilings;
    for (const auto &s : surfaces)
      if (s.facing == Facing::down &&
          static_cast<double>(s.support) >=
              opt.peak_min_share * static_cast<double>(floor.support))
        ceilings.push_back(&s);
    const Level cl =
        first_level_from(ceilings, floor.height + opt.min_storey_height_m, opt);

    const Surface *next = nullptr;
    if (cl.surfaces.empty()) {
      span.ceiling_assumed = true;
      span.reason = no_ceiling_reason(opt);
      const Level fl =
          first_level_from(floors, floor.height + opt.min_storey_height_m, opt);
      if (!fl.surfaces.empty())
        next = &fl.strongest();
    } else {
      span.ceiling = cl.strongest();
      const double lo = cl.bottom().height;
      const double hi = cl.top().height + opt.slab_merge_m;
      for (const auto &s : surfaces)
        if (s.facing == Facing::up && s.height > lo && s.height <= hi &&
            (next == nullptr || s.support > next->support))
          next = &s;
    }
    spans.push_back(std::move(span));
    if (next == nullptr)
      break;
    floor = *next;
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

double angle_deg(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return std::acos(std::clamp(a.dot(b), -1.0, 1.0)) * kRadToDeg;
}

/// Evidence for one surface; tilt_deg is the angle between its fitted normal
/// and the vertical @p v of the frame its storey is expressed in.
nlohmann::json surface_json(const Surface &s, const Eigen::Vector3d &v) {
  return {{"points", s.points},
          {"support", s.support},
          {"rms_m", s.rms},
          {"fitted", s.fitted},
          {"tilt_deg", angle_deg(s.normal, v)}};
}

/// A storey's own vertical: the bisector of its fitted floor and ceiling
/// normals (the floor's alone without a fitted ceiling); @p shared when
/// neither is fitted.
Eigen::Vector3d storey_vertical(const StoreySpan &sp,
                                const Eigen::Vector3d &shared) {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  if (sp.floor.fitted)
    sum += sp.floor.normal;
  if (!sp.ceiling_assumed && sp.ceiling.fitted)
    sum += sp.ceiling.normal;
  return sum.norm() < 1e-12 ? shared : sum.normalized();
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

  // Horizontal planes, normals oriented to world +z; the stored sign is
  // kept as the plane's facing.
  const size_t n_planes = plane_normals.size();
  std::vector<int> hplane_of(n_planes, -1);
  std::vector<Eigen::Vector3d> normal(n_planes);
  std::vector<HPlane> hplanes;
  StoreysResult result;
  for (size_t k = 0; k < n_planes; ++k) {
    const auto &pn = plane_normals.points[k];
    Eigen::Vector3d n(pn.normal_x, pn.normal_y, pn.normal_z);
    const double len = n.norm();
    if (!(len > 0.0) || !std::isfinite(len))
      continue;
    n /= len;
    const Facing facing = n.z() < 0.0 ? Facing::down : Facing::up;
    if (n.z() < 0.0)
      n = -n;
    normal[k] = n;
    if (n.z() > opt.horizontal_cos) {
      hplane_of[k] = static_cast<int>(hplanes.size());
      HPlane hp;
      hp.facing = facing;
      hp.index = k;
      hp.normal = n;
      hplanes.push_back(std::move(hp));
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
    if (hplane_of[k] < 0)
      continue;
    const auto &p = cloud.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    hplanes[static_cast<size_t>(hplane_of[k])].idx.push_back(pts.size());
    pts.emplace_back(p.x, p.y, p.z);
    v += normal[k];
  }
  hplanes.erase(std::remove_if(hplanes.begin(), hplanes.end(),
                               [](const HPlane &h) { return h.idx.empty(); }),
                hplanes.end());

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
                    {"ceiling_assumed", true},
                    {"frame", "shared"}});
  }

  // Does the stored sign say floor / ceiling? Normals from before
  // reusex-0pi all face the world origin, which makes every plane above the
  // origin "down" — upper floors included. A sensor-oriented cloud has
  // floors seen from above facing up; once any horizontal plane faces away
  // from the origin the sign cannot be origin-made, and both facings must
  // be present for it to tell anything apart.
  {
    size_t up = 0, down = 0, away = 0;
    for (const auto &h : hplanes) {
      Eigen::Vector3d c = Eigen::Vector3d::Zero();
      for (size_t i : h.idx)
        c += pts[i];
      c /= static_cast<double>(h.idx.size());
      const double nz = h.facing == Facing::up ? 1.0 : -1.0;
      (h.facing == Facing::up ? up : down) += h.idx.size();
      // n . (c - 0) > 0: the normal points away from the origin (n ~ ±z).
      if (nz * c.z() > 0.0)
        away += h.idx.size();
    }
    result.normal_sign_used =
        opt.use_normal_sign && up > 0 && down > 0 && away > 0;
    if (opt.use_normal_sign && !result.normal_sign_used)
      reusex::info("detect_storeys: plane normal sign not used ({} up-, {} "
                   "down-facing points, {} facing away from the world origin); "
                   "floors and ceilings are told apart by height alone",
                   up, down, away);
    if (!result.normal_sign_used)
      for (auto &h : hplanes)
        h.facing = Facing::any;
  }

  v.normalize();
  std::vector<Surface> surfaces;
  std::vector<StoreySpan> spans;
  for (int pass = 0; pass < kMaxPasses; ++pass) {
    for (auto &h : hplanes) {
      double sum = 0.0;
      for (size_t i : h.idx)
        sum += pts[i].dot(v);
      h.height = sum / static_cast<double>(h.idx.size());
    }
    // Floor/ceiling candidates: planes within surface_max_tilt_deg of the
    // current vertical. A sloped ceiling or a ramp passes horizontal_cos
    // (measured against world z, generous so a tilted scan still has
    // horizontal planes) but is not a floor or ceiling, and as a surface of
    // its own it would pull the vertical.
    std::vector<HPlane> level_planes;
    const double max_tilt_cos = std::cos(opt.surface_max_tilt_deg / kRadToDeg);
    for (const auto &h : hplanes)
      if (h.normal.dot(v) >= max_tilt_cos)
        level_planes.push_back(h);
    if (level_planes.empty())
      level_planes = hplanes;
    surfaces = build_surfaces(pts, level_planes, v, opt);
    spans =
        result.normal_sign_used
            ? walk_by_facing(surfaces, opt)
            : walk_by_height(
                  group_levels(significant(surfaces, Facing::any, opt), opt),
                  opt);
    if (spans.empty()) // sign in use but no significant up-facing surface
      spans = walk_by_height(
          group_levels(significant(surfaces, Facing::any, opt), opt), opt);

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

  // Levels: significant surfaces (per facing when the sign is in use)
  // grouped by height.
  {
    std::vector<const Surface *> sig;
    if (result.normal_sign_used) {
      for (Facing f : {Facing::up, Facing::down})
        for (const Surface *s : significant(surfaces, f, opt))
          sig.push_back(s);
      std::sort(sig.begin(), sig.end(), [](const Surface *a, const Surface *b) {
        return a->height < b->height;
      });
    } else {
      sig = significant(surfaces, Facing::any, opt);
    }
    result.levels = static_cast<int>(group_levels(sig, opt).size());
  }
  result.vertical = {v.x(), v.y(), v.z()};

  // The shared level frame: origin on the storey-0 floor centroid. Storey
  // i's floor height is its floor centroid's height there, and its ceiling
  // height is measured on the vertical through that centroid (the storey's
  // own height, not one extrapolated across a tilted surface). A storey
  // whose own vertical is more than storey_frame_tol_deg off v gets a frame
  // rotated to its own vertical and placed so that its floor centroid keeps
  // its shared-frame level coordinates: its floor stays at z_floor and its
  // x/y agree with the shared frame's there.
  const Eigen::Vector3d origin = spans.front().floor.centroid;
  const LevelFrame shared = make_level_frame(v, origin);
  const auto shared_transform = shared.to_array();

  for (size_t i = 0; i < spans.size(); ++i) {
    const auto &sp = spans[i];
    const Eigen::Vector3d own_v = storey_vertical(sp, v);
    const double tilt = angle_deg(own_v, v);
    result.max_storey_tilt_deg = std::max(result.max_storey_tilt_deg, tilt);
    const bool own = opt.per_storey_frame && tilt > opt.storey_frame_tol_deg;

    core::StoreyRecord s;
    s.idx = static_cast<int>(i);
    const Eigen::Vector3d anchor = sp.floor.centroid;
    const Eigen::Vector3d at = shared.to_level(anchor);
    s.z_floor = i == 0 ? 0.0 : at.z();
    const Eigen::Vector3d frame_v = own ? own_v : v;
    if (own) {
      ++result.own_frames;
      LevelFrame f = make_level_frame(own_v, anchor);
      f.world_to_level.translation() +=
          Eigen::Vector3d(at.x(), at.y(), s.z_floor);
      s.level_transform = f.to_array();
    } else {
      s.level_transform = shared_transform;
    }
    s.z_ceiling =
        s.z_floor + (sp.ceiling_assumed
                         ? opt.default_height_m
                         : height_on_axis(sp.ceiling, anchor, frame_v));
    nlohmann::json meta = {{"floor", surface_json(sp.floor, frame_v)},
                           {"ceiling_assumed", sp.ceiling_assumed},
                           {"frame", own ? "storey" : "shared"},
                           {"vertical", {own_v.x(), own_v.y(), own_v.z()}},
                           {"tilt_vs_shared_deg", tilt},
                           {"normal_sign_used", result.normal_sign_used}};
    if (!sp.ceiling_assumed)
      meta["ceiling"] = surface_json(sp.ceiling, frame_v);
    if (!sp.reason.empty())
      meta["reason"] = sp.reason;
    s.metadata = meta.dump();
    result.storeys.push_back(std::move(s));
  }
  // An assumed ceiling must not reach into the storey above.
  for (size_t i = 0; i + 1 < result.storeys.size(); ++i)
    if (spans[i].ceiling_assumed)
      result.storeys[i].z_ceiling =
          std::min(result.storeys[i].z_ceiling, result.storeys[i + 1].z_floor);

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
                 "planes ({} points); storey verticals up to {:.3f} deg off "
                 "the shared one, {} storey(s) in their own frame",
                 result.storeys.size(), result.levels, result.horizontal_planes,
                 pts.size(), result.max_storey_tilt_deg, result.own_frames);
  }
  for (size_t i = 0; i < spans.size(); ++i)
    if (spans[i].ceiling_assumed)
      reusex::warn("detect_storeys: storey {} has no ceiling level; height "
                   "defaulted to {:.2f} m",
                   i, opt.default_height_m);
  return result;
}

} // namespace reusex::geometry
