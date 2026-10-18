// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segmentation/classify_planes.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"
#include "geometry/level_frame.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace reusex::geometry {

std::string_view to_string(RoleSource source) noexcept {
  switch (source) {
  case RoleSource::none:
    return "none";
  case RoleSource::semantic:
    return "semantic";
  case RoleSource::rules:
    return "rules";
  case RoleSource::overruled:
    return "overruled";
  }
  return "none";
}

namespace {

using core::PlaneRole;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

enum class Orientation { horizontal, vertical, slanted };

const char *orientation_name(Orientation o) {
  switch (o) {
  case Orientation::horizontal:
    return "horizontal";
  case Orientation::vertical:
    return "vertical";
  case Orientation::slanted:
    return "slanted";
  }
  return "slanted";
}

/// Exact order statistic (lower), @p q in [0, 1], of a sorted vector.
double quantile(const std::vector<double> &sorted, double q) {
  const auto n = static_cast<double>(sorted.size() - 1);
  return sorted[static_cast<size_t>(std::floor(q * n))];
}

/// Area of the points @p idx on the plane with unit normal @p n: occupied
/// cells of a grid in the plane, anchored at the world origin.
double occupied_area(const Cloud &cloud, const std::vector<size_t> &idx,
                     Eigen::Vector3d n, double cell) {
  // Canonical sign, so a flipped normal gives the same grid.
  int big = 0;
  n.cwiseAbs().maxCoeff(&big);
  if (n[big] < 0.0)
    n = -n;
  const Eigen::Vector3d u = n.unitOrthogonal();
  const Eigen::Vector3d w = n.cross(u);
  std::vector<std::pair<int64_t, int64_t>> cells;
  cells.reserve(idx.size());
  for (size_t i : idx) {
    const auto &p = cloud.points[i];
    const Eigen::Vector3d x(p.x, p.y, p.z);
    cells.emplace_back(static_cast<int64_t>(std::floor(x.dot(u) / cell)),
                       static_cast<int64_t>(std::floor(x.dot(w) / cell)));
  }
  std::sort(cells.begin(), cells.end());
  const auto unique = static_cast<double>(
      std::unique(cells.begin(), cells.end()) - cells.begin());
  return unique * cell * cell;
}

/// Heights of the plane's points, in one frame.
struct Heights {
  std::vector<double> sorted;
  [[nodiscard]] double median() const { return quantile(sorted, 0.5); }
};

Heights heights_in(const Cloud &cloud, const std::vector<size_t> &idx,
                   const LevelFrame &frame) {
  // z of the level frame = third row of world_to_level.
  const Eigen::Matrix4d m = frame.world_to_level.matrix();
  Heights h;
  h.sorted.reserve(idx.size());
  for (size_t i : idx) {
    const auto &p = cloud.points[i];
    h.sorted.push_back(m(2, 0) * p.x + m(2, 1) * p.y + m(2, 2) * p.z + m(2, 3));
  }
  std::sort(h.sorted.begin(), h.sorted.end());
  return h;
}

/// Floor / ceiling heights the rules compare against (NaN = none).
struct Reference {
  double floor = kNaN;
  double ceiling = kNaN;
  /// Heights of split-level floors in the storey (floor-role planes outside
  /// the floor band); a wall may stand on one.
  std::vector<double> split_floors;
  [[nodiscard]] bool has_floor() const { return !std::isnan(floor); }
  [[nodiscard]] bool has_ceiling() const { return !std::isnan(ceiling); }
  /// Mid-height of the storey, NaN without both surfaces.
  [[nodiscard]] double middle() const { return 0.5 * (floor + ceiling); }
};

struct PlaneGeometry {
  Orientation orientation = Orientation::slanted;
  /// +1 the normal faces up, -1 down, 0 unknown or ignored.
  int facing = 0;
  double area = 0.0;
  double z_med = 0.0, z_lo = 0.0, z_hi = 0.0; ///< robust, plane's frame
  double z_min = 0.0, z_max = 0.0;
  [[nodiscard]] bool may_face_up() const { return facing >= 0; }
  [[nodiscard]] bool may_face_down() const { return facing <= 0; }
};

bool in_floor_band(const PlaneGeometry &g, const Reference &ref,
                   const ClassifyPlanesOptions &opt) {
  return ref.has_floor() && std::abs(g.z_med - ref.floor) <= opt.band_m;
}

bool in_ceiling_band(const PlaneGeometry &g, const Reference &ref,
                     const ClassifyPlanesOptions &opt) {
  return ref.has_ceiling() && std::abs(g.z_med - ref.ceiling) <= opt.band_m;
}

/// Below / above the storey's mid-height (false when it is unknown).
bool lower_half(const PlaneGeometry &g, const Reference &ref) {
  return ref.has_floor() && ref.has_ceiling() && g.z_med < ref.middle();
}
bool upper_half(const PlaneGeometry &g, const Reference &ref) {
  return ref.has_floor() && ref.has_ceiling() && g.z_med > ref.middle();
}

/// Rules for a horizontal plane (they do not depend on other planes).
PlaneRole horizontal_rule(const PlaneGeometry &g, const Reference &ref,
                          const ClassifyPlanesOptions &opt) {
  const bool large = g.area >= opt.min_large_area_m2;
  const bool structural = g.area > opt.max_furniture_area_m2;
  if (large && g.may_face_up() && in_floor_band(g, ref, opt))
    return PlaneRole::floor;
  if (large && g.may_face_down() && in_ceiling_band(g, ref, opt))
    return PlaneRole::ceiling;
  if (structural && g.may_face_up() && lower_half(g, ref))
    return PlaneRole::floor; // split level / raised floor
  if (structural && g.may_face_down() && upper_half(g, ref))
    return PlaneRole::ceiling; // lowered ceiling, soffit
  if (!ref.has_floor())
    return PlaneRole::unknown;
  const double above = g.z_med - ref.floor;
  if (!structural && g.may_face_up() && above >= opt.table_min_m &&
      above <= opt.table_max_m)
    return PlaneRole::furniture;
  return PlaneRole::clutter;
}

/// The geometric rules (Poux 2019/2020).
PlaneRole rule_role(const PlaneGeometry &g, const Reference &ref,
                    bool in_storey, const ClassifyPlanesOptions &opt) {
  if (!in_storey)
    return PlaneRole::unknown;
  switch (g.orientation) {
  case Orientation::horizontal:
    return horizontal_rule(g, ref, opt);
  case Orientation::vertical: {
    bool touches = (!ref.has_floor() && !ref.has_ceiling()) ||
                   (ref.has_floor() && g.z_lo <= ref.floor + opt.band_m) ||
                   (ref.has_ceiling() && g.z_hi >= ref.ceiling - opt.band_m);
    // The floor it stands on: the storey's, or a split level it touches.
    double base = ref.floor;
    for (double f : ref.split_floors)
      if (std::abs(g.z_lo - f) <= opt.band_m) {
        touches = true;
        base = f;
        break;
      }
    double min_height = opt.min_wall_height_m;
    if (!std::isnan(base) && ref.has_ceiling())
      min_height = std::min(min_height,
                            opt.min_wall_height_share * (ref.ceiling - base));
    if (touches && g.z_hi - g.z_lo >= min_height)
      return PlaneRole::wall;
    return PlaneRole::clutter;
  }
  case Orientation::slanted:
    if (g.area >= opt.min_large_area_m2 && g.may_face_down() &&
        upper_half(g, ref))
      return PlaneRole::ceiling; // pitched ceiling
    return PlaneRole::unknown;
  }
  return PlaneRole::unknown;
}

/// Does the geometry allow the role the class map gave?
bool class_role_fits(PlaneRole role, PlaneRole rules, const PlaneGeometry &g,
                     const Reference &ref, const ClassifyPlanesOptions &opt) {
  const bool horizontal = g.orientation == Orientation::horizontal;
  switch (role) {
  case PlaneRole::wall:
    return g.orientation == Orientation::vertical;
  case PlaneRole::floor:
    return horizontal && g.may_face_up() &&
           (!ref.has_floor() || rules == PlaneRole::floor ||
            in_floor_band(g, ref, opt));
  case PlaneRole::ceiling:
    if (!g.may_face_down())
      return false;
    if (horizontal)
      return !ref.has_ceiling() || rules == PlaneRole::ceiling ||
             in_ceiling_band(g, ref, opt);
    // A pitched (attic) ceiling: slanted and above table height.
    return g.orientation == Orientation::slanted &&
           (!ref.has_floor() || g.z_med - ref.floor >= opt.table_max_m);
  case PlaneRole::slab:
    return horizontal;
  case PlaneRole::furniture:
  case PlaneRole::clutter:
    // A large horizontal plane in the floor / ceiling band is structure,
    // whatever is standing on or hanging from it.
    return rules != PlaneRole::floor && rules != PlaneRole::ceiling;
  case PlaneRole::unknown:
    return true;
  }
  return true;
}

/// The role most of a plane's labelled points vote for through their class,
/// when it holds at least @p min_share of the @p labelled votes; ties go to
/// the lower PlaneRole value (deterministic). Points of a class with no name
/// or no role vote for nothing but still count in @p labelled.
std::optional<PlaneRole>
majority_role(const std::vector<std::pair<uint32_t, int>> &votes,
              const std::map<uint32_t, std::string> &class_names,
              const core::ClassRoleMap &role_map, int labelled,
              double min_share) {
  if (labelled <= 0)
    return std::nullopt;
  std::map<uint32_t, int> per_class;
  for (const auto &[label, n] : votes)
    per_class[label] += n;
  std::map<PlaneRole, int> per_role;
  for (const auto &[label, n] : per_class) {
    const auto name = class_names.find(label);
    if (name == class_names.end())
      continue;
    if (const auto role = role_map.role_of(name->second))
      per_role[*role] += n;
  }
  std::optional<PlaneRole> best;
  int best_n = 0;
  for (const auto &[role, n] : per_role)
    if (n > best_n) {
      best = role;
      best_n = n;
    }
  const auto share = [labelled](int n) {
    return static_cast<double>(n) / static_cast<double>(labelled);
  };
  if (best && share(best_n) >= min_share)
    return best;
  // No single role has the majority, but the object roles together may: a
  // pier or cupboard box votes "column" and "cabinet" (clutter + furniture)
  // over a little "wall". Then it is an object, whichever of the two has
  // more votes (ties: furniture).
  const auto votes_for = [&per_role](PlaneRole role) {
    const auto it = per_role.find(role);
    return it == per_role.end() ? 0 : it->second;
  };
  const int furniture = votes_for(PlaneRole::furniture);
  const int clutter = votes_for(PlaneRole::clutter);
  if (share(furniture + clutter) >= min_share)
    return furniture >= clutter ? PlaneRole::furniture : PlaneRole::clutter;
  return std::nullopt;
}

void check_options(const ClassifyPlanesOptions &o) {
  const auto bad = [](const char *what) {
    throw std::invalid_argument(
        fmt::format("classify_planes: option {} is out of range", what));
  };
  if (!(o.horizontal_cos > 0.0 && o.horizontal_cos <= 1.0))
    bad("horizontal_cos");
  if (!(o.vertical_cos >= 0.0 && o.vertical_cos < o.horizontal_cos))
    bad("vertical_cos");
  if (!(o.area_cell_m > 0.0))
    bad("area_cell_m");
  if (!(o.band_m >= 0.0))
    bad("band_m");
  if (!(o.extent_quantile >= 0.0 && o.extent_quantile < 0.5))
    bad("extent_quantile");
  if (!(o.table_min_m <= o.table_max_m))
    bad("table_min_m/table_max_m");
  if (!(o.min_wall_height_share >= 0.0))
    bad("min_wall_height_share");
  if (!(o.min_class_share >= 0.0 && o.min_class_share <= 1.0))
    bad("min_class_share");
  if (!(o.min_labelled_share >= 0.0 && o.min_labelled_share <= 1.0))
    bad("min_labelled_share");
}

} // namespace

ClassifyPlanesResult classify_planes(
    const Cloud &cloud, const CloudL &plane_labels, const CloudN &plane_normals,
    const std::vector<core::StoreyRecord> &storeys,
    const SemanticLabels &semantic, const core::ClassRoleMap &role_map,
    const ClassifyPlanesOptions &opt) {
  check_options(opt);
  if (cloud.size() != plane_labels.size())
    throw std::invalid_argument(fmt::format(
        "classify_planes: cloud has {} points but the planes cloud has {}",
        cloud.size(), plane_labels.size()));
  if (semantic.labels && semantic.labels->size() != cloud.size())
    throw std::invalid_argument(fmt::format(
        "classify_planes: cloud has {} points but the labels cloud has {}",
        cloud.size(), semantic.labels->size()));

  // Inliers per plane, in point order.
  const size_t n_planes = plane_normals.size();
  std::vector<std::vector<size_t>> inliers(n_planes);
  for (size_t i = 0; i < cloud.size(); ++i) {
    const uint32_t label = plane_labels.points[i].label;
    if (!core::is_valid_label(label) || core::is_out_of_contract_label(label))
      continue;
    const size_t k = core::label_to_index(label);
    if (k >= n_planes)
      throw std::invalid_argument(fmt::format(
          "classify_planes: point {} has plane label {} but only {} planes "
          "exist",
          i, label, n_planes));
    const auto &p = cloud.points[i];
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      inliers[k].push_back(i);
  }

  // Storeys in idx order; each has its own level frame (in one run they
  // share it, but a locked row may keep an older one).
  std::vector<core::StoreyRecord> st = storeys;
  std::stable_sort(st.begin(), st.end(),
                   [](const auto &a, const auto &b) { return a.idx < b.idx; });
  std::vector<LevelFrame> frames;
  frames.reserve(st.size());
  for (const auto &s : st)
    frames.push_back(LevelFrame::from_record(s));
  const LevelFrame ref_frame = st.empty() ? LevelFrame{} : frames.front();
  const Eigen::Vector3d up =
      ref_frame.world_to_level.linear().transpose() * Eigen::Vector3d::UnitZ();

  ClassifyPlanesResult result;
  result.implicit_floor_z = kNaN;
  result.implicit_ceiling_z = kNaN;

  // Geometry per plane, in the reference frame.
  std::vector<PlaneGeometry> geo(n_planes);
  std::vector<int> storey_pos(n_planes, -1); // index into st
  for (size_t k = 0; k < n_planes; ++k) {
    auto &g = geo[k];
    const auto &pn = plane_normals.points[k];
    Eigen::Vector3d n(pn.normal_x, pn.normal_y, pn.normal_z);
    const double len = n.norm();
    if (!(len > 0.0) || !std::isfinite(len)) {
      g.orientation = Orientation::slanted;
      n = Eigen::Vector3d::UnitZ();
    } else {
      n /= len;
      const double c = std::abs(n.dot(up));
      g.orientation = c > opt.horizontal_cos ? Orientation::horizontal
                      : c < opt.vertical_cos ? Orientation::vertical
                                             : Orientation::slanted;
      if (opt.oriented_normals && g.orientation != Orientation::vertical)
        g.facing = n.dot(up) > 0.0 ? 1 : -1;
    }
    if (inliers[k].empty())
      continue;
    g.area = occupied_area(cloud, inliers[k], n, opt.area_cell_m);

    // Storey: among those whose [floor, ceiling] (in its own frame) lies
    // within band_m of the plane's median height, the nearest; a horizontal
    // plane of known facing goes to the storey whose floor (facing up) or
    // ceiling (facing down) is nearest, so the surfaces either side of a
    // slab split correctly. Ties go to the lower idx.
    LevelFrame frame = ref_frame;
    if (!st.empty()) {
      const bool by_surface =
          g.orientation == Orientation::horizontal && g.facing != 0;
      double best = std::numeric_limits<double>::infinity();
      for (size_t s = 0; s < st.size(); ++s) {
        const double z = heights_in(cloud, inliers[k], frames[s]).median();
        const double d =
            std::max({st[s].z_floor - z, z - st[s].z_ceiling, 0.0});
        if (d > opt.band_m)
          continue;
        const double key = !by_surface    ? d
                           : g.facing > 0 ? std::abs(z - st[s].z_floor)
                                          : std::abs(z - st[s].z_ceiling);
        if (key < best) {
          best = key;
          storey_pos[k] = static_cast<int>(s);
        }
      }
      if (storey_pos[k] >= 0)
        frame = frames[static_cast<size_t>(storey_pos[k])];
    }
    const Heights h = heights_in(cloud, inliers[k], frame);
    g.z_med = h.median();
    g.z_lo = quantile(h.sorted, opt.extent_quantile);
    g.z_hi = quantile(h.sorted, 1.0 - opt.extent_quantile);
    g.z_min = h.sorted.front();
    g.z_max = h.sorted.back();
  }

  // No storeys: one implicit storey from the large horizontal planes.
  Reference implicit;
  if (st.empty()) {
    for (size_t k = 0; k < n_planes; ++k)
      if (!inliers[k].empty() &&
          geo[k].orientation == Orientation::horizontal &&
          geo[k].area >= opt.min_large_area_m2 && geo[k].may_face_up() &&
          (!implicit.has_floor() || geo[k].z_med < implicit.floor))
        implicit.floor = geo[k].z_med;
    if (implicit.has_floor())
      for (size_t k = 0; k < n_planes; ++k)
        if (!inliers[k].empty() &&
            geo[k].orientation == Orientation::horizontal &&
            geo[k].area >= opt.min_large_area_m2 && geo[k].may_face_down() &&
            geo[k].z_med - implicit.floor >= opt.min_storey_height_m &&
            (!implicit.has_ceiling() || geo[k].z_med > implicit.ceiling))
          implicit.ceiling = geo[k].z_med;
    result.implicit_floor_z = implicit.floor;
    result.implicit_ceiling_z = implicit.ceiling;
    if (!implicit.has_floor())
      reusex::warn("classify_planes: no storeys and no horizontal plane of "
                   "at least {:.2f} m^2 among {} planes; floor/ceiling rules "
                   "cannot apply",
                   opt.min_large_area_m2, n_planes);
  }

  // Each plane's reference surfaces; split-level floors (floor by the
  // rules, outside the floor band) join their storey's reference so a wall
  // standing on one touches a floor.
  std::vector<Reference> refs(n_planes, implicit);
  std::vector<char> in_storey(n_planes, 1);
  for (size_t k = 0; k < n_planes; ++k) {
    if (st.empty())
      continue;
    if (storey_pos[k] < 0) {
      in_storey[k] = 0;
      continue;
    }
    const auto &s = st[static_cast<size_t>(storey_pos[k])];
    refs[k].floor = s.z_floor;
    refs[k].ceiling = s.z_ceiling;
  }
  std::map<int, std::vector<double>> split_floors; // by storey_pos
  for (size_t k = 0; k < n_planes; ++k)
    if (!inliers[k].empty() && in_storey[k] &&
        geo[k].orientation == Orientation::horizontal &&
        horizontal_rule(geo[k], refs[k], opt) == PlaneRole::floor &&
        !in_floor_band(geo[k], refs[k], opt))
      split_floors[storey_pos[k]].push_back(geo[k].z_med);
  for (size_t k = 0; k < n_planes; ++k) {
    const auto it = split_floors.find(storey_pos[k]);
    if (in_storey[k] && it != split_floors.end())
      refs[k].split_floors = it->second;
  }

  int labelled_planes = 0;
  result.planes.reserve(n_planes);
  result.role_source.reserve(n_planes);
  for (size_t k = 0; k < n_planes; ++k) {
    const auto &g = geo[k];
    core::PlaneClassRecord r;
    r.plane_label = static_cast<int>(k + 1);
    r.orientation = orientation_name(g.orientation);
    r.point_count = static_cast<int>(inliers[k].size());

    if (inliers[k].empty()) {
      r.area = 0.0;
      r.role = std::string(core::to_string(PlaneRole::unknown));
      r.role_source = std::string(to_string(RoleSource::none));
      result.planes.push_back(std::move(r));
      result.role_source.push_back(RoleSource::none);
      continue;
    }
    r.area = g.area;
    r.z_min = g.z_min;
    r.z_max = g.z_max;

    const Reference &ref = refs[k];
    if (storey_pos[k] >= 0)
      r.storey = st[static_cast<size_t>(storey_pos[k])].idx;

    // Majority class over the inliers.
    std::optional<PlaneRole> class_role;
    if (semantic.labels) {
      std::vector<std::pair<uint32_t, int>> votes;
      votes.reserve(inliers[k].size());
      int labelled = 0;
      for (size_t i : inliers[k]) {
        const uint32_t l = semantic.labels->points[i].label;
        if (!core::is_valid_label(l) || core::is_out_of_contract_label(l))
          continue;
        votes.emplace_back(l, 1);
        ++labelled;
      }
      const uint32_t winner = core::majority_label(votes);
      if (winner != core::kUnlabeled) {
        ++labelled_planes;
        const auto won =
            std::count_if(votes.begin(), votes.end(), [winner](const auto &v) {
              return v.first == winner;
            });
        r.semantic_class = core::point_label_to_api(winner);
        r.class_confidence =
            static_cast<double>(won) / static_cast<double>(labelled);
        const double labelled_share = static_cast<double>(labelled) /
                                      static_cast<double>(inliers[k].size());
        // The role is voted by role, not by class (reusex-rqn.4): a richer
        // class list splits one object over near-synonyms ("cabinet" and
        // "shelf" on one cupboard front) that share a role. A labelled point
        // whose class has no name or no role votes for no role.
        if (labelled_share >= opt.min_labelled_share)
          class_role = majority_role(votes, semantic.class_names, role_map,
                                     labelled, opt.min_class_share);
      }
    }

    const PlaneRole rules = rule_role(g, ref, in_storey[k] != 0, opt);
    PlaneRole role = rules;
    RoleSource source = RoleSource::rules;
    if (class_role) {
      if (class_role_fits(*class_role, rules, g, ref, opt)) {
        role = *class_role;
        source = RoleSource::semantic;
        ++result.from_semantic;
      } else {
        source = RoleSource::overruled;
        ++result.overruled;
      }
    }
    r.role = std::string(core::to_string(role));
    r.role_source = std::string(to_string(source));
    result.planes.push_back(std::move(r));
    result.role_source.push_back(source);
  }

  for (const auto &r : result.planes)
    ++result.role_counts[r.role];

  if (n_planes == 0) {
    reusex::warn("classify_planes: no planes to classify");
    return result;
  }
  if (semantic.labels && labelled_planes == 0)
    reusex::warn("classify_planes: a labels cloud was given but none of the "
                 "{} planes has a labelled inlier; roles come from the rules "
                 "only",
                 n_planes);
  const auto count = [&](PlaneRole role) {
    const auto it = result.role_counts.find(std::string(core::to_string(role)));
    return it == result.role_counts.end() ? 0 : it->second;
  };
  if (count(PlaneRole::unknown) == static_cast<int>(n_planes))
    reusex::warn("classify_planes: all {} planes have role unknown", n_planes);
  reusex::info("classify_planes: {} planes: {} wall, {} floor, {} ceiling, "
               "{} slab, {} furniture, {} clutter, {} unknown; {} by class, "
               "{} class roles overruled by geometry",
               n_planes, count(PlaneRole::wall), count(PlaneRole::floor),
               count(PlaneRole::ceiling), count(PlaneRole::slab),
               count(PlaneRole::furniture), count(PlaneRole::clutter),
               count(PlaneRole::unknown), result.from_semantic,
               result.overruled);
  return result;
}

} // namespace reusex::geometry
