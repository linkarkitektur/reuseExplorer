// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/walls.hpp"

#include "reusex/geometry/transform_utils.hpp"
#include "reusex/utils/tolerances.hpp"

#include <fmt/format.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <tuple>

namespace reusex::geometry {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

/// @p a reduced into [0, period).
double wrap(double a, double period) {
  double r = std::fmod(a, period);
  if (r < 0.0)
    r += period;
  if (r >= period) // fmod of a tiny negative can round up to period
    r = 0.0;
  return r;
}

/// Distance between two angles on a circle of @p period [deg].
double circ_dist(double a, double b, double period) {
  const double d = wrap(a - b, period);
  return std::min(d, period - d);
}

struct DirItem {
  double a180 = 0.0; ///< line angle in [0, 180) [deg]
  double w = 0.0;
};

/// Weighted circular mean of k * angle over @p items with @p take(i) true,
/// as an angle of period 360 / k [deg]; NaN when the resultant vanishes.
template <class Pred>
double circular_mean(const std::vector<DirItem> &items, double k, Pred take) {
  double c = 0.0, s = 0.0;
  for (size_t i = 0; i < items.size(); ++i) {
    if (!take(i))
      continue;
    c += items[i].w * std::cos(k * items[i].a180 * kDegToRad);
    s += items[i].w * std::sin(k * items[i].a180 * kDegToRad);
  }
  if (std::hypot(c, s) <= kEpsilonRelative)
    return std::numeric_limits<double>::quiet_NaN();
  return wrap(std::atan2(s, c) * kRadToDeg / k, 360.0 / k);
}

AxisFamily make_family(int index, double angle_deg, bool dominant) {
  AxisFamily f;
  f.index = index;
  f.angle_deg = wrap(angle_deg, 180.0);
  f.d = Eigen::Vector2d(std::cos(f.angle_deg * kDegToRad),
                        std::sin(f.angle_deg * kDegToRad));
  f.dominant = dominant;
  return f;
}

} // namespace

std::vector<AxisFamily>
axis_families(const std::vector<WeightedNormal> &normals,
              const WallOptions &opt) {
  // Line angles in [0, 180), sorted so the result does not depend on the
  // input order (sums run in this order).
  std::vector<DirItem> items;
  items.reserve(normals.size());
  double total = 0.0;
  for (const auto &n : normals) {
    if (!(n.weight > 0.0) || !std::isfinite(n.weight) || !n.n.allFinite() ||
        n.n.norm() <= kEpsilonLengthM)
      continue;
    items.push_back(
        {wrap(std::atan2(n.n.y(), n.n.x()) * kRadToDeg, 180.0), n.weight});
    total += n.weight;
  }
  if (items.empty())
    return {};
  std::sort(items.begin(), items.end(), [](const auto &a, const auto &b) {
    return std::tie(a.a180, a.w) < std::tie(b.a180, b.w);
  });
  const double tol = opt.family_angle_deg;

  // Dominant pair, seeded at the weighted mode of the angles mod 90 (a plain
  // 4-theta mean of everything is pulled off the grid by a heavy oblique
  // wing), then refined as the 4-theta circular mean of its inliers.
  double phi = 0.0, best = -1.0;
  for (const auto &seed : items) {
    const double a = wrap(seed.a180, 90.0);
    double score = 0.0;
    for (const auto &o : items)
      if (circ_dist(wrap(o.a180, 90.0), a, 90.0) <= tol)
        score += o.w;
    if (score > best) { // strict: ties keep the smallest angle (sorted)
      best = score;
      phi = a;
    }
  }
  // The inlier window halves each round down to family_refine_deg: planes a
  // few degrees off (furniture, drifted fragments) must not tilt the grid
  // that every wall is squared to.
  double window = tol;
  for (int it = 0; it < opt.family_refine_iterations; ++it) {
    const double next = circular_mean(items, 4.0, [&](size_t i) {
      return circ_dist(wrap(items[i].a180, 90.0), phi, 90.0) <= window;
    });
    window = std::max(opt.family_refine_deg, 0.5 * window);
    if (std::isnan(next))
      break;
    phi = next;
  }

  std::vector<AxisFamily> families = {make_family(0, phi, true),
                                      make_family(1, phi + 90.0, true)};

  // Extra families: normals off the pair, clustered by angle (mod 180).
  std::vector<size_t> rest;
  for (size_t i = 0; i < items.size(); ++i)
    if (circ_dist(wrap(items[i].a180, 90.0), phi, 90.0) > tol)
      rest.push_back(i);
  if (!rest.empty() && opt.max_families > 0) {
    // Start the circular sweep after the widest gap so no cluster straddles
    // the 0/180 seam.
    size_t start = 0;
    double widest = -1.0;
    for (size_t k = 0; k < rest.size(); ++k) {
      const double prev = items[rest[(k + rest.size() - 1) % rest.size()]].a180;
      const double gap = wrap(items[rest[k]].a180 - prev, 180.0);
      const double g = rest.size() == 1 ? 180.0 : gap;
      if (g > widest) {
        widest = g;
        start = k;
      }
    }
    struct Cluster {
      std::vector<size_t> members;
      double weight = 0.0;
      double angle = 0.0;
    };
    std::vector<Cluster> clusters;
    for (size_t k = 0; k < rest.size(); ++k) {
      const size_t i = rest[(start + k) % rest.size()];
      const size_t prev = rest[(start + k + rest.size() - 1) % rest.size()];
      if (k == 0 || wrap(items[i].a180 - items[prev].a180, 180.0) > tol)
        clusters.emplace_back();
      clusters.back().members.push_back(i);
      clusters.back().weight += items[i].w;
    }
    for (auto &c : clusters) {
      std::vector<bool> in(items.size(), false);
      for (const size_t i : c.members)
        in[i] = true;
      c.angle = circular_mean(items, 2.0, [&](size_t i) { return in[i]; });
      if (std::isnan(c.angle))
        c.angle = items[c.members.front()].a180;
    }
    std::stable_sort(clusters.begin(), clusters.end(),
                     [](const Cluster &a, const Cluster &b) {
                       if (a.weight != b.weight)
                         return a.weight > b.weight;
                       return a.angle < b.angle;
                     });
    for (const auto &c : clusters) {
      if (static_cast<int>(families.size()) - 2 >= opt.max_families)
        break;
      if (c.weight < opt.min_family_share * total)
        break; // sorted by weight: nothing lighter clears the share either
      families.push_back(
          make_family(static_cast<int>(families.size()), c.angle, false));
    }
  }

  // Each normal's mass goes to the family nearest to it.
  for (const auto &item : items) {
    size_t nearest = 0;
    double dmin = std::numeric_limits<double>::infinity();
    for (size_t f = 0; f < families.size(); ++f) {
      const double d = circ_dist(item.a180, families[f].angle_deg, 180.0);
      if (d < dmin) {
        dmin = d;
        nearest = f;
      }
    }
    families[nearest].weight += item.w;
  }
  return families;
}

std::vector<AxisFamily>
axis_families(const std::vector<core::PlaneClassRecord> &planes,
              const CloudN &plane_normals, const core::StoreyRecord &storey,
              const WallOptions &opt) {
  for (const double v : storey.level_transform)
    if (!std::isfinite(v))
      throw std::invalid_argument(fmt::format(
          "axis_families: storey {} has a level_transform that is not finite",
          storey.idx));
  const Eigen::Matrix3d rot = to_affine(storey.level_transform).linear();

  auto collect = [&](bool walls_only) {
    std::vector<WeightedNormal> out;
    for (const auto &p : planes) {
      if (p.storey != storey.idx || p.orientation != "vertical" ||
          p.point_count <= 0)
        continue;
      if (walls_only && p.role != "wall")
        continue;
      if (p.plane_label < 1 ||
          static_cast<size_t>(p.plane_label) > plane_normals.size())
        throw std::invalid_argument(fmt::format(
            "axis_families: plane {} has no normal ('plane_normals' holds {})",
            p.plane_label, plane_normals.size()));
      const auto &pn =
          plane_normals.points[static_cast<size_t>(p.plane_label - 1)];
      const Eigen::Vector3d n =
          rot * Eigen::Vector3d(pn.normal_x, pn.normal_y, pn.normal_z);
      out.push_back(
          {Eigen::Vector2d(n.x(), n.y()), static_cast<double>(p.point_count)});
    }
    return out;
  };
  auto normals = collect(opt.wall_role_only);
  if (normals.empty() && opt.wall_role_only)
    normals = collect(false);
  return axis_families(normals, opt);
}

} // namespace reusex::geometry
