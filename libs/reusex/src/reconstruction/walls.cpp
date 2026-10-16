// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/walls.hpp"

#include "reusex/core/logging.hpp"
#include "reusex/geometry/transform_utils.hpp"
#include "reusex/utils/tolerances.hpp"

#include <fmt/format.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
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

namespace {

/// The along-axis direction of a family: d rotated +90 deg.
Eigen::Vector2d tangent(const Eigen::Vector2d &d) { return {-d.y(), d.x()}; }

double median_of(std::vector<double> v) {
  if (v.empty())
    return std::numeric_limits<double>::quiet_NaN();
  const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
  std::nth_element(v.begin(), mid, v.end());
  if (v.size() % 2 == 1)
    return *mid;
  const double hi = *mid;
  const double lo = *std::max_element(v.begin(), mid);
  return 0.5 * (lo + hi);
}

int cell_index(double v, double cell) {
  return static_cast<int>(std::floor(v / cell));
}

/// A wall or other point in one family's coordinates.
struct EvidenceSample {
  double s = 0.0, z = 0.0, o = 0.0;
  int side = 0; ///< wall points: the side they were seen from; others: 0
};

/// Per family (by position in the families vector), sorted by offset.
struct EvidenceIndex {
  std::vector<std::vector<EvidenceSample>> wall;
  std::vector<std::vector<EvidenceSample>> others;
};

void check_aligned(const WallPoints &wp, const char *who) {
  if (wp.normals.size() != wp.points.size())
    throw std::invalid_argument(fmt::format("{}: {} wall points but {} normals",
                                            who, wp.points.size(),
                                            wp.normals.size()));
  if (!wp.plane.empty() && wp.plane.size() != wp.points.size())
    throw std::invalid_argument(
        fmt::format("{}: {} wall points but {} plane ids", who,
                    wp.points.size(), wp.plane.size()));
}

void sort_by_offset(std::vector<EvidenceSample> &v) {
  std::sort(v.begin(), v.end(), [](const auto &a, const auto &b) {
    return std::tie(a.o, a.s, a.z, a.side) < std::tie(b.o, b.s, b.z, b.side);
  });
}

/// The samples of a sorted evidence vector with offsets in [o0, o1].
std::pair<std::vector<EvidenceSample>::const_iterator,
          std::vector<EvidenceSample>::const_iterator>
offset_range(const std::vector<EvidenceSample> &v, double o0, double o1) {
  const auto lo = std::lower_bound(
      v.begin(), v.end(), o0,
      [](const EvidenceSample &q, double o) { return q.o < o; });
  const auto hi =
      std::upper_bound(lo, v.end(), o1, [](double o, const EvidenceSample &q) {
        return o < q.o;
      });
  return {lo, hi};
}

/// Project the evidence into every family: wall points into the family
/// their plane normal is nearest (squared about their plane's centroid when
/// within square_deg), and, with @p with_others, every other point into
/// every family.
EvidenceIndex index_evidence(const WallPoints &wp,
                             const std::vector<AxisFamily> &families,
                             const WallOptions &opt, bool with_others) {
  check_aligned(wp, "walls");
  EvidenceIndex ev;
  ev.wall.resize(families.size());
  ev.others.resize(families.size());
  if (families.empty())
    return ev;
  const double cos_assign = std::cos(opt.assign_angle_deg * kDegToRad);
  const double cos_square = std::cos(opt.square_deg * kDegToRad);

  auto nearest_family = [&](const Eigen::Vector2d &n) {
    size_t best = 0;
    double best_c = -1.0;
    for (size_t f = 0; f < families.size(); ++f) {
      const double c = std::abs(n.dot(families[f].d));
      if (c > best_c) {
        best_c = c;
        best = f;
      }
    }
    return std::pair{best, best_c};
  };

  // Per plane: centroid and mean unit normal of its points.
  struct PlaneAcc {
    Eigen::Vector2d sum_p = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum_n = Eigen::Vector2d::Zero();
    size_t n = 0;
  };
  std::map<int, PlaneAcc> acc;
  if (!wp.plane.empty())
    for (size_t i = 0; i < wp.points.size(); ++i) {
      const Eigen::Vector2d n = wp.normals[i];
      const double len = n.norm();
      if (wp.plane[i] < 0 || !wp.points[i].allFinite() || !n.allFinite() ||
          len <= kEpsilonLengthM)
        continue;
      auto &a = acc[wp.plane[i]];
      a.sum_p += wp.points[i].head<2>();
      // Orient consistently with the first normal of the plane so opposite
      // signs (should not happen within a plane) do not cancel.
      const Eigen::Vector2d u = n / len;
      a.sum_n += (a.n > 0 && u.dot(a.sum_n) < 0.0) ? -u : u;
      ++a.n;
    }
  struct PlaneFrame {
    size_t family = 0;
    bool assigned = false, squared = false;
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    Eigen::Vector2d n = Eigen::Vector2d::UnitX(); ///< sign of d
  };
  std::map<int, PlaneFrame> frames;
  for (const auto &[id, a] : acc) {
    PlaneFrame pf;
    if (a.sum_n.norm() > kEpsilonLengthM) {
      const Eigen::Vector2d n = a.sum_n.normalized();
      const auto [f, c] = nearest_family(n);
      pf.family = f;
      pf.assigned = c >= cos_assign;
      pf.squared = c >= cos_square;
      pf.centroid = a.sum_p / static_cast<double>(a.n);
      pf.n = n.dot(families[f].d) >= 0.0 ? n : Eigen::Vector2d(-n);
    }
    frames[id] = pf;
  }

  for (size_t i = 0; i < wp.points.size(); ++i) {
    const Eigen::Vector3d &p = wp.points[i];
    Eigen::Vector2d n = wp.normals[i];
    const double len = n.norm();
    if (!p.allFinite() || !n.allFinite() || len <= kEpsilonLengthM)
      continue;
    n /= len;
    const Eigen::Vector2d xy = p.head<2>();
    size_t f = 0;
    double o = 0.0;
    const PlaneFrame *pf = nullptr;
    if (!wp.plane.empty() && wp.plane[i] >= 0)
      pf = &frames.at(wp.plane[i]);
    if (pf) {
      if (!pf->assigned)
        continue;
      f = pf->family;
      o = pf->squared
              ? pf->centroid.dot(families[f].d) + (xy - pf->centroid).dot(pf->n)
              : xy.dot(families[f].d);
    } else {
      const auto [best, c] = nearest_family(n);
      if (c < cos_assign)
        continue;
      f = best;
      o = xy.dot(families[f].d);
    }
    const int side =
        opt.use_normal_sign ? (n.dot(families[f].d) > 0.0 ? 1 : -1) : 0;
    ev.wall[f].push_back({xy.dot(tangent(families[f].d)), p.z(), o, side});
  }
  if (with_others)
    for (size_t f = 0; f < families.size(); ++f) {
      const Eigen::Vector2d d = families[f].d, t = tangent(d);
      auto &v = ev.others[f];
      v.reserve(wp.others.size());
      for (const auto &p : wp.others)
        if (p.allFinite())
          v.push_back({p.head<2>().dot(t), p.z(), p.head<2>().dot(d), 0});
    }
  for (size_t f = 0; f < families.size(); ++f) {
    sort_by_offset(ev.wall[f]);
    sort_by_offset(ev.others[f]);
  }
  return ev;
}

/// An offset histogram anchored at offset 0, smoothed with [1,2,3,2,1]/9.
struct Profile {
  long k0 = 0;           ///< absolute bin index of h[0]
  std::vector<double> h; ///< smoothed counts
  double bin = 0.01;

  [[nodiscard]] double centre(size_t i) const {
    return (static_cast<double>(k0 + static_cast<long>(i)) + 0.5) * bin;
  }
};

Profile smoothed_profile(const std::vector<double> &offsets, double bin) {
  Profile p;
  p.bin = bin;
  if (offsets.empty())
    return p;
  long kmin = std::numeric_limits<long>::max(),
       kmax = std::numeric_limits<long>::min();
  for (const double o : offsets) {
    const long k = static_cast<long>(std::floor(o / bin));
    kmin = std::min(kmin, k);
    kmax = std::max(kmax, k);
  }
  constexpr long kPad = 2;
  p.k0 = kmin - kPad;
  std::vector<double> c(static_cast<size_t>(kmax - kmin + 1 + 2 * kPad), 0.0);
  for (const double o : offsets)
    c[static_cast<size_t>(static_cast<long>(std::floor(o / bin)) - p.k0)] += 1;
  constexpr std::array<double, 5> kKernel = {1, 2, 3, 2, 1};
  p.h.assign(c.size(), 0.0);
  for (size_t i = 0; i < c.size(); ++i)
    for (int j = -2; j <= 2; ++j) {
      const long q = static_cast<long>(i) + j;
      if (q >= 0 && q < static_cast<long>(c.size()))
        p.h[i] +=
            kKernel[static_cast<size_t>(j + 2)] * c[static_cast<size_t>(q)];
    }
  for (double &v : p.h)
    v /= 9.0;
  return p;
}

/// Peaks of @p p (indices into p.h), sorted by position: local maxima of
/// enough height and prominence, thinned by separation (higher first).
std::vector<size_t> profile_peaks(const Profile &p, const WallOptions &opt) {
  const auto &h = p.h;
  std::vector<size_t> cand;
  for (size_t i = 1; i + 1 < h.size(); ++i) {
    if (!(h[i] > h[i - 1] && h[i] >= h[i + 1]) || h[i] < opt.min_peak_points)
      continue;
    double left = h[i], right = h[i];
    for (size_t j = i; j-- > 0;) {
      if (h[j] > h[i])
        break;
      left = std::min(left, h[j]);
    }
    for (size_t j = i + 1; j < h.size(); ++j) {
      if (h[j] > h[i])
        break;
      right = std::min(right, h[j]);
    }
    if (h[i] - std::max(left, right) >= opt.min_peak_prominence_share * h[i])
      cand.push_back(i);
  }
  std::stable_sort(cand.begin(), cand.end(),
                   [&](size_t a, size_t b) { return h[a] > h[b]; });
  std::vector<size_t> kept;
  for (const size_t c : cand) {
    bool near = false;
    for (const size_t k : kept)
      if (std::abs(p.centre(c) - p.centre(k)) <
          opt.min_peak_separation_m - kEpsilonLengthM)
        near = true;
    if (!near)
      kept.push_back(c);
  }
  std::sort(kept.begin(), kept.end());
  return kept;
}

void fill_face_stats(WallFace &f, const WallOptions &opt) {
  std::vector<double> o;
  o.reserve(f.samples.size());
  f.z0 = std::numeric_limits<double>::infinity();
  f.z1 = -std::numeric_limits<double>::infinity();
  f.cells.clear();
  for (const auto &q : f.samples) {
    o.push_back(q.o);
    f.z0 = std::min(f.z0, q.z);
    f.z1 = std::max(f.z1, q.z);
    ++f.cells[{cell_index(q.s, opt.cell_m), cell_index(q.z, opt.cell_m)}];
  }
  f.offset = median_of(std::move(o));
  f.s0 = f.samples.front().s;
  f.s1 = f.samples.back().s;
  f.point_count = static_cast<int>(f.samples.size());
}

/// The faces of one (family, side) profile.
void profile_faces(std::vector<FaceSample> samples, int family, int side,
                   double z_floor, double z_ceiling, const WallOptions &opt,
                   std::vector<WallFace> &out) {
  std::vector<double> offsets;
  offsets.reserve(samples.size());
  for (const auto &q : samples)
    offsets.push_back(q.o);
  const Profile prof = smoothed_profile(offsets, opt.bin_m);
  const auto peaks = profile_peaks(prof, opt);
  if (peaks.empty())
    return;
  std::vector<double> centre(peaks.size()), half(peaks.size());
  for (size_t j = 0; j < peaks.size(); ++j)
    centre[j] = prof.centre(peaks[j]);
  for (size_t j = 0; j < peaks.size(); ++j) {
    half[j] = opt.face_window_m;
    if (j > 0)
      half[j] = std::min(half[j], 0.5 * (centre[j] - centre[j - 1]));
    if (j + 1 < peaks.size())
      half[j] = std::min(half[j], 0.5 * (centre[j + 1] - centre[j]));
  }
  // Members of each peak, in (s, z, o) order so runs and medians do not
  // depend on the input order.
  std::sort(samples.begin(), samples.end(), [](const auto &a, const auto &b) {
    return std::tie(a.s, a.z, a.o) < std::tie(b.s, b.z, b.o);
  });
  std::vector<std::vector<FaceSample>> members(peaks.size());
  for (const auto &q : samples) {
    const auto it = std::lower_bound(centre.begin(), centre.end(), q.o);
    for (const auto j : {it - centre.begin() - 1, it - centre.begin()}) {
      if (j < 0 || j >= static_cast<std::ptrdiff_t>(centre.size()))
        continue;
      const auto u = static_cast<size_t>(j);
      if (std::abs(q.o - centre[u]) <= half[u]) {
        members[u].push_back(q);
        break;
      }
    }
  }
  // Runs along the axis over "solid" columns: a cell_m wide column of the
  // face is evidence when its points cover at least min_column_share of the
  // storey's cell rows, so a long line of furniture ends that happen to
  // share an offset is not a wall. Runs break at stretches without a solid
  // column longer than run_gap_m.
  const int iz_lo = cell_index(z_floor, opt.cell_m);
  const int iz_hi = cell_index(z_ceiling - kEpsilonLengthM, opt.cell_m);
  const double rows = std::max(1, iz_hi - iz_lo + 1);
  for (size_t j = 0; j < peaks.size(); ++j) {
    const auto &m = members[j];
    std::map<int, std::set<int>> col_rows;
    for (const auto &q : m)
      col_rows[cell_index(q.s, opt.cell_m)].insert(cell_index(q.z, opt.cell_m));
    std::vector<int> solid;
    for (const auto &[c, r] : col_rows)
      if (static_cast<double>(r.size()) >= opt.min_column_share * rows)
        solid.push_back(c);
    size_t begin = 0;
    for (size_t k = 1; k <= solid.size(); ++k) {
      if (k < solid.size() &&
          (solid[k] - solid[k - 1] - 1) * opt.cell_m <= opt.run_gap_m)
        continue;
      const int c0 = solid[begin], c1 = solid[k - 1];
      begin = k;
      WallFace f;
      for (const auto &q : m) {
        const int c = cell_index(q.s, opt.cell_m);
        if (c >= c0 && c <= c1)
          f.samples.push_back(q);
      }
      if (f.samples.empty() ||
          f.samples.back().s - f.samples.front().s < opt.min_run_m ||
          static_cast<int>(f.samples.size()) < opt.min_run_points)
        continue;
      f.family = family;
      f.side = side;
      f.peak_height = prof.h[peaks[j]];
      fill_face_stats(f, opt);
      out.push_back(std::move(f));
    }
  }
}

} // namespace

std::vector<WallFace> wall_faces(const WallPoints &wp,
                                 const std::vector<AxisFamily> &families,
                                 const WallOptions &opt) {
  if (families.empty() || wp.points.empty()) {
    check_aligned(wp, "wall_faces");
    return {};
  }
  const EvidenceIndex ev = index_evidence(wp, families, opt, false);

  // One bucket per (family, side); std::map keeps them in key order.
  std::map<std::pair<size_t, int>, std::vector<FaceSample>> buckets;
  for (size_t f = 0; f < families.size(); ++f)
    for (const auto &q : ev.wall[f])
      buckets[{f, q.side}].push_back({q.s, q.z, q.o});

  std::vector<WallFace> faces;
  for (auto &[key, samples] : buckets)
    profile_faces(std::move(samples), families[key.first].index, key.second,
                  wp.z_floor, wp.z_ceiling, opt, faces);
  std::sort(faces.begin(), faces.end(), [](const auto &a, const auto &b) {
    return std::tie(a.family, a.offset, a.s0, a.side) <
           std::tie(b.family, b.offset, b.s0, b.side);
  });
  return faces;
}

namespace {

/// Cell medians of a face's samples with s in [s0, s1]: (z, o) per cell
/// with at least WallOptions::min_cell_points samples, in cell order.
std::vector<Eigen::Vector2d> cell_medians(const WallFace &f, double s0,
                                          double s1, const WallOptions &opt) {
  std::map<FaceCell, std::pair<std::vector<double>, std::vector<double>>> by;
  for (const auto &q : f.samples) {
    if (q.s < s0 || q.s > s1)
      continue;
    auto &c = by[{cell_index(q.s, opt.cell_m), cell_index(q.z, opt.cell_m)}];
    c.first.push_back(q.z);
    c.second.push_back(q.o);
  }
  std::vector<Eigen::Vector2d> out;
  for (auto &[key, zo] : by)
    if (static_cast<int>(zo.first.size()) >= opt.min_cell_points)
      out.emplace_back(median_of(std::move(zo.first)),
                       median_of(std::move(zo.second)));
  return out;
}

struct LineFit {
  double a = 0.0, b = 0.0; ///< o = a + b z
  bool ok = false;
  [[nodiscard]] double at(double z) const { return a + b * z; }
};

/// Ordinary least squares o = a + b z over @p pts (z, o); slope 0 when the
/// heights do not spread.
LineFit ols(const std::vector<Eigen::Vector2d> &pts) {
  LineFit f;
  if (pts.empty())
    return f;
  double mz = 0.0, mo = 0.0;
  for (const auto &p : pts) {
    mz += p.x();
    mo += p.y();
  }
  mz /= static_cast<double>(pts.size());
  mo /= static_cast<double>(pts.size());
  double szz = 0.0, szo = 0.0;
  for (const auto &p : pts) {
    szz += (p.x() - mz) * (p.x() - mz);
    szo += (p.x() - mz) * (p.y() - mo);
  }
  f.b = szz > kEpsilonLengthM ? szo / szz : 0.0;
  f.a = mo - f.b * mz;
  f.ok = true;
  return f;
}

/// Theil-Sen fit of o(z), then the cells within outlier_mad_k robust
/// sigmas of it (the furniture or lining cells a face also caught).
std::vector<Eigen::Vector2d> robust_inliers(std::vector<Eigen::Vector2d> pts,
                                            const WallOptions &opt) {
  if (pts.size() < 3)
    return pts;
  std::vector<double> slopes;
  for (size_t i = 0; i < pts.size(); ++i)
    for (size_t j = i + 1; j < pts.size(); ++j) {
      const double dz = pts[j].x() - pts[i].x();
      if (std::abs(dz) > kEpsilonLengthM)
        slopes.push_back((pts[j].y() - pts[i].y()) / dz);
    }
  const double b = slopes.empty() ? 0.0 : median_of(std::move(slopes));
  std::vector<double> icpt;
  icpt.reserve(pts.size());
  for (const auto &p : pts)
    icpt.push_back(p.y() - b * p.x());
  const double a = median_of(icpt);
  std::vector<double> absres;
  absres.reserve(pts.size());
  for (const auto &p : pts)
    absres.push_back(std::abs(p.y() - a - b * p.x()));
  constexpr double kMadToSigma = 1.4826;
  const double sigma =
      std::max(kMadToSigma * median_of(absres), opt.min_robust_sigma_m);
  std::vector<Eigen::Vector2d> kept;
  for (const auto &p : pts)
    if (std::abs(p.y() - a - b * p.x()) <= opt.outlier_sigma_k * sigma)
      kept.push_back(p);
  return kept;
}

/// Nearest-rank quantile of a sorted vector.
double quantile_sorted(const std::vector<double> &v, double q) {
  if (v.empty())
    return std::numeric_limits<double>::quiet_NaN();
  const auto rank = static_cast<size_t>(
      std::clamp(std::ceil(q * static_cast<double>(v.size())), 1.0,
                 static_cast<double>(v.size())));
  return v[rank - 1];
}

} // namespace

ThicknessFit measure_thickness(const WallFace &lo, const WallFace &hi,
                               double s0, double s1, const WallOptions &opt) {
  ThicknessFit r;
  const auto a = robust_inliers(cell_medians(lo, s0, s1, opt), opt);
  const auto b = robust_inliers(cell_medians(hi, s0, s1, opt), opt);
  r.cells_lo = static_cast<int>(a.size());
  r.cells_hi = static_cast<int>(b.size());
  if (a.size() < 2 || b.size() < 2)
    return r;

  // Mid-height of the heights both faces were seen at (of either, when
  // they do not overlap).
  auto zrange = [](const std::vector<Eigen::Vector2d> &v) {
    double lo_z = std::numeric_limits<double>::infinity(), hi_z = -lo_z;
    for (const auto &p : v) {
      lo_z = std::min(lo_z, p.x());
      hi_z = std::max(hi_z, p.x());
    }
    return std::pair{lo_z, hi_z};
  };
  const auto [alo, ahi] = zrange(a);
  const auto [blo, bhi] = zrange(b);
  const double zlo = std::max(alo, blo), zhi = std::min(ahi, bhi);
  r.z_mid = zlo <= zhi ? 0.5 * (zlo + zhi)
                       : 0.5 * (std::min(alo, blo) + std::max(ahi, bhi));

  const LineFit fa = ols(a), fb = ols(b);
  r.offset_lo = fa.at(r.z_mid);
  r.offset_hi = fb.at(r.z_mid);
  r.thickness = r.offset_hi - r.offset_lo;
  r.lean_deg = std::atan(0.5 * (fa.b + fb.b)) * kRadToDeg;

  // Bootstrap over cells, seeded (STANDARDS §6). Draws use the raw engine
  // output so the sequence is the same on every standard library.
  std::mt19937_64 rng(opt.seed);
  std::vector<double> boot;
  boot.reserve(static_cast<size_t>(std::max(opt.bootstrap_samples, 0)));
  std::vector<Eigen::Vector2d> ra(a.size()), rb(b.size());
  for (int k = 0; k < opt.bootstrap_samples; ++k) {
    for (auto &p : ra)
      p = a[static_cast<size_t>(rng() % a.size())];
    for (auto &p : rb)
      p = b[static_cast<size_t>(rng() % b.size())];
    boot.push_back(ols(rb).at(r.z_mid) - ols(ra).at(r.z_mid));
  }
  std::sort(boot.begin(), boot.end());
  const double tail = 0.5 * (1.0 - opt.ci_level);
  r.ci_lo = boot.empty() ? r.thickness : quantile_sorted(boot, tail);
  r.ci_hi = boot.empty() ? r.thickness : quantile_sorted(boot, 1.0 - tail);
  r.ok = true;
  return r;
}

namespace {

using Interval = std::pair<double, double>;

/// @p free minus [s0, s1].
std::vector<Interval> subtract(const std::vector<Interval> &free, double s0,
                               double s1) {
  std::vector<Interval> out;
  for (const auto &[a, b] : free) {
    if (b <= s0 || a >= s1) {
      out.emplace_back(a, b);
      continue;
    }
    if (a < s0)
      out.emplace_back(a, s0);
    if (b > s1)
      out.emplace_back(s1, b);
  }
  return out;
}

/// The longest piece of the intersection of two interval lists.
std::optional<Interval> longest_common(const std::vector<Interval> &x,
                                       const std::vector<Interval> &y) {
  std::optional<Interval> best;
  for (const auto &[a0, a1] : x)
    for (const auto &[b0, b1] : y) {
      const double lo = std::max(a0, b0), hi = std::min(a1, b1);
      if (hi > lo && (!best || hi - lo > best->second - best->first))
        best = Interval{lo, hi};
    }
  return best;
}

/// Raw point counts per offset bin of every point (wall or not) of family
/// position @p fam within s in [s0, s1] and offsets in [o0, o1].
std::map<long, int> raw_counts(const EvidenceIndex &ev, size_t fam, double s0,
                               double s1, double o0, double o1,
                               const WallOptions &opt) {
  std::map<long, int> c;
  for (const auto *v : {&ev.wall[fam], &ev.others[fam]}) {
    const auto [lo, hi] = offset_range(*v, o0, o1);
    for (auto it = lo; it != hi; ++it)
      if (it->s >= s0 && it->s <= s1)
        ++c[static_cast<long>(std::floor(it->o / opt.bin_m))];
  }
  return c;
}

/// Valley share between two faces: the emptiest bin between them over the
/// weaker face's peak bin, on raw counts of all the storey's points.
double valley_share(const EvidenceIndex &ev, size_t fam, double olo, double ohi,
                    double s0, double s1, const WallOptions &opt) {
  const double bin = opt.bin_m;
  const auto c = raw_counts(ev, fam, s0, s1, olo - 2 * bin, ohi + 2 * bin, opt);
  auto count = [&](long k) {
    const auto it = c.find(k);
    return it == c.end() ? 0 : it->second;
  };
  auto peak = [&](double o) {
    const long k = static_cast<long>(std::floor(o / bin));
    return std::max({count(k - 1), count(k), count(k + 1)});
  };
  const double weaker = std::min(peak(olo), peak(ohi));
  if (weaker <= 0)
    return 1.0;
  const double margin = std::min(opt.face_window_m, 0.25 * (ohi - olo));
  int valley = std::numeric_limits<int>::max();
  for (long k = static_cast<long>(std::floor((olo + margin) / bin));
       k <= static_cast<long>(std::floor((ohi - margin) / bin)); ++k) {
    const double centre = (static_cast<double>(k) + 0.5) * bin;
    if (centre <= olo + margin || centre >= ohi - margin)
      continue;
    valley = std::min(valley, count(k));
  }
  if (valley == std::numeric_limits<int>::max())
    return 0.0; // no bin between: nothing contradicts a void
  return static_cast<double>(valley) / weaker;
}

/// Cells (s, z) seen on both faces over those seen on either, over [s0, s1].
double co_visibility(const WallFace &lo, const WallFace &hi, double s0,
                     double s1, const WallOptions &opt) {
  auto in_range = [&](const FaceCell &c) {
    const double centre = (static_cast<double>(c.first) + 0.5) * opt.cell_m;
    return centre >= s0 && centre <= s1;
  };
  int both = 0, either = 0;
  for (const auto &[c, n] : lo.cells) {
    if (!in_range(c))
      continue;
    ++either;
    if (hi.cells.count(c))
      ++both;
  }
  for (const auto &[c, n] : hi.cells)
    if (in_range(c) && !lo.cells.count(c))
      ++either;
  return either > 0 ? static_cast<double>(both) / either : 0.0;
}

} // namespace

FacePairing pair_faces(const std::vector<WallFace> &faces, const WallPoints &wp,
                       const std::vector<AxisFamily> &families,
                       const WallOptions &opt) {
  FacePairing out;
  if (faces.empty())
    return out;
  const EvidenceIndex ev = index_evidence(wp, families, opt, true);
  std::map<int, size_t> pos; // AxisFamily::index -> position
  for (size_t f = 0; f < families.size(); ++f)
    pos[families[f].index] = f;

  struct Candidate {
    size_t lo, hi;
    double overlap, gap;
  };
  std::vector<Candidate> cand;
  for (size_t i = 0; i < faces.size(); ++i)
    for (size_t j = 0; j < faces.size(); ++j) {
      const auto &a = faces[i], &b = faces[j];
      if (i == j || a.family != b.family || !(a.offset < b.offset) ||
          !pos.count(a.family))
        continue;
      // Both faces' outward normals point away from the wall: the lower
      // face is seen from -d, the upper from +d. Faces the other way round
      // look at each other: a room -- unless they are too close for a
      // sensor to have seen into the gap between them, which only drift
      // between the two rooms' scans explains ("crossed").
      const double gap = b.offset - a.offset;
      const bool crossed = a.side == 1 && b.side == -1;
      if (crossed) {
        if (gap > opt.max_crossed_gap_m)
          continue;
      } else {
        if ((a.side != 0 && a.side != -1) || (b.side != 0 && b.side != 1))
          continue;
        if (gap < opt.min_gap_m || gap > opt.max_gap_m)
          continue;
      }
      const double overlap = std::min(a.s1, b.s1) - std::max(a.s0, b.s0);
      if (overlap < opt.min_shared_m)
        continue;
      cand.push_back({i, j, overlap, gap});
    }
  std::stable_sort(cand.begin(), cand.end(),
                   [](const Candidate &x, const Candidate &y) {
                     if (x.overlap != y.overlap)
                       return x.overlap > y.overlap;
                     if (x.gap != y.gap)
                       return x.gap < y.gap;
                     return std::tie(x.lo, x.hi) < std::tie(y.lo, y.hi);
                   });

  std::vector<std::vector<Interval>> free(faces.size());
  for (size_t i = 0; i < faces.size(); ++i)
    free[i] = {{faces[i].s0, faces[i].s1}};
  for (const auto &c : cand) {
    const auto piece = longest_common(free[c.lo], free[c.hi]);
    if (!piece || piece->second - piece->first < opt.min_shared_m)
      continue;
    const auto &a = faces[c.lo], &b = faces[c.hi];
    FacePair p;
    p.face_lo = static_cast<int>(c.lo);
    p.face_hi = static_cast<int>(c.hi);
    p.s0 = piece->first;
    p.s1 = piece->second;
    p.gap = c.gap;
    p.crossed = a.side == 1 && b.side == -1;
    p.valley_share = p.crossed ? 0.0
                               : valley_share(ev, pos.at(a.family), a.offset,
                                              b.offset, p.s0, p.s1, opt);
    p.co_visibility = co_visibility(a, b, p.s0, p.s1, opt);
    if (p.valley_share > opt.max_valley_share ||
        p.co_visibility < opt.min_co_visibility) {
      ++out.rejected;
      continue;
    }
    free[c.lo] = subtract(free[c.lo], p.s0, p.s1);
    free[c.hi] = subtract(free[c.hi], p.s0, p.s1);
    out.pairs.push_back(p);
  }

  // What no pair took: one-sided stretches, trimmed to their points.
  for (size_t i = 0; i < faces.size(); ++i)
    for (const auto &[s0, s1] : free[i]) {
      double lo = std::numeric_limits<double>::infinity(), hi = -lo;
      int n = 0;
      for (const auto &q : faces[i].samples)
        if (q.s >= s0 && q.s <= s1) {
          lo = std::min(lo, q.s);
          hi = std::max(hi, q.s);
          ++n;
        }
      if (n >= opt.min_run_points && hi - lo >= opt.min_run_m)
        out.singles.push_back({static_cast<int>(i), lo, hi});
    }
  return out;
}

namespace {

/// Non-wall points per m^2 of the strip [o0, o1] x [s0, s1], below
/// WallOptions::external_max_z_m above the floor.
double strip_density(const EvidenceIndex &ev, size_t fam, double o0, double o1,
                     double s0, double s1, const WallPoints &wp,
                     const WallOptions &opt) {
  const double area = (o1 - o0) * (s1 - s0);
  if (!(area > 0.0))
    return 0.0;
  const double zlo = wp.z_floor - opt.external_strip_near_m;
  const double zhi = wp.z_floor + opt.external_max_z_m;
  int n = 0;
  const auto [lo, hi] = offset_range(ev.others[fam], o0, o1);
  for (auto it = lo; it != hi; ++it)
    if (it->s >= s0 && it->s <= s1 && it->z >= zlo && it->z <= zhi)
      ++n;
  return n / area;
}

int samples_in(const WallFace &f, double s0, double s1) {
  return static_cast<int>(std::count_if(
      f.samples.begin(), f.samples.end(),
      [&](const FaceSample &q) { return q.s >= s0 && q.s <= s1; }));
}

} // namespace

std::vector<WallSegment> measure_walls(const std::vector<WallFace> &faces,
                                       const FacePairing &pairing,
                                       const WallPoints &wp,
                                       const std::vector<AxisFamily> &families,
                                       const WallOptions &opt) {
  std::vector<WallSegment> out;
  if (faces.empty())
    return out;
  const EvidenceIndex ev = index_evidence(wp, families, opt, true);
  const double near = opt.external_strip_near_m, far = opt.external_strip_far_m;
  auto external = [&](size_t fam, double olo, double ohi, double s0,
                      double s1) {
    const double below =
        strip_density(ev, fam, olo - far, olo - near, s0, s1, wp, opt);
    const double above =
        strip_density(ev, fam, ohi + near, ohi + far, s0, s1, wp, opt);
    return std::min(below, above) < opt.min_room_density;
  };
  auto family_pos = [&](int index) -> size_t {
    for (size_t f = 0; f < families.size(); ++f)
      if (families[f].index == index)
        return f;
    throw std::invalid_argument(
        fmt::format("measure_walls: a face names family {}, which is not "
                    "among the {} families given",
                    index, families.size()));
  };

  for (const auto &p : pairing.pairs) {
    const auto &lo = faces[static_cast<size_t>(p.face_lo)];
    const auto &hi = faces[static_cast<size_t>(p.face_hi)];
    const size_t fam = family_pos(lo.family);
    WallSegment w;
    w.family = lo.family;
    w.d = families[fam].d;
    w.s0 = p.s0;
    w.s1 = p.s1;
    w.face_lo_index = p.face_lo;
    w.face_hi_index = p.face_hi;
    w.points_lo = samples_in(lo, p.s0, p.s1);
    w.points_hi = samples_in(hi, p.s0, p.s1);
    w.co_visibility = p.co_visibility;
    if (p.crossed) {
      // The two rooms' scans disagree about where the wall is: its centre
      // is between them, its thickness unknown.
      const auto fit = measure_thickness(lo, hi, p.s0, p.s1, opt);
      w.axis_offset = fit.ok ? 0.5 * (fit.offset_lo + fit.offset_hi)
                             : 0.5 * (lo.offset + hi.offset);
      w.assumed = true;
      w.thickness = opt.default_thickness_m;
      w.is_external = external(fam, w.axis_offset - 0.5 * w.thickness,
                               w.axis_offset + 0.5 * w.thickness, w.s0, w.s1);
      out.push_back(w);
      continue;
    }
    const auto fit = measure_thickness(lo, hi, p.s0, p.s1, opt);
    if (fit.ok && fit.thickness > 0.0) {
      w.face_lo = fit.offset_lo;
      w.face_hi = fit.offset_hi;
      w.thickness = fit.thickness;
      w.thickness_ci_lo = fit.ci_lo;
      w.thickness_ci_hi = fit.ci_hi;
      w.lean_deg = fit.lean_deg;
    } else { // too few cells for a fit: the profile peaks, no CI
      w.face_lo = lo.offset;
      w.face_hi = hi.offset;
      w.thickness = hi.offset - lo.offset;
    }
    w.axis_offset = 0.5 * (w.face_lo + w.face_hi);
    w.is_external = external(fam, w.face_lo, w.face_hi, w.s0, w.s1);
    out.push_back(w);
  }

  const double t = opt.default_thickness_m;
  for (const auto &sp : pairing.singles) {
    const auto &f = faces[static_cast<size_t>(sp.face)];
    const size_t fam = family_pos(f.family);
    WallSegment w;
    w.family = f.family;
    w.d = families[fam].d;
    w.s0 = sp.s0;
    w.s1 = sp.s1;
    w.assumed = true;
    w.thickness = t;

    const auto cells = robust_inliers(cell_medians(f, sp.s0, sp.s1, opt), opt);
    const LineFit fit = ols(cells);
    const double zm = 0.5 * (f.z0 + f.z1);
    const double o = fit.ok ? fit.at(zm) : f.offset;
    if (fit.ok)
      w.lean_deg = std::atan(fit.b) * kRadToDeg;

    // Which side the room is on: the face's normal, else the side with
    // more floor-level evidence in front of it.
    int side = f.side;
    if (side == 0)
      side = strip_density(ev, fam, o + near, o + far, sp.s0, sp.s1, wp, opt) >=
                     strip_density(ev, fam, o - far, o - near, sp.s0, sp.s1, wp,
                                   opt)
                 ? 1
                 : -1;
    // The axis is the seen face (reusex-v8p): the thickness is unknown, and
    // the reference plans draw a one-sided wall on its seen face too. The
    // assumed body (default_thickness_m) lies behind it, away from the side
    // it was seen from; body() and the external test use that.
    w.axis_offset = o;
    if (side > 0) { // seen from +d: the wall's body lies below the face
      w.face_hi = o;
      w.points_hi = samples_in(f, sp.s0, sp.s1);
      w.face_hi_index = sp.face;
      w.is_external = external(fam, o - t, o, w.s0, w.s1);
    } else {
      w.face_lo = o;
      w.points_lo = samples_in(f, sp.s0, sp.s1);
      w.face_lo_index = sp.face;
      w.is_external = external(fam, o, o + t, w.s0, w.s1);
    }
    out.push_back(w);
  }
  std::stable_sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
    return std::tie(a.family, a.axis_offset, a.s0) <
           std::tie(b.family, b.axis_offset, b.s0);
  });
  return out;
}

namespace {

/// The face offsets a wall occupies: [lo, hi]. A one-sided wall's unseen
/// face sits its (default) thickness behind the seen one; a wall with no
/// face (a crossed pair) is its axis +- half its thickness.
std::pair<double, double> body(const WallSegment &w) {
  const bool lo_seen = std::isfinite(w.face_lo);
  const bool hi_seen = std::isfinite(w.face_hi);
  if (lo_seen && hi_seen)
    return {w.face_lo, w.face_hi};
  if (hi_seen)
    return {w.face_hi - w.thickness, w.face_hi};
  if (lo_seen)
    return {w.face_lo, w.face_lo + w.thickness};
  return {w.axis_offset - 0.5 * w.thickness, w.axis_offset + 0.5 * w.thickness};
}

/// The middle of a wall's body: its centre line, whatever its axis is.
double body_centre(const WallSegment &w) {
  const auto [lo, hi] = body(w);
  return 0.5 * (lo + hi);
}

/// The axis from the faces: the middle of two, the one seen face of a
/// one-sided wall (reusex-v8p); unchanged with none.
void set_axis_from_faces(WallSegment &w) {
  if (std::isfinite(w.face_lo) && std::isfinite(w.face_hi))
    w.axis_offset = 0.5 * (w.face_lo + w.face_hi);
  else if (std::isfinite(w.face_hi))
    w.axis_offset = w.face_hi;
  else if (std::isfinite(w.face_lo))
    w.axis_offset = w.face_lo;
}

/// Median offset of the family's wall evidence seen from @p side within
/// [o0, o1] and s in [s0, s1]; NaN when there is none.
double median_offset(const EvidenceIndex &ev, size_t fam, int side, double o0,
                     double o1, double s0, double s1) {
  std::vector<double> o;
  const auto [lo, hi] = offset_range(ev.wall[fam], o0, o1);
  for (auto it = lo; it != hi; ++it)
    if ((it->side == side || it->side == 0) && it->s >= s0 && it->s <= s1)
      o.push_back(it->o);
  return median_of(std::move(o));
}

/// Snap a wall's seen faces to the median of the raw wall points near them:
/// a wide pass, then a narrow one around the result.
void snap(WallSegment &w, const EvidenceIndex &ev, size_t fam,
          const WallOptions &opt) {
  // A face never snaps past the middle of the body (a one-sided wall's
  // axis is its face, so the axis cannot be the bound).
  const double axis = body_centre(w);
  for (const double r : {opt.snap_wide_m, opt.snap_narrow_m}) {
    if (std::isfinite(w.face_lo)) {
      const double m = median_offset(ev, fam, -1, w.face_lo - r,
                                     std::min(w.face_lo + r, axis), w.s0, w.s1);
      if (std::isfinite(m))
        w.face_lo = m;
    }
    if (std::isfinite(w.face_hi)) {
      const double m = median_offset(ev, fam, 1, std::max(w.face_hi - r, axis),
                                     w.face_hi + r, w.s0, w.s1);
      if (std::isfinite(m))
        w.face_hi = m;
    }
  }
  if (std::isfinite(w.face_lo) && std::isfinite(w.face_hi) && !w.assumed) {
    // Keep the measured thickness; move the axis with the faces.
    w.axis_offset = 0.5 * (w.face_lo + w.face_hi);
  } else {
    set_axis_from_faces(w);
  }
}

/// Are two walls of one family on the same line? Compare a face both saw,
/// else the middles of their bodies (one wall seen from opposite sides in
/// two stretches).
bool collinear(const WallSegment &a, const WallSegment &b, double tol) {
  if (std::isfinite(a.face_lo) && std::isfinite(b.face_lo))
    return std::abs(a.face_lo - b.face_lo) <= tol;
  if (std::isfinite(a.face_hi) && std::isfinite(b.face_hi))
    return std::abs(a.face_hi - b.face_hi) <= tol;
  return std::abs(body_centre(a) - body_centre(b)) <= tol;
}

/// The one face a one-sided wall was seen on: +1 its face_hi, -1 its
/// face_lo, 0 none or both.
int seen_side(const WallSegment &w) {
  const bool lo = std::isfinite(w.face_lo), hi = std::isfinite(w.face_hi);
  return lo == hi ? 0 : (hi ? 1 : -1);
}

/// Two one-sided walls seen from the same side, one behind the other within
/// a wall's thickness: the farther face is a recess (a window niche, a
/// reveal) of the nearer wall -- the room cannot see a separate wall
/// standing in another wall's body. Only short pieces: a long step in a
/// facade stays two walls.
bool is_recess(const WallSegment &a, const WallSegment &b,
               const WallOptions &opt) {
  const int side = seen_side(a);
  if (!a.assumed || !b.assumed || side == 0 || side != seen_side(b))
    return false;
  const double fa = side > 0 ? a.face_hi : a.face_lo;
  const double fb = side > 0 ? b.face_hi : b.face_lo;
  if (std::abs(fa - fb) > opt.max_gap_m)
    return false;
  return std::min(a.s1 - a.s0, b.s1 - b.s0) <= opt.recess_max_length_m;
}

/// Merge the recess pair @p b into @p a: the face with more points is the
/// wall's face, the extent is the union.
void merge_recess(WallSegment &a, const WallSegment &b) {
  const int pa = a.points_lo + a.points_hi, pb = b.points_lo + b.points_hi;
  WallSegment out = pa >= pb ? a : b;
  out.s0 = std::min(a.s0, b.s0);
  out.s1 = std::max(a.s1, b.s1);
  out.points_lo = a.points_lo + b.points_lo;
  out.points_hi = a.points_hi + b.points_hi;
  a = out;
}

/// Are @p a and @p b one wall seen twice (reusex-1wp)? Parallel and
/// overlapping along the axis by duplicate_min_overlap of the shorter (or
/// by duplicate_min_overlap_m), and
///  - both one-sided, seen from the same side, faces within
///    duplicate_max_offset_m: the nearer face would hide the farther one
///    over the shared stretch, so drift put one face in two places;
///  - both measured with overlapping bodies (a crossed pair's body is a
///    guess, so it never decides);
///  - one of each, the one-sided face within bridge_offset_m of the
///    two-sided wall's face on its side (the same face, seen twice).
/// A one-sided face near but not on a two-sided wall stays apart: the
/// two-sided wall's pairing may be wrong, and merging would move a
/// well-placed face onto it. So do two two-sided walls with a gap between
/// their bodies (a duct) and faces looking at each other (a corridor).
bool is_duplicate(const WallSegment &a, const WallSegment &b,
                  const WallOptions &opt) {
  if (a.family != b.family)
    return false;
  const double shorter = std::min(a.s1 - a.s0, b.s1 - b.s0);
  const double overlap = std::min(a.s1, b.s1) - std::max(a.s0, b.s0);
  if (!(shorter > 0.0) || !(overlap > 0.0) ||
      (overlap < opt.duplicate_min_overlap * shorter &&
       overlap < opt.duplicate_min_overlap_m))
    return false;
  const int sa = seen_side(a), sb = seen_side(b);
  auto face = [](const WallSegment &w, int side) {
    return side > 0 ? w.face_hi : w.face_lo;
  };
  if (sa != 0 && sb != 0)
    return sa == sb &&
           std::abs(face(a, sa) - face(b, sb)) <= opt.duplicate_max_offset_m;
  if (sa == 0 && sb == 0) {
    if (a.assumed || b.assumed)
      return false;
    const auto [alo, ahi] = body(a);
    const auto [blo, bhi] = body(b);
    return std::min(ahi, bhi) > std::max(alo, blo);
  }
  const WallSegment &one = sa != 0 ? a : b, &two = sa != 0 ? b : a;
  const int side = sa != 0 ? sa : sb;
  const double f = face(one, side);
  const auto [lo, hi] = body(two);
  return std::abs(f - (side > 0 ? hi : lo)) <= opt.bridge_offset_m;
}

/// Does @p a dominate its duplicate @p b: measured over assumed, then more
/// points, then longer?
bool dominates(const WallSegment &a, const WallSegment &b) {
  auto rank = [](const WallSegment &w) {
    return std::make_tuple(!w.assumed, w.points_lo + w.points_hi, w.s1 - w.s0);
  };
  return rank(a) >= rank(b);
}

/// Merge duplicate @p minor into @p dom (reusex-1wp): the dominant wall
/// keeps its faces over the union of the two extents -- a straight wall
/// scanned twice with drift, not two walls -- and takes the minor's points
/// when both were seen from the same side(s).
void merge_duplicate(WallSegment &dom, const WallSegment &minor) {
  dom.s0 = std::min(dom.s0, minor.s0);
  dom.s1 = std::max(dom.s1, minor.s1);
  if (std::isfinite(dom.face_lo) == std::isfinite(minor.face_lo) &&
      std::isfinite(dom.face_hi) == std::isfinite(minor.face_hi)) {
    dom.points_lo += minor.points_lo;
    dom.points_hi += minor.points_hi;
  }
}

/// Is the gap between collinear walls @p a and @p b an opening (reusex-3e9)?
/// The face line they share (the face both saw, else the middle of their
/// bodies) is seen over the gap above lintel_min_z_m in lintel_min_share
/// of its columns -- a lintel over a door -- or the capture path crosses
/// that line in the gap.
bool gap_is_opening(const WallSegment &a, const WallSegment &b,
                    const EvidenceIndex &ev, size_t fam, const WallPoints &wp,
                    const WallOptions &opt) {
  const double g0 = std::min(a.s1, b.s1), g1 = std::max(a.s0, b.s0);
  if (!(g1 > g0))
    return false;
  double f = 0.5 * (body_centre(a) + body_centre(b));
  int side = 0;
  if (std::isfinite(a.face_hi) && std::isfinite(b.face_hi)) {
    f = 0.5 * (a.face_hi + b.face_hi);
    side = 1;
  } else if (std::isfinite(a.face_lo) && std::isfinite(b.face_lo)) {
    f = 0.5 * (a.face_lo + b.face_lo);
    side = -1;
  }
  // Lintel: wall evidence on the face line over the gap, above the door.
  const int c0 = cell_index(g0, opt.cell_m), c1 = cell_index(g1, opt.cell_m);
  std::set<int> cols;
  const double band = opt.snap_wide_m;
  const auto [lo, hi] = offset_range(ev.wall[fam], f - band, f + band);
  for (auto q = lo; q != hi; ++q)
    if ((side == 0 || q->side == 0 || q->side == side) && q->s > g0 &&
        q->s < g1 && q->z >= wp.z_floor + opt.lintel_min_z_m &&
        q->z <= wp.z_ceiling)
      cols.insert(cell_index(q->s, opt.cell_m));
  const double ncols = std::max(1, c1 - c0 + 1);
  if (static_cast<double>(cols.size()) >= opt.lintel_min_share * ncols)
    return true;
  // The capture path walking through the gap.
  const Eigen::Vector2d t = tangent(a.d);
  for (size_t k = 0; k + 1 < wp.trajectory.size(); ++k) {
    const Eigen::Vector3d &p = wp.trajectory[k], &q = wp.trajectory[k + 1];
    if (!p.allFinite() || !q.allFinite())
      continue;
    if ((q.head<2>() - p.head<2>()).norm() > opt.crossing_max_step_m)
      continue;
    const double op = p.head<2>().dot(a.d) - f, oq = q.head<2>().dot(a.d) - f;
    if (op * oq > 0.0 || op == oq)
      continue;
    const double u = op / (op - oq);
    const double sc = (1.0 - u) * p.head<2>().dot(t) + u * q.head<2>().dot(t);
    const double zc = (1.0 - u) * p.z() + u * q.z();
    if (sc >= g0 && sc <= g1 && zc >= wp.z_floor && zc <= wp.z_ceiling + 0.5)
      return true;
  }
  return false;
}

/// Merge @p b into @p a (collinear, close along the axis).
void merge_into(WallSegment &a, const WallSegment &b) {
  const double la = a.s1 - a.s0, lb = b.s1 - b.s0;
  const double wa = la / std::max(la + lb, kEpsilonLengthM), wb = 1.0 - wa;
  auto blend = [&](double x, double y) {
    if (std::isfinite(x) && std::isfinite(y))
      return wa * x + wb * y;
    return std::isfinite(x) ? x : y;
  };
  if (a.assumed != b.assumed) {
    // A measured wall decides the thickness of the whole.
    const WallSegment &m = a.assumed ? b : a;
    WallSegment out = m;
    out.s0 = std::min(a.s0, b.s0);
    out.s1 = std::max(a.s1, b.s1);
    out.points_lo = a.points_lo + b.points_lo;
    out.points_hi = a.points_hi + b.points_hi;
    out.is_external = (la >= lb ? a : b).is_external;
    a = out;
    return;
  }
  a.face_lo = blend(a.face_lo, b.face_lo);
  a.face_hi = blend(a.face_hi, b.face_hi);
  if (!a.assumed) {
    a.thickness = wa * a.thickness + wb * b.thickness;
    a.thickness_ci_lo = wa * a.thickness_ci_lo + wb * b.thickness_ci_lo;
    a.thickness_ci_hi = wa * a.thickness_ci_hi + wb * b.thickness_ci_hi;
    a.co_visibility = wa * a.co_visibility + wb * b.co_visibility;
  }
  a.lean_deg = wa * a.lean_deg + wb * b.lean_deg;
  a.is_external = (la >= lb ? a : b).is_external;
  a.s0 = std::min(a.s0, b.s0);
  a.s1 = std::max(a.s1, b.s1);
  a.points_lo += b.points_lo;
  a.points_hi += b.points_hi;
  if (std::isfinite(a.face_lo) || std::isfinite(a.face_hi))
    set_axis_from_faces(a);
  else // crossed pairs: no face, the axis is all there is
    a.axis_offset = wa * a.axis_offset + wb * b.axis_offset;
}

} // namespace

WallGraph join_walls(std::vector<WallSegment> walls,
                     const std::vector<AxisFamily> &families,
                     const WallPoints &wp, const WallOptions &opt) {
  WallGraph g;
  if (walls.empty())
    return g;
  const EvidenceIndex ev = index_evidence(wp, families, opt, false);
  auto family_pos = [&](int index) -> size_t {
    for (size_t f = 0; f < families.size(); ++f)
      if (families[f].index == index)
        return f;
    throw std::invalid_argument(fmt::format(
        "join_walls: a wall names family {}, which is not among the {} "
        "families given",
        index, families.size()));
  };

  // 1. Squaring happened on the evidence (wall_faces turns every wall plane
  //    within square_deg onto its family axis), so each wall is already
  //    parallel to its family. 2. Snap the seen faces to the raw points.
  for (auto &w : walls)
    snap(w, ev, family_pos(w.family), opt);

  // 3. Bridge collinear gaps: merge, or join end to end when the measured
  //    thicknesses disagree; and merge duplicates -- one wall seen twice
  //    (drift, a split face) -- into their dominant copy (reusex-1wp). Both
  //    in one loop: a facade broken by windows only overlaps its drifted
  //    copy once its pieces are bridged, so duplicates are merged only when
  //    nothing is left to bridge (and bridging resumes after). Bounded: every
  //    merge removes a wall.
  auto merge_duplicates = [&] {
    for (size_t i = 0; i < walls.size(); ++i)
      for (size_t j = i + 1; j < walls.size(); ++j)
        if (is_duplicate(walls[i], walls[j], opt)) {
          const bool keep_i = dominates(walls[i], walls[j]);
          const size_t d = keep_i ? i : j, m = keep_i ? j : i;
          merge_duplicate(walls[d], walls[m]);
          walls.erase(walls.begin() + static_cast<std::ptrdiff_t>(m));
          return true;
        }
    return false;
  };
  for (bool merged = true; merged;) {
    merged = false;
    std::stable_sort(walls.begin(), walls.end(),
                     [](const auto &x, const auto &y) {
                       return std::tie(x.family, x.s0, x.axis_offset) <
                              std::tie(y.family, y.s0, y.axis_offset);
                     });
    for (size_t i = 0; i < walls.size() && !merged; ++i)
      for (size_t j = i + 1; j < walls.size() && !merged; ++j) {
        auto &a = walls[i];
        const auto &b = walls[j];
        if (a.family != b.family)
          continue;
        const double gap = std::max(a.s0, b.s0) - std::min(a.s1, b.s1);
        if (gap > opt.bridge_gap_m) {
          // A wider gap only over an opening, between collinear walls whose
          // thicknesses agree (reusex-3e9).
          if (gap > opt.door_bridge_gap_m ||
              !collinear(a, b, opt.bridge_offset_m) ||
              (!a.assumed && !b.assumed &&
               std::abs(a.thickness - b.thickness) > opt.bridge_thickness_m) ||
              !gap_is_opening(a, b, ev, family_pos(a.family), wp, opt))
            continue;
          merge_into(a, b);
          walls.erase(walls.begin() + static_cast<std::ptrdiff_t>(j));
          merged = true;
          ++g.door_bridges;
          continue;
        }
        if (is_recess(a, b, opt)) {
          merge_recess(a, b);
          walls.erase(walls.begin() + static_cast<std::ptrdiff_t>(j));
          merged = true;
          continue;
        }
        if (!collinear(a, b, opt.bridge_offset_m))
          continue;
        if (!a.assumed && !b.assumed &&
            std::abs(a.thickness - b.thickness) > opt.bridge_thickness_m)
          continue; // joined end to end below
        merge_into(a, b);
        walls.erase(walls.begin() + static_cast<std::ptrdiff_t>(j));
        merged = true;
      }
    if (!merged)
      merged = merge_duplicates();
  }
  // Collinear walls of different measured thickness meet in the gap middle.
  std::vector<std::array<bool, 2>> end_used(walls.size(), {false, false});
  for (size_t i = 0; i < walls.size(); ++i)
    for (size_t j = 0; j < walls.size(); ++j) {
      auto &a = walls[i];
      auto &b = walls[j];
      if (i == j || a.family != b.family || end_used[i][1] || end_used[j][0] ||
          !collinear(a, b, opt.bridge_offset_m))
        continue;
      const double gap = b.s0 - a.s1;
      if (gap < -kToleranceCoplanarM || gap > opt.bridge_gap_m)
        continue;
      const double mid = 0.5 * (a.s1 + b.s0);
      a.s1 = mid;
      b.s0 = mid;
      end_used[i][1] = end_used[j][0] = true;
      g.joins.push_back({static_cast<int>(i), static_cast<int>(j),
                         WallJoinKind::collinear, WallEnd::end,
                         WallEnd::start});
    }

  // 4. Corners and tees between non-parallel walls, nearest first.
  struct Meet {
    size_t a, b;
    double sa, sb;    ///< the crossing, along each wall
    int end_a, end_b; ///< nearer end of each (0 start, 1 end)
    double da, db;    ///< distances of those ends to the crossing
  };
  std::vector<Meet> meets;
  for (size_t i = 0; i < walls.size(); ++i)
    for (size_t j = i + 1; j < walls.size(); ++j) {
      const auto &a = walls[i], &b = walls[j];
      if (a.family == b.family)
        continue;
      Eigen::Matrix2d m;
      m.row(0) = a.d.transpose();
      m.row(1) = b.d.transpose();
      if (std::abs(m.determinant()) <= kEpsilonRelative)
        continue;
      const Eigen::Vector2d x =
          m.inverse() * Eigen::Vector2d(a.axis_offset, b.axis_offset);
      Meet mt{i, j, x.dot(tangent(a.d)), x.dot(tangent(b.d)), 0, 0, 0, 0};
      mt.end_a = std::abs(mt.sa - a.s0) <= std::abs(mt.sa - a.s1) ? 0 : 1;
      mt.end_b = std::abs(mt.sb - b.s0) <= std::abs(mt.sb - b.s1) ? 0 : 1;
      mt.da = std::abs(mt.sa - (mt.end_a == 0 ? a.s0 : a.s1));
      mt.db = std::abs(mt.sb - (mt.end_b == 0 ? b.s0 : b.s1));
      if (std::min(mt.da, mt.db) <= opt.tee_m)
        meets.push_back(mt);
    }
  std::stable_sort(
      meets.begin(), meets.end(), [](const Meet &x, const Meet &y) {
        const double kx = std::min(x.da, x.db) + std::max(x.da, x.db);
        const double ky = std::min(y.da, y.db) + std::max(y.da, y.db);
        if (kx != ky)
          return kx < ky;
        return std::tie(x.a, x.b) < std::tie(y.a, y.b);
      });
  auto move_end = [&](size_t w, int end, double s) {
    auto &x = walls[w];
    const double s0 = end == 0 ? s : x.s0, s1 = end == 1 ? s : x.s1;
    if (s1 - s0 < opt.min_run_m)
      return false;
    x.s0 = s0;
    x.s1 = s1;
    end_used[w][static_cast<size_t>(end)] = true;
    return true;
  };
  auto on_wall = [&](size_t w, double s) {
    return s >= walls[w].s0 - opt.tee_on_wall_m &&
           s <= walls[w].s1 + opt.tee_on_wall_m;
  };
  for (const auto &mt : meets) {
    const bool free_a = !end_used[mt.a][static_cast<size_t>(mt.end_a)];
    const bool free_b = !end_used[mt.b][static_cast<size_t>(mt.end_b)];
    const auto end_name = [](int e) {
      return e == 0 ? WallEnd::start : WallEnd::end;
    };
    if (free_a && free_b && mt.da <= opt.corner_m && mt.db <= opt.corner_m) {
      const WallSegment keep_a = walls[mt.a];
      if (move_end(mt.a, mt.end_a, mt.sa)) {
        if (move_end(mt.b, mt.end_b, mt.sb))
          g.joins.push_back({static_cast<int>(mt.a), static_cast<int>(mt.b),
                             WallJoinKind::corner, end_name(mt.end_a),
                             end_name(mt.end_b)});
        else { // undo: a corner moves both ends or none
          walls[mt.a] = keep_a;
          end_used[mt.a][static_cast<size_t>(mt.end_a)] = false;
        }
      }
      continue;
    }
    // Tee: one wall ends near the crossing, which lies on the other one
    // away from its ends.
    if (free_a && mt.da <= opt.tee_m && mt.db > opt.corner_m &&
        on_wall(mt.b, mt.sb) && move_end(mt.a, mt.end_a, mt.sa)) {
      g.joins.push_back({static_cast<int>(mt.a), static_cast<int>(mt.b),
                         WallJoinKind::tee, end_name(mt.end_a), WallEnd::path});
      continue;
    }
    if (free_b && mt.db <= opt.tee_m && mt.da > opt.corner_m &&
        on_wall(mt.a, mt.sa) && move_end(mt.b, mt.end_b, mt.sb))
      g.joins.push_back({static_cast<int>(mt.b), static_cast<int>(mt.a),
                         WallJoinKind::tee, end_name(mt.end_b), WallEnd::path});
  }

  // 5. Seen share: (s, z) cells of the wall's face area with evidence.
  for (auto &w : walls) {
    const size_t fam = family_pos(w.family);
    const auto [lo, hi] = body(w);
    const double pad = opt.face_window_m;
    const int is0 = cell_index(w.s0, opt.cell_m),
              is1 = cell_index(w.s1, opt.cell_m);
    const int iz0 = cell_index(wp.z_floor, opt.cell_m);
    const int iz1 = cell_index(wp.z_ceiling - kEpsilonLengthM, opt.cell_m);
    std::set<FaceCell> seen;
    const auto [qlo, qhi] = offset_range(ev.wall[fam], lo - pad, hi + pad);
    for (auto q = qlo; q != qhi; ++q)
      if (q->s >= w.s0 && q->s <= w.s1 && q->z >= wp.z_floor &&
          q->z <= wp.z_ceiling)
        seen.insert(
            {cell_index(q->s, opt.cell_m), cell_index(q->z, opt.cell_m)});
    const double total = static_cast<double>(is1 - is0 + 1) * (iz1 - iz0 + 1);
    w.seen_share = total > 0.0 ? static_cast<double>(seen.size()) / total : 0.0;
    w.low_seen = w.seen_share < opt.min_seen_share;
    if (w.low_seen)
      ++g.low_seen;
  }
  g.walls = std::move(walls);
  return g;
}

namespace {

const char *to_text(WallJoinKind k) {
  switch (k) {
  case WallJoinKind::corner:
    return "corner";
  case WallJoinKind::tee:
    return "tee";
  case WallJoinKind::collinear:
    return "collinear";
  }
  return "corner";
}

const char *to_text(WallEnd e) {
  switch (e) {
  case WallEnd::start:
    return "start";
  case WallEnd::end:
    return "end";
  case WallEnd::path:
    return "path";
  }
  return "path";
}

/// @p v rounded to 5 cm, printed without a negative zero.
std::string key_coord(double v) {
  constexpr double kKeyGridM = 0.05;
  const double r = std::round(v / kKeyGridM) * kKeyGridM + 0.0;
  return fmt::format("{:.2f}", r == 0.0 ? 0.0 : r);
}

} // namespace

WallsResult
detect_walls(const Cloud &cloud, const CloudL &planes,
             const CloudN &plane_normals,
             const std::vector<core::PlaneClassRecord> &plane_classes,
             const core::StoreyRecord &storey, const WallOptions &opt,
             const std::vector<Eigen::Vector3d> &camera_path) {
  WallsResult r;
  r.families = axis_families(plane_classes, plane_normals, storey, opt);
  if (r.families.empty()) {
    warn("walls: storey {} has no vertical plane to take wall directions "
         "from ({} plane classes); no walls",
         storey.idx, plane_classes.size());
    return r;
  }
  WallPoints wp =
      wall_points(cloud, planes, plane_normals, plane_classes, storey, opt);
  const Eigen::Affine3d T = to_affine(storey.level_transform);
  wp.trajectory.reserve(camera_path.size());
  for (const auto &c : camera_path)
    wp.trajectory.push_back(T * c);
  r.faces = wall_faces(wp, r.families, opt);
  r.pairing = pair_faces(r.faces, wp, r.families, opt);
  r.graph = join_walls(measure_walls(r.faces, r.pairing, wp, r.families, opt),
                       r.families, wp, opt);
  if (r.graph.walls.empty())
    warn("walls: storey {} has no wall: {} wall points, {} faces in {} "
         "families",
         storey.idx, wp.points.size(), r.faces.size(), r.families.size());
  return r;
}

std::pair<std::vector<core::WallRecord>, std::vector<core::WallJoinRecord>>
wall_records(const WallGraph &graph, const core::StoreyRecord &storey) {
  std::vector<core::WallRecord> walls;
  std::map<std::string, int> used;
  for (const auto &g : graph.walls) {
    const Eigen::Vector2d t = tangent(g.d);
    const Eigen::Vector2d a = g.axis_offset * g.d + g.s0 * t;
    const Eigen::Vector2d b = g.axis_offset * g.d + g.s1 * t;
    core::WallRecord w;
    std::string key =
        fmt::format("walls/{}/{},{},{},{}", storey.idx, key_coord(a.x()),
                    key_coord(a.y()), key_coord(b.x()), key_coord(b.y()));
    if (const int n = used[key]++; n > 0)
      key += fmt::format("#{}", n);
    w.guid = key;
    w.storey = storey.idx;
    w.x0 = a.x();
    w.y0 = a.y();
    w.x1 = b.x();
    w.y1 = b.y();
    w.z_base = storey.z_floor;
    w.height = storey.z_ceiling - storey.z_floor;
    // One-sided: the axis is the seen face and the thickness unknown
    // (reusex-v8p); the default thickness was only the body the graph used.
    w.thickness = seen_side(g) != 0 ? -1.0 : g.thickness;
    w.thickness_ci_lo = g.assumed ? -1.0 : g.thickness_ci_lo;
    w.thickness_ci_hi = g.assumed ? -1.0 : g.thickness_ci_hi;
    w.lean_deg = g.lean_deg;
    w.is_external = g.is_external;
    w.assumed = g.assumed;
    w.seen_share = g.seen_share;
    w.face_a_points = g.points_lo;
    w.face_b_points = g.points_hi;
    w.co_visibility = g.co_visibility;
    w.axis_family = g.family;
    w.source = "detected";
    if (g.low_seen)
      w.metadata = R"({"low_seen_share":true})";
    walls.push_back(std::move(w));
  }
  std::vector<core::WallJoinRecord> joins;
  for (const auto &j : graph.joins)
    joins.push_back({walls[static_cast<size_t>(j.a)].guid,
                     walls[static_cast<size_t>(j.b)].guid, to_text(j.kind),
                     to_text(j.at_a), to_text(j.at_b)});
  return {std::move(walls), std::move(joins)};
}

WallPoints wall_points(const Cloud &cloud, const CloudL &planes,
                       const CloudN &plane_normals,
                       const std::vector<core::PlaneClassRecord> &plane_classes,
                       const core::StoreyRecord &storey,
                       const WallOptions &opt) {
  if (planes.size() != cloud.size())
    throw std::invalid_argument(
        fmt::format("wall_points: cloud has {} points but the planes cloud "
                    "has {}",
                    cloud.size(), planes.size()));
  for (const double v : storey.level_transform)
    if (!std::isfinite(v))
      throw std::invalid_argument(fmt::format(
          "wall_points: storey {} has a level_transform that is not finite",
          storey.idx));
  const Eigen::Affine3d T = to_affine(storey.level_transform);
  const Eigen::Matrix3d R = T.linear();

  // Level-frame horizontal normal per wall plane label.
  std::map<uint32_t, Eigen::Vector2d> wall_normal;
  for (const auto &pc : plane_classes) {
    if (pc.role != "wall" || pc.plane_label < 1)
      continue;
    const auto k = static_cast<size_t>(pc.plane_label - 1);
    if (k >= plane_normals.size())
      throw std::invalid_argument(fmt::format(
          "wall_points: wall plane {} has no normal ('plane_normals' holds "
          "{})",
          pc.plane_label, plane_normals.size()));
    const auto &pn = plane_normals.points[k];
    const Eigen::Vector3d n =
        R * Eigen::Vector3d(pn.normal_x, pn.normal_y, pn.normal_z);
    wall_normal[static_cast<uint32_t>(pc.plane_label)] = n.head<2>();
  }

  WallPoints wp;
  wp.z_floor = storey.z_floor;
  wp.z_ceiling = storey.z_ceiling;
  const double zlo = storey.z_floor - opt.storey_margin_m;
  const double zhi = storey.z_ceiling + opt.storey_margin_m;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto &q = cloud.points[i];
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
      continue;
    const Eigen::Vector3d p = T * Eigen::Vector3d(q.x, q.y, q.z);
    if (p.z() < zlo || p.z() > zhi)
      continue;
    const uint32_t label = planes.points[i].label;
    const auto it = wall_normal.find(label);
    if (it == wall_normal.end()) {
      wp.others.push_back(p);
      continue;
    }
    wp.points.push_back(p);
    wp.normals.push_back(it->second);
    wp.plane.push_back(static_cast<int>(label));
  }
  return wp;
}

} // namespace reusex::geometry
