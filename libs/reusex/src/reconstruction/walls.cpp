// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/walls.hpp"

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
#include <set>
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
