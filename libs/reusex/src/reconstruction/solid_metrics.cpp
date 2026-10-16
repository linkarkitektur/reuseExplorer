// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/solid_metrics.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/geometry/level_frame.hpp"
#include "reusex/geometry/point_mesh_distance.hpp"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace reusex::geometry {
namespace {

// ---------------------------------------------------------------------------
// One-to-one greedy matching

/// A candidate pair (reference index, detected index) and its score. Pairs are
/// taken best score first; ties break by reference index, then detected
/// index, so the result never depends on container or hash order.
struct Candidate {
  double score;
  std::size_t ref;
  std::size_t det;
};

/// Greedy one-to-one assignment. @p higher_is_better orders the scores.
std::vector<Candidate> greedy_match(std::vector<Candidate> cands,
                                    std::size_t n_ref, std::size_t n_det,
                                    bool higher_is_better) {
  std::sort(cands.begin(), cands.end(),
            [higher_is_better](const Candidate &a, const Candidate &b) {
              if (a.score != b.score)
                return higher_is_better ? a.score > b.score : a.score < b.score;
              return std::tie(a.ref, a.det) < std::tie(b.ref, b.det);
            });
  std::vector<bool> ref_used(n_ref, false), det_used(n_det, false);
  std::vector<Candidate> out;
  for (const auto &c : cands) {
    if (ref_used[c.ref] || det_used[c.det])
      continue;
    ref_used[c.ref] = det_used[c.det] = true;
    out.push_back(c);
  }
  return out;
}

std::optional<double> ratio(std::size_t num, std::size_t den) {
  if (den == 0)
    return std::nullopt;
  return static_cast<double>(num) / static_cast<double>(den);
}

// ---------------------------------------------------------------------------
// Wall axes

struct Axis {
  double ax, ay; ///< start
  double dx, dy; ///< unit direction
  double length;
};

Axis axis_of(const core::WallRecord &w) {
  const double len = std::hypot(w.x1 - w.x0, w.y1 - w.y0);
  if (!(len > 0.0))
    throw std::invalid_argument(
        fmt::format("solid metrics: wall '{}' has a zero-length axis", w.guid));
  return {w.x0, w.y0, (w.x1 - w.x0) / len, (w.y1 - w.y0) / len, len};
}

/// Signed position of (x, y) along @p a, from its start [m].
double along(const Axis &a, double x, double y) {
  return (x - a.ax) * a.dx + (y - a.ay) * a.dy;
}

/// Distance of (x, y) from the infinite line through @p a [m].
double off_line(const Axis &a, double x, double y) {
  return std::abs((x - a.ax) * a.dy - (y - a.ay) * a.dx);
}

/// Undirected angle between two axes [deg].
double angle_deg(const Axis &a, const Axis &b) {
  const double c = std::min(1.0, std::abs(a.dx * b.dx + a.dy * b.dy));
  return std::acos(c) * 180.0 / M_PI;
}

/// Whether detected axis @p d can be reference axis @p r: near-parallel, its
/// midpoint close to r's line, and covering enough of r along r (1D IoU of
/// the two extents projected on r >= wall_match_min_overlap).
bool walls_compatible(const Axis &r, const Axis &d,
                      const SolidMetricsOptions &opt, double &mid_dist) {
  if (angle_deg(r, d) > opt.wall_match_angle_deg)
    return false;
  const double ex = d.ax + d.dx * d.length, ey = d.ay + d.dy * d.length;
  const double mx = 0.5 * (d.ax + ex), my = 0.5 * (d.ay + ey);
  mid_dist = off_line(r, mx, my);
  if (mid_dist > opt.wall_match_dist_m)
    return false;
  const double s0 = along(r, d.ax, d.ay), s1 = along(r, ex, ey);
  const double lo = std::min(s0, s1), hi = std::max(s0, s1);
  const double overlap = std::min(hi, r.length) - std::max(lo, 0.0);
  if (overlap <= 0.0)
    return false;
  const double extent = std::max(hi, r.length) - std::min(lo, 0.0);
  return overlap / extent >= opt.wall_match_min_overlap;
}

/// Nearest-rank percentile of @p v (sorted in place), 0 < p <= 100.
double percentile(std::vector<double> &v, double p) {
  std::sort(v.begin(), v.end());
  const auto rank = static_cast<std::size_t>(
      std::ceil(p / 100.0 * static_cast<double>(v.size())));
  return v[std::max<std::size_t>(rank, 1) - 1];
}

void compare_walls(const core::SolidModel &det, const core::SolidModel &ref,
                   const SolidMetricsOptions &opt, SolidModelReport &r) {
  r.reference_walls = ref.walls.size();
  r.detected_walls = det.walls.size();

  std::vector<Axis> ra, da;
  for (const auto &w : ref.walls)
    ra.push_back(axis_of(w));
  for (const auto &w : det.walls)
    da.push_back(axis_of(w));

  std::vector<Candidate> cands;
  for (std::size_t i = 0; i < ra.size(); ++i)
    for (std::size_t j = 0; j < da.size(); ++j) {
      if (ref.walls[i].storey != det.walls[j].storey)
        continue;
      double mid = 0.0;
      if (walls_compatible(ra[i], da[j], opt, mid))
        cands.push_back({mid, i, j});
    }
  const auto matches = greedy_match(std::move(cands), ra.size(), da.size(),
                                    /*higher_is_better=*/false);
  r.matched_walls = matches.size();
  r.wall_recall = ratio(matches.size(), ra.size());
  r.wall_precision = ratio(matches.size(), da.size());

  if (matches.empty())
    return;
  double sq = 0.0;
  std::vector<double> thick_err;
  for (const auto &m : matches) {
    const auto &dw = det.walls[m.det];
    const double d0 = off_line(ra[m.ref], dw.x0, dw.y0);
    const double d1 = off_line(ra[m.ref], dw.x1, dw.y1);
    sq += d0 * d0 + d1 * d1;
    const auto &rw = ref.walls[m.ref];
    if (rw.thickness > 0.0 && dw.thickness > 0.0) {
      if (rw.assumed || dw.assumed)
        ++r.thickness_assumed_excluded;
      else
        thick_err.push_back(std::abs(dw.thickness - rw.thickness));
    }
  }
  r.wall_position_rms_m =
      std::sqrt(sq / (2.0 * static_cast<double>(matches.size())));
  r.thickness_pairs = thick_err.size();
  if (!thick_err.empty()) {
    double sum = 0.0;
    for (double e : thick_err)
      sum += e;
    r.wall_thickness_err_mean_m = sum / static_cast<double>(thick_err.size());
    r.wall_thickness_err_p95_m = percentile(thick_err, 95.0);
  }
}

// ---------------------------------------------------------------------------
// Spaces

using EK = CGAL::Exact_predicates_exact_constructions_kernel;
using Polygon = CGAL::Polygon_2<EK>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<EK>;

Polygon to_polygon(const std::vector<core::PlanPoint> &ring,
                   const std::string &guid, bool hole) {
  Polygon p;
  for (const auto &q : ring)
    p.push_back(EK::Point_2(q[0], q[1]));
  if (p.size() < 3 || !p.is_simple())
    throw std::invalid_argument(
        fmt::format("solid metrics: space '{}': {} is not a simple polygon",
                    guid, hole ? "a hole" : "the outer ring"));
  if (p.orientation() == CGAL::ZERO)
    throw std::invalid_argument(
        fmt::format("solid metrics: space '{}': {} has zero area", guid,
                    hole ? "a hole" : "the outer ring"));
  // Boolean set operations want the outer ring counter-clockwise and holes
  // clockwise; authored files come in either winding.
  const bool want_ccw = !hole;
  if ((p.orientation() == CGAL::COUNTERCLOCKWISE) != want_ccw)
    p.reverse_orientation();
  return p;
}

PolygonWithHoles to_pwh(const core::SpaceRecord &s) {
  PolygonWithHoles pwh(to_polygon(s.polygon.outer, s.guid, false));
  for (const auto &h : s.polygon.holes)
    pwh.add_hole(to_polygon(h, s.guid, true));
  return pwh;
}

double area_of(const PolygonWithHoles &p) {
  double a = CGAL::to_double(p.outer_boundary().area());
  for (auto it = p.holes_begin(); it != p.holes_end(); ++it)
    a -= std::abs(CGAL::to_double(it->area()));
  return a;
}

void compare_spaces(const core::SolidModel &det, const core::SolidModel &ref,
                    SolidModelReport &r) {
  r.reference_spaces = ref.spaces.size();
  r.detected_spaces = det.spaces.size();
  r.room_count_error =
      static_cast<int>(det.spaces.size()) - static_cast<int>(ref.spaces.size());

  std::vector<PolygonWithHoles> rp, dp;
  std::vector<double> ra, da;
  for (const auto &s : ref.spaces) {
    rp.push_back(to_pwh(s));
    ra.push_back(area_of(rp.back()));
  }
  for (const auto &s : det.spaces) {
    dp.push_back(to_pwh(s));
    da.push_back(area_of(dp.back()));
  }

  std::vector<Candidate> cands;
  for (std::size_t i = 0; i < rp.size(); ++i)
    for (std::size_t j = 0; j < dp.size(); ++j) {
      if (ref.spaces[i].storey != det.spaces[j].storey)
        continue;
      if (!CGAL::do_intersect(rp[i], dp[j]))
        continue;
      std::vector<PolygonWithHoles> inter;
      CGAL::intersection(rp[i], dp[j], std::back_inserter(inter));
      double ia = 0.0;
      for (const auto &p : inter)
        ia += area_of(p);
      const double ua = ra[i] + da[j] - ia;
      if (ia > 0.0 && ua > 0.0)
        cands.push_back({ia / ua, i, j});
    }
  const auto matches = greedy_match(std::move(cands), rp.size(), dp.size(),
                                    /*higher_is_better=*/true);
  r.matched_spaces = matches.size();
  if (rp.empty())
    return; // nothing to average over: IoU stays n/a
  double sum = 0.0;
  for (const auto &m : matches)
    sum += m.score;
  r.room_iou_mean = sum / static_cast<double>(rp.size());
}

// ---------------------------------------------------------------------------
// Openings

/// An opening placed in plan: its two ends along the host axis, its z range,
/// and the host axis it lies on.
struct PlacedOpening {
  Axis host;
  int storey; ///< the host wall's storey
  double x0, y0, x1, y1;
  double z0, z1;
};

std::vector<PlacedOpening> place_openings(const core::SolidModel &m,
                                          const char *which) {
  std::unordered_map<std::string, std::size_t> by_guid;
  for (std::size_t i = 0; i < m.walls.size(); ++i)
    by_guid.emplace(m.walls[i].guid, i);
  std::vector<PlacedOpening> out;
  for (const auto &o : m.openings) {
    const auto it = by_guid.find(o.wall);
    if (it == by_guid.end())
      throw std::invalid_argument(fmt::format(
          "solid metrics: {} opening '{}' names host wall '{}', which the "
          "model does not contain",
          which, o.guid, o.wall));
    const auto &host = m.walls[it->second];
    const Axis a = axis_of(host);
    out.push_back({a, host.storey, a.ax + a.dx * o.s_start,
                   a.ay + a.dy * o.s_start, a.ax + a.dx * o.s_end,
                   a.ay + a.dy * o.s_end, o.z_sill, o.z_head});
  }
  return out;
}

/// IoU of detected opening @p d with reference opening @p r, in r's host
/// wall face coordinates (s along r's axis, z); 0 when d is not on r's wall.
double opening_iou(const PlacedOpening &r, const PlacedOpening &d,
                   const SolidMetricsOptions &opt) {
  if (r.storey != d.storey)
    return 0.0;
  if (angle_deg(r.host, d.host) > opt.wall_match_angle_deg)
    return 0.0;
  const double mx = 0.5 * (d.x0 + d.x1), my = 0.5 * (d.y0 + d.y1);
  if (off_line(r.host, mx, my) > opt.wall_match_dist_m)
    return 0.0;
  const double r0 = along(r.host, r.x0, r.y0), r1 = along(r.host, r.x1, r.y1);
  const double d0 = along(r.host, d.x0, d.y0), d1 = along(r.host, d.x1, d.y1);
  const double rs0 = std::min(r0, r1), rs1 = std::max(r0, r1);
  const double ds0 = std::min(d0, d1), ds1 = std::max(d0, d1);
  const double iw = std::min(rs1, ds1) - std::max(rs0, ds0);
  const double ih = std::min(r.z1, d.z1) - std::max(r.z0, d.z0);
  if (iw <= 0.0 || ih <= 0.0)
    return 0.0;
  const double inter = iw * ih;
  const double uni =
      (rs1 - rs0) * (r.z1 - r.z0) + (ds1 - ds0) * (d.z1 - d.z0) - inter;
  return uni > 0.0 ? inter / uni : 0.0;
}

void compare_openings(const core::SolidModel &det, const core::SolidModel &ref,
                      const SolidMetricsOptions &opt, SolidModelReport &r) {
  const auto ro = place_openings(ref, "reference");
  const auto dop = place_openings(det, "detected");
  r.reference_openings = ro.size();
  r.detected_openings = dop.size();
  std::vector<Candidate> cands, any;
  for (std::size_t i = 0; i < ro.size(); ++i)
    for (std::size_t j = 0; j < dop.size(); ++j) {
      const double iou = opening_iou(ro[i], dop[j], opt);
      if (iou > 0.0)
        any.push_back({iou, i, j});
      if (iou >= opt.opening_match_iou && iou > 0.0)
        cands.push_back({iou, i, j});
    }
  const auto matches = greedy_match(std::move(cands), ro.size(), dop.size(),
                                    /*higher_is_better=*/true);
  r.matched_openings = matches.size();
  r.opening_recall = ratio(matches.size(), ro.size());
  r.opening_precision = ratio(matches.size(), dop.size());
  if (!ro.empty()) {
    double sum = 0.0;
    for (const auto &m : greedy_match(std::move(any), ro.size(), dop.size(),
                                      /*higher_is_better=*/true))
      sum += m.score;
    r.opening_iou_mean = sum / static_cast<double>(ro.size());
  }
}

void evidence_shares(const core::SolidModel &det, SolidModelReport &r) {
  if (det.walls.empty())
    return;
  double seen = 0.0;
  std::size_t seen_n = 0, assumed = 0;
  for (const auto &w : det.walls) {
    if (w.seen_share >= 0.0) {
      seen += w.seen_share;
      ++seen_n;
    }
    assumed += w.assumed ? 1 : 0;
  }
  if (seen_n > 0)
    r.seen_share_mean = seen / static_cast<double>(seen_n);
  r.assumed_share = ratio(assumed, det.walls.size());
}

/// The storeys @p ref covers: its `storeys` plus every storey its walls and
/// spaces stand on. Empty when it names none.
std::set<int> covered_storeys(const core::SolidModel &ref) {
  std::set<int> out;
  for (const auto &s : ref.storeys)
    out.insert(s.idx);
  for (const auto &w : ref.walls)
    out.insert(w.storey);
  for (const auto &s : ref.spaces)
    out.insert(s.storey);
  return out;
}

// ---------------------------------------------------------------------------
// Level frames (reusex-4kt)

/// The world -> level frame of each storey record of a model, by idx. A
/// storey without a record has the identity frame (its level frame is world).
std::map<int, LevelFrame> level_frames(const core::SolidModel &m,
                                       const char *which) {
  std::map<int, LevelFrame> out;
  for (const auto &s : m.storeys) {
    const LevelFrame f = LevelFrame::from_record(s);
    const double det = f.world_to_level.linear().determinant();
    bool finite = std::isfinite(det);
    for (double v : s.level_transform)
      finite = finite && std::isfinite(v);
    if (!finite || std::abs(det) < 1e-9)
      throw std::invalid_argument(fmt::format(
          "solid metrics: {} storey {} ('{}') has a singular or non-finite "
          "level_transform (determinant {})",
          which, s.idx, s.guid, det));
    out.emplace(s.idx, f);
  }
  return out;
}

LevelFrame frame_of(const std::map<int, LevelFrame> &frames, int idx) {
  const auto it = frames.find(idx);
  return it == frames.end() ? LevelFrame{} : it->second;
}

/// Which reference storey each detected storey is compared with.
///
/// When both models carry storey records, a detected storey corresponds to
/// the reference storey whose floor is nearest in world z (floor = the
/// storey's z_floor taken through its level_transform), within
/// SolidMetricsOptions::storey_match_dist_m, one-to-one, closest first: a
/// detector numbers storeys from the lowest one it saw, which need not be
/// the reference's storey 0. Otherwise storeys correspond by idx.
struct StoreyMap {
  bool by_height = false;
  std::map<int, int> det_to_ref;

  [[nodiscard]] std::optional<int> operator()(int det_idx) const {
    if (!by_height)
      return det_idx;
    const auto it = det_to_ref.find(det_idx);
    if (it == det_to_ref.end())
      return std::nullopt;
    return it->second;
  }
};

StoreyMap match_storeys(const core::SolidModel &det,
                        const core::SolidModel &ref,
                        const std::map<int, LevelFrame> &det_frames,
                        const std::map<int, LevelFrame> &ref_frames,
                        const SolidMetricsOptions &opt) {
  StoreyMap out;
  if (det.storeys.empty() || ref.storeys.empty())
    return out;
  out.by_height = true;
  // World z of the storey's floor where it crosses the level z axis (as
  // LevelFrame::world_z_at, with a general inverse: the reader does not
  // require level_transform to be exactly rigid).
  auto floor_z = [](const core::StoreyRecord &s, const LevelFrame &f) {
    return (f.world_to_level.inverse(Eigen::Affine) *
            Eigen::Vector3d(0.0, 0.0, s.z_floor))
        .z();
  };
  std::vector<double> rz, dz;
  for (const auto &s : ref.storeys)
    rz.push_back(floor_z(s, ref_frames.at(s.idx)));
  for (const auto &s : det.storeys)
    dz.push_back(floor_z(s, det_frames.at(s.idx)));
  std::vector<Candidate> cands;
  for (std::size_t i = 0; i < rz.size(); ++i)
    for (std::size_t j = 0; j < dz.size(); ++j) {
      const double d = std::abs(rz[i] - dz[j]);
      if (d <= opt.storey_match_dist_m)
        cands.push_back({d, i, j});
    }
  for (const auto &m :
       greedy_match(std::move(cands), rz.size(), dz.size(), false))
    out.det_to_ref.emplace(det.storeys[m.det].idx, ref.storeys[m.ref].idx);
  for (std::size_t j = 0; j < dz.size(); ++j)
    if (!out.det_to_ref.count(det.storeys[j].idx))
      reusex::debug("solid metrics: detected storey {} (floor at world z "
                    "{:.3f} m) has no reference storey within {:.2f} m",
                    det.storeys[j].idx, dz[j], opt.storey_match_dist_m);
  if (out.det_to_ref.empty()) {
    std::string ref_list, det_list;
    for (double z : rz)
      ref_list += fmt::format(" {:.3f}", z);
    for (double z : dz)
      det_list += fmt::format(" {:.3f}", z);
    reusex::warn("solid metrics: no detected storey lies within {:.2f} m of a "
                 "reference storey (floors at world z: reference{}, "
                 "detected{}); nothing detected is scored. Check both "
                 "models' level_transform",
                 opt.storey_match_dist_m, ref_list, det_list);
  }
  return out;
}

/// @p det re-expressed for comparison with @p ref: every wall, space and
/// opening moved to the reference storey it corresponds to (StoreyMap) and
/// into that storey's level frame, through world (reference level frame
/// <- world <- detected level frame). Elements on a detected storey with no
/// corresponding reference storey, or one @p covered (when non-empty) does
/// not contain, are left out and counted in @p dropped.
core::SolidModel to_reference_frames(const core::SolidModel &det,
                                     const core::SolidModel &ref,
                                     const std::set<int> &covered,
                                     const SolidMetricsOptions &opt,
                                     std::size_t &dropped) {
  const auto det_frames = level_frames(det, "detected");
  const auto ref_frames = level_frames(ref, "reference");
  const StoreyMap storey_of =
      match_storeys(det, ref, det_frames, ref_frames, opt);

  // The reference storey a detected storey is scored on (nullopt = left
  // out), and the detected level -> reference level transform between them.
  auto target = [&](int det_idx) -> std::optional<int> {
    const auto r = storey_of(det_idx);
    if (!r || (!covered.empty() && !covered.count(*r)))
      return std::nullopt;
    return r;
  };
  auto det_to_ref = [&](int det_idx, int ref_idx) {
    return Eigen::Affine3d(
        frame_of(ref_frames, ref_idx).world_to_level *
        frame_of(det_frames, det_idx).world_to_level.inverse(Eigen::Affine));
  };

  core::SolidModel out = det;
  out.walls.clear();
  out.spaces.clear();
  out.openings.clear();

  // Walls: axis endpoints taken at mid-height, so a small tilt between the
  // two frames' verticals moves them by the least.
  // guid -> (index in det.walls, index in out.walls; nullopt = left out)
  std::unordered_map<std::string,
                     std::pair<std::size_t, std::optional<std::size_t>>>
      host;
  for (std::size_t i = 0; i < det.walls.size(); ++i) {
    const auto &w = det.walls[i];
    const auto r = target(w.storey);
    if (!r) {
      host.emplace(w.guid, std::make_pair(i, std::nullopt));
      ++dropped;
      continue;
    }
    const Eigen::Affine3d T = det_to_ref(w.storey, *r);
    const double zm = w.z_base + 0.5 * std::max(w.height, 0.0);
    const Eigen::Vector3d a = T * Eigen::Vector3d(w.x0, w.y0, zm);
    const Eigen::Vector3d b = T * Eigen::Vector3d(w.x1, w.y1, zm);
    core::WallRecord m = w;
    m.storey = *r;
    m.x0 = a.x();
    m.y0 = a.y();
    m.x1 = b.x();
    m.y1 = b.y();
    m.z_base = (T * Eigen::Vector3d(w.x0, w.y0, w.z_base)).z();
    host.emplace(w.guid, std::make_pair(i, out.walls.size()));
    out.walls.push_back(std::move(m));
  }

  // Spaces: polygons taken at the space's floor height.
  for (const auto &sp : det.spaces) {
    const auto r = target(sp.storey);
    if (!r) {
      ++dropped;
      continue;
    }
    const Eigen::Affine3d T = det_to_ref(sp.storey, *r);
    core::SpaceRecord m = sp;
    m.storey = *r;
    auto map_ring = [&](std::vector<core::PlanPoint> &ring) {
      for (auto &q : ring) {
        const Eigen::Vector3d p = T * Eigen::Vector3d(q[0], q[1], sp.z_floor);
        q = {p.x(), p.y()};
      }
    };
    map_ring(m.polygon.outer);
    for (auto &h : m.polygon.holes)
      map_ring(h);
    out.spaces.push_back(std::move(m));
  }

  // Openings: each end placed in 3D on its host wall (detected frame), at the
  // opening's mid-height, then measured along the re-expressed host axis;
  // sill and head are the reference-frame heights above the opening's centre.
  for (const auto &o : det.openings) {
    const auto it = host.find(o.wall);
    if (it == host.end()) {
      out.openings.push_back(o); // place_openings reports the unknown host
      continue;
    }
    if (!it->second.second) {
      ++dropped;
      continue;
    }
    const core::WallRecord &dw = det.walls[it->second.first];
    const core::WallRecord &mw = out.walls[*it->second.second];
    const Eigen::Affine3d T = det_to_ref(dw.storey, mw.storey);
    const Axis da = axis_of(dw);
    const Axis ma = axis_of(mw);
    const double zm = 0.5 * (o.z_sill + o.z_head);
    auto at = [&](double s, double z) {
      return Eigen::Vector3d(
          T * Eigen::Vector3d(da.ax + da.dx * s, da.ay + da.dy * s, z));
    };
    const Eigen::Vector3d p0 = at(o.s_start, zm), p1 = at(o.s_end, zm);
    const double sc = 0.5 * (o.s_start + o.s_end);
    core::OpeningRecord m = o;
    m.s_start = along(ma, p0.x(), p0.y());
    m.s_end = along(ma, p1.x(), p1.y());
    m.z_sill = at(sc, o.z_sill).z();
    m.z_head = at(sc, o.z_head).z();
    out.openings.push_back(std::move(m));
  }
  return out;
}

nlohmann::ordered_json opt_json(const std::optional<double> &v) {
  return v ? nlohmann::ordered_json(*v) : nlohmann::ordered_json(nullptr);
}

} // namespace

std::string SolidModelReport::to_json(int indent) const {
  nlohmann::ordered_json j;
  j["room_count_error"] = room_count_error
                              ? nlohmann::ordered_json(*room_count_error)
                              : nlohmann::ordered_json(nullptr);
  j["room_iou_mean"] = opt_json(room_iou_mean);
  j["wall_position_rms_m"] = opt_json(wall_position_rms_m);
  j["wall_thickness_err_mean_m"] = opt_json(wall_thickness_err_mean_m);
  j["wall_thickness_err_p95_m"] = opt_json(wall_thickness_err_p95_m);
  j["wall_recall"] = opt_json(wall_recall);
  j["wall_precision"] = opt_json(wall_precision);
  j["opening_recall"] = opt_json(opening_recall);
  j["opening_precision"] = opt_json(opening_precision);
  j["opening_iou_mean"] = opt_json(opening_iou_mean);
  j["fit_mean_m"] = opt_json(fit_mean_m);
  j["fit_within_share"] = opt_json(fit_within_share);
  j["mesh_area_m2"] = opt_json(mesh_area_m2);
  j["mesh_unsupported_share"] = opt_json(mesh_unsupported_share);
  j["seen_share_mean"] = opt_json(seen_share_mean);
  j["assumed_share"] = opt_json(assumed_share);
  j["fit_source"] = fit_source.empty() ? nlohmann::ordered_json(nullptr)
                                       : nlohmann::ordered_json(fit_source);
  j["counts"] = {{"reference_walls", reference_walls},
                 {"detected_walls", detected_walls},
                 {"matched_walls", matched_walls},
                 {"reference_spaces", reference_spaces},
                 {"detected_spaces", detected_spaces},
                 {"matched_spaces", matched_spaces},
                 {"reference_openings", reference_openings},
                 {"detected_openings", detected_openings},
                 {"matched_openings", matched_openings},
                 {"fit_points", fit_points},
                 {"support_samples", support_samples},
                 {"thickness_pairs", thickness_pairs},
                 {"thickness_assumed_excluded", thickness_assumed_excluded},
                 {"detected_outside_reference", detected_outside_reference}};
  return j.dump(indent);
}

SolidModelReport compare_solid_models(const core::SolidModel &detected_in,
                                      const core::SolidModel &reference,
                                      const SolidMetricsOptions &opt) {
  SolidModelReport r;
  const std::set<int> covered = covered_storeys(reference);
  const core::SolidModel detected = to_reference_frames(
      detected_in, reference, covered, opt, r.detected_outside_reference);
  if (r.detected_outside_reference > 0)
    reusex::debug("solid metrics: {} detected elements stand on storeys the "
                  "reference does not cover and are not scored",
                  r.detected_outside_reference);
  if (detected.has_walls() && reference.has_walls())
    compare_walls(detected, reference, opt, r);
  if (detected.has_spaces() && reference.has_spaces())
    compare_spaces(detected, reference, r);
  if (detected.has_openings() && reference.has_openings())
    compare_openings(detected, reference, opt, r);
  if (detected.has_walls())
    evidence_shares(detected, r);
  return r;
}

void fit_report(const Cloud &cloud, const pcl::PolygonMesh &mesh,
                const SolidMetricsOptions &opt, SolidModelReport &report) {
  // One definition of "distance from a scan point to the mesh", shared with
  // the render deviation layer (geometry/point_mesh_distance.hpp).
  const std::vector<double> d = point_to_mesh_distances(cloud, mesh);
  double sum = 0.0;
  std::size_t n = 0, within = 0;
  for (const double v : d) {
    if (!std::isfinite(v))
      continue;
    sum += v;
    ++n;
    within += v <= opt.fit_within_m ? 1 : 0;
  }
  if (n == 0)
    throw std::invalid_argument(fmt::format(
        "fit_report: cloud has no finite point ({} points)", cloud.size()));

  const MeshSupport support =
      mesh_support(cloud, mesh, opt.support_dist_m, opt.support_spacing_m);

  report.fit_points = n;
  report.fit_mean_m = sum / static_cast<double>(n);
  report.fit_within_share =
      static_cast<double>(within) / static_cast<double>(n);
  report.support_samples = support.samples;
  report.mesh_area_m2 = support.area_m2;
  report.mesh_unsupported_share = support.unsupported_area_m2 / support.area_m2;
  reusex::debug("fit_report: {} points against {} faces, mean {:.4f} m, "
                "{:.1f}% within {:.3f} m; {:.1f}% of {:.1f} m^2 mesh without "
                "a point within {:.2f} m",
                n, mesh.polygons.size(), *report.fit_mean_m,
                100.0 * *report.fit_within_share, opt.fit_within_m,
                100.0 * *report.mesh_unsupported_share, support.area_m2,
                opt.support_dist_m);
}

} // namespace reusex::geometry
