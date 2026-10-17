// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/segmentation/spaces.hpp"

#include "reusex/core/logging.hpp"

#include <CGAL/Arr_batched_point_location.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include <fmt/format.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace reusex::geometry {

namespace {

using K = CGAL::Exact_predicates_exact_constructions_kernel;
using Point = K::Point_2;
using Segment = K::Segment_2;
using ArrTraits = CGAL::Arr_segment_traits_2<K>;
using Arr = CGAL::Arrangement_2<ArrTraits>;
using GpsTraits = CGAL::Gps_segment_traits_2<K>;
using Polygon = GpsTraits::Polygon_2;
using PolygonWithHoles = GpsTraits::Polygon_with_holes_2;
using PolygonSet = CGAL::Polygon_set_2<K>;
using Vec2 = Eigen::Vector2d;

constexpr double kPi = 3.14159265358979323846;

Vec2 to_vec(const Point &p) {
  return {CGAL::to_double(p.x()), CGAL::to_double(p.y())};
}

/// Drop the vertices of a closed ring that lie on the line through their
/// neighbours (exactly), until none is left.
std::vector<Point> drop_collinear(std::vector<Point> ring) {
  bool changed = true;
  while (changed && ring.size() >= 3) {
    changed = false;
    std::vector<Point> out;
    out.reserve(ring.size());
    const size_t n = ring.size();
    for (size_t i = 0; i < n; ++i) {
      const Point &prev = out.empty() ? ring[(i + n - 1) % n] : out.back();
      const Point &next = ring[(i + 1) % n];
      if (CGAL::collinear(prev, ring[i], next)) {
        changed = true;
        continue;
      }
      out.push_back(ring[i]);
    }
    ring = std::move(out);
  }
  return ring;
}

/// Split a closed vertex walk at every repeated vertex into loops that each
/// visit a vertex once (a walk that touches itself at a pinch).
std::vector<std::vector<Point>>
split_at_repeats(const std::vector<Point> &walk) {
  std::vector<std::vector<Point>> loops;
  std::vector<Point> stack;
  std::map<Point, size_t> at;
  for (const auto &p : walk) {
    auto it = at.find(p);
    if (it != at.end()) {
      const size_t i = it->second;
      std::vector<Point> loop(stack.begin() + static_cast<std::ptrdiff_t>(i),
                              stack.end());
      for (size_t k = i + 1; k < stack.size(); ++k)
        at.erase(stack[k]);
      stack.resize(i + 1);
      if (loop.size() >= 3)
        loops.push_back(std::move(loop));
      continue;
    }
    at.emplace(p, stack.size());
    stack.push_back(p);
  }
  if (stack.size() >= 3)
    loops.push_back(std::move(stack));
  return loops;
}

/// Chain directed edges (every vertex as many in as out) into closed walks
/// and split those into simple loops without collinear vertices.
std::vector<std::vector<Point>>
trace_loops(const std::vector<std::pair<Point, Point>> &edges) {
  std::map<Point, std::vector<size_t>> out;
  for (size_t i = 0; i < edges.size(); ++i)
    out[edges[i].first].push_back(i);
  std::vector<uint8_t> used(edges.size(), 0);
  std::vector<std::vector<Point>> loops;
  for (size_t e0 = 0; e0 < edges.size(); ++e0) {
    if (used[e0])
      continue;
    std::vector<Point> walk;
    size_t e = e0;
    const Point start = edges[e0].first;
    while (true) {
      used[e] = 1;
      walk.push_back(edges[e].first);
      const Point &v = edges[e].second;
      if (v == start)
        break;
      auto it = out.find(v);
      size_t next = edges.size();
      if (it != out.end())
        for (size_t c : it->second)
          if (!used[c]) {
            next = c;
            break;
          }
      if (next == edges.size())
        break; // unbalanced input; close the walk as it is
      e = next;
    }
    for (auto &loop : split_at_repeats(walk)) {
      loop = drop_collinear(std::move(loop));
      if (loop.size() >= 3)
        loops.push_back(std::move(loop));
    }
  }
  return loops;
}

/// The region bounded by a set of directed edges with the region on their
/// left: the loops of trace_loops() combined by symmetric difference -- for
/// loops that never cross, that is the region whatever the chaining at a
/// pinch vertex.
PolygonSet
region_from_edges(const std::vector<std::pair<Point, Point>> &edges) {
  PolygonSet set;
  for (const auto &loop : trace_loops(edges)) {
    Polygon poly(loop.begin(), loop.end());
    const auto orient = poly.orientation();
    if (orient == CGAL::COLLINEAR)
      continue;
    if (orient == CGAL::CLOCKWISE)
      poly.reverse_orientation();
    set.symmetric_difference(poly);
  }
  return set;
}

/// The directed cell edges around the cells where @p in holds (the region
/// on their left), on the level-frame lattice of @p g.
template <typename In>
std::vector<std::pair<Point, Point>> cell_edges(const RoomGrid &g,
                                                const In &in) {
  std::vector<std::pair<Point, Point>> edges;
  const double cell = g.cell_size;
  auto corner = [&](int c, int r) {
    return Point((g.col0 + c) * cell, (g.row0 + r) * cell);
  };
  auto is = [&](int c, int r) {
    return c >= 0 && r >= 0 && c < g.width && r < g.height && in(g.index(c, r));
  };
  for (int r = 0; r < g.height; ++r)
    for (int c = 0; c < g.width; ++c) {
      if (!is(c, r))
        continue;
      if (!is(c, r - 1))
        edges.emplace_back(corner(c, r), corner(c + 1, r));
      if (!is(c + 1, r))
        edges.emplace_back(corner(c + 1, r), corner(c + 1, r + 1));
      if (!is(c, r + 1))
        edges.emplace_back(corner(c + 1, r + 1), corner(c, r + 1));
      if (!is(c - 1, r))
        edges.emplace_back(corner(c, r + 1), corner(c, r));
    }
  return edges;
}

/// Douglas-Peucker on an open polyline [first, last] (both kept).
void simplify_run(const std::vector<Vec2> &pts, size_t first, size_t last,
                  double tol, std::vector<uint8_t> &keep) {
  if (last <= first + 1)
    return;
  const Vec2 a = pts[first];
  const Vec2 d = pts[last] - a;
  const double len = d.norm();
  double worst = -1.0;
  size_t at = first;
  for (size_t i = first + 1; i < last; ++i) {
    const Vec2 v = pts[i] - a;
    const double dist =
        len > 1e-12 ? std::abs(d.x() * v.y() - d.y() * v.x()) / len : v.norm();
    if (dist > worst) {
      worst = dist;
      at = i;
    }
  }
  if (worst > tol) {
    keep[at] = 1;
    simplify_run(pts, first, at, tol, keep);
    simplify_run(pts, at, last, tol, keep);
  }
}

/// A closed loop simplified to within @p tol (Douglas-Peucker from its
/// first point and the point farthest from it).
std::vector<Vec2> simplify_loop(const std::vector<Point> &loop, double tol) {
  std::vector<Vec2> pts;
  pts.reserve(loop.size() + 1);
  for (const auto &p : loop)
    pts.push_back(to_vec(p));
  if (pts.size() < 4)
    return pts;
  size_t far = 0;
  for (size_t i = 1; i < pts.size(); ++i)
    if ((pts[i] - pts[0]).squaredNorm() > (pts[far] - pts[0]).squaredNorm())
      far = i;
  pts.push_back(pts[0]);
  std::vector<uint8_t> keep(pts.size(), 0);
  keep[0] = keep[far] = keep.back() = 1;
  simplify_run(pts, 0, far, tol, keep);
  simplify_run(pts, far, pts.size() - 1, tol, keep);
  std::vector<Vec2> out;
  for (size_t i = 0; i + 1 < pts.size(); ++i)
    if (keep[i])
      out.push_back(pts[i]);
  return out;
}

double area_of(const PolygonWithHoles &p) {
  double a = std::abs(CGAL::to_double(p.outer_boundary().area()));
  for (auto h = p.holes_begin(); h != p.holes_end(); ++h)
    a -= std::abs(CGAL::to_double(h->area()));
  return a;
}

double area_of(const PolygonSet &s) {
  std::vector<PolygonWithHoles> parts;
  s.polygons_with_holes(std::back_inserter(parts));
  double a = 0.0;
  for (const auto &p : parts)
    a += area_of(p);
  return a;
}

/// The raster region of room @p label: the cells' outline (cell corners on
/// the level-frame lattice), holes included.
PolygonSet raster_region(const RoomGrid &g, const std::vector<int32_t> &labels,
                         int32_t label) {
  return region_from_edges(
      cell_edges(g, [&](size_t i) { return labels[i] == label; }));
}

/// The room labels with every pocket a room encloses filled with the room
/// (unlabelled cells only: furniture, a column, band-wall clutter -- not a
/// room inside a room). @p holes receives, per room, the pockets of at
/// least @p min_hole_area [m^2].
std::vector<int32_t> fill_room_pockets(const RoomGrid &g,
                                       const std::set<int32_t> &rooms,
                                       double min_hole_area,
                                       std::map<int32_t, int> &holes) {
  std::vector<int32_t> filled = g.labels;
  const int w = g.width;
  const int h = g.height;
  const size_t n = filled.size();
  const double cell_area = g.cell_size * g.cell_size;
  std::vector<uint8_t> seen(n);
  std::vector<int> queue;
  queue.reserve(n);
  for (const int32_t L : rooms) {
    // Flood from the border through every cell that is not L.
    std::fill(seen.begin(), seen.end(), uint8_t{0});
    queue.clear();
    auto push = [&](int c, int r) {
      const size_t i = g.index(c, r);
      if (!seen[i] && g.labels[i] != L) {
        seen[i] = 1;
        queue.push_back(static_cast<int>(i));
      }
    };
    for (int c = 0; c < w; ++c) {
      push(c, 0);
      push(c, h - 1);
    }
    for (int r = 0; r < h; ++r) {
      push(0, r);
      push(w - 1, r);
    }
    auto flood = [&](size_t from) {
      for (size_t k = from; k < queue.size(); ++k) {
        const int i = queue[k];
        const int c = i % w;
        const int r = i / w;
        if (c > 0)
          push(c - 1, r);
        if (c + 1 < w)
          push(c + 1, r);
        if (r > 0)
          push(c, r - 1);
        if (r + 1 < h)
          push(c, r + 1);
      }
    };
    flood(0);
    // What the flood did not reach and is not L is a pocket of L.
    int count = 0;
    for (size_t i = 0; i < n; ++i) {
      if (seen[i] || g.labels[i] == L)
        continue;
      const size_t from = queue.size();
      seen[i] = 1;
      queue.push_back(static_cast<int>(i));
      flood(from);
      if (static_cast<double>(queue.size() - from) * cell_area >= min_hole_area)
        ++count;
      for (size_t k = from; k < queue.size(); ++k)
        if (g.labels[static_cast<size_t>(queue[k])] == 0)
          filled[static_cast<size_t>(queue[k])] = L;
    }
    holes[L] = count;
  }
  return filled;
}

/// One wall of the storey, in the form the arrangement and the boundaries
/// need.
struct PlacedWall {
  const core::WallRecord *rec = nullptr;
  Vec2 a, u, n; ///< axis start, unit direction, unit left normal
  double len = 0.0;
  double half = 0.0; ///< 0: the axis is the (only) face
};

struct FaceInfo {
  Arr::Face_const_handle f;
  double area = 0.0;
  double perimeter = 0.0;
  std::map<int32_t, size_t> cells; ///< room -> cell centres inside
  enum Class { mixed, empty, pure } cls = mixed;
  int32_t label = 0; ///< the room of a pure face
  /// Other rooms that live mostly in a pure face (a meeting room the wall
  /// graph does not close off): they keep their cells, the face's room
  /// gets the rest.
  std::vector<int32_t> guests;
  bool sliver = false;
};

/// Walk a CCB: its signed area (shoelace over the walk) and length.
template <typename Circ> std::pair<double, double> ccb_measure(Circ c) {
  double a = 0.0;
  double len = 0.0;
  const Circ start = c;
  do {
    const Vec2 p = to_vec(c->source()->point());
    const Vec2 q = to_vec(c->target()->point());
    a += p.x() * q.y() - q.x() * p.y();
    len += (q - p).norm();
  } while (++c != start);
  return {0.5 * a, len};
}

/// The union of the bounded faces for which @p in holds.
template <typename Pred>
PolygonSet region_of_faces(const Arr &arr, const Pred &in) {
  std::vector<std::pair<Point, Point>> edges;
  for (auto e = arr.halfedges_begin(); e != arr.halfedges_end(); ++e) {
    const auto f = e->face();
    const auto t = e->twin()->face();
    // An arrangement face lies to the left of its halfedges.
    if (!f->is_unbounded() && in(f) && (t->is_unbounded() || !in(t)))
      edges.emplace_back(e->source()->point(), e->target()->point());
  }
  return region_from_edges(edges);
}

/// A ring in doubles: counter-clockwise for an outer ring, clockwise for a
/// hole, starting at its lexicographically smallest vertex.
std::vector<core::PlanPoint> to_plan_ring(const Polygon &poly, bool hole) {
  std::vector<core::PlanPoint> ring;
  for (auto v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
    ring.push_back({CGAL::to_double(v->x()), CGAL::to_double(v->y())});
  const bool ccw = poly.orientation() == CGAL::COUNTERCLOCKWISE;
  if (ccw == hole)
    std::reverse(ring.begin(), ring.end());
  const auto first = std::min_element(ring.begin(), ring.end());
  std::rotate(ring.begin(), first, ring.end());
  return ring;
}

std::vector<Point> polygon_points(const Polygon &p) {
  return {p.vertices_begin(), p.vertices_end()};
}

/// A ring snapped to a 1 um lattice, so that the doubles a SpaceRecord
/// holds are the points every check below ran on (exact constructions can
/// put two vertices 1e-16 m apart, which round to one double), without
/// consecutive repeats.
std::vector<Point> snapped(const std::vector<Point> &ring) {
  constexpr double kSnap = 1e6;
  std::vector<Point> out;
  out.reserve(ring.size());
  for (const auto &p : ring) {
    const Point q(std::round(CGAL::to_double(p.x()) * kSnap) / kSnap,
                  std::round(CGAL::to_double(p.y()) * kSnap) / kSnap);
    if (out.empty() || out.back() != q)
      out.push_back(q);
  }
  while (out.size() > 1 && out.front() == out.back())
    out.pop_back();
  return out;
}

/// The simple loops of a snapped ring, without collinear vertices.
std::vector<std::vector<Point>> clean_loops(const std::vector<Point> &ring) {
  std::vector<std::vector<Point>> out;
  for (auto &loop : split_at_repeats(snapped(ring))) {
    loop = drop_collinear(std::move(loop));
    if (loop.size() < 3)
      continue;
    if (!CGAL::is_simple_2(loop.begin(), loop.end(), K()))
      continue; // snapping folded it (a sliver below 1 um)
    out.push_back(std::move(loop));
  }
  return out;
}

/// The chosen piece made valid for a SpaceRecord: a simple outer ring (the
/// largest loop of a self-touching one), holes of at least min_hole_area
/// that lie strictly inside it and apart from each other, no collinear
/// vertices, every vertex on a 1 um lattice.
PolygonWithHoles clean_piece(const PolygonWithHoles &piece,
                             const SpacesOptions &o) {
  Polygon outer;
  double best = 0.0;
  for (auto &loop : clean_loops(polygon_points(piece.outer_boundary()))) {
    Polygon p(loop.begin(), loop.end());
    const double a = std::abs(CGAL::to_double(p.area()));
    if (a > best) {
      best = a;
      outer = std::move(p);
    }
  }
  if (outer.size() >= 3 && outer.orientation() == CGAL::CLOCKWISE)
    outer.reverse_orientation();
  PolygonWithHoles out(outer);
  if (outer.size() < 3)
    return out;
  std::vector<Polygon> holes;
  for (auto h = piece.holes_begin(); h != piece.holes_end(); ++h)
    for (auto &loop : clean_loops(polygon_points(*h))) {
      Polygon p(loop.begin(), loop.end());
      if (std::abs(CGAL::to_double(p.area())) < o.min_hole_area)
        continue;
      if (p.orientation() == CGAL::COUNTERCLOCKWISE)
        p.reverse_orientation();
      holes.push_back(std::move(p));
    }
  // Largest first, so a hole that touches a bigger one is the one dropped.
  std::stable_sort(holes.begin(), holes.end(),
                   [](const auto &a, const auto &b) {
                     return CGAL::abs(a.area()) > CGAL::abs(b.area());
                   });
  GpsTraits traits;
  for (auto &h : holes) {
    PolygonWithHoles trial = out;
    trial.add_hole(h);
    if (CGAL::is_valid_polygon_with_holes(trial, traits))
      out = std::move(trial);
  }
  return out;
}

/// Aspect (long / short side) of the minimum-area rectangle around @p ring.
double min_rect_aspect(const std::vector<core::PlanPoint> &ring) {
  double best_area = std::numeric_limits<double>::infinity();
  double aspect = 1.0;
  for (size_t i = 0; i < ring.size(); ++i) {
    const auto &p = ring[i];
    const auto &q = ring[(i + 1) % ring.size()];
    Vec2 u(q[0] - p[0], q[1] - p[1]);
    const double l = u.norm();
    if (!(l > 1e-9))
      continue;
    u /= l;
    const Vec2 v(-u.y(), u.x());
    double s0 = std::numeric_limits<double>::infinity(), s1 = -s0;
    double t0 = s0, t1 = -s0;
    for (const auto &r : ring) {
      const Vec2 w(r[0], r[1]);
      s0 = std::min(s0, w.dot(u));
      s1 = std::max(s1, w.dot(u));
      t0 = std::min(t0, w.dot(v));
      t1 = std::max(t1, w.dot(v));
    }
    const double a = (s1 - s0) * (t1 - t0);
    if (a < best_area) {
      best_area = a;
      const double lo = std::min(s1 - s0, t1 - t0);
      const double hi = std::max(s1 - s0, t1 - t0);
      aspect = lo > 1e-9 ? hi / lo : std::numeric_limits<double>::infinity();
    }
  }
  return aspect;
}

double ring_length(const std::vector<core::PlanPoint> &ring) {
  double len = 0.0;
  for (size_t i = 0; i < ring.size(); ++i) {
    const auto &p = ring[i];
    const auto &q = ring[(i + 1) % ring.size()];
    len += std::hypot(q[0] - p[0], q[1] - p[1]);
  }
  return len;
}

double plan_ring_area(const std::vector<core::PlanPoint> &ring) {
  double a = 0.0;
  for (size_t i = 0; i < ring.size(); ++i) {
    const auto &p = ring[i];
    const auto &q = ring[(i + 1) % ring.size()];
    a += p[0] * q[1] - q[0] * p[1];
  }
  return 0.5 * a;
}

/// The space boundaries of one space: its ring edges that lie on a wall
/// face, merged per (wall, side) into one s range.
void add_boundaries(const core::SpaceRecord &space,
                    const std::vector<PlacedWall> &walls,
                    const SpacesOptions &o,
                    std::vector<core::SpaceBoundaryRecord> &out) {
  const double sin_max = std::sin(o.boundary_max_angle_deg * kPi / 180.0);
  // (wall index, side a?) -> s range
  std::map<std::pair<size_t, bool>, std::pair<double, double>> ranges;
  auto scan = [&](const std::vector<core::PlanPoint> &ring) {
    for (size_t i = 0; i < ring.size(); ++i) {
      const Vec2 p(ring[i][0], ring[i][1]);
      const Vec2 q(ring[(i + 1) % ring.size()][0],
                   ring[(i + 1) % ring.size()][1]);
      const double l = (q - p).norm();
      if (!(l > 1e-9))
        continue;
      const Vec2 e = (q - p) / l;
      // Outer rings run counter-clockwise and holes clockwise, so the space
      // is left of every edge.
      const Vec2 left(-e.y(), e.x());
      for (size_t w = 0; w < walls.size(); ++w) {
        const auto &W = walls[w];
        if (std::abs(e.x() * W.u.y() - e.y() * W.u.x()) > sin_max)
          continue;
        const double side = left.dot(W.n);
        if (side == 0.0)
          continue;
        const bool a_side = side > 0.0;
        // The face the space sees: +half on side a, -half on side b.
        const double face = a_side ? W.half : -W.half;
        const double dp = (p - W.a).dot(W.n) - face;
        const double dq = (q - W.a).dot(W.n) - face;
        if (std::abs(dp) > o.boundary_tolerance_m ||
            std::abs(dq) > o.boundary_tolerance_m)
          continue;
        const double sp = (p - W.a).dot(W.u);
        const double sq = (q - W.a).dot(W.u);
        const double lo = std::max(0.0, std::min(sp, sq));
        const double hi = std::min(W.len, std::max(sp, sq));
        if (hi - lo < o.boundary_min_length_m)
          continue;
        auto [it, fresh] = ranges.try_emplace({w, a_side}, lo, hi);
        if (!fresh) {
          it->second.first = std::min(it->second.first, lo);
          it->second.second = std::max(it->second.second, hi);
        }
      }
    }
  };
  scan(space.polygon.outer);
  for (const auto &h : space.polygon.holes)
    scan(h);
  for (const auto &[key, range] : ranges) {
    core::SpaceBoundaryRecord b;
    b.space = space.guid;
    b.wall = walls[key.first].rec->guid;
    b.side = key.second ? "a" : "b";
    b.s_start = range.first;
    b.s_end = range.second;
    out.push_back(std::move(b));
  }
}

} // namespace

auto polygonise_spaces(const std::vector<RoomGrid> &grids,
                       const std::vector<core::StoreyRecord> &storeys,
                       const std::vector<core::WallRecord> &all_walls,
                       const SpacesOptions &o) -> SpacesResult {
  SpacesResult result;
  for (const auto &g : grids) {
    SpacesStoreyStats st;
    st.storey_idx = g.storey_idx;
    const double cell = g.cell_size;
    const double cell_area = cell * cell;
    const core::StoreyRecord *storey = nullptr;
    for (const auto &s : storeys)
      if (s.idx == g.storey_idx)
        storey = &s;

    std::set<int32_t> labels;
    for (int32_t l : g.labels)
      if (l > 0)
        labels.insert(l);
    st.rooms = static_cast<int>(labels.size());
    if (labels.empty()) {
      result.stats.push_back(st);
      continue;
    }
    // Furniture pockets belong to their room: the faces count them and the
    // raster fallback has no furniture holes.
    std::map<int32_t, int> furniture;
    std::vector<int32_t> filled =
        fill_room_pockets(g, labels, o.open_plan_min_hole_area, furniture);

    // The storey's walls, by guid so the result does not depend on their
    // order.
    std::vector<PlacedWall> walls;
    if (o.use_walls)
      for (const auto &w : all_walls) {
        if (w.storey != g.storey_idx)
          continue;
        if (w.seen_share >= 0.0 && w.seen_share < o.wall_min_seen_share)
          continue;
        PlacedWall P;
        P.rec = &w;
        P.a = Vec2(w.x0, w.y0);
        const Vec2 d(w.x1 - w.x0, w.y1 - w.y0);
        P.len = d.norm();
        if (!(P.len > 1e-6) || !std::isfinite(P.len))
          continue;
        P.u = d / P.len;
        P.n = Vec2(-P.u.y(), P.u.x());
        P.half = (!o.axes_only && w.thickness > 0.0) ? 0.5 * w.thickness : 0.0;
        walls.push_back(P);
      }
    std::stable_sort(walls.begin(), walls.end(),
                     [](const PlacedWall &a, const PlacedWall &b) {
                       return a.rec->guid < b.rec->guid;
                     });
    st.walls = static_cast<int>(walls.size());

    // --- 0. exterior beyond the external walls (reusex-e0k) --------------
    if (o.exterior_depth_m > 0.0) {
      std::vector<uint8_t> outside(filled.size(), 0);
      for (const auto &W : walls) {
        if (!W.rec->is_external)
          continue;
        // The cells within exterior_depth_m of the wall's run, by side.
        const Vec2 b = W.a + W.len * W.u;
        const double reach = std::max(W.half, 0.0) + o.exterior_depth_m;
        const auto lo = [&](double v, int origin) {
          return static_cast<int>(std::floor(v / cell)) - origin;
        };
        const int c0 =
            std::max(0, lo(std::min(W.a.x(), b.x()) - reach, g.col0));
        const int c1 =
            std::min(g.width - 1, lo(std::max(W.a.x(), b.x()) + reach, g.col0));
        const int r0 =
            std::max(0, lo(std::min(W.a.y(), b.y()) - reach, g.row0));
        const int r1 = std::min(g.height - 1,
                                lo(std::max(W.a.y(), b.y()) + reach, g.row0));
        std::vector<size_t> left, right; // room cells beyond each face
        size_t n_left = 0, n_right = 0;
        for (int r = r0; r <= r1; ++r)
          for (int c = c0; c <= c1; ++c) {
            const size_t i = g.index(c, r);
            if (filled[i] <= 0)
              continue;
            const Vec2 q((g.col0 + c + 0.5) * cell, (g.row0 + r + 0.5) * cell);
            const double s = (q - W.a).dot(W.u);
            const double d = (q - W.a).dot(W.n);
            if (s < 0.0 || s > W.len || std::abs(d) > reach)
              continue;
            (d > 0.0 ? n_left : n_right) += 1;
            if (std::abs(d) > W.half)
              (d > 0.0 ? left : right).push_back(i);
          }
        const bool in_left =
            static_cast<double>(n_left) >=
                o.exterior_min_ratio * static_cast<double>(n_right) &&
            n_left > 0;
        const bool in_right =
            static_cast<double>(n_right) >=
                o.exterior_min_ratio * static_cast<double>(n_left) &&
            n_right > 0;
        if (in_left == in_right) {
          debug("spaces: storey {}: external wall {}: {} room cells left, {} "
                "right; inside unclear, not cut",
                g.storey_idx, W.rec->guid, n_left, n_right);
          continue;
        }
        const auto &cut = in_left ? right : left;
        if (cut.empty())
          continue;
        ++st.exterior_walls;
        for (size_t i : cut)
          outside[i] = 1;
      }
      std::map<int32_t, size_t> before, after;
      for (size_t i = 0; i < filled.size(); ++i) {
        if (filled[i] <= 0)
          continue;
        ++before[filled[i]];
        if (outside[i]) {
          filled[i] = 0;
          st.exterior_area += cell_area;
        } else {
          ++after[filled[i]];
        }
      }
      for (const auto &[L, n] : before)
        if (!after.count(L)) {
          ++st.exterior_rooms;
          labels.erase(L);
          info("spaces: storey {}: room {} ({:.2f} m^2) lies beyond the "
               "external walls; no space",
               g.storey_idx, L, static_cast<double>(n) * cell_area);
        }
    }

    // --- 1. arrangement of the wall faces --------------------------------
    Arr arr;
    std::vector<FaceInfo> faces;
    std::unordered_map<const void *, size_t> face_index;
    if (!walls.empty()) {
      using Key = std::array<double, 4>;
      std::vector<Key> keys;
      auto add = [&](const Vec2 &p, const Vec2 &q) {
        if ((q - p).norm() < 1e-9)
          return;
        Key k{p.x(), p.y(), q.x(), q.y()};
        if (std::tie(k[2], k[3]) < std::tie(k[0], k[1]))
          k = {q.x(), q.y(), p.x(), p.y()};
        keys.push_back(k);
      };
      for (const auto &W : walls) {
        const Vec2 b = W.a + W.len * W.u;
        const Vec2 ext = o.join_extension_m * W.u;
        if (W.half > 0.0) {
          for (double s : {W.half, -W.half})
            add(W.a + s * W.n - ext, b + s * W.n + ext);
          add(W.a + W.half * W.n, W.a - W.half * W.n);
          add(b + W.half * W.n, b - W.half * W.n);
        } else {
          add(W.a - ext, b + ext);
        }
      }
      // Bridges across offset runs of one wall line (reusex-ajf).
      if (o.bridge_max_offset_m > 0.0) {
        const double sin_max = std::sin(o.bridge_max_angle_deg * kPi / 180.0);
        for (size_t i = 0; i < walls.size(); ++i)
          for (size_t j = i + 1; j < walls.size(); ++j) {
            // Each pair is measured from the wall whose axis comes first by
            // its coordinates, so the bridges depend on the geometry alone,
            // not on the walls' GUIDs (they are new on every fresh run).
            const auto geo = [](const PlacedWall &W) {
              const Vec2 e = W.a + W.len * W.u;
              auto k = std::make_tuple(W.a.x(), W.a.y(), e.x(), e.y());
              if (std::tie(std::get<2>(k), std::get<3>(k)) <
                  std::tie(std::get<0>(k), std::get<1>(k)))
                k = std::make_tuple(e.x(), e.y(), W.a.x(), W.a.y());
              return k;
            };
            const bool swap = geo(walls[j]) < geo(walls[i]);
            const auto &A = swap ? walls[j] : walls[i];
            const auto &B = swap ? walls[i] : walls[j];
            if (std::abs(A.u.x() * B.u.y() - A.u.y() * B.u.x()) > sin_max)
              continue;
            const Vec2 b0 = B.a;
            const Vec2 b1 = B.a + B.len * B.u;
            const double off =
                0.5 * ((b0 - A.a).dot(A.n) + (b1 - A.a).dot(A.n));
            if (std::abs(off) > o.bridge_max_offset_m)
              continue;
            const double t0 =
                std::min((b0 - A.a).dot(A.u), (b1 - A.a).dot(A.u));
            const double t1 =
                std::max((b0 - A.a).dot(A.u), (b1 - A.a).dot(A.u));
            // B after A along A, or before it.
            double s_mid = 0.0;
            if (std::abs(t0 - A.len) <= o.bridge_max_gap_m && t1 > A.len)
              s_mid = 0.5 * (A.len + t0);
            else if (std::abs(t1) <= o.bridge_max_gap_m && t0 < 0.0)
              s_mid = 0.5 * t1;
            else
              continue;
            const double lo = std::min(-A.half, off - B.half);
            const double hi = std::max(A.half, off + B.half);
            if (!(hi - lo > 1e-6))
              continue;
            const Vec2 m = A.a + s_mid * A.u;
            add(m + lo * A.n, m + hi * A.n);
            ++st.bridges;
          }
      }
      // The outline of the storey's interior closes what the wall graph
      // leaves open (a gap between two offset wall runs, a doorway with no
      // wall either side, a missing exterior wall), so a room there is
      // still a face rather than part of the unbounded one.
      if (o.close_at_interior && g.interior.size() == g.labels.size()) {
        const auto loops = trace_loops(
            cell_edges(g, [&](size_t i) { return g.interior[i] != 0; }));
        for (const auto &loop : loops) {
          const auto pts = simplify_loop(loop, o.interior_simplify_m);
          for (size_t i = 0; i < pts.size(); ++i)
            add(pts[i], pts[(i + 1) % pts.size()]);
        }
      }
      std::sort(keys.begin(), keys.end());
      keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
      std::vector<Segment> segs;
      segs.reserve(keys.size());
      for (const auto &k : keys)
        segs.emplace_back(Point(k[0], k[1]), Point(k[2], k[3]));
      CGAL::insert(arr, segs.begin(), segs.end());

      // Stubs: edges with the same face on both sides (an extension that
      // ends in a room) bound nothing.
      std::vector<Arr::Halfedge_handle> stubs;
      for (auto e = arr.edges_begin(); e != arr.edges_end(); ++e)
        if (e->face() == e->twin()->face())
          stubs.push_back(e);
      for (auto e : stubs)
        arr.remove_edge(e);

      for (auto f = arr.faces_begin(); f != arr.faces_end(); ++f) {
        if (f->is_unbounded())
          continue;
        FaceInfo fi;
        fi.f = f;
        auto [a, len] = ccb_measure(f->outer_ccb());
        for (auto h = f->inner_ccbs_begin(); h != f->inner_ccbs_end(); ++h) {
          auto [ha, hl] = ccb_measure(*h);
          a += ha;
          len += hl;
        }
        fi.area = a;
        fi.perimeter = len;
        fi.sliver = a < std::max(o.small_face_area, 2.0 * cell_area) ||
                    2.0 * a / len < cell;
        face_index.emplace(&*f, faces.size());
        faces.push_back(std::move(fi));
      }
      st.faces = static_cast<int>(faces.size());

      // --- 2. labels: count the room cells in each face --------------------
      std::vector<Point> centres;
      for (int r = 0; r < g.height; ++r)
        for (int c = 0; c < g.width; ++c)
          if (filled[g.index(c, r)] > 0)
            centres.emplace_back((g.col0 + c + 0.5) * cell,
                                 (g.row0 + r + 0.5) * cell);
      using Located =
          std::pair<Point, CGAL::Arr_point_location_result<Arr>::Type>;
      std::vector<Located> located;
      located.reserve(centres.size());
      CGAL::locate(arr, centres.begin(), centres.end(),
                   std::back_inserter(located));
      for (const auto &[p, obj] : located) {
        Arr::Face_const_handle f;
        if (const auto *fh = std::get_if<Arr::Face_const_handle>(&obj))
          f = *fh;
        else if (const auto *he = std::get_if<Arr::Halfedge_const_handle>(&obj))
          f = (*he)->face();
        else
          continue; // on a vertex
        if (f->is_unbounded())
          continue;
        const int c =
            static_cast<int>(std::floor(CGAL::to_double(p.x()) / cell)) -
            g.col0;
        const int r =
            static_cast<int>(std::floor(CGAL::to_double(p.y()) / cell)) -
            g.row0;
        if (c < 0 || r < 0 || c >= g.width || r >= g.height)
          continue;
        ++faces[face_index.at(&*f)].cells[filled[g.index(c, r)]];
      }

      std::map<int32_t, size_t> room_cells;
      for (int32_t l : filled)
        if (l > 0)
          ++room_cells[l];
      for (auto &fi : faces) {
        if (fi.sliver)
          continue;
        size_t total = 0, top = 0, second = 0;
        int32_t top_label = 0;
        for (const auto &[l, n] : fi.cells) {
          total += n;
          if (n > top) {
            second = top;
            top = n;
            top_label = l;
          } else if (n > second) {
            second = n;
          }
        }
        const double area = std::max(fi.area, cell_area);
        const double top_share = static_cast<double>(top) * cell_area / area;
        const double second_share =
            static_cast<double>(second) * cell_area / area;
        const double share = static_cast<double>(total) * cell_area / area;
        if (top_share >= o.pure_min_share) {
          fi.cls = FaceInfo::pure;
          fi.label = top_label;
          for (const auto &[l, n] : fi.cells)
            if (l != top_label &&
                static_cast<double>(n) >=
                    o.guest_min_share * static_cast<double>(room_cells[l]))
              fi.guests.push_back(l);
        } else if (share <= o.empty_max_share) {
          fi.cls = FaceInfo::empty;
        } else {
          fi.cls = FaceInfo::mixed;
        }
        debug("spaces: storey {}: face {:.2f} m^2: rooms cover {:.2f}, room "
              "{} {:.2f}, next {:.2f} -> {} ({} guest(s))",
              g.storey_idx, fi.area, share, top_label, top_share, second_share,
              fi.cls == FaceInfo::pure    ? "pure"
              : fi.cls == FaceInfo::empty ? "empty"
                                          : "mixed",
              fi.guests.size());
      }
      // A sliver takes the class of the neighbour it shares the longest
      // edge with (a gap between two almost collinear faces, a wall end
      // past a corner); one between two rooms is empty.
      for (auto &fi : faces) {
        if (!fi.sliver)
          continue;
        ++st.sliver_faces;
        std::map<size_t, double> shared;
        auto visit = [&](auto c) {
          const auto start = c;
          do {
            const auto t = c->twin()->face();
            if (!t->is_unbounded()) {
              const size_t k = face_index.at(&*t);
              if (!faces[k].sliver)
                shared[k] += (to_vec(c->target()->point()) -
                              to_vec(c->source()->point()))
                                 .norm();
            }
          } while (++c != start);
        };
        visit(fi.f->outer_ccb());
        for (auto h = fi.f->inner_ccbs_begin(); h != fi.f->inner_ccbs_end();
             ++h)
          visit(*h);
        fi.cls = FaceInfo::mixed;
        double best = 0.0;
        std::set<int32_t> rooms_around;
        for (const auto &[k, len] : shared) {
          if (faces[k].cls == FaceInfo::pure)
            rooms_around.insert(faces[k].label);
          if (len > best) {
            best = len;
            fi.cls = faces[k].cls;
            fi.label = faces[k].label;
          }
        }
        // Between two rooms (the rest of a wall gap that the extension
        // closed): part of neither.
        if (rooms_around.size() > 1) {
          fi.cls = FaceInfo::empty;
          fi.label = 0;
        }
      }
      for (const auto &fi : faces) {
        if (fi.sliver)
          continue;
        st.pure_faces += fi.cls == FaceInfo::pure;
        st.empty_faces += fi.cls == FaceInfo::empty;
        st.mixed_faces += fi.cls == FaceInfo::mixed;
      }
    }

    auto info_of = [&](Arr::Face_const_handle f) -> const FaceInfo & {
      return faces[face_index.at(&*f)];
    };
    auto is_guest = [](const FaceInfo &fi, int32_t L) {
      return std::find(fi.guests.begin(), fi.guests.end(), L) !=
             fi.guests.end();
    };
    // Everything a face decides: pure faces and empty ones.
    PolygonSet decided;
    bool any_guest = false;
    if (!faces.empty()) {
      decided = region_of_faces(arr, [&](Arr::Face_const_handle f) {
        return info_of(f).cls != FaceInfo::mixed;
      });
      for (const auto &fi : faces)
        any_guest = any_guest || !fi.guests.empty();
    }
    // The raster of every room (guests keep theirs inside a host's face).
    // A guest's raster grown by guest_grow_m over unlabelled cells, so it
    // reaches the walls its host's face is bounded by (the room cells stop
    // a wall's half-width short of them) instead of leaving the host a
    // ring around it.
    std::set<int32_t> all_guests;
    for (const auto &fi : faces)
      all_guests.insert(fi.guests.begin(), fi.guests.end());
    const int grow = static_cast<int>(std::lround(o.guest_grow_m / cell));
    std::map<int32_t, PolygonSet> raster;
    for (const int32_t L : labels) {
      if (!all_guests.count(L) || grow <= 0) {
        raster.emplace(L, raster_region(g, filled, L));
        continue;
      }
      std::vector<int32_t> grown = filled;
      for (int r = 0; r < g.height; ++r)
        for (int c = 0; c < g.width; ++c) {
          if (filled[g.index(c, r)] != 0)
            continue;
          bool near = false;
          for (int dr = -grow; dr <= grow && !near; ++dr)
            for (int dc = -grow; dc <= grow && !near; ++dc) {
              const int cc = c + dc;
              const int rr = r + dr;
              near = dc * dc + dr * dr <= grow * grow && cc >= 0 && rr >= 0 &&
                     cc < g.width && rr < g.height &&
                     filled[g.index(cc, rr)] == L;
            }
          if (near)
            grown[g.index(c, r)] = L;
        }
      raster.emplace(L, raster_region(g, grown, L));
    }

    // --- 3. polygons per room --------------------------------------------
    double total_area = 0.0;
    double arrangement_area = 0.0;
    for (const int32_t L : labels) {
      PolygonSet region = raster.at(L);
      const int furniture_holes = furniture[L];
      PolygonSet from_faces;
      if (!faces.empty()) {
        // Its raster outside every decided face but those it is a guest in.
        bool guest_somewhere = false;
        for (const auto &fi : faces)
          guest_somewhere = guest_somewhere || is_guest(fi, L);
        region.difference(guest_somewhere
                              ? region_of_faces(arr,
                                                [&](Arr::Face_const_handle f) {
                                                  const auto &fi = info_of(f);
                                                  return fi.cls !=
                                                             FaceInfo::mixed &&
                                                         !is_guest(fi, L);
                                                })
                              : decided);
        // Its own faces, less the cells of their guests.
        from_faces = region_of_faces(arr, [&](Arr::Face_const_handle f) {
          const auto &fi = info_of(f);
          return fi.cls == FaceInfo::pure && fi.label == L;
        });
        if (any_guest) {
          std::set<int32_t> guests;
          for (const auto &fi : faces)
            if (fi.cls == FaceInfo::pure && fi.label == L)
              guests.insert(fi.guests.begin(), fi.guests.end());
          for (const int32_t M : guests)
            from_faces.difference(raster.at(M));
        }
        region.join(from_faces);
      }
      std::vector<PolygonWithHoles> pieces;
      region.polygons_with_holes(std::back_inserter(pieces));
      if (pieces.empty()) {
        ++st.lost_rooms;
        warn("spaces: storey {}: room {} ({:.2f} m^2 of cells) has no polygon "
             "left: faces of other rooms cover all of it",
             g.storey_idx, L, [&] {
               size_t n = 0;
               for (int32_t l : g.labels)
                 n += l == L;
               return static_cast<double>(n) * cell_area;
             }());
        continue;
      }
      size_t best = 0;
      for (size_t i = 1; i < pieces.size(); ++i)
        if (area_of(pieces[i]) > area_of(pieces[best]))
          best = i;
      for (size_t i = 0; i < pieces.size(); ++i)
        if (i != best && area_of(pieces[i]) >= o.min_piece_area)
          warn("spaces: storey {}: room {} has a second piece of {:.2f} m^2 "
               "(kept the largest, {:.2f} m^2)",
               g.storey_idx, L, area_of(pieces[i]), area_of(pieces[best]));
      const PolygonWithHoles piece = clean_piece(pieces[best], o);
      if (piece.outer_boundary().size() < 3) {
        ++st.lost_rooms;
        warn("spaces: storey {}: room {}'s polygon degenerated while "
             "cleaning; no space",
             g.storey_idx, L);
        continue;
      }

      core::SpaceRecord s;
      s.storey = g.storey_idx;
      s.guid = fmt::format("space-{}-{}", g.storey_idx, L);
      s.name = fmt::format("Space {}", L);
      s.room_label = L;
      s.polygon.outer = to_plan_ring(piece.outer_boundary(), false);
      for (auto h = piece.holes_begin(); h != piece.holes_end(); ++h)
        s.polygon.holes.push_back(to_plan_ring(*h, true));
      std::sort(s.polygon.holes.begin(), s.polygon.holes.end());
      double area = plan_ring_area(s.polygon.outer);
      for (const auto &h : s.polygon.holes)
        area -= std::abs(plan_ring_area(h));
      s.area = area;
      s.perimeter = ring_length(s.polygon.outer);
      s.compactness = s.perimeter > 0.0
                          ? 4.0 * kPi * area / (s.perimeter * s.perimeter)
                          : -1.0;
      if (storey) {
        s.z_floor = storey->z_floor;
        s.z_ceiling = storey->z_ceiling;
        if (storey->z_ceiling > storey->z_floor)
          s.volume = area * (storey->z_ceiling - storey->z_floor);
      }
      const double aspect = min_rect_aspect(s.polygon.outer);
      if (aspect > o.corridor_min_aspect)
        s.kind = "corridor";
      else if (area >= o.open_plan_min_area &&
               furniture_holes >= o.open_plan_min_holes)
        s.kind = "open_plan";
      else
        s.kind = "room";
      const double face_area = faces.empty() ? 0.0 : area_of(from_faces);
      const double share =
          area > 0.0 ? std::clamp(face_area / area, 0.0, 1.0) : 0.0;
      s.confidence = share;
      s.metadata = fmt::format(
          R"({{"arrangement_share":{:.4f},"aspect":{:.3f},"furniture_holes":{}}})",
          share, aspect, furniture_holes);
      total_area += area;
      arrangement_area += share * area;

      add_boundaries(s, walls, o, result.boundaries);
      result.spaces.push_back(std::move(s));
      ++st.spaces;
    }
    st.arrangement_share =
        total_area > 0.0 ? arrangement_area / total_area : 0.0;

    info("spaces: storey {}: {} wall(s), {} bridge(s), {:.2f} m^2 beyond {} "
         "external wall(s) ({} room(s)), {} face(s) ({} pure, {} empty, {} "
         "mixed, {} sliver), {} room(s) -> {} space(s), {} lost; {:.0f} % of "
         "the area on wall faces",
         st.storey_idx, st.walls, st.bridges, st.exterior_area,
         st.exterior_walls, st.exterior_rooms, st.faces, st.pure_faces,
         st.empty_faces, st.mixed_faces, st.sliver_faces, st.rooms, st.spaces,
         st.lost_rooms, 100.0 * st.arrangement_share);
    if (o.use_walls && walls.empty())
      warn("spaces: storey {} has no wall (of seen share >= {}); its spaces "
           "are the raster outlines of the rooms",
           st.storey_idx, o.wall_min_seen_share);
    result.stats.push_back(st);
  }
  return result;
}

} // namespace reusex::geometry
