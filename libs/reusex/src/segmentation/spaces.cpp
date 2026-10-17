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

double signed_area(const std::vector<Point> &ring) {
  double a = 0.0;
  for (size_t i = 0; i < ring.size(); ++i) {
    const Vec2 p = to_vec(ring[i]);
    const Vec2 q = to_vec(ring[(i + 1) % ring.size()]);
    a += p.x() * q.y() - q.x() * p.y();
  }
  return 0.5 * a;
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

/// The region bounded by a set of directed edges with the region on their
/// left (every vertex as many edges in as out). Edges are chained into
/// closed walks, split into simple loops, and the loops' interiors combined
/// by symmetric difference: for loops that never cross, that is the region
/// whatever the chaining at a pinch vertex. @p loop_areas, when set,
/// receives each loop's signed area (> 0: counter-clockwise, an outer
/// boundary; < 0: a hole).
PolygonSet region_from_edges(const std::vector<std::pair<Point, Point>> &edges,
                             std::vector<double> *loop_areas = nullptr) {
  std::map<Point, std::vector<size_t>> out;
  for (size_t i = 0; i < edges.size(); ++i)
    out[edges[i].first].push_back(i);
  std::vector<uint8_t> used(edges.size(), 0);
  PolygonSet set;
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
      if (loop.size() < 3)
        continue;
      Polygon poly(loop.begin(), loop.end());
      const auto orient = poly.orientation();
      if (orient == CGAL::COLLINEAR)
        continue;
      if (loop_areas)
        loop_areas->push_back(signed_area(loop));
      if (orient == CGAL::CLOCKWISE)
        poly.reverse_orientation();
      set.symmetric_difference(poly);
    }
  }
  return set;
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
                         int32_t label, std::vector<double> *loop_areas) {
  std::vector<std::pair<Point, Point>> edges;
  const double cell = g.cell_size;
  auto corner = [&](int c, int r) {
    return Point((g.col0 + c) * cell, (g.row0 + r) * cell);
  };
  auto is = [&](int c, int r) {
    return c >= 0 && r >= 0 && c < g.width && r < g.height &&
           labels[g.index(c, r)] == label;
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
  return region_from_edges(edges, loop_areas);
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
    const std::vector<int32_t> filled =
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
        if (top_share >= o.pure_min_share &&
            second_share <= o.other_max_share) {
          fi.cls = FaceInfo::pure;
          fi.label = top_label;
        } else if (share <= o.empty_max_share) {
          fi.cls = FaceInfo::empty;
        } else {
          fi.cls = FaceInfo::mixed;
        }
        debug("spaces: storey {}: face {:.2f} m^2: rooms cover {:.2f}, room "
              "{} {:.2f}, next {:.2f} -> {}",
              g.storey_idx, fi.area, share, top_label, top_share, second_share,
              fi.cls == FaceInfo::pure    ? "pure"
              : fi.cls == FaceInfo::empty ? "empty"
                                          : "mixed");
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
    // Everything a face decides: pure faces and empty ones.
    PolygonSet decided;
    if (!faces.empty())
      decided = region_of_faces(arr, [&](Arr::Face_const_handle f) {
        return info_of(f).cls != FaceInfo::mixed;
      });

    // --- 3. polygons per room --------------------------------------------
    double total_area = 0.0;
    double arrangement_area = 0.0;
    for (const int32_t L : labels) {
      PolygonSet region = raster_region(g, filled, L, nullptr);
      const int furniture_holes = furniture[L];
      PolygonSet from_faces;
      if (!faces.empty()) {
        region.difference(decided);
        from_faces = region_of_faces(arr, [&](Arr::Face_const_handle f) {
          const auto &fi = info_of(f);
          return fi.cls == FaceInfo::pure && fi.label == L;
        });
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

    info("spaces: storey {}: {} wall(s), {} face(s) ({} pure, {} empty, {} "
         "mixed, {} sliver), {} room(s) -> {} space(s), {} lost; {:.0f} % of "
         "the area on wall faces",
         st.storey_idx, st.walls, st.faces, st.pure_faces, st.empty_faces,
         st.mixed_faces, st.sliver_faces, st.rooms, st.spaces, st.lost_rooms,
         100.0 * st.arrangement_share);
    if (o.use_walls && walls.empty())
      warn("spaces: storey {} has no wall (of seen share >= {}); its spaces "
           "are the raster outlines of the rooms",
           st.storey_idx, o.wall_min_seen_share);
    result.stats.push_back(st);
  }
  return result;
}

} // namespace reusex::geometry
