// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/reconstruction/solids.hpp"

#include "reusex/core/logging.hpp"
#include "reusex/geometry/transform_utils.hpp"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/convex_hull_2.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <list>
#include <map>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>

namespace reusex::geometry {
namespace {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Mesh = CGAL::Surface_mesh<K::Point_3>;
using EK = CGAL::Exact_predicates_exact_constructions_kernel;
using EPolygon = CGAL::Polygon_2<EK>;
using EPolygonWithHoles = CGAL::Polygon_with_holes_2<EK>;
using EPolygonSet = CGAL::Polygon_set_2<EK>;
namespace PMP = CGAL::Polygon_mesh_processing;

using Vec2 = Eigen::Vector2d;
using Ring = std::vector<Vec2>;

// ── Constrained triangulation of a polygon with holes ───────────────────────

struct FaceInfo {
  int nesting = -1;
  bool in_domain() const { return nesting % 2 == 1; }
};
using Vb = CGAL::Triangulation_vertex_base_2<K>;
using Fbi = CGAL::Triangulation_face_base_with_info_2<FaceInfo, K>;
using Fb = CGAL::Constrained_triangulation_face_base_2<K, Fbi>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using Cdt =
    CGAL::Constrained_Delaunay_triangulation_2<K, Tds,
                                               CGAL::Exact_predicates_tag>;

/// Flood the faces from @p start with nesting level @p index, stopping at
/// constrained edges, which open the next level (the CGAL polygon example).
void mark_domains(Cdt &cdt, Cdt::Face_handle start, int index,
                  std::list<Cdt::Edge> &border) {
  if (start->info().nesting != -1)
    return;
  std::list<Cdt::Face_handle> queue{start};
  while (!queue.empty()) {
    auto fh = queue.front();
    queue.pop_front();
    if (fh->info().nesting != -1)
      continue;
    fh->info().nesting = index;
    for (int i = 0; i < 3; ++i) {
      const Cdt::Edge e(fh, i);
      auto n = fh->neighbor(i);
      if (n->info().nesting == -1) {
        if (cdt.is_constrained(e))
          border.push_back(e);
        else
          queue.push_back(n);
      }
    }
  }
}

void mark_domains(Cdt &cdt) {
  for (auto f = cdt.all_faces_begin(); f != cdt.all_faces_end(); ++f)
    f->info().nesting = -1;
  std::list<Cdt::Edge> border;
  mark_domains(cdt, cdt.infinite_face(), 0, border);
  while (!border.empty()) {
    const Cdt::Edge e = border.front();
    border.pop_front();
    auto n = e.first->neighbor(e.second);
    if (n->info().nesting == -1)
      mark_domains(cdt, n, e.first->info().nesting + 1, border);
  }
}

double signed_area(const Ring &r) {
  double a = 0.0;
  for (std::size_t i = 0; i < r.size(); ++i) {
    const Vec2 &p = r[i], &q = r[(i + 1) % r.size()];
    a += p.x() * q.y() - q.x() * p.y();
  }
  return 0.5 * a;
}

/// Drop repeated points (and a closing repeat); orient CCW (outer) or CW.
Ring clean_ring(const Ring &in, bool ccw) {
  Ring r;
  for (const auto &p : in) {
    if (!p.allFinite())
      continue;
    if (!r.empty() && (p - r.back()).norm() < 1e-9)
      continue;
    r.push_back(p);
  }
  while (r.size() > 1 && (r.front() - r.back()).norm() < 1e-9)
    r.pop_back();
  if (r.size() >= 3 && ((signed_area(r) > 0.0) != ccw))
    std::reverse(r.begin(), r.end());
  return r;
}

/// A closed prism: @p outer (with @p holes) from z0 up to z1, outward faces.
/// std::nullopt for a degenerate polygon or z1 <= z0.
std::optional<Mesh> extrude(const Ring &outer_in,
                            const std::vector<Ring> &holes_in, double z0,
                            double z1) {
  if (!(z1 > z0) || !std::isfinite(z0) || !std::isfinite(z1))
    return std::nullopt;
  const Ring outer = clean_ring(outer_in, true);
  if (outer.size() < 3 || std::abs(signed_area(outer)) < 1e-9)
    return std::nullopt;
  std::vector<Ring> holes;
  for (const auto &h : holes_in) {
    Ring c = clean_ring(h, false);
    if (c.size() >= 3 && std::abs(signed_area(c)) >= 1e-9)
      holes.push_back(std::move(c));
  }

  Cdt cdt;
  std::vector<std::vector<Cdt::Vertex_handle>> rings;
  const auto insert_ring = [&](const Ring &r) {
    std::vector<Cdt::Vertex_handle> vh;
    for (const auto &p : r)
      vh.push_back(cdt.insert(K::Point_2(p.x(), p.y())));
    for (std::size_t i = 0; i < vh.size(); ++i)
      if (vh[i] != vh[(i + 1) % vh.size()])
        cdt.insert_constraint(vh[i], vh[(i + 1) % vh.size()]);
    rings.push_back(std::move(vh));
  };
  insert_ring(outer);
  for (const auto &h : holes)
    insert_ring(h);
  mark_domains(cdt);

  Mesh m;
  std::map<Cdt::Vertex_handle,
           std::pair<Mesh::Vertex_index, Mesh::Vertex_index>>
      vmap;
  const auto vert = [&](Cdt::Vertex_handle v) {
    auto it = vmap.find(v);
    if (it != vmap.end())
      return it->second;
    const auto &p = v->point();
    auto b = m.add_vertex(K::Point_3(p.x(), p.y(), z0));
    auto t = m.add_vertex(K::Point_3(p.x(), p.y(), z1));
    return vmap.emplace(v, std::make_pair(b, t)).first->second;
  };
  bool ok = true;
  const auto face = [&](Mesh::Vertex_index a, Mesh::Vertex_index b,
                        Mesh::Vertex_index c) {
    if (m.add_face(a, b, c) == Mesh::null_face())
      ok = false;
  };
  for (auto f = cdt.finite_faces_begin(); f != cdt.finite_faces_end(); ++f) {
    if (!f->info().in_domain())
      continue;
    const auto a = vert(f->vertex(0)), b = vert(f->vertex(1)),
               c = vert(f->vertex(2));
    face(a.second, b.second, c.second); // top, +z
    face(a.first, c.first, b.first);    // bottom, -z
  }
  // Sides: an outer ring runs CCW and a hole CW, so (b - a) x up points out of
  // the solid either way.
  for (const auto &vh : rings)
    for (std::size_t i = 0; i < vh.size(); ++i) {
      const auto va = vh[i], vb = vh[(i + 1) % vh.size()];
      if (va == vb)
        continue;
      const auto a = vert(va), b = vert(vb);
      face(a.first, b.first, b.second);
      face(a.first, b.second, a.second);
    }
  if (!ok)
    debug("solids: extrusion: a face could not be added (non-manifold "
          "polygon); the solid is not closed");
  return m;
}

// ── Frames ──────────────────────────────────────────────────────────────────

/// Level -> world of each storey (the inverse of its level_transform).
struct Frames {
  std::map<int, Eigen::Affine3d> to_world;
  const Eigen::Affine3d &of(int storey) const {
    static const Eigen::Affine3d identity = Eigen::Affine3d::Identity();
    auto it = to_world.find(storey);
    return it == to_world.end() ? identity : it->second;
  }
};

Frames make_frames(const core::SolidModel &model) {
  Frames f;
  for (const auto &s : model.storeys) {
    const Eigen::Affine3d T = to_affine(s.level_transform);
    if (!T.matrix().allFinite() || std::abs(T.linear().determinant()) < 1e-12) {
      warn("solids: storey {} has a singular level_transform; its elements "
           "are placed as if in world coordinates",
           s.idx);
      continue;
    }
    f.to_world[s.idx] = T.inverse();
  }
  return f;
}

std::array<double, 16> row_major(const Eigen::Affine3d &a) {
  std::array<double, 16> out{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out[static_cast<std::size_t>(4 * r + c)] = a.matrix()(r, c);
  return out;
}

/// A local frame in level coordinates: origin @p o (x, y, z), x along @p dx
/// (in plan), z up.
Eigen::Affine3d local_frame(const Vec2 &o, double z, const Vec2 &dx) {
  Eigen::Affine3d l = Eigen::Affine3d::Identity();
  l.linear().col(0) = Eigen::Vector3d(dx.x(), dx.y(), 0.0);
  l.linear().col(1) = Eigen::Vector3d(-dx.y(), dx.x(), 0.0);
  l.linear().col(2) = Eigen::Vector3d::UnitZ();
  l.translation() = Eigen::Vector3d(o.x(), o.y(), z);
  return l;
}

std::vector<core::PlanPoint> to_plan(const Ring &r) {
  std::vector<core::PlanPoint> out;
  out.reserve(r.size());
  for (const auto &p : r)
    out.push_back({p.x(), p.y()});
  return out;
}

Ring to_ring(const std::vector<core::PlanPoint> &r) {
  Ring out;
  out.reserve(r.size());
  for (const auto &p : r)
    out.emplace_back(p[0], p[1]);
  return out;
}

/// Close, volume, world-space PCL triangles of @p m (level frame @p T).
void finish(ElementSolid &e, Mesh &m, const Eigen::Affine3d &T) {
  if (!CGAL::is_triangle_mesh(m))
    PMP::triangulate_faces(m);
  m.collect_garbage();
  e.closed = CGAL::is_closed(m) && m.number_of_faces() > 0;
  e.volume = e.closed ? PMP::volume(m) : 0.0;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(m.number_of_vertices());
  for (const auto v : m.vertices()) {
    const auto &p = m.point(v);
    const Eigen::Vector3d w = T * Eigen::Vector3d(p.x(), p.y(), p.z());
    cloud.push_back(pcl::PointXYZ(static_cast<float>(w.x()),
                                  static_cast<float>(w.y()),
                                  static_cast<float>(w.z())));
  }
  pcl::toPCLPointCloud2(cloud, e.mesh.cloud);
  e.mesh.polygons.clear();
  e.mesh.polygons.reserve(m.number_of_faces());
  for (const auto f : m.faces()) {
    pcl::Vertices tri;
    for (const auto v : CGAL::vertices_around_face(m.halfedge(f), m))
      tri.vertices.push_back(static_cast<std::uint32_t>(v.idx()));
    e.mesh.polygons.push_back(std::move(tri));
  }
}

bool inside_ring(const Ring &r, const Vec2 &p) {
  bool in = false;
  for (std::size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
    const Vec2 &a = r[i], &b = r[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) &&
        p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())
      in = !in;
  }
  return in;
}

bool inside_space(const core::SpaceRecord &s, const Vec2 &p) {
  if (s.polygon.outer.size() < 3 || !inside_ring(to_ring(s.polygon.outer), p))
    return false;
  for (const auto &h : s.polygon.holes)
    if (h.size() >= 3 && inside_ring(to_ring(h), p))
      return false;
  return true;
}

// ── Walls ───────────────────────────────────────────────────────────────────

/// One wall's box in its frame: s0..s1 along the axis from (x0, y0), lo..hi
/// across it (left of the axis positive), z0..z1.
struct WallBox {
  const core::WallRecord *wall = nullptr;
  Vec2 p0, d;
  double len = 0.0, s0 = 0.0, s1 = 0.0, lo = 0.0, hi = 0.0, z0 = 0.0, z1 = 0.0;
  bool valid = false;

  Vec2 at(double s, double t) const {
    return p0 + d * s + Vec2(-d.y(), d.x()) * t;
  }
  /// The footprint, counter-clockwise.
  Ring footprint() const {
    return {at(s0, lo), at(s1, lo), at(s1, hi), at(s0, hi)};
  }
};

std::vector<WallBox> wall_boxes(const core::SolidModel &model,
                                const SolidsOptions &opts,
                                std::size_t &skipped) {
  std::vector<WallBox> boxes(model.walls.size());
  std::unordered_map<std::string, std::size_t> by_guid;
  for (std::size_t i = 0; i < model.walls.size(); ++i) {
    const auto &w = model.walls[i];
    WallBox &b = boxes[i];
    b.wall = &w;
    b.p0 = Vec2(w.x0, w.y0);
    const Vec2 p1(w.x1, w.y1);
    b.len = (p1 - b.p0).norm();
    if (!(b.len > 1e-6) || !std::isfinite(b.len) || !(w.height > 0.0) ||
        !std::isfinite(w.z_base) || !std::isfinite(w.height)) {
      ++skipped;
      continue;
    }
    b.d = (p1 - b.p0) / b.len;
    b.s0 = 0.0;
    b.s1 = b.len;
    b.z0 = w.z_base;
    b.z1 = w.z_base + w.height;
    if (w.thickness > 0.0) {
      b.lo = -0.5 * w.thickness;
      b.hi = 0.5 * w.thickness;
    } else {
      // The axis is the seen face: grow away from the space it faces.
      const double t = opts.unknown_wall_thickness_m;
      const Vec2 mid = b.p0 + b.d * (0.5 * b.len);
      const Vec2 n(-b.d.y(), b.d.x());
      const double probe = std::max(0.15, t);
      bool left = false, right = false;
      for (const auto &s : model.spaces) {
        if (s.storey != w.storey)
          continue;
        left = left || inside_space(s, mid + n * probe);
        right = right || inside_space(s, mid - n * probe);
      }
      if (left && !right) {
        b.lo = -t;
        b.hi = 0.0;
      } else if (right && !left) {
        b.lo = 0.0;
        b.hi = t;
      } else {
        b.lo = -0.5 * t;
        b.hi = 0.5 * t;
      }
    }
    b.valid = true;
    by_guid.emplace(w.guid, i);
  }
  if (opts.extend_at_joins) {
    // Reach the far face of the wall met at a corner or a tee.
    const auto reach = [&](const WallBox &other) {
      return std::max(std::abs(other.lo), std::abs(other.hi));
    };
    for (const auto &j : model.wall_joins) {
      if (j.kind != "corner" && j.kind != "tee")
        continue;
      const auto a = by_guid.find(j.wall_a), b = by_guid.find(j.wall_b);
      if (a == by_guid.end() || b == by_guid.end())
        continue;
      WallBox &A = boxes[a->second], &B = boxes[b->second];
      const auto extend = [&](WallBox &w, const std::string &at, double r) {
        if (at == "start")
          w.s0 = std::min(w.s0, -r);
        else if (at == "end")
          w.s1 = std::max(w.s1, w.len + r);
      };
      extend(A, j.at_a, reach(B));
      extend(B, j.at_b, reach(A));
    }
  }
  return boxes;
}

ElementSolid build_wall(const WallBox &b,
                        const std::vector<const core::OpeningRecord *> &ops,
                        const Frames &frames, const SolidsOptions &opts,
                        SolidsResult &result) {
  const core::WallRecord &w = *b.wall;
  const Eigen::Affine3d &T = frames.of(w.storey);
  ElementSolid e;
  e.type = SolidType::wall;
  e.source_guid = w.guid;
  e.storey = w.storey;
  e.predefined = "STANDARD";
  e.profile.outer = {{b.s0, b.lo}, {b.s1, b.lo}, {b.s1, b.hi}, {b.s0, b.hi}};
  e.profile.depth = b.z1 - b.z0;
  e.profile.placement = row_major(T * local_frame(b.p0, b.z0, b.d));

  auto box = extrude(b.footprint(), {}, b.z0, b.z1);
  Mesh m = box ? std::move(*box) : Mesh{};
  const double eps = 1e-3, over = opts.opening_overcut_m;
  for (const auto *o : ops) {
    double a = std::max(o->s_start, b.s0), c = std::min(o->s_end, b.s1);
    double zs = o->z_sill, zh = o->z_head;
    if (c - a < eps || zh <= b.z0 + eps || zs >= b.z1 - eps ||
        !std::isfinite(a + c + zs + zh)) {
      ++result.openings_skipped;
      debug("solids: opening {} lies outside wall {} (s {:.2f}..{:.2f} of "
            "{:.2f}..{:.2f}, z {:.2f}..{:.2f} of {:.2f}..{:.2f}); not cut",
            o->guid, w.guid, o->s_start, o->s_end, b.s0, b.s1, zs, zh, b.z0,
            b.z1);
      continue;
    }
    // Run the cut through every face it touches, so no coplanar sliver
    // remains: across the wall always, past the ends, floor and top when the
    // opening reaches them.
    if (a <= b.s0 + eps)
      a = b.s0 - over;
    if (c >= b.s1 - eps)
      c = b.s1 + over;
    if (zs <= b.z0 + eps)
      zs = b.z0 - over;
    if (zh >= b.z1 - eps)
      zh = b.z1 + over;
    const double lo = b.lo - over, hi = b.hi + over;
    OpeningVoid v;
    v.guid = o->guid;
    v.kind = o->kind;
    v.profile.outer = {
        {0.0, 0.0}, {c - a, 0.0}, {c - a, hi - lo}, {0.0, hi - lo}};
    v.profile.depth = zh - zs;
    v.profile.placement = row_major(T * local_frame(b.at(a, lo), zs, b.d));
    auto cut = extrude({b.at(a, lo), b.at(c, lo), b.at(c, hi), b.at(a, hi)}, {},
                       zs, zh);
    bool done = false;
    if (cut && box) {
      Mesh keep = m;
      try {
        Mesh out;
        done = PMP::corefine_and_compute_difference(m, *cut, out) &&
               CGAL::is_closed(out);
        if (done)
          m = std::move(out);
        else
          m = std::move(keep);
      } catch (const std::exception &ex) {
        debug("solids: corefinement threw for opening {} in wall {}: {}",
              o->guid, w.guid, ex.what());
        m = std::move(keep);
        done = false;
      }
    }
    if (done) {
      ++result.openings_cut;
      e.voids.push_back(std::move(v));
    } else {
      ++result.openings_failed;
      ++e.voids_failed;
      warn("solids: could not subtract opening {} ({}) from wall {}; the wall "
           "is left uncut there",
           o->guid, o->kind, w.guid);
    }
  }
  finish(e, m, T);
  return e;
}

// ── Slabs ───────────────────────────────────────────────────────────────────

/// One piece of a storey's floor polygon.
struct Piece {
  Ring outer;
  std::vector<Ring> holes;
  double area = 0.0;
};

EPolygon epolygon(const Ring &r) {
  EPolygon p;
  for (const auto &q : r)
    p.push_back(EK::Point_2(q.x(), q.y()));
  return p;
}

Ring dring(const EPolygon &p) {
  Ring r;
  for (auto v = p.vertices_begin(); v != p.vertices_end(); ++v)
    r.emplace_back(CGAL::to_double(v->x()), CGAL::to_double(v->y()));
  return r;
}

/// The union of @p rings (each simple), split into pieces, largest first.
std::vector<Piece> union_pieces(const std::vector<Ring> &rings,
                                const SolidsOptions &opts,
                                std::size_t &not_simple) {
  EPolygonSet set;
  for (const auto &r0 : rings) {
    const Ring r = clean_ring(r0, true);
    if (r.size() < 3)
      continue;
    EPolygon p = epolygon(r);
    if (!p.is_simple()) {
      ++not_simple;
      continue;
    }
    if (p.orientation() == CGAL::CLOCKWISE)
      p.reverse_orientation();
    set.join(p);
  }
  std::vector<EPolygonWithHoles> pwhs;
  set.polygons_with_holes(std::back_inserter(pwhs));
  std::vector<Piece> out;
  for (const auto &pwh : pwhs) {
    if (pwh.is_unbounded())
      continue;
    Piece piece;
    piece.outer = dring(pwh.outer_boundary());
    piece.area = std::abs(signed_area(piece.outer));
    for (auto h = pwh.holes_begin(); h != pwh.holes_end(); ++h) {
      Ring hr = dring(*h);
      const double ha = std::abs(signed_area(hr));
      if (ha < opts.slab_min_hole_area_m2)
        continue;
      piece.area -= ha;
      piece.holes.push_back(std::move(hr));
    }
    if (piece.area >= opts.slab_min_area_m2)
      out.push_back(std::move(piece));
  }
  std::stable_sort(out.begin(), out.end(), [](const Piece &a, const Piece &b) {
    return a.area > b.area;
  });
  return out;
}

std::vector<Piece> hull_piece(const std::vector<Ring> &rings) {
  std::vector<K::Point_2> pts, hull;
  for (const auto &r : rings)
    for (const auto &p : r)
      pts.emplace_back(p.x(), p.y());
  if (pts.size() < 3)
    return {};
  CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
  Piece piece;
  for (const auto &p : hull)
    piece.outer.emplace_back(p.x(), p.y());
  piece.area = std::abs(signed_area(piece.outer));
  if (piece.outer.size() < 3 || piece.area < 1e-9)
    return {};
  return {piece};
}

ElementSolid build_slab(const core::StoreyRecord &st, const Piece &piece,
                        int part, double z_bottom, double thickness,
                        const char *predefined, const Frames &frames) {
  const Eigen::Affine3d &T = frames.of(st.idx);
  ElementSolid e;
  e.type = SolidType::slab;
  e.source_guid = st.guid;
  e.storey = st.idx;
  e.predefined = predefined;
  e.part = part;
  e.profile.outer = to_plan(clean_ring(piece.outer, true));
  for (const auto &h : piece.holes)
    e.profile.holes.push_back(to_plan(clean_ring(h, false)));
  e.profile.depth = thickness;
  e.profile.placement =
      row_major(T * local_frame(Vec2::Zero(), z_bottom, Vec2::UnitX()));
  auto m = extrude(piece.outer, piece.holes, z_bottom, z_bottom + thickness);
  Mesh mesh = m ? std::move(*m) : Mesh{};
  finish(e, mesh, T);
  return e;
}

Vec2 centroid(const Ring &r) {
  Vec2 c = Vec2::Zero();
  for (const auto &p : r)
    c += p;
  return r.empty() ? c : Vec2(c / static_cast<double>(r.size()));
}

} // namespace

std::string_view to_string(SolidType type) {
  switch (type) {
  case SolidType::wall:
    return "wall";
  case SolidType::slab:
    return "slab";
  case SolidType::space:
    return "space";
  }
  return "unknown";
}

SolidsResult build_solids(const core::SolidModel &model,
                          const SolidsOptions &opts) {
  SolidsResult result;
  const Frames frames = make_frames(model);

  // Walls, with their openings cut out.
  const auto boxes = wall_boxes(model, opts, result.walls_skipped);
  std::unordered_map<std::string, std::vector<const core::OpeningRecord *>>
      openings;
  std::unordered_map<std::string, bool> valid_wall;
  for (const auto &b : boxes)
    valid_wall[b.wall->guid] = valid_wall[b.wall->guid] || b.valid;
  for (const auto &o : model.openings) {
    if (const auto it = valid_wall.find(o.wall);
        it == valid_wall.end() || !it->second) {
      ++result.openings_skipped;
      debug("solids: opening {} names wall {}, which the model does not hold "
            "(or which has no solid)",
            o.guid, o.wall);
      continue;
    }
    openings[o.wall].push_back(&o);
  }
  for (const auto &b : boxes) {
    if (!b.valid)
      continue;
    static const std::vector<const core::OpeningRecord *> none;
    const auto it = openings.find(b.wall->guid);
    result.walls.push_back(build_wall(
        b, it == openings.end() ? none : it->second, frames, opts, result));
  }

  // Spaces.
  for (const auto &s : model.spaces) {
    const Eigen::Affine3d &T = frames.of(s.storey);
    std::vector<Ring> holes;
    for (const auto &h : s.polygon.holes)
      holes.push_back(to_ring(h));
    auto m = extrude(to_ring(s.polygon.outer), holes, s.z_floor, s.z_ceiling);
    if (!m) {
      ++result.spaces_skipped;
      continue;
    }
    ElementSolid e;
    e.type = SolidType::space;
    e.source_guid = s.guid;
    e.storey = s.storey;
    e.predefined = s.kind.empty() ? "unknown" : s.kind;
    e.profile.outer = to_plan(clean_ring(to_ring(s.polygon.outer), true));
    for (const auto &h : holes)
      e.profile.holes.push_back(to_plan(clean_ring(h, false)));
    e.profile.depth = s.z_ceiling - s.z_floor;
    e.profile.placement =
        row_major(T * local_frame(Vec2::Zero(), s.z_floor, Vec2::UnitX()));
    finish(e, *m, T);
    result.spaces.push_back(std::move(e));
  }

  // Slabs: one floor slab per storey (its floor polygon, down to the storey
  // below), and a roof over the top one.
  std::vector<const core::StoreyRecord *> storeys;
  for (const auto &s : model.storeys)
    storeys.push_back(&s);
  std::stable_sort(
      storeys.begin(), storeys.end(),
      [](const auto *a, const auto *b) { return a->idx < b->idx; });
  std::size_t not_simple = 0;
  std::vector<Piece> top_pieces;
  for (std::size_t k = 0; k < storeys.size(); ++k) {
    const auto &st = *storeys[k];
    std::vector<Ring> space_rings, wall_rings;
    for (const auto &s : model.spaces)
      if (s.storey == st.idx && s.polygon.outer.size() >= 3)
        space_rings.push_back(to_ring(s.polygon.outer));
    for (const auto &b : boxes)
      if (b.valid && b.wall->storey == st.idx)
        wall_rings.push_back(b.footprint());
    std::vector<Piece> pieces;
    if (!space_rings.empty()) {
      std::vector<Ring> all = space_rings;
      all.insert(all.end(), wall_rings.begin(), wall_rings.end());
      pieces = union_pieces(all, opts, not_simple);
    } else {
      pieces = hull_piece(wall_rings);
    }
    if (pieces.empty()) {
      warn("solids: storey {} has no space and no wall to draw a floor from; "
           "no slab",
           st.idx);
      continue;
    }
    // The gap to the storey below, in world z at the floor's centre.
    double thickness = opts.slab_thickness_m;
    if (k > 0) {
      const auto &below = *storeys[k - 1];
      const Vec2 c = centroid(pieces.front().outer);
      const double z_floor =
          (frames.of(st.idx) * Eigen::Vector3d(c.x(), c.y(), st.z_floor)).z();
      const double z_ceiling = (frames.of(below.idx) *
                                Eigen::Vector3d(c.x(), c.y(), below.z_ceiling))
                                   .z();
      const double gap = z_floor - z_ceiling;
      if (gap > 1e-3 && gap <= opts.max_slab_thickness_m)
        thickness = gap;
      else
        info("solids: storey {}: the gap to storey {} is {:.2f} m (not in "
             "(0, {:.2f}]); its floor slab gets the default {:.2f} m",
             st.idx, below.idx, gap, opts.max_slab_thickness_m,
             opts.slab_thickness_m);
    }
    for (std::size_t i = 0; i < pieces.size(); ++i)
      result.slabs.push_back(build_slab(st, pieces[i], static_cast<int>(i),
                                        st.z_floor - thickness, thickness,
                                        "FLOOR", frames));
    if (k + 1 == storeys.size())
      top_pieces = std::move(pieces);
  }
  if (opts.roof && !storeys.empty() && !top_pieces.empty()) {
    const auto &top = *storeys.back();
    for (std::size_t i = 0; i < top_pieces.size(); ++i)
      result.slabs.push_back(build_slab(top, top_pieces[i], static_cast<int>(i),
                                        top.z_ceiling, opts.slab_thickness_m,
                                        "ROOF", frames));
  }
  if (not_simple > 0)
    warn("solids: {} space / wall outline(s) are not simple polygons and were "
         "left out of the slabs",
         not_simple);

  for (const auto *v : {&result.walls, &result.slabs, &result.spaces})
    for (const auto &e : *v)
      if (!e.closed)
        ++result.not_closed;
  return result;
}

} // namespace reusex::geometry
