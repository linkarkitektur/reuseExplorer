// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/geometry/point_mesh_distance.hpp"

#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fmt/format.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace reusex::geometry {

std::vector<double> point_to_mesh_distances(const Cloud &cloud,
                                            const pcl::PolygonMesh &mesh) {
  using K = CGAL::Exact_predicates_inexact_constructions_kernel;
  using Triangle = K::Triangle_3;
  using Iterator = std::vector<Triangle>::const_iterator;
  using Primitive = CGAL::AABB_triangle_primitive_3<K, Iterator>;
  using Tree = CGAL::AABB_tree<CGAL::AABB_traits_3<K, Primitive>>;

  pcl::PointCloud<pcl::PointXYZ> verts;
  if (!mesh.cloud.data.empty())
    pcl::fromPCLPointCloud2(mesh.cloud, verts);

  std::vector<Triangle> tris;
  for (const auto &poly : mesh.polygons) {
    const auto &v = poly.vertices;
    if (v.size() < 3)
      continue;
    auto pt = [&](std::uint32_t idx) {
      if (idx >= verts.size())
        throw std::invalid_argument(
            fmt::format("point_to_mesh_distances: mesh face references vertex "
                        "{} of {}",
                        idx, verts.size()));
      const auto &p = verts[idx];
      return K::Point_3(p.x, p.y, p.z);
    };
    for (std::size_t k = 1; k + 1 < v.size(); ++k) {
      Triangle t(pt(v[0]), pt(v[k]), pt(v[k + 1]));
      if (!t.is_degenerate())
        tris.push_back(t);
    }
  }
  if (tris.empty())
    throw std::invalid_argument(
        fmt::format("point_to_mesh_distances: mesh has no usable triangle ({} "
                    "faces, {} vertices)",
                    mesh.polygons.size(), verts.size()));

  Tree tree(tris.cbegin(), tris.cend());
  tree.accelerate_distance_queries();

  std::vector<double> out(cloud.size(),
                          std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const auto &p = cloud[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    out[i] = std::sqrt(
        CGAL::to_double(tree.squared_distance(K::Point_3(p.x, p.y, p.z))));
  }
  return out;
}

namespace {

/// Usable triangles of @p mesh as vertex triples (polygons fanned).
std::vector<std::array<Eigen::Vector3d, 3>>
mesh_triangles(const pcl::PolygonMesh &mesh, const char *who) {
  pcl::PointCloud<pcl::PointXYZ> verts;
  if (!mesh.cloud.data.empty())
    pcl::fromPCLPointCloud2(mesh.cloud, verts);
  std::vector<std::array<Eigen::Vector3d, 3>> tris;
  for (const auto &poly : mesh.polygons) {
    const auto &v = poly.vertices;
    if (v.size() < 3)
      continue;
    auto pt = [&](std::uint32_t idx) {
      if (idx >= verts.size())
        throw std::invalid_argument(
            fmt::format("{}: mesh face references vertex {} of {}", who, idx,
                        verts.size()));
      const auto &p = verts[idx];
      return Eigen::Vector3d(p.x, p.y, p.z);
    };
    for (std::size_t k = 1; k + 1 < v.size(); ++k) {
      const std::array<Eigen::Vector3d, 3> t{pt(v[0]), pt(v[k]), pt(v[k + 1])};
      if ((t[1] - t[0]).cross(t[2] - t[0]).norm() > 1e-12)
        tris.push_back(t);
    }
  }
  if (tris.empty())
    throw std::invalid_argument(
        fmt::format("{}: mesh has no usable triangle ({} faces, {} vertices)",
                    who, mesh.polygons.size(), verts.size()));
  return tris;
}

/// Uniform hash grid over a cloud's finite points; cell size = the query
/// radius, so a radius query visits the 27 cells around the query point.
class PointGrid {
    public:
  PointGrid(const Cloud &cloud, double cell) : cell_(cell) {
    for (const auto &p : cloud) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      cells_[key(cell_of(p.x), cell_of(p.y), cell_of(p.z))].push_back(
          Eigen::Vector3d(p.x, p.y, p.z));
      ++points_;
    }
  }
  [[nodiscard]] std::size_t points() const { return points_; }
  [[nodiscard]] bool any_within(const Eigen::Vector3d &q, double r) const {
    const double r2 = r * r;
    const auto cx = cell_of(q.x()), cy = cell_of(q.y()), cz = cell_of(q.z());
    for (std::int64_t dx = -1; dx <= 1; ++dx)
      for (std::int64_t dy = -1; dy <= 1; ++dy)
        for (std::int64_t dz = -1; dz <= 1; ++dz) {
          const auto it = cells_.find(key(cx + dx, cy + dy, cz + dz));
          if (it == cells_.end())
            continue;
          for (const auto &p : it->second)
            if ((p - q).squaredNorm() <= r2)
              return true;
        }
    return false;
  }

    private:
  [[nodiscard]] std::int64_t cell_of(double v) const {
    return static_cast<std::int64_t>(std::floor(v / cell_));
  }
  static std::uint64_t key(std::int64_t x, std::int64_t y, std::int64_t z) {
    // 21 bits per axis (+-1e6 cells): far beyond any building at cm cells.
    const auto m = [](std::int64_t v) {
      return static_cast<std::uint64_t>(v + (1LL << 20)) & ((1ULL << 21) - 1);
    };
    return (m(x) << 42) | (m(y) << 21) | m(z);
  }
  double cell_;
  std::size_t points_ = 0;
  std::unordered_map<std::uint64_t, std::vector<Eigen::Vector3d>> cells_;
};

} // namespace

MeshSupport mesh_support(const Cloud &cloud, const pcl::PolygonMesh &mesh,
                         double support_dist, double spacing) {
  if (!(std::isfinite(support_dist) && support_dist > 0.0))
    throw std::invalid_argument(fmt::format(
        "mesh_support: support distance must be > 0 (got {})", support_dist));
  if (!(std::isfinite(spacing) && spacing > 0.0))
    throw std::invalid_argument(fmt::format(
        "mesh_support: sample spacing must be > 0 (got {})", spacing));
  const auto tris = mesh_triangles(mesh, "mesh_support");
  const PointGrid grid(cloud, support_dist);
  if (grid.points() == 0)
    throw std::invalid_argument(fmt::format(
        "mesh_support: cloud has no finite point ({} points)", cloud.size()));

  // R2 low-discrepancy sequence (Roberts 2018): even coverage of the unit
  // square for any sample count, folded onto the triangle.
  constexpr double g = 1.32471795724474602596; // plastic number
  constexpr double a1 = 1.0 / g, a2 = 1.0 / (g * g);
  MeshSupport s;
  for (const auto &t : tris) {
    const Eigen::Vector3d e1 = t[1] - t[0], e2 = t[2] - t[0];
    const double area = 0.5 * e1.cross(e2).norm();
    const auto n = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(area / (spacing * spacing))));
    const double w = area / static_cast<double>(n);
    std::size_t unsupported = 0;
    for (std::size_t i = 0; i < n; ++i) {
      double u = std::fmod(0.5 + a1 * static_cast<double>(i + 1), 1.0);
      double v = std::fmod(0.5 + a2 * static_cast<double>(i + 1), 1.0);
      if (u + v > 1.0) {
        u = 1.0 - u;
        v = 1.0 - v;
      }
      if (!grid.any_within(t[0] + u * e1 + v * e2, support_dist))
        ++unsupported;
    }
    s.area_m2 += area;
    s.unsupported_area_m2 += w * static_cast<double>(unsupported);
    s.samples += n;
  }
  return s;
}

DistanceSummary summarize_distances(const std::vector<double> &distances,
                                    double within, double beyond) {
  DistanceSummary s;
  std::vector<double> d;
  d.reserve(distances.size());
  for (const double v : distances)
    if (std::isfinite(v))
      d.push_back(v);
  if (d.empty())
    return s;
  std::sort(d.begin(), d.end());
  const std::size_t n = d.size();
  // Nearest rank: the smallest value with at least q * n values at or below.
  const auto rank = [&](double q) {
    const auto k =
        static_cast<std::size_t>(std::ceil(q * static_cast<double>(n)));
    return d[std::clamp<std::size_t>(k, 1, n) - 1];
  };
  double sum = 0.0;
  std::size_t in = 0, out = 0;
  for (const double v : d) {
    sum += v;
    in += v <= within ? 1 : 0;
    out += v > beyond ? 1 : 0;
  }
  s.points = n;
  s.mean_m = sum / static_cast<double>(n);
  s.median_m = rank(0.5);
  s.p95_m = rank(0.95);
  s.max_m = d.back();
  s.within_share = static_cast<double>(in) / static_cast<double>(n);
  s.beyond_share = static_cast<double>(out) / static_cast<double>(n);
  return s;
}

} // namespace reusex::geometry
