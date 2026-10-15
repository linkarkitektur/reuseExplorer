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

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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
