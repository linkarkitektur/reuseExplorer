// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "reconstruction/Solidifier.hpp"
#include "utils/tolerances.hpp"

/*
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/circulator.h>
#include <CGAL/polygon_mesh_processing.h>
*/
#include <range/v3/view/enumerate.hpp>

#include <algorithm>
#include <array>
#include <numeric>

// using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
// using Kernel = CGAL::Simple_cartesian<double>;
// using Point = Kernel::Point_3;
// using Mesh = CGAL::Surface_mesh<Point>;
//
// namespace PMP = CGAL::Polygon_mesh_processing;
// Forward declaration of internal helper
namespace reusex::geometry {
std::pair<Eigen::MatrixXd, Eigen::MatrixXi>
toMesh_impl(const std::shared_ptr<const CellComplex> &cc,
            std::function<bool(CellComplex::Vertex)> filter);
}

namespace reusex::geometry {

namespace {

/// Triangles over a convex planar polygon, as corner indices in its winding.
///
/// A fan from corner 0, as before, unless a fan triangle would be degenerate.
/// That happens when the polygon has collinear corners: a wall face of a
/// storey-local cell complex (reusex-sba) carries the vertices where a finer
/// section above or below meets it, so the mesh has no T-junctions there.
/// Such a polygon is ear-clipped instead, only ever at a true corner and
/// never down to a sliver, so every triangle has area.
std::vector<std::array<size_t, 3>>
triangulate_convex(const std::vector<Eigen::Vector3d> &p) {
  std::vector<std::array<size_t, 3>> out;
  const size_t n = p.size();
  if (n < 3)
    return out;
  double extent = 0.0;
  for (const auto &q : p)
    extent = std::max(extent, (q - p[0]).norm());
  const double eps = 1e-9 * extent * extent; // on twice the triangle area
  auto twice_area = [&](size_t a, size_t b, size_t c) {
    return (p[b] - p[a]).cross(p[c] - p[a]).norm();
  };

  bool fan_ok = true;
  for (size_t i = 1; i + 1 < n && fan_ok; ++i)
    fan_ok = twice_area(0, i, i + 1) > eps;
  if (fan_ok) {
    for (size_t i = 1; i + 1 < n; ++i)
      out.push_back({0, i, i + 1});
    return out;
  }

  std::vector<size_t> ring(n);
  std::iota(ring.begin(), ring.end(), size_t{0});
  auto ring_area = [&](const std::vector<size_t> &r) {
    Eigen::Vector3d s = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < r.size(); ++i)
      s += p[r[i]].cross(p[r[(i + 1) % r.size()]]);
    return s.norm();
  };
  while (ring.size() > 3) {
    const double whole = ring_area(ring);
    bool clipped = false;
    for (size_t i = 0; i < ring.size() && !clipped; ++i) {
      const size_t a = ring[(i + ring.size() - 1) % ring.size()];
      const size_t b = ring[i];
      const size_t c = ring[(i + 1) % ring.size()];
      const double ear = twice_area(a, b, c);
      if (ear <= eps || whole - ear <= eps)
        continue; // not a corner, or clipping it leaves a sliver
      out.push_back({a, b, c});
      ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(i));
      clipped = true;
    }
    if (!clipped)
      return out; // the rest is degenerate: nothing with area left
  }
  if (twice_area(ring[0], ring[1], ring[2]) > eps)
    out.push_back({ring[0], ring[1], ring[2]});
  return out;
}

} // namespace

std::pair<Eigen::MatrixXd, Eigen::MatrixXi>
Solidifier::toMesh(std::function<bool(const Cd)> filter) {
  // Delegate to helper function
  return toMesh_impl(get_cell_complex(), filter);
}

// Internal implementation (has access to CellComplex)
std::pair<Eigen::MatrixXd, Eigen::MatrixXi>
toMesh_impl(const std::shared_ptr<const CellComplex> &_cc,
            std::function<bool(CellComplex::Vertex)> filter) {

  /*
 Mesh mesh;

 // INFO: Create vertices
 std::map<size_t, Mesh::Vertex_index> vertex_map;
 for (auto vit = _cc->vertices_begin(); vit != _cc->vertices_end(); ++vit) {
   auto &v = (*_cc)[*vit];
   vertex_map[v.id] = mesh.add_vertex(Point(v.pos[0], v.pos[1], v.pos[2]));
 }

 for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {

   if (!filter(*cit))
     continue; // Skip cells that do not pass the filter

   for (auto fit = _cc->faces_begin(*cit); fit != _cc->faces_end(*cit);
        ++fit) {

     std::vector<Mesh::Vertex_index> verts{};
     for (auto vit = _cc->vertices_begin(*fit); vit != _cc->vertices_end(*fit);
          ++vit)
       verts.push_back(vertex_map[(*_cc)[*vit].id]);

     mesh.add_face(verts);
   }
 }

 // INFO: Merge volumes using boolean union
 reusex::trace("Corefining and computing union");

 // mesh.collect_garbage();
 PMP::triangulate_faces(mesh);

 // if (!PMP::corefine_and_compute_union(mesh, mesh, mesh))
 //   reusex::error("Boolean union failed");

 // PMP::triangulate_faces(mesh);
 // PMP::remove_isolated_vertices(mesh);

 reusex::debug("Mesh is triangulated: {}",
               CGAL::is_triangle_mesh(mesh) ? "yes" : "no");
 */

  // INFO: New section
  std::map<int, CellComplex::Vertex> vertex_indices{};
  std::vector<Eigen::Vector3i> faces{};

  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    if (!filter(*cit))
      continue; // Skip cells that do not pass the filter

    auto cell = *cit;
    for (auto fit = _cc->faces_of_cell_begin(*cit);
         fit != _cc->faces_of_cell_end(*cit); ++fit) {

      std::vector<CellComplex::Vertex> verts{};

      std::vector<CellComplex::Vertex> neighbor_cells{};
      neighbor_cells.reserve(2);

      auto [start, end] = boost::adjacent_vertices(*fit, *_cc);
      for (auto vit = start; vit != end; ++vit) {
        // Get neighboring cells
        if ((*_cc)[*vit].type == CellNodeType::cell)
          neighbor_cells.push_back(*vit);
        // Get face vertices
        else if ((*_cc)[*vit].type == CellNodeType::vertex)
          verts.push_back(*vit);
      }

      if (neighbor_cells.size() > 2)
        reusex::warn("Face {} has more than two adjacent cells",
                     (*_cc)[*fit].id);

      if (neighbor_cells.size() == 2) {
        const auto a = neighbor_cells[0];
        const auto b = neighbor_cells[1];

        if (filter(a) && filter(b))
          continue; // Skip faces between two valid cells
      }

      // Orient the face so its normal points OUTWARD from `cell`: compute a
      // Newell-style normal as the sum of edge cross products around the
      // face centroid, then flip the winding if it points toward the cell
      // interior. Degenerate (near-zero) normals fall through unflipped.
      bool flip = false;
      if (neighbor_cells.size() >= 1) {

        Eigen::Vector3d face_normal = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < verts.size(); ++i) {
          const auto v0 = (*_cc)[verts[i]].pos;
          const auto v1 = (*_cc)[verts[(i + 1) % verts.size()]].pos;
          face_normal += (v0 - (*_cc)[*fit].pos)
                             .cross(v1 - (*_cc)[*fit].pos)
                             .template head<3>();
        }
        const double normal_norm = face_normal.norm();
        if (normal_norm < reusex::kEpsilonLengthM) {
          reusex::warn("Face {} has degenerate normal (norm={:g}); leaving "
                       "winding unchanged",
                       (*_cc)[*fit].id, normal_norm);
        } else {
          face_normal /= normal_norm;
          Eigen::Vector3d cell_dir =
              ((*_cc)[cell].pos - (*_cc)[*fit].pos).template head<3>();
          const double cell_norm = cell_dir.norm();
          if (cell_norm > reusex::kEpsilonLengthM) {
            cell_dir /= cell_norm;
            if (face_normal.dot(cell_dir) > 0)
              flip = true;
          }
        }
      }

      // INFO: Create face
      if (verts.size() < 3) {
        reusex::error("Face {} has less than 3 vertices", (*_cc)[*fit].id);
        continue;
      }

      for (auto v : verts)
        vertex_indices[(*_cc)[v].id] = v;

      if (flip)
        std::reverse(verts.begin(), verts.end());

      std::vector<Eigen::Vector3d> corners;
      corners.reserve(verts.size());
      for (auto v : verts)
        corners.push_back((*_cc)[v].pos.template head<3>());
      for (const auto &[a, b, c] : triangulate_convex(corners))
        faces.emplace_back((*_cc)[verts[a]].id, (*_cc)[verts[b]].id,
                           (*_cc)[verts[c]].id);

    } // end for faces
  } // end for cells

  // INFO: Renumber mesh
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  const size_t nV = vertex_indices.size();
  const size_t nF = faces.size();

  reusex::trace("Converted mesh has {} vertices and {} faces", nV, nF);
  V.resize(nV, 3);
  F.resize(nF, 3);

  // Map vertex indices
  reusex::trace("Mapping {} vertices", nV);
  std::unordered_map<int, int> vmap;
  vmap.reserve(nV);

  int id = 0;
  for (const auto &[old_id, v] : vertex_indices) {
    V.row(id) << (*_cc)[v].pos.template head<3>().transpose();
    vmap[old_id] = id++;
  }

  // Convert faces
  reusex::trace("Mapping {} faces", nF);
  for (size_t i = 0; i < faces.size(); ++i)
    F.row(i) = faces[i];
  F = F.unaryExpr([&](int x) { return vmap[x]; });

  reusex::trace("Converted mesh to Eigen matrices");

  return {V, F};
}

} // namespace reusex::geometry
