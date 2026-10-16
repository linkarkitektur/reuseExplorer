// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reconstruction/mesh.hpp"
#include "core/logging.hpp"
#include "core/processing_observer.hpp"
#include "core/visual_observer.hpp"
#include "reconstruction/CellComplex.hpp"
#include "reconstruction/Solidifier.hpp"
#include "reconstruction/regularization.hpp"
#include "reconstruction/structural_planes.hpp"
// #include "geometry/utils.hpp"
// #include "io/reusex.hpp"

// #include <algorithm>
// #include <filesystem>
// #include <latch>
// #include <mutex>
// #include <thread>

#include <fmt/color.h>
#include <fmt/ranges.h>
#include <fmt/std.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace reusex::geometry {

namespace {

/** \brief Convert Eigen matrices to PCL PolygonMesh.
 *
 * \param vertices Nx3 matrix of vertex coordinates (x, y, z)
 * \param faces Mx3 matrix of triangle face indices
 * \return PCL polygon mesh with vertices and faces populated
 */
pcl::PolygonMeshPtr eigen_to_polygon_mesh(const Eigen::MatrixXd &vertices,
                                          const Eigen::MatrixXi &faces) {
  auto result = pcl::PolygonMeshPtr(new pcl::PolygonMesh());

  // Convert vertices: Eigen::MatrixXd → CloudLoc → PCLPointCloud2
  CloudLocPtr mesh_vertices(new CloudLoc);
  mesh_vertices->points.resize(vertices.rows());
  mesh_vertices->width = static_cast<uint32_t>(vertices.rows());
  mesh_vertices->height = 1;

  for (int i = 0; i < vertices.rows(); ++i) {
    mesh_vertices->points[i].x = static_cast<float>(vertices(i, 0));
    mesh_vertices->points[i].y = static_cast<float>(vertices(i, 1));
    mesh_vertices->points[i].z = static_cast<float>(vertices(i, 2));
  }
  pcl::toPCLPointCloud2(*mesh_vertices, result->cloud);

  // Convert faces: Eigen::MatrixXi → pcl::Vertices
  result->polygons.resize(faces.rows());
  for (int i = 0; i < faces.rows(); ++i) {
    pcl::Vertices polygon;
    polygon.vertices.reserve(faces.cols());
    for (int j = 0; j < faces.cols(); ++j) {
      polygon.vertices.push_back(faces(i, j));
    }
    result->polygons[i] = polygon;
  }

  return result;
}

} // anonymous namespace

std::vector<size_t>
select_section_planes(const EigenVectorContainer<double, 4> &planes,
                      const std::vector<size_t> &horizontals,
                      const std::vector<double> &section_heights,
                      double tolerance_m) {
  if (section_heights.empty())
    return horizontals;
  std::vector<size_t> out;
  std::vector<double> dropped;
  for (size_t id : horizontals) {
    const auto &p = planes[id];
    const double z = -p[3] / p[2]; // n . x + d = 0 with n ~ (0, 0, +-1)
    const bool near =
        std::any_of(section_heights.begin(), section_heights.end(),
                    [&](double h) { return std::abs(z - h) <= tolerance_m; });
    if (near)
      out.push_back(id);
    else
      dropped.push_back(z);
  }
  if (out.size() < 2) {
    reusex::warn("mesh: {} of {} horizontal plane(s) lie within {:.2f} m of "
                 "the {} storey floor/ceiling height(s) [{:.2f}]; the storeys "
                 "table does not describe these planes, so every horizontal "
                 "plane stays a section plane. Re-run `rux create storeys`",
                 out.size(), horizontals.size(), tolerance_m,
                 section_heights.size(), fmt::join(section_heights, ", "));
    return horizontals;
  }
  reusex::info("mesh: {} of {} horizontal plane(s) are storey floors or "
               "ceilings and become section planes; {} left out at z = "
               "[{:.2f}] m",
               out.size(), horizontals.size(), dropped.size(),
               fmt::join(dropped, ", "));
  return out;
}

std::vector<std::array<double, 2>>
wall_z_ranges(const EigenVectorContainer<double, 4> &planes,
              const std::vector<IndicesPtr> &inliers, CloudConstPtr cloud,
              const std::vector<std::pair<size_t, size_t>> &pairs) {
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kVerticalEps = 0.1; // |n_z| below this: a wall
  constexpr double kTrim = 0.02;       // percentile cut at each end
  std::vector<std::array<double, 2>> out(planes.size(), {nan, nan});
  if (planes.size() != inliers.size())
    throw std::invalid_argument(
        fmt::format("wall_z_ranges: {} planes but {} inlier lists",
                    planes.size(), inliers.size()));
  std::vector<bool> wall(planes.size(), false);
  for (size_t i = 0; i < planes.size(); ++i) {
    wall[i] = std::abs(planes[i][2]) < kVerticalEps;
    if (!wall[i] || !inliers[i] || inliers[i]->empty() || !cloud)
      continue;
    std::vector<float> z;
    z.reserve(inliers[i]->size());
    for (int idx : *inliers[i])
      z.push_back(cloud->points[static_cast<size_t>(idx)].z);
    std::sort(z.begin(), z.end());
    const auto last = static_cast<double>(z.size() - 1);
    const auto lo = static_cast<size_t>(std::floor(kTrim * last));
    const auto hi = static_cast<size_t>(std::ceil((1.0 - kTrim) * last));
    out[i] = {z[lo], z[hi]};
  }
  // The two faces of a wall share one range.
  for (const auto &[a, b] : pairs) {
    if (a >= out.size() || b >= out.size() || !wall[a] || !wall[b])
      continue;
    const auto ra = out[a], rb = out[b];
    if (std::isnan(ra[0]) && std::isnan(rb[0]))
      continue;
    const double lo = std::isnan(ra[0])   ? rb[0]
                      : std::isnan(rb[0]) ? ra[0]
                                          : std::min(ra[0], rb[0]);
    const double hi = std::isnan(ra[1])   ? rb[1]
                      : std::isnan(rb[1]) ? ra[1]
                                          : std::max(ra[1], rb[1]);
    out[a] = out[b] = {lo, hi};
  }
  return out;
}

pcl::PolygonMeshPtr mesh(CloudConstPtr cloud, CloudNConstPtr normals,
                         EigenVectorContainer<double, 4> &planes,
                         EigenVectorContainer<double, 3> &centroids,
                         std::vector<IndicesPtr> &inliers, CloudLConstPtr rooms,
                         MeshOptions const opt) {

  // If filter is provided in options, filter the inliers lists
  if (opt.filter) {
    std::unordered_set<int> filtered_set(opt.filter->begin(),
                                         opt.filter->end());
    reusex::debug("Mesh generation using {} filtered points",
                  opt.filter->size());

    // Filter each inlier list to only include filtered indices
    for (auto &inlier_list : inliers) {
      IndicesPtr filtered_inliers = std::make_shared<Indices>();
      filtered_inliers->reserve(inlier_list->size());
      for (int idx : *inlier_list) {
        if (filtered_set.find(idx) != filtered_set.end()) {
          filtered_inliers->push_back(idx);
        }
      }
      inlier_list = filtered_inliers;
    }
  }

  // May be null when running headless (no observer registered) — every use
  // below must be guarded.
  auto *observer = reusex::core::get_visual_observer();
  constexpr auto stage = reusex::core::Stage::mesh_generation;

  if (observer)
    observer->viewer_add_geometry("input_cloud", cloud, stage);

  // Planes the classification calls furniture or clutter never reach the
  // cell complex (reusex-z1v.4.3): drop them before they can pull a wall or
  // floor towards them in regularisation and merging.
  drop_planes_by_role(planes, inliers, centroids, opt.plane_roles);

  planes = regularizePlanes<double, PointT>(planes, cloud, inliers, 10.0);

  std::tie(planes, inliers, centroids) =
      merge_planes(planes, inliers, centroids, cloud);
  reusex::debug("Number of planes after merging: {}", planes.size());

  planes = force_orthogonal_planes(planes);
  reusex::debug("Number of planes after forcing orthogonality: {}",
                planes.size());

  // Only wall lines and storey floors/ceilings become arrangement lines and
  // sections; jambs, reveals and furniture tops stay in the `planes` cloud
  // but would each cut the whole building (reusex-8l3). A wall's two faces
  // are the ones make_pairs pairs, so use its pairing distance.
  auto structural = opt.structural;
  structural.wall_pair_max_m = opt.search_threshold;
  select_structural_planes(planes, inliers, centroids, cloud, structural);

  auto pairs = make_pairs(planes, inliers, centroids, opt.search_threshold,
                          opt.new_plane_offset);
  reusex::debug("Number of plane pairs: {}", pairs.size());

  auto [vertical, horizontal] = separate_planes(planes);

  // Storey floors and ceilings only as section planes, when the storeys
  // table gave their heights (reusex-z1v.3.4).
  if (!opt.section_heights.empty())
    horizontal = select_section_planes(planes, horizontal, opt.section_heights,
                                       opt.section_tolerance_m);

  // Each section gets only the walls that reach into it (reusex-sba).
  std::vector<std::array<double, 2>> z_ranges;
  if (opt.storey_local)
    z_ranges = wall_z_ranges(planes, inliers, cloud, pairs);
  reusex::debug("Number indices in inliers [{}]",
                fmt::join(inliers | ranges::views::transform([](auto const &i) {
                            return i->size();
                          }),
                          ", "));
  reusex::debug("Number of horizonal planes: {}", horizontal.size());
  reusex::debug("Number of vertical planes: {}", vertical.size());

  reusex::debug("Vertical planes [{}]", fmt::join(vertical, ", "));
  reusex::debug("Horizontal planes [{}]", fmt::join(horizontal, ", "));

  auto sel = [&](size_t idx) {
    return std::make_pair(planes[idx], centroids[idx]);
  };

  auto vPlanes =
      vertical | ranges::views::transform(sel) | ranges::to<std::vector>();

  auto hPlanes =
      horizontal | ranges::views::transform(sel) | ranges::to<std::vector>();

  auto pPairs = pairs | ranges::views::transform([&](auto const &p) {
                  return std::make_pair(sel(p.first), sel(p.second));
                }) |
                ranges::to<std::vector>();

  if (observer) {
    observer->viewer_add_geometries("vertical_planes", vPlanes, stage);
    observer->viewer_add_geometries("horizontal_planes", hPlanes, stage);
    observer->viewer_add_geometries("plane_pairs", pPairs, stage);
  }

  // viewer->addPlanes(vertical_planes, "vertical_planes", vps->at(0));
  // viewer->addPlanes(horizontal_planes, "horizontal_planes", vps->at(0));
  // viewer->addPlanePairs(plane_pairs, "plain_pairs", vps->at(0));

  PointT min, max;
  pcl::getMinMax3D(*cloud, min, max);

  std::shared_ptr<CellComplex> cc = std::make_shared<CellComplex>(
      planes, vertical, horizontal, pairs,
      std::array<double, 2>{min.x - 1, min.y - 1},
      std::array<double, 2>{max.x + 1, max.y + 1}, std::nullopt, opt.max_cells,
      z_ranges, opt.section_overlap_m);

  reusex::debug("Cell complex: {}", *cc);

  // INFO: Display cell complex
  // if (viewer)
  //  viewer->addCellComplex(cc, "cell_complex", vps->at(1));

  cc->compute_room_probabilities<PointT, NormalT, LabelT>(cloud, normals,
                                                          rooms);
  // // INFO: Display face support probabilities
  // if (false && viewer)
  //   viewer->addSupportProbabilities(cc, "face_support_probablilites",
  //   vps->at(2));

  cc->compute_face_coverage<PointT>(cloud, planes, inliers);
  // INFO: Display room probabilities
  if (observer)
    observer->viewer_add_geometry("cell_complex", *cc, stage);
  //   viewer->addRoomProbabilities(cc, "room_probablilites", vps->at(2));

  reusex::debug("Cell complex: {}", *cc);

  reusex::trace("Initializing Solidifier");
  reusex::geometry::SolidifierOptions solidifier_opts;
  solidifier_opts.time_limit_seconds = opt.time_limit_seconds;
  solidifier_opts.alpha = opt.alpha;
  solidifier_opts.sectioned = opt.sectioned;
  solidifier_opts.sectioned_threshold = opt.sectioned_threshold;
  solidifier_opts.solver = opt.solver;
  solidifier_opts.cuopt_max_variables = opt.cuopt_max_variables;
  reusex::geometry::Solidifier solidifier(cc, solidifier_opts);
  auto results = solidifier.solve();

  if (!results)
    reusex::warn("Solidification failed to find a solution (status={})",
                 reusex::geometry::to_string(solidifier.last_solve_status()));

  // INFO: Display results
  if (observer && results.has_value())
    observer->viewer_add_geometry("rooms", results.value(), stage);
  // viewer->addRooms(cc, results.value(), "rooms", vps->at(3));

  if (results) {
    auto [room_labels, wall_labels] = results.value();
    auto [vertices, faces] = solidifier.toMesh(
        [&](const CellComplex::Vertex v) { return room_labels[v] > 0; });
    return eigen_to_polygon_mesh(vertices, faces);
  }

  // Fallback: return empty mesh when solidification fails
  return pcl::PolygonMeshPtr(new pcl::PolygonMesh());
}
} // namespace reusex::geometry
