// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// A partition whose two faces cross (reusex-wm0).
//
// house1's wall i-n1 between the hall and the entrance is a full-height
// partition, and the band rooms split the two rooms along it, but the mesh
// put no wall there: the two rooms' scans drifted apart by more than the
// wall is thick, so the hall's face of the wall (normal +x) lies 0.09 m west
// of the entrance's face (normal -x). make_pairs paired the two faces, and
// the cell complex makes a cell a wall cell only where it lies behind both
// faces of a pair -- nowhere, for a crossed pair. With no wall cell to
// separate them, the MIP had to give both rooms one label. This test builds
// two rooms side by side with such a crossed partition and runs the whole
// mesh stage over them.

#include <reusex/reconstruction/mesh.hpp>
#include <reusex/types.hpp>

#include <pcl/conversions.h>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;

namespace {

struct Scene {
  CloudPtr cloud = std::make_shared<Cloud>();
  CloudNPtr normals = std::make_shared<CloudN>();
  CloudLPtr rooms = std::make_shared<CloudL>();
  EigenVectorContainer<double, 4> planes;
  EigenVectorContainer<double, 3> centroids;
  std::vector<IndicesPtr> inliers;

  /// One plane: the axis-aligned rectangle origin + [0, u] x [0, v] with
  /// normal n, sampled every `step`, its points labelled `room`.
  void patch(const Eigen::Vector3d &origin, const Eigen::Vector3d &u,
             const Eigen::Vector3d &v, const Eigen::Vector3d &n,
             std::uint32_t room, double step = 0.05) {
    IndicesPtr idx(new Indices);
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    const int nu = static_cast<int>(std::round(u.norm() / step));
    const int nv = static_cast<int>(std::round(v.norm() / step));
    for (int i = 0; i <= nu; ++i)
      for (int j = 0; j <= nv; ++j) {
        const Eigen::Vector3d p =
            origin + u * (double(i) / nu) + v * (double(j) / nv);
        PointT q;
        q.x = static_cast<float>(p.x());
        q.y = static_cast<float>(p.y());
        q.z = static_cast<float>(p.z());
        idx->push_back(static_cast<int>(cloud->size()));
        cloud->push_back(q);
        normals->push_back(NormalT(static_cast<float>(n.x()),
                                   static_cast<float>(n.y()),
                                   static_cast<float>(n.z())));
        LabelT l;
        l.label = room;
        rooms->push_back(l);
        sum += p;
      }
    const Eigen::Vector3d c = sum / static_cast<double>(idx->size());
    planes.emplace_back(n.x(), n.y(), n.z(), -n.dot(c));
    centroids.push_back(c);
    inliers.push_back(idx);
  }

  /// A box room [x0, x1] x [0, 3] x [0, 2.5], every face facing inwards.
  void room(double x0, double x1, std::uint32_t label) {
    const double w = x1 - x0, d = 3.0, h = 2.5;
    const Eigen::Vector3d ex(w, 0, 0), ey(0, d, 0), ez(0, 0, h);
    const Eigen::Vector3d o(x0, 0, 0);
    patch(o, ex, ey, {0, 0, 1}, label);       // floor
    patch(o + ez, ex, ey, {0, 0, -1}, label); // ceiling
    patch(o, ey, ez, {1, 0, 0}, label);       // x = x0
    patch(o + ex, ey, ez, {-1, 0, 0}, label); // x = x1
    patch(o, ex, ez, {0, 1, 0}, label);       // y = 0
    patch(o + ey, ex, ez, {0, -1, 0}, label); // y = d
  }
};

} // namespace

TEST_CASE("Mesh_TwoRoomsSplitByACrossedPartition_KeepTheWall",
          "[reconstruction][mesh][make_pairs]") {
  // The west room's face of the partition (normal -x) at x = 3.05, the east
  // room's (normal +x) at x = 2.95: crossed by 0.1 m, as house1's i-n1.
  Scene s;
  s.room(0.0, 3.05, 1);
  s.room(2.95, 6.0, 2);
  s.cloud->width = static_cast<std::uint32_t>(s.cloud->size());
  s.cloud->height = 1;
  s.normals->width = s.cloud->width;
  s.normals->height = 1;
  s.rooms->width = s.cloud->width;
  s.rooms->height = 1;

  MeshOptions opt;
  opt.solver = SolverChoice::highs;
  auto mesh = geometry::mesh(s.cloud, s.normals, s.planes, s.centroids,
                             s.inliers, s.rooms, opt);
  REQUIRE(mesh);
  REQUIRE_FALSE(mesh->polygons.empty());

  CloudLoc v;
  pcl::fromPCLPointCloud2(mesh->cloud, v);
  // The partition's faces: the mesh's area in the planes x = const near
  // x = 3. Without a wall between the rooms, the mesh is one box over both
  // and has none (its outer walls only carry the vertices where the
  // partition's arrangement lines cut them).
  double partition_area = 0.0;
  for (const auto &poly : mesh->polygons) {
    REQUIRE(poly.vertices.size() == 3);
    const Eigen::Vector3f a = v.points[poly.vertices[0]].getVector3fMap();
    const Eigen::Vector3f b = v.points[poly.vertices[1]].getVector3fMap();
    const Eigen::Vector3f c = v.points[poly.vertices[2]].getVector3fMap();
    const bool in_x_plane =
        std::abs(a.x() - b.x()) < 1e-4F && std::abs(a.x() - c.x()) < 1e-4F;
    if (in_x_plane && std::abs(a.x() - 3.0F) < 0.1F)
      partition_area += 0.5 * (b - a).cross(c - a).norm();
  }
  INFO("mesh: " << v.size() << " vertices, " << mesh->polygons.size()
                << " faces, " << partition_area << " m^2 on the partition");
  // Both faces of a 3 x 2.5 m partition.
  CHECK(partition_area > 14.0);
}
