// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for select_structural_planes() (reusex-8l3): the step in
// `create mesh` that keeps the cell complex down to wall lines and storey
// sections. These tests pin what it keeps and what it drops:
//   - collinear same-facing wall pieces, far apart  -> one line
//   - the two faces of a thin wall                  -> both kept
//   - a jamb (small, perpendicular)                 -> dropped
//   - a small pier collinear with a facade          -> merged, kept
//   - a desk top / desk underside over the floor    -> dropped
//   - an upper storey floor, a split level          -> kept
//   - disabled                                      -> unchanged
//   - a door leaf in a hall (short, not full height,
//     no wall in the walls table)                   -> dropped (reusex-72a)
//   - a long wall seen only to door height, a short
//     one the walls table has, a full-height pier   -> kept
// and for drop_planes_by_role() (reusex-z1v.4.3 / reusex-vwa): planes whose
// plane_classes role is furniture or clutter never reach the cell complex.

#include <reusex/reconstruction/structural_planes.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;

namespace {

struct Scene {
  CloudPtr cloud{new Cloud};
  EigenVectorContainer<double, 4> planes;
  std::vector<IndicesPtr> inliers;
  EigenVectorContainer<double, 3> centroids;

  /// Wall on x = x0 (normal +-x), spanning y in [y0, y1], z in [z0, z1].
  void x_wall(double x0, double sign, double y0, double y1, double z0,
              double z1, double step = 0.05) {
    IndicesPtr idx(new Indices);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (double y = y0; y <= y1 + 1e-9; y += step)
      for (double z = z0; z <= z1 + 1e-9; z += step)
        add(idx, c, x0, y, z);
    push(Eigen::Vector4d(sign, 0, 0, -sign * x0), idx, c);
  }

  /// Wall on y = y0 (normal +-y), spanning x in [x0, x1], z in [z0, z1].
  void y_wall(double y0, double sign, double x0, double x1, double z0,
              double z1, double step = 0.05) {
    IndicesPtr idx(new Indices);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (double x = x0; x <= x1 + 1e-9; x += step)
      for (double z = z0; z <= z1 + 1e-9; z += step)
        add(idx, c, x, y0, z);
    push(Eigen::Vector4d(0, sign, 0, -sign * y0), idx, c);
  }

  /// Horizontal patch at height h; sign +1 faces up (floor, desk top),
  /// -1 faces down (ceiling, desk underside).
  void slab(double h, double sign, double x0, double x1, double y0, double y1,
            double step = 0.05) {
    IndicesPtr idx(new Indices);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (double x = x0; x <= x1 + 1e-9; x += step)
      for (double y = y0; y <= y1 + 1e-9; y += step)
        add(idx, c, x, y, h);
    push(Eigen::Vector4d(0, 0, sign, -sign * h), idx, c);
  }

  /// Vertical face through (x0, y0) -> (x1, y1), z in [z0, z1]; @p flip
  /// turns its normal to the other side.
  void face(double x0, double y0, double x1, double y1, double z0, double z1,
            bool flip, double step = 0.05) {
    IndicesPtr idx(new Indices);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    const Eigen::Vector2d a(x0, y0), b(x1, y1);
    const double len = (b - a).norm();
    for (double t = 0.0; t <= len + 1e-9; t += step)
      for (double z = z0; z <= z1 + 1e-9; z += step) {
        const Eigen::Vector2d q = a + (b - a) * (t / len);
        add(idx, c, q.x(), q.y(), z);
      }
    Eigen::Vector2d n((b - a).y(), -(b - a).x());
    n.normalize();
    if (flip)
      n = -n;
    push(Eigen::Vector4d(n.x(), n.y(), 0, -n.dot(a)), idx, c);
  }

    private:
  void add(IndicesPtr &idx, Eigen::Vector3d &c, double x, double y, double z) {
    PointT p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    idx->push_back(static_cast<int>(cloud->size()));
    cloud->push_back(p);
    c += Eigen::Vector3d(x, y, z);
  }
  void push(const Eigen::Vector4d &plane, const IndicesPtr &idx,
            const Eigen::Vector3d &sum) {
    planes.push_back(plane);
    inliers.push_back(idx);
    centroids.push_back(sum / static_cast<double>(idx->size()));
  }
};

std::size_t count_kind(const EigenVectorContainer<double, 4> &planes,
                       bool vertical) {
  std::size_t n = 0;
  for (const auto &p : planes)
    if ((std::abs(p[2]) < 0.1) == vertical)
      ++n;
  return n;
}

bool has_height(const EigenVectorContainer<double, 4> &planes, double h) {
  for (const auto &p : planes)
    if (std::abs(p[2]) > 0.9 && std::abs(-p[3] / p[2] - h) < 1e-6)
      return true;
  return false;
}

/// A hall 4 x 6 m, 2.6 m high: its four walls, both faces 0.12 m apart,
/// floor and ceiling. The walls table has all four walls.
void hall(Scene &s, StructuralEvidence &ev) {
  s.slab(0.0, +1, 0.0, 4.0, 0.0, 6.0);
  s.slab(2.6, -1, 0.0, 4.0, 0.0, 6.0);
  s.x_wall(0.0, +1, 0.0, 6.0, 0.0, 2.6);
  s.x_wall(-0.12, -1, 0.0, 6.0, 0.0, 2.6);
  s.x_wall(4.0, -1, 0.0, 6.0, 0.0, 2.6);
  s.x_wall(4.12, +1, 0.0, 6.0, 0.0, 2.6);
  s.y_wall(0.0, +1, 0.0, 4.0, 0.0, 2.6);
  s.y_wall(-0.12, -1, 0.0, 4.0, 0.0, 2.6);
  s.y_wall(6.0, -1, 0.0, 4.0, 0.0, 2.6);
  s.y_wall(6.12, +1, 0.0, 4.0, 0.0, 2.6);
  ev.storeys = {{0.0, 2.6}};
  ev.walls = {{{-0.06, 0.0}, {-0.06, 6.0}, 0.0, 2.6, 0.12},
              {{4.06, 0.0}, {4.06, 6.0}, 0.0, 2.6, 0.12},
              {{0.0, -0.06}, {4.0, -0.06}, 0.0, 2.6, 0.12},
              {{0.0, 6.06}, {4.0, 6.06}, 0.0, 2.6, 0.12}};
}

/// Is there a vertical plane with (horizontal) normal +-@p n?
bool has_wall_facing(const EigenVectorContainer<double, 4> &planes,
                     const Eigen::Vector2d &n) {
  for (const auto &p : planes)
    if (std::abs(p[2]) < 0.1 &&
        std::abs(std::abs(p.head<2>().normalized().dot(n.normalized())) - 1.0) <
            1e-6)
      return true;
  return false;
}

} // namespace

TEST_CASE("structural planes: collinear wall pieces become one line",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 3.0, 0.0, 2.5);   // 7.5 m^2
  s.x_wall(0.05, +1, 8.0, 10.0, 0.0, 2.5); // same line, 5 m along, 5 cm off
  const auto before = s.inliers[0]->size() + s.inliers[1]->size();

  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  REQUIRE(s.planes.size() == 1);
  CHECK(s.inliers[0]->size() == before);
  CHECK(r.vertical_in == 2);
  CHECK(r.vertical_merged == 1);
  // The best-supported piece keeps its plane.
  CHECK(s.planes[0][3] == 0.0);
}

TEST_CASE("structural planes: the two faces of a thin wall both stay",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 4.0, 0.0, 2.5);
  s.x_wall(0.10, -1, 0.0, 4.0, 0.0, 2.5); // other face, 10 cm behind
  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(s.planes.size() == 2);
  CHECK(r.vertical_merged == 0);
  CHECK(r.vertical_small == 0);
}

TEST_CASE("structural planes: a jamb is dropped, a collinear pier is kept",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 3.0, 0.0, 2.5);  // facade piece, 7.5 m^2
  s.x_wall(0.02, +1, 4.0, 4.5, 0.0, 2.0); // pier, 1.0 m^2, same line
  s.y_wall(3.0, +1, 0.0, 0.25, 0.0, 2.0); // jamb, 0.5 m^2, perpendicular
  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  REQUIRE(s.planes.size() == 1);
  CHECK(std::abs(s.planes[0][0]) == 1.0); // the x wall line survives
  CHECK(r.vertical_merged == 1);
  CHECK(r.vertical_small == 1);
}

TEST_CASE("structural planes: a short thin wall counts both faces",
          "[reconstruction][structural_planes]") {
  // Each face of this 0.5 m stub is ~1.1 m^2, under the threshold, but the
  // wall it belongs to has twice that: both faces must stay (seen on house1,
  // where dropping them merged two rooms).
  Scene s;
  s.y_wall(1.0, +1, 0.0, 0.45, 0.0, 2.4);
  s.y_wall(1.1, -1, 0.0, 0.45, 0.0, 2.4);
  s.x_wall(5.0, +1, 0.0, 0.25, 0.0, 2.0); // a lone jamb still goes
  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(s.planes.size() == 2);
  CHECK(r.vertical_small == 1);
}

TEST_CASE("structural planes: a lone small wall stays when the threshold "
          "allows it",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.y_wall(3.0, +1, 0.0, 0.8, 0.0, 2.5); // 2.0 m^2 stub
  select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(s.planes.size() == 1);
}

TEST_CASE("structural planes: furniture tops do not become storey sections",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);  // floor
  s.slab(2.7, -1, 0.0, 10.0, 0.0, 8.0);  // ceiling
  s.slab(0.75, +1, 2.0, 4.0, 2.0, 3.0);  // desk top
  s.slab(0.72, -1, 2.0, 4.0, 2.0, 3.0);  // desk underside
  s.slab(3.0, +1, 0.0, 10.0, 0.0, 8.0);  // upper storey floor
  s.slab(0.6, +1, 12.0, 16.0, 0.0, 4.0); // split level beside the floor

  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(r.horizontal_in == 6);
  CHECK(r.horizontal_furniture == 2);
  CHECK(s.planes.size() == 4);
  CHECK(has_height(s.planes, 0.0));
  CHECK(has_height(s.planes, 2.7));
  CHECK(has_height(s.planes, 3.0));
  CHECK(has_height(s.planes, 0.6));
  CHECK_FALSE(has_height(s.planes, 0.75));
  CHECK_FALSE(has_height(s.planes, 0.72));
}

TEST_CASE("structural planes: a large raised floor over a small one stays",
          "[reconstruction][structural_planes]") {
  // The clearance rule only drops a plane over a LARGER one: a big upper
  // floor stays even when a small cabinet top lies 1.5 m below it.
  Scene s;
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);
  s.slab(1.5, +1, 2.0, 2.6, 2.0, 2.6); // cabinet top
  s.slab(3.0, +1, 0.0, 10.0, 0.0, 8.0);
  select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(has_height(s.planes, 3.0));
  CHECK_FALSE(has_height(s.planes, 1.5));
}

TEST_CASE("structural planes: disabled leaves the planes alone",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 3.0, 0.0, 2.5);
  s.y_wall(3.0, +1, 0.0, 0.25, 0.0, 2.0);
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);
  s.slab(0.75, +1, 2.0, 4.0, 2.0, 3.0);
  StructuralPlaneOptions opt;
  opt.enabled = false;
  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud, opt);
  CHECK(s.planes.size() == 4);
  CHECK(r.planes_out == 4);
  CHECK(count_kind(s.planes, true) == 2);
}

TEST_CASE("structural planes: mismatched inputs throw",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 3.0, 0.0, 2.5);
  s.centroids.clear();
  CHECK_THROWS_AS(
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud),
      std::runtime_error);
}

// ── drop_planes_by_role (reusex-z1v.4.3) ──────────────────────────────────

TEST_CASE("plane roles: furniture and clutter planes are dropped",
          "[reconstruction][structural_planes][plane_roles]") {
  using core::PlaneRole;
  Scene s;
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);  // 0 floor
  s.slab(2.7, -1, 0.0, 10.0, 0.0, 8.0);  // 1 ceiling
  s.slab(0.75, +1, 2.0, 4.0, 2.0, 3.0);  // 2 desk top
  s.x_wall(0.0, +1, 0.0, 8.0, 0.0, 2.7); // 3 wall
  s.slab(1.9, +1, 6.0, 6.5, 1.0, 2.0);   // 4 shelf
  s.x_wall(9.0, -1, 0.0, 1.0, 0.0, 2.0); // 5 unknown
  const std::vector<PlaneRole> roles{PlaneRole::floor,     PlaneRole::ceiling,
                                     PlaneRole::furniture, PlaneRole::wall,
                                     PlaneRole::clutter,   PlaneRole::unknown};
  const auto floor_inliers = s.inliers[0];
  const auto wall_plane = s.planes[3];

  const auto r = drop_planes_by_role(s.planes, s.inliers, s.centroids, roles);
  CHECK(r.furniture == 1);
  CHECK(r.clutter == 1);
  CHECK(r.vertical_clutter_kept == 0);
  CHECK(r.planes_out == 4);
  REQUIRE(s.planes.size() == 4);
  REQUIRE(s.inliers.size() == 4);
  REQUIRE(s.centroids.size() == 4);
  // Input order is kept and each survivor keeps its own inliers.
  CHECK(s.inliers[0] == floor_inliers);
  CHECK(s.planes[2] == wall_plane);
  CHECK(has_height(s.planes, 0.0));
  CHECK(has_height(s.planes, 2.7));
  CHECK_FALSE(has_height(s.planes, 0.75));
  CHECK_FALSE(has_height(s.planes, 1.9));
  CHECK(count_kind(s.planes, true) == 2); // the wall and the unknown plane
}

TEST_CASE("plane roles: vertical clutter is left to the structural filter",
          "[reconstruction][structural_planes][plane_roles]") {
  // reusex-rtx: "clutter" on a vertical plane is the height rule's verdict
  // (it does not reach floor to ceiling): a hallway wall seen to door
  // height, a wall stub behind furniture, but also a desk screen. Only the
  // structural filter can tell those apart (wall-line area with the wall's
  // far face), so the plane goes on to it: the hallway stub seen from both
  // sides is a wall line, the lone desk screen is not.
  using core::PlaneRole;
  Scene s;
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);   // 0 floor
  s.slab(2.7, -1, 0.0, 10.0, 0.0, 8.0);   // 1 ceiling
  s.x_wall(0.0, +1, 0.0, 8.0, 0.0, 2.7);  // 2 wall
  s.x_wall(5.0, +1, 0.0, 0.6, 0.0, 1.7);  // 3 hallway stub, room side
  s.x_wall(5.12, -1, 0.0, 0.6, 0.0, 1.7); // 4 hallway stub, far side
  s.y_wall(4.0, +1, 2.0, 2.8, 0.75, 1.2); // 5 desk screen
  const std::vector<PlaneRole> roles{PlaneRole::floor,   PlaneRole::ceiling,
                                     PlaneRole::wall,    PlaneRole::clutter,
                                     PlaneRole::clutter, PlaneRole::clutter};

  const auto r = drop_planes_by_role(s.planes, s.inliers, s.centroids, roles);
  CHECK(r.clutter == 0);
  CHECK(r.vertical_clutter_kept == 3);
  CHECK(r.planes_out == 6);

  select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(count_kind(s.planes, true) == 3); // the wall and both stub faces
}

TEST_CASE("plane roles: only horizontal clutter is dropped",
          "[reconstruction][structural_planes][plane_roles]") {
  // The classification calls a plane vertical up to 10 deg off; before
  // regularisation a wall piece can lean that much. It is not horizontal,
  // so it is not dropped either (house2: one such plane cost 2.6 cm of fit).
  using core::PlaneRole;
  Scene s;
  s.x_wall(7.0, +1, 0.0, 2.0, 0.0, 1.5); // 0 leaning wall piece
  const double c = std::cos(7.0 * std::numbers::pi / 180.0);
  const double z = std::sin(7.0 * std::numbers::pi / 180.0);
  s.planes.back() = Eigen::Vector4d(c, 0, z, -c * 7.0 - z * 0.75);
  s.slab(1.9, +1, 6.0, 6.5, 1.0, 2.0); // 1 shelf
  const auto r = drop_planes_by_role(s.planes, s.inliers, s.centroids,
                                     {PlaneRole::clutter, PlaneRole::clutter});
  CHECK(r.clutter == 1);
  CHECK(r.vertical_clutter_kept == 1);
  REQUIRE(s.planes.size() == 1);
  CHECK(std::abs(s.planes[0][2]) == Catch::Approx(z));
}

TEST_CASE("plane roles: no roles leaves the planes alone",
          "[reconstruction][structural_planes][plane_roles]") {
  Scene s;
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);
  s.slab(0.75, +1, 2.0, 4.0, 2.0, 3.0);
  const auto r = drop_planes_by_role(s.planes, s.inliers, s.centroids, {});
  CHECK(s.planes.size() == 2);
  CHECK(r.planes_out == 2);
  CHECK(r.furniture == 0);
}

TEST_CASE("plane roles: a role count that disagrees with the planes throws",
          "[reconstruction][structural_planes][plane_roles]") {
  Scene s;
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);
  s.slab(0.75, +1, 2.0, 4.0, 2.0, 3.0);
  CHECK_THROWS_AS(drop_planes_by_role(s.planes, s.inliers, s.centroids,
                                      {core::PlaneRole::floor}),
                  std::invalid_argument);
}

TEST_CASE("plane roles: catch a desk row the clearance rule cannot",
          "[reconstruction][structural_planes][plane_roles]") {
  // An open-plan office: a long row of desks hides the floor under it, so
  // the desk top is larger than the floor seen beside it and hardly lies
  // over it. The geometric clearance rule needs a larger plane under most
  // of the footprint, so the desk row survives it and becomes a storey
  // section at desk height (reusex-vwa). Its classified role removes it.
  using core::PlaneRole;
  Scene s;
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 1.0);  // floor strip left visible
  s.slab(0.75, +1, 0.0, 10.0, 1.0, 4.0); // desk row
  s.slab(2.7, -1, 0.0, 10.0, 0.0, 4.0);  // ceiling

  Scene g = s; // the geometry-only path keeps the desk row
  select_structural_planes(g.planes, g.inliers, g.centroids, g.cloud);
  CHECK(has_height(g.planes, 0.75));

  drop_planes_by_role(
      s.planes, s.inliers, s.centroids,
      {PlaneRole::floor, PlaneRole::furniture, PlaneRole::ceiling});
  select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK_FALSE(has_height(s.planes, 0.75));
  CHECK(has_height(s.planes, 0.0));
  CHECK(has_height(s.planes, 2.7));
}

// ── Short free-standing planes (reusex-72a) ─────────────────────────────────

TEST_CASE("structural planes: an open door leaf in a hall is no wall line",
          "[reconstruction][structural_planes][short_planes]") {
  // house2's hall: an open door leaf (0.9 m wide, 2.0 m tall, at an angle,
  // seen from both sides) is a 1.8 m^2 vertical pair the area rule keeps.
  // As an arrangement line it runs across the whole storey, and the MIP
  // used it as a 3 m diagonal room boundary. It is short, does not reach
  // the ceiling, and the walls stage found no wall on it.
  Scene s;
  StructuralEvidence ev;
  hall(s, ev);
  const Eigen::Vector2d a(1.5, 2.0), b(1.5 + 0.9 * 0.8, 2.0 + 0.9 * 0.6);
  s.face(a.x(), a.y(), b.x(), b.y(), 0.0, 2.0, false);
  const Eigen::Vector2d off = Eigen::Vector2d(0.6, -0.8) * 0.04;
  s.face(a.x() + off.x(), a.y() + off.y(), b.x() + off.x(), b.y() + off.y(),
         0.0, 2.0, true);

  const auto r = select_structural_planes(s.planes, s.inliers, s.centroids,
                                          s.cloud, {}, ev);
  CHECK(r.vertical_unsupported == 2);
  CHECK_FALSE(has_wall_facing(s.planes, {0.6, -0.8}));
  CHECK(count_kind(s.planes, true) == 8); // the hall's walls all stay
  CHECK(count_kind(s.planes, false) == 2);
}

TEST_CASE("structural planes: walls seen only to door height stay",
          "[reconstruction][structural_planes][short_planes]") {
  // reusex-rtx: dropping walls seen only to door height cost house2 most of
  // its ground floor. None of these may go:
  //   - a long corridor wall seen to 2.0 m, missing from the walls table
  //   - a short wall stub seen to 2.0 m that the walls table has
  //   - a short full-height pier the walls table missed
  Scene s;
  StructuralEvidence ev;
  hall(s, ev);
  s.y_wall(3.0, +1, 0.0, 3.0, 0.0, 2.0); // corridor wall, 3 m long
  s.y_wall(3.1, -1, 0.0, 3.0, 0.0, 2.0);
  s.x_wall(2.0, +1, 4.5, 5.5, 0.0, 2.0); // stub, 1 m long, in the table
  s.x_wall(2.1, -1, 4.5, 5.5, 0.0, 2.0);
  ev.walls.push_back({{2.05, 4.5}, {2.05, 5.5}, 0.0, 2.6, 0.1});
  s.face(3.0, 1.0, 3.6, 1.6, 0.0, 2.6, false); // pier, 0.85 m, full height

  const auto r = select_structural_planes(s.planes, s.inliers, s.centroids,
                                          s.cloud, {}, ev);
  CHECK(r.vertical_unsupported == 0);
  CHECK(count_kind(s.planes, true) == 8 + 5);
}

TEST_CASE("structural planes: without walls or storeys a leaf stays",
          "[reconstruction][structural_planes][short_planes]") {
  // No evidence to judge it by: the behaviour before reusex-72a.
  Scene s;
  StructuralEvidence ev;
  hall(s, ev);
  s.face(1.5, 2.0, 2.22, 2.54, 0.0, 2.0, false);
  s.face(1.53, 1.97, 2.25, 2.51, 0.0, 2.0, true);
  StructuralEvidence no_walls = ev;
  no_walls.walls.clear();
  Scene t = s;
  auto r = select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud,
                                    {}, no_walls);
  CHECK(r.vertical_unsupported == 0);
  CHECK(count_kind(s.planes, true) == 10);
  StructuralEvidence no_storeys = ev;
  no_storeys.storeys.clear();
  r = select_structural_planes(t.planes, t.inliers, t.centroids, t.cloud, {},
                               no_storeys);
  CHECK(r.vertical_unsupported == 0);
  CHECK(count_kind(t.planes, true) == 10);
}
