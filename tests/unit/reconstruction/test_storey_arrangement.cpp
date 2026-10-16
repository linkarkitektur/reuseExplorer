// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Storey-local cell complex arrangement (reusex-sba) and storey section
// planes (reusex-z1v.3.4).
//
// The cell complex used to extend every wall across every section, so a
// multi-storey scan got (faces of ALL storeys' walls) x (sections) cells.
// With per-plane z-ranges a section's 2D arrangement holds only the walls
// that reach into it, and the horizontal faces between two sections are the
// overlay of the two arrangements. These tests pin:
//   - two storeys with different wall layouts: cells = sum of the per-storey
//     arrangements, not the product;
//   - the faces of a level between sections are shared, consistently, by
//     exactly one cell below and one above, with matching areas;
//   - the outer boundary stays watertight (no T-junctions, no degenerate
//     triangles) where a coarse section meets a finer one;
//   - a single storey is unchanged, and so is any input without z-ranges;
//   - select_section_planes() keeps every horizontal inside the building
//     (storeys, split levels, an attic under the roof) and drops those
//     outside it, and the horizontals alone without (matching) storeys;
//   - wall_z_ranges() measures a wall's height from its inliers and gives an
//     invented back face its partner's range.

#include <reusex/reconstruction/CellComplex.hpp>
#include <reusex/reconstruction/Solidifier.hpp>
#include <reusex/reconstruction/mesh.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

using namespace reusex::geometry;
using Catch::Approx;

namespace {

using PlaneVec =
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;
using Range = std::array<double, 2>;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// An 8 x 6 m two-storey house: levels at z = 0, 2.5 (ground ceiling), 2.8
// (upper floor), 5.3. Exterior walls run the full height. The ground floor
// is split by walls at x = 2 and x = 4; the upper floor by walls at y = 2
// and y = 4, which the ground floor does not have.
struct House {
  PlaneVec planes;
  std::vector<size_t> verticals;
  std::vector<size_t> horizontals;
  std::vector<std::pair<size_t, size_t>> pairs;
  std::vector<Range> z_ranges; // index-aligned with planes
};

House make_house(bool with_ranges = true) {
  House h;
  auto wall = [&](Eigen::Vector4d p, Range r) {
    h.planes.push_back(p);
    h.verticals.push_back(h.planes.size() - 1);
    h.z_ranges.push_back(r);
  };
  const Range full{0.0, 5.3}, ground{0.0, 2.5}, upper{2.8, 5.3};
  wall({1, 0, 0, 0}, full);    // x = 0
  wall({1, 0, 0, -8}, full);   // x = 8
  wall({0, 1, 0, 0}, full);    // y = 0
  wall({0, 1, 0, -6}, full);   // y = 6
  wall({1, 0, 0, -2}, ground); // x = 2, ground floor only
  wall({1, 0, 0, -4}, ground); // x = 4, ground floor only
  wall({0, 1, 0, -2}, upper);  // y = 2, upper floor only
  wall({0, 1, 0, -4}, upper);  // y = 4, upper floor only
  for (double z : {0.0, 2.5, 2.8, 5.3}) {
    h.planes.emplace_back(0, 0, 1, -z);
    h.horizontals.push_back(h.planes.size() - 1);
    h.z_ranges.push_back({kNaN, kNaN});
  }
  if (!with_ranges)
    h.z_ranges.clear();
  return h;
}

std::shared_ptr<CellComplex> build(House &h, size_t max_cells = 5000) {
  return std::make_shared<CellComplex>(
      h.planes, h.verticals, h.horizontals, h.pairs,
      std::array<double, 2>{-1.0, -1.0}, std::array<double, 2>{9.0, 7.0},
      std::nullopt, max_cells, h.z_ranges);
}

int cells_of_face(const CellComplex &cc, CellComplex::Vertex f) {
  int n = 0;
  for (auto [a, e] = boost::adjacent_vertices(f, cc); a != e; ++a)
    n += cc[*a].type == CellNodeType::cell;
  return n;
}

bool is_horizontal_face_at(const CellComplex &cc, CellComplex::Vertex f,
                           double z) {
  for (auto [a, e] = boost::adjacent_vertices(f, cc); a != e; ++a)
    if (cc[*a].type == CellNodeType::vertex &&
        std::abs(cc[*a].pos.z() - z) > 1e-9)
      return false;
  return true;
}

// Every undirected edge of a closed triangle mesh is used exactly twice.
std::map<std::pair<int, int>, int> edge_uses(const Eigen::MatrixXi &F) {
  std::map<std::pair<int, int>, int> uses;
  for (int i = 0; i < F.rows(); ++i)
    for (int k = 0; k < 3; ++k) {
      int a = F(i, k), b = F(i, (k + 1) % 3);
      if (a > b)
        std::swap(a, b);
      ++uses[{a, b}];
    }
  return uses;
}

} // namespace

TEST_CASE("StoreyArrangement_TwoStoreys_CellsAreSumNotProduct",
          "[geometry][cellcomplex][storey_local]") {
  // Without z-ranges (legacy): lines x = 0,2,4,8 and y = 0,2,4,6 cut every
  // section into 3 x 3 = 9 faces; 3 sections -> 27 cells.
  auto legacy = make_house(false);
  auto cc_legacy = build(legacy);
  CHECK(cc_legacy->num_cells() == 27);

  // Storey-local: ground floor 3 cells, slab (exterior walls only) 1 cell,
  // upper floor 3 cells.
  auto house = make_house();
  auto cc = build(house);
  REQUIRE(cc->n_sections == 3);
  CHECK(cc->num_cells() == 7);

  auto section = cc->property_map<CellComplex::Vertex, int>("c:section");
  std::map<int, int> per_section;
  for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c)
    ++per_section[section[*c]];
  CHECK(per_section == std::map<int, int>{{0, 3}, {1, 1}, {2, 3}});

  // Total volume is the bounding box of the house either way.
  auto volume = cc->property_map<CellComplex::Vertex, double>("c:volume");
  double v = 0.0;
  for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c)
    v += volume[*c];
  CHECK(v == Approx(8.0 * 6.0 * 5.3));
}

TEST_CASE("StoreyArrangement_LevelBetweenSections_FacesSharedConsistently",
          "[geometry][cellcomplex][storey_local]") {
  auto house = make_house();
  auto cc = build(house);
  auto area = cc->property_map<CellComplex::Vertex, double>("f:area");
  auto section = cc->property_map<CellComplex::Vertex, int>("c:section");

  // z = 2.8: the 1-cell slab under the 3-cell upper floor -> 3 faces, each
  // with the slab cell below and one upper cell above.
  // z = 2.5: the 3-cell ground floor under the slab -> 3 faces.
  for (double z : {2.5, 2.8}) {
    INFO("level z = " << z);
    int faces = 0;
    double total = 0.0;
    for (auto f = cc->faces_begin(); f != cc->faces_end(); ++f) {
      if (!is_horizontal_face_at(*cc, *f, z))
        continue;
      ++faces;
      total += area[*f];
      REQUIRE(cells_of_face(*cc, *f) == 2);
      // One cell below, one above, in adjacent sections; get_a / get_b
      // resolve (the Solidifier needs both sides).
      const auto a = cc->get_a(*f);
      const auto b = cc->get_b(*f);
      CHECK(std::abs(section[a] - section[b]) == 1);
      CHECK(cc->get_a(*f) != cc->get_b(*f));
    }
    CHECK(faces == 3);
    CHECK(total == Approx(48.0));
  }

  // Each cell's top faces cover exactly its own footprint.
  auto volume = cc->property_map<CellComplex::Vertex, double>("c:volume");
  for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c) {
    const double zc = (*cc)[*c].pos.z();
    double top = 0.0, bottom = 0.0, zt = -1e9, zb = 1e9;
    for (auto f = cc->faces_of_cell_begin(*c); f != cc->faces_of_cell_end(*c);
         ++f) {
      const double z = (*cc)[*f].pos.z();
      bool horizontal = true;
      for (auto v = cc->vertices_of_face_begin(*f);
           v != cc->vertices_of_face_end(*f); ++v)
        horizontal &= std::abs((*cc)[*v].pos.z() - z) < 1e-9;
      if (!horizontal)
        continue;
      if (z > zc) {
        top += area[*f];
        zt = z;
      } else {
        bottom += area[*f];
        zb = z;
      }
    }
    CHECK(top == Approx(bottom));
    CHECK(volume[*c] == Approx(top * (zt - zb)));
  }
}

TEST_CASE("StoreyArrangement_AllCellsSolid_MeshIsWatertight",
          "[geometry][cellcomplex][storey_local][mesh]") {
  // With every cell inside, toMesh emits only the house's outer boundary.
  // The slab's outer wall faces meet finer upper-floor faces along z = 2.8
  // and finer ground-floor faces along z = 2.5: without the extra vertices on
  // those edges (or with fan triangles over collinear corners) the mesh has
  // T-junctions, i.e. edges used once.
  auto house = make_house();
  std::shared_ptr<const CellComplex> cc = build(house);
  Solidifier solidifier(cc);
  auto [V, F] = solidifier.toMesh([](CellComplex::Vertex) { return true; });
  REQUIRE(F.rows() > 0);

  for (const auto &[edge, n] : edge_uses(F)) {
    INFO("edge " << edge.first << "-" << edge.second);
    CHECK(n == 2);
  }
  for (int i = 0; i < F.rows(); ++i) {
    const Eigen::Vector3d a = V.row(F(i, 0)), b = V.row(F(i, 1)),
                          c = V.row(F(i, 2));
    CHECK((b - a).cross(c - a).norm() > 1e-9);
  }
  // Surface area of the 8 x 6 x 5.3 box.
  double surface = 0.0;
  for (int i = 0; i < F.rows(); ++i) {
    const Eigen::Vector3d a = V.row(F(i, 0)), b = V.row(F(i, 1)),
                          c = V.row(F(i, 2));
    surface += 0.5 * (b - a).cross(c - a).norm();
  }
  CHECK(surface == Approx(2 * (8 * 6 + 8 * 5.3 + 6 * 5.3)));
}

TEST_CASE("StoreyArrangement_SingleStorey_UnchangedByZRanges",
          "[geometry][cellcomplex][storey_local]") {
  // One storey with thin slabs above and below it (what make_pairs invents
  // for an unpaired floor and ceiling): the slabs reach no wall, so they take
  // the storey's walls, and the complex is the legacy one, node for node.
  auto make = [](bool ranges) {
    House h;
    const Range storey{0.02, 2.48};
    for (auto p :
         {Eigen::Vector4d(1, 0, 0, 0), Eigen::Vector4d(1, 0, 0, -8),
          Eigen::Vector4d(0, 1, 0, 0), Eigen::Vector4d(0, 1, 0, -6),
          Eigen::Vector4d(1, 0, 0, -3), Eigen::Vector4d(0, 1, 0, -2)}) {
      h.planes.push_back(p);
      h.verticals.push_back(h.planes.size() - 1);
      h.z_ranges.push_back(storey);
    }
    for (double z : {-0.25, 0.0, 2.5, 2.75}) {
      h.planes.emplace_back(0, 0, 1, -z);
      h.horizontals.push_back(h.planes.size() - 1);
      h.z_ranges.push_back({kNaN, kNaN});
    }
    h.pairs = {{0, 1}};
    if (!ranges)
      h.z_ranges.clear();
    return h;
  };
  auto a = make(false), b = make(true);
  auto ca = build(a), cb = build(b);
  REQUIRE(ca->num_cells() == 2 * 2 * 3); // 2 x 2 faces, 3 sections
  REQUIRE(cb->num_cells() == ca->num_cells());
  REQUIRE(cb->num_faces() == ca->num_faces());
  REQUIRE(cb->num_vertices() == ca->num_vertices());
  REQUIRE(boost::num_vertices(*cb) == boost::num_vertices(*ca));
  REQUIRE(boost::num_edges(*cb) == boost::num_edges(*ca));
  for (size_t i = 0; i < boost::num_vertices(*ca); ++i) {
    INFO("node " << i);
    REQUIRE((*cb)[i].type == (*ca)[i].type);
    REQUIRE((*cb)[i].id == (*ca)[i].id);
    REQUIRE(((*cb)[i].pos - (*ca)[i].pos).norm() < 1e-12);
  }
  for (auto c = ca->cells_begin(); c != ca->cells_end(); ++c)
    CHECK(std::get<CellData>((*ca)[*c].data).wall_ids ==
          std::get<CellData>((*cb)[*c].data).wall_ids);
}

TEST_CASE("StoreyArrangement_WallsOutsideASection_DoNotCountItsWalls",
          "[geometry][cellcomplex][storey_local]") {
  // A wall that only exists upstairs carries no wall id downstairs, even
  // where a big ground-floor cell's centre falls between its two faces.
  House h;
  const Range full{0.0, 5.3}, upper{2.8, 5.3};
  auto wall = [&](Eigen::Vector4d p, Range r) {
    h.planes.push_back(p);
    h.verticals.push_back(h.planes.size() - 1);
    h.z_ranges.push_back(r);
  };
  wall({1, 0, 0, 0}, full);
  wall({1, 0, 0, -8}, full);
  wall({0, 1, 0, 0}, full);
  wall({0, 1, 0, -6}, full);
  // A wall's faces point away from its core: x = 3.9 faces -x, x = 4.1
  // faces +x, so the core (3.9 < x < 4.1) is behind both.
  wall({-1, 0, 0, 3.9}, upper);
  wall({1, 0, 0, -4.1}, upper);
  h.pairs = {{4, 5}};
  for (double z : {0.0, 2.5, 2.8, 5.3}) {
    h.planes.emplace_back(0, 0, 1, -z);
    h.horizontals.push_back(h.planes.size() - 1);
    h.z_ranges.push_back({kNaN, kNaN});
  }
  auto cc = build(h);
  auto section = cc->property_map<CellComplex::Vertex, int>("c:section");
  int upper_wall_cells = 0;
  for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c) {
    const auto &ids = std::get<CellData>((*cc)[*c].data).wall_ids;
    if (section[*c] < 2)
      CHECK(ids.empty());
    else
      upper_wall_cells += static_cast<int>(ids.size());
  }
  CHECK(upper_wall_cells == 1); // the 0.2 m cell between the two faces
}

TEST_CASE("StoreyArrangement_SectionWithoutFaces_TakesNearestWalls",
          "[geometry][cellcomplex][storey_local]") {
  // A storey [0, 2.5], a slab [2.5, 2.75] no wall reaches, and an attic
  // [2.75, 4] with one wall of its own (its roof is oblique). Neither the
  // slab nor the attic encloses a face alone. The slab takes the storey's
  // walls (plus the attic wall), not every wall: the stray wall at x = 4,
  // whose range reaches no section, does not cut it. The attic, thick
  // enough to be a room, takes every wall while the cell count allows
  // (reusex-e6z), and the nearest walls when it does not.
  House h;
  auto wall = [&](Eigen::Vector4d p, Range r) {
    h.planes.push_back(p);
    h.verticals.push_back(h.planes.size() - 1);
    h.z_ranges.push_back(r);
  };
  const Range storey{0.0, 2.5};
  wall({1, 0, 0, 0}, storey);
  wall({1, 0, 0, -8}, storey);
  wall({0, 1, 0, 0}, storey);
  wall({0, 1, 0, -6}, storey);
  wall({1, 0, 0, -2}, storey);
  wall({1, 0, 0, -6}, {2.75, 4.0});  // the attic's own wall
  wall({1, 0, 0, -4}, {10.0, 12.0}); // reaches no section
  for (double z : {0.0, 2.5, 2.75, 4.0}) {
    h.planes.emplace_back(0, 0, 1, -z);
    h.horizontals.push_back(h.planes.size() - 1);
    h.z_ranges.push_back({kNaN, kNaN});
  }
  auto count = [&h](size_t max_cells) {
    auto copy = h;
    auto cc = build(copy, max_cells);
    auto section = cc->property_map<CellComplex::Vertex, int>("c:section");
    std::map<int, int> per_section;
    for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c)
      ++per_section[section[*c]];
    return per_section;
  };
  // Storey: x = 0, 2, 8 -> 2 faces. Slab: x = 0, 2, 6, 8 -> 3. Attic: every
  // wall, x = 0, 2, 4, 6, 8 -> 4.
  CHECK(count(5000) == std::map<int, int>{{0, 2}, {1, 3}, {2, 4}});
  // 9 cells would exceed a limit of 8: the attic keeps the nearest walls.
  CHECK(count(8) == std::map<int, int>{{0, 2}, {1, 3}, {2, 3}});
}

namespace {

// A one-storey 8 x 6 m house [0, 2.5] (split by a wall at x = 4) under a
// pitched roof (reusex-e6z): the ceiling slab [2.5, 2.75] and an attic
// [2.75, 4.0] whose only walls of their own are the two gables (y = 0,
// y = 6). The roof planes rise at 45 degrees from the eaves at x = 0 and
// x = 8 (z = 2.75) towards the ridge, so at the attic's mid-height
// (z = 3.375) they stand at x = 0.625 and x = 7.375.
struct AtticHouse : House {
  size_t roof_left = 0, roof_right = 0;
  std::vector<size_t> obliques;
};

AtticHouse make_attic_house() {
  AtticHouse h;
  auto wall = [&](Eigen::Vector4d p, Range r) {
    h.planes.push_back(p);
    h.verticals.push_back(h.planes.size() - 1);
    h.z_ranges.push_back(r);
  };
  const Range storey{0.0, 2.5}, gable{0.0, 4.0};
  wall({1, 0, 0, 0}, storey);
  wall({1, 0, 0, -8}, storey);
  wall({1, 0, 0, -4}, storey);
  wall({0, 1, 0, 0}, gable);
  wall({0, 1, 0, -6}, gable);
  const double s = std::sqrt(0.5);
  // Left roof: z - 2.75 = x, normal pointing down into the attic.
  h.planes.emplace_back(s, 0, -s, 2.75 * s);
  h.roof_left = h.planes.size() - 1;
  h.z_ranges.push_back({2.75, 4.0});
  // Right roof: z - 2.75 = 8 - x.
  h.planes.emplace_back(-s, 0, -s, 10.75 * s);
  h.roof_right = h.planes.size() - 1;
  h.z_ranges.push_back({2.75, 4.0});
  h.obliques = {h.roof_left, h.roof_right};
  for (double z : {0.0, 2.5, 2.75, 4.0}) {
    h.planes.emplace_back(0, 0, 1, -z);
    h.horizontals.push_back(h.planes.size() - 1);
    h.z_ranges.push_back({kNaN, kNaN});
  }
  return h;
}

std::shared_ptr<CellComplex> build_attic(AtticHouse &h, bool with_roof) {
  return std::make_shared<CellComplex>(
      h.planes, h.verticals, h.horizontals, h.pairs,
      std::array<double, 2>{-1.0, -1.0}, std::array<double, 2>{9.0, 7.0},
      std::nullopt, 5000, h.z_ranges,
      CellComplex::default_min_section_overlap_m,
      with_roof ? h.obliques : std::vector<size_t>{});
}

} // namespace

TEST_CASE("StoreyArrangement_AtticUnderObliqueRoof_TakesRoofLines",
          "[geometry][cellcomplex][storey_local][attic]") {
  // The attic encloses no face with its gables alone, so it takes the
  // storey's walls (x = 0, 4, 8). Those put its outer cells under the eaves,
  // where the roof leaves no room, and nothing marks where the roof is: the
  // MIP found no attic it could select on house2 (reusex-e6z). The roof
  // planes that reach the attic now cut it where they cross its mid-height.
  auto h = make_attic_house();
  auto without = build_attic(h, false);
  auto section0 = without->property_map<CellComplex::Vertex, int>("c:section");
  std::map<int, int> before;
  for (auto c = without->cells_begin(); c != without->cells_end(); ++c)
    ++before[section0[*c]];
  CHECK(before == std::map<int, int>{{0, 2}, {1, 2}, {2, 2}});

  auto h2 = make_attic_house();
  auto cc = build_attic(h2, true);
  auto section = cc->property_map<CellComplex::Vertex, int>("c:section");
  std::map<int, int> per_section;
  std::vector<double> attic_xs;
  for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c) {
    ++per_section[section[*c]];
    if (section[*c] == 2)
      attic_xs.push_back((*cc)[*c].pos.x());
  }
  // Storey and slab unchanged (the roof does not reach the slab); the attic
  // is cut at x = 0.625 and x = 7.375: 0 | 0.625 | 4 | 7.375 | 8.
  CHECK(per_section == std::map<int, int>{{0, 2}, {1, 2}, {2, 4}});

  // The attic's wall faces on the roof lines carry the roof planes, so face
  // coverage reads the roof's inliers there.
  std::set<int> roof_faces;
  for (auto f = cc->faces_begin(); f != cc->faces_end(); ++f) {
    const int id = std::get<FaceData>((*cc)[*f].data).plane_id;
    if (id != static_cast<int>(h2.roof_left) &&
        id != static_cast<int>(h2.roof_right))
      continue;
    roof_faces.insert(id);
    const double x = (*cc)[*f].pos.x();
    CHECK((std::abs(x - 0.625) < 1e-6 || std::abs(x - 7.375) < 1e-6));
    CHECK((*cc)[*f].pos.z() == Approx(3.375));
  }
  CHECK(roof_faces.size() == 2);

  // Still watertight with every cell solid.
  std::shared_ptr<const CellComplex> ccc = cc;
  Solidifier solidifier(ccc);
  auto [V, F] = solidifier.toMesh([](CellComplex::Vertex) { return true; });
  for (const auto &[edge, n] : edge_uses(F))
    CHECK(n == 2);
}

namespace {

// A scanned one-storey house with a pitched-roof attic, for a full solve:
// planes with their invented back faces (wall pairs), inliers, z-ranges, and
// a labelled cloud (storey = room 1, attic = room 2). The same geometry as
// make_attic_house(), without the interior wall, plus a flat collar ceiling
// at z = 4 between the roof slopes (x = 1.25 .. 6.75).
struct ScannedAttic {
  PlaneVec planes;
  std::vector<size_t> verticals, horizontals, obliques;
  std::vector<std::pair<size_t, size_t>> pairs;
  std::vector<Range> z_ranges;
  std::vector<reusex::IndicesPtr> inliers;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud{
      new pcl::PointCloud<pcl::PointXYZRGB>};
  pcl::PointCloud<pcl::Normal>::Ptr normals{new pcl::PointCloud<pcl::Normal>};
  pcl::PointCloud<pcl::Label>::Ptr labels{new pcl::PointCloud<pcl::Label>};

  size_t add_plane(Eigen::Vector4d p, Range r, std::vector<size_t> &kind) {
    planes.push_back(p);
    z_ranges.push_back(r);
    inliers.push_back(std::make_shared<reusex::Indices>());
    kind.push_back(planes.size() - 1);
    return planes.size() - 1;
  }
  // A face and its invented back face 0.25 m behind it.
  void add_pair(Eigen::Vector4d p, Range r, std::vector<size_t> &kind) {
    const size_t a = add_plane(p, r, kind);
    const size_t b =
        add_plane(Eigen::Vector4d(-p[0], -p[1], -p[2], -p[3] - 0.25), r, kind);
    pairs.emplace_back(a, b);
  }
  void add_point(size_t plane, const Eigen::Vector3d &x, std::uint32_t room) {
    pcl::PointXYZRGB q;
    q.x = static_cast<float>(x.x());
    q.y = static_cast<float>(x.y());
    q.z = static_cast<float>(x.z());
    cloud->push_back(q);
    const auto &n = planes[plane];
    normals->push_back(pcl::Normal(static_cast<float>(n[0]),
                                   static_cast<float>(n[1]),
                                   static_cast<float>(n[2])));
    pcl::Label l;
    l.label = room;
    labels->push_back(l);
    inliers[plane]->push_back(static_cast<int>(cloud->size() - 1));
  }
};

ScannedAttic make_scanned_attic() {
  ScannedAttic h;
  const double s = std::sqrt(0.5);
  const Range storey{0.0, 2.5}, gable{0.0, 4.0}, roof{2.75, 4.0};
  // Inner faces, normals into the building.
  h.add_pair({1, 0, 0, 0}, storey, h.verticals);        // 0: x = 0
  h.add_pair({-1, 0, 0, 8}, storey, h.verticals);       // 2: x = 8
  h.add_pair({0, 1, 0, 0}, gable, h.verticals);         // 4: y = 0
  h.add_pair({0, -1, 0, 6}, gable, h.verticals);        // 6: y = 6
  h.add_pair({s, 0, -s, 2.75 * s}, roof, h.obliques);   // 8: left roof
  h.add_pair({-s, 0, -s, 10.75 * s}, roof, h.obliques); // 10: right roof
  const Range none{kNaN, kNaN};
  h.add_plane({0, 0, 1, 0}, none, h.horizontals);     // 12: floor
  h.add_plane({0, 0, -1, 2.5}, none, h.horizontals);  // 13: storey ceiling
  h.add_plane({0, 0, 1, -2.75}, none, h.horizontals); // 14: attic floor
  h.add_plane({0, 0, -1, 4.0}, none, h.horizontals);  // 15: collar ceiling

  const double st = 0.1;
  for (double x = 0; x <= 8 + 1e-9; x += st)
    for (double y = 0; y <= 6 + 1e-9; y += st) {
      h.add_point(12, {x, y, 0.0}, 1);
      h.add_point(13, {x, y, 2.5}, 1);
      h.add_point(14, {x, y, 2.75}, 2);
      if (x >= 1.25 && x <= 6.75)
        h.add_point(15, {x, y, 4.0}, 2);
      if (x <= 1.25)
        h.add_point(8, {x, y, x + 2.75}, 2);
      if (x >= 6.75)
        h.add_point(10, {x, y, 10.75 - x}, 2);
    }
  for (double z = 0; z <= 2.5 + 1e-9; z += st) {
    for (double y = 0; y <= 6 + 1e-9; y += st) {
      h.add_point(0, {0.0, y, z}, 1);
      h.add_point(2, {8.0, y, z}, 1);
    }
    for (double x = 0; x <= 8 + 1e-9; x += st) {
      h.add_point(4, {x, 0.0, z}, 1);
      h.add_point(6, {x, 6.0, z}, 1);
    }
  }
  // The gables above the attic floor, inside the roof profile.
  for (double z = 2.75; z <= 4.0 + 1e-9; z += st)
    for (double x = z - 2.75; x <= 10.75 - z + 1e-9; x += st) {
      h.add_point(4, {x, 0.0, z}, 2);
      h.add_point(6, {x, 6.0, z}, 2);
    }
  return h;
}

} // namespace

TEST_CASE("StoreyArrangement_AtticUnderObliqueRoof_IsSelectedUnderTheRoof",
          "[geometry][cellcomplex][storey_local][attic][solidifier]") {
  // The attic is the scan's second room. Solved per section, it is selected
  // and ends at the roof lines (x = 0.625 / 7.375 at its mid-height), not
  // at the storey's outer walls under the eaves (reusex-e6z).
  auto h = make_scanned_attic();
  auto cc = std::make_shared<CellComplex>(
      h.planes, h.verticals, h.horizontals, h.pairs,
      std::array<double, 2>{-1.0, -1.0}, std::array<double, 2>{9.0, 7.0},
      std::nullopt, 5000, h.z_ranges,
      CellComplex::default_min_section_overlap_m, h.obliques);
  REQUIRE(cc->n_sections == 3);
  cc->compute_room_probabilities<pcl::PointXYZRGB, pcl::Normal, pcl::Label>(
      h.cloud, h.normals, h.labels);
  cc->compute_face_coverage<pcl::PointXYZRGB>(h.cloud, h.planes, h.inliers);

  SolidifierOptions opt;
  opt.sectioned_threshold = 0; // per section, like a multi-storey scan
  opt.solver = SolverChoice::highs;
  Solidifier solidifier(cc, opt);
  const auto result = solidifier.solve();
  REQUIRE(result.has_value());
  const auto &rooms = result->first;

  auto section = cc->property_map<CellComplex::Vertex, int>("c:section");
  auto volume = cc->property_map<CellComplex::Vertex, double>("c:volume");
  double attic = 0.0, storey = 0.0;
  for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c) {
    if (rooms.at(*c) <= 0)
      continue;
    const double x = (*cc)[*c].pos.x();
    if (section[*c] == 2) {
      attic += volume[*c];
      CHECK(x > 0.625);
      CHECK(x < 7.375);
    } else if (section[*c] == 0) {
      storey += volume[*c];
    }
  }
  CHECK(storey == Approx(8.0 * 6.0 * 2.5));
  // Between the roof lines at mid-height: 6.75 x 6 x 1.25 m.
  CHECK(attic == Approx(6.75 * 6.0 * 1.25));
}

TEST_CASE("StoreyArrangement_ProjectedCellsGuard_UsesStoreyLocalCount",
          "[geometry][cellcomplex][storey_local]") {
  // 27 legacy cells would trip a 10-cell limit; the 7 storey-local cells
  // do not.
  auto legacy = make_house(false);
  CHECK_THROWS(build(legacy, 10));
  auto house = make_house();
  CHECK_NOTHROW(build(house, 10));
}

TEST_CASE("SelectSectionPlanes_KeepsSplitLevelAndAttic_DropsOutsideBuilding",
          "[geometry][mesh][storey_sections]") {
  // Two storeys [0, 2.5] and [2.75, 5.25] from the storeys table, a split
  // level at 1.2 m (with its invented underside at 0.95), an attic ceiling
  // at 6.5 m under a roof that reaches 6.4 m, a plane at 9 m and one at
  // -3 m that nothing of the building reaches (on NewOffice: tilted planes
  // force_orthogonal_planes moved out of the building), and the invented
  // faces of the outermost floor (-0.25) and attic ceiling (6.75). Only the
  // horizontals outside the building go (reusex-x8s).
  reusex::EigenVectorContainer<double, 4> planes;
  std::vector<reusex::IndicesPtr> inliers;
  std::vector<Range> z_ranges;
  std::vector<size_t> horizontals;
  auto horizontal = [&](double z, bool measured) {
    planes.emplace_back(0, 0, 1, -z);
    inliers.push_back(std::make_shared<reusex::Indices>(
        measured ? reusex::Indices{0} : reusex::Indices{}));
    z_ranges.push_back({kNaN, kNaN});
    horizontals.push_back(planes.size() - 1);
  };
  for (double z : {0.0, 2.5, 2.75, 5.25, 1.2, 6.5, 9.0, -3.0})
    horizontal(z, true); // 0..7
  for (double z : {-0.25, 6.75, 0.95})
    horizontal(z, false);          // 8..10
  planes.emplace_back(1, 0, 0, 0); // a wall over both storeys
  inliers.push_back(std::make_shared<reusex::Indices>(reusex::Indices{0}));
  z_ranges.push_back({0.0, 5.25});
  const double s = std::sqrt(0.5);
  planes.emplace_back(s, 0, -s, 0); // the roof, up to 6.4 m
  inliers.push_back(std::make_shared<reusex::Indices>(reusex::Indices{0}));
  z_ranges.push_back({5.25, 6.4});
  const std::vector<double> heights{0.05, 2.45, 2.8, 5.2};

  SECTION("split level, attic and inner invented faces stay") {
    const auto got = select_section_planes(planes, horizontals, heights, 0.15,
                                           z_ranges, inliers);
    CHECK(got == std::vector<size_t>{0, 1, 2, 3, 4, 5, 10});
  }
  SECTION("without the roof's range the attic ceiling is outside too") {
    auto no_roof = z_ranges;
    no_roof.back() = {kNaN, kNaN};
    const auto got = select_section_planes(planes, horizontals, heights, 0.15,
                                           no_roof, inliers);
    CHECK(got == std::vector<size_t>{0, 1, 2, 3, 4, 10});
  }
  SECTION("no storey heights: every horizontal stays") {
    CHECK(select_section_planes(planes, horizontals, {}, 0.15, z_ranges,
                                inliers) == horizontals);
  }
  SECTION("fewer than two planes inside: every horizontal stays") {
    std::vector<size_t> two{6, 7}; // 9 m and -3 m only
    CHECK(select_section_planes(planes, two, heights, 0.15, z_ranges,
                                inliers) == two);
  }
}

TEST_CASE("WallZRanges_FromInliers_InventedFaceTakesPartnerRange",
          "[geometry][mesh][storey_local]") {
  auto cloud = std::make_shared<reusex::Cloud>();
  auto add = [&](double x, double z) {
    reusex::PointT p;
    p.x = static_cast<float>(x);
    p.y = 1.0F;
    p.z = static_cast<float>(z);
    cloud->push_back(p);
    return static_cast<int>(cloud->size() - 1);
  };
  reusex::EigenVectorContainer<double, 4> planes;
  std::vector<reusex::IndicesPtr> inliers;
  // Plane 0: a wall from z = 0 to 2.5 with one stray point at 4.0.
  planes.emplace_back(1, 0, 0, 0);
  inliers.push_back(std::make_shared<reusex::Indices>());
  for (int i = 0; i <= 200; ++i)
    inliers.back()->push_back(add(0.0, 2.5 * i / 200.0));
  inliers.back()->push_back(add(0.0, 4.0));
  // Plane 1: its invented back face (no inliers).
  planes.emplace_back(-1, 0, 0, -0.25);
  inliers.push_back(std::make_shared<reusex::Indices>());
  // Plane 2: a floor, not a wall.
  planes.emplace_back(0, 0, 1, 0);
  inliers.push_back(std::make_shared<reusex::Indices>(reusex::Indices{0}));
  // Plane 3: a wall without inliers or partner.
  planes.emplace_back(0, 1, 0, -3);
  inliers.push_back(std::make_shared<reusex::Indices>());

  const auto r = wall_z_ranges(planes, inliers, cloud, {{0, 1}});
  REQUIRE(r.size() == 4);
  CHECK(r[0][0] == Approx(0.0).margin(0.06));
  CHECK(r[0][1] == Approx(2.5).margin(0.06)); // the stray point is trimmed
  CHECK(r[1][0] == r[0][0]);
  CHECK(r[1][1] == r[0][1]);
  CHECK(std::isnan(r[2][0])); // not a wall: spans every section
  CHECK(std::isnan(r[3][0])); // no evidence: spans every section
}
