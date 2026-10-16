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
//   - select_section_planes() keeps storey floors/ceilings, drops a table
//     top, and leaves the horizontals alone without (matching) storeys;
//   - wall_z_ranges() measures a wall's height from its inliers and gives an
//     invented back face its partner's range.

#include <reusex/reconstruction/CellComplex.hpp>
#include <reusex/reconstruction/Solidifier.hpp>
#include <reusex/reconstruction/mesh.hpp>

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
  // slab nor the attic encloses a face alone, so both take the storey's
  // walls (plus the attic wall), not every wall: the stray wall at x = 4,
  // whose range reaches no section, cuts nothing.
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
  auto cc = build(h);
  auto section = cc->property_map<CellComplex::Vertex, int>("c:section");
  std::map<int, int> per_section;
  for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c)
    ++per_section[section[*c]];
  // Storey: x = 0, 2, 8 -> 2 faces. Slab and attic: x = 0, 2, 6, 8 -> 3.
  CHECK(per_section == std::map<int, int>{{0, 2}, {1, 3}, {2, 3}});
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

TEST_CASE("SelectSectionPlanes_KeepsStoreyLevels_DropsTableTop",
          "[geometry][mesh][storey_sections]") {
  reusex::EigenVectorContainer<double, 4> planes;
  for (double z : {0.0, 0.75, 2.6, 2.85, 5.2})
    planes.emplace_back(0, 0, 1, -z);
  planes.emplace_back(0, 0, -1, 2.6); // ceiling face, normal down
  const std::vector<size_t> horizontals{0, 1, 2, 3, 4, 5};

  SECTION("storey heights pick floors and ceilings within tolerance") {
    const auto got = select_section_planes(planes, horizontals,
                                           {0.05, 2.55, 2.9, 5.25}, 0.15);
    CHECK(got == std::vector<size_t>{0, 2, 3, 4, 5});
  }
  SECTION("no storey heights: every horizontal stays") {
    CHECK(select_section_planes(planes, horizontals, {}, 0.15) == horizontals);
  }
  SECTION("heights matching fewer than two planes: every horizontal stays") {
    CHECK(select_section_planes(planes, horizontals, {10.0, 20.0}, 0.15) ==
          horizontals);
    CHECK(select_section_planes(planes, horizontals, {0.75}, 0.15) ==
          horizontals);
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
