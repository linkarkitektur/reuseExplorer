// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/class_role_map.hpp>
#include <reusex/core/label_semantics.hpp>
#include <reusex/geometry/level_frame.hpp>
#include <reusex/segmentation/classify_planes.hpp>
#include <reusex/types/point_types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Approx;

namespace {

/// API classes of the synthetic prompt list (point label = API + 1).
enum Api : int {
  kWall = 0,
  kFloor = 1,
  kCeiling = 2,
  kTable = 3,
  kCabinet = 4
};

/// Synthetic scan: planar patches in a "true" level frame (z up, storey-0
/// floor at z = 0), moved to world by `pose`. Each patch is one plane label
/// (k + 1) and may carry a per-point semantic API class (-1 = background).
struct Scene {
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  /// Point spacing; finer than the default 0.1 m area cell.
  double step = 0.05;

  struct Patch {
    std::vector<Eigen::Vector3d> pts;
    Eigen::Vector3d normal;
    std::vector<int> api; ///< per point, -1 = unlabeled
  };
  std::vector<Patch> patches;

  /// Horizontal rectangle at height z. Its normal faces the sensor
  /// (CONTRACTS.md): up for a surface seen from above, down (`down`) for one
  /// seen from below.
  size_t horizontal(double x0, double x1, double y0, double y1, double z,
                    int api = -1, bool down = false) {
    Patch p;
    for (double x = x0; x <= x1 + 1e-9; x += step)
      for (double y = y0; y <= y1 + 1e-9; y += step)
        p.pts.emplace_back(x, y, z);
    p.normal = Eigen::Vector3d(0, 0, down ? -1 : 1);
    p.api.assign(p.pts.size(), api);
    patches.push_back(std::move(p));
    return patches.size() - 1;
  }

  /// Vertical patch x = const.
  size_t wall_x(double x, double y0, double y1, double z0, double z1,
                int api = -1) {
    Patch p;
    for (double y = y0; y <= y1 + 1e-9; y += step)
      for (double z = z0; z <= z1 + 1e-9; z += step)
        p.pts.emplace_back(x, y, z);
    p.normal = Eigen::Vector3d::UnitX();
    p.api.assign(p.pts.size(), api);
    patches.push_back(std::move(p));
    return patches.size() - 1;
  }

  /// Vertical patch y = const.
  size_t wall_y(double y, double x0, double x1, double z0, double z1,
                int api = -1) {
    Patch p;
    for (double x = x0; x <= x1 + 1e-9; x += step)
      for (double z = z0; z <= z1 + 1e-9; z += step)
        p.pts.emplace_back(x, y, z);
    p.normal = Eigen::Vector3d::UnitY();
    p.api.assign(p.pts.size(), api);
    patches.push_back(std::move(p));
    return patches.size() - 1;
  }

  /// Plane tilted 45 deg about the y axis, through (x0, *, z0), seen from
  /// below (normal facing down).
  size_t slanted(double x0, double y0, double y1, double z0, double run) {
    Patch p;
    for (double s = 0; s <= run + 1e-9; s += step)
      for (double y = y0; y <= y1 + 1e-9; y += step)
        p.pts.emplace_back(x0 + s, y, z0 + s);
    p.normal = Eigen::Vector3d(1, 0, -1).normalized();
    p.api.assign(p.pts.size(), -1);
    patches.push_back(std::move(p));
    return patches.size() - 1;
  }

  /// A room [0,5]x[0,4], floor at z0, ceiling at z0 + 2.6, four walls and a
  /// 1 x 2 m table top 0.75 m above the floor. Returns the patch indices
  /// {floor, ceiling, wall x0, wall x5, wall y0, wall y4, table}.
  std::vector<size_t> room(double z0, bool label = false) {
    const double zc = z0 + 2.6;
    std::vector<size_t> ids;
    ids.push_back(horizontal(0, 5, 0, 4, z0, label ? kFloor : -1));
    ids.push_back(horizontal(0, 5, 0, 4, zc, label ? kCeiling : -1, true));
    ids.push_back(wall_x(0, 0, 4, z0, zc, label ? kWall : -1));
    ids.push_back(wall_x(5, 0, 4, z0, zc, label ? kWall : -1));
    ids.push_back(wall_y(0, 0, 5, z0, zc, label ? kWall : -1));
    ids.push_back(wall_y(4, 0, 5, z0, zc, label ? kWall : -1));
    ids.push_back(horizontal(2, 3, 1, 3, z0 + 0.75, label ? kTable : -1));
    return ids;
  }

  struct Clouds {
    Cloud cloud;
    CloudL planes;
    CloudN normals;
    CloudL labels;
  };

  [[nodiscard]] Clouds build() const {
    Clouds c;
    for (size_t k = 0; k < patches.size(); ++k) {
      for (size_t i = 0; i < patches[k].pts.size(); ++i) {
        const Eigen::Vector3d w = pose * patches[k].pts[i];
        PointT pt;
        pt.x = static_cast<float>(w.x());
        pt.y = static_cast<float>(w.y());
        pt.z = static_cast<float>(w.z());
        c.cloud.push_back(pt);
        LabelT l;
        l.label = static_cast<uint32_t>(k + 1);
        c.planes.push_back(l);
        LabelT s;
        s.label = core::api_to_point_label(patches[k].api[i]);
        c.labels.push_back(s);
      }
      const Eigen::Vector3d n = pose.linear() * patches[k].normal;
      NormalT nt;
      nt.normal_x = static_cast<float>(n.x());
      nt.normal_y = static_cast<float>(n.y());
      nt.normal_z = static_cast<float>(n.z());
      c.normals.push_back(nt);
    }
    return c;
  }
};

SemanticLabels semantic_of(const CloudL &labels) {
  SemanticLabels s;
  s.labels = &labels;
  s.class_names = {{core::api_to_point_label(kWall), "wall"},
                   {core::api_to_point_label(kFloor), "floor"},
                   {core::api_to_point_label(kCeiling), "Ceiling"},
                   {core::api_to_point_label(kTable), "table"},
                   {core::api_to_point_label(kCabinet), "cabinet"}};
  return s;
}

core::StoreyRecord storey(int idx, double z_floor, double z_ceiling) {
  core::StoreyRecord s;
  s.idx = idx;
  s.z_floor = z_floor;
  s.z_ceiling = z_ceiling;
  return s;
}

const core::PlaneClassRecord &rec(const ClassifyPlanesResult &r, size_t patch) {
  return r.planes.at(patch);
}

} // namespace

// ── class -> role map ─────────────────────────────────────────────────────

TEST_CASE("DefaultClassRoleMap_BuiltIn_MapsStructureAndFurniture",
          "[classify_planes]") {
  const auto &m = core::default_class_role_map();
  CHECK(m.role_of("wall") == core::PlaneRole::wall);
  CHECK(m.role_of("floor") == core::PlaneRole::floor);
  CHECK(m.role_of("ceiling") == core::PlaneRole::ceiling);
  CHECK(m.role_of(" Table ") == core::PlaneRole::furniture);
  CHECK(m.role_of("cabinet") == core::PlaneRole::furniture);
  CHECK_FALSE(m.role_of("column").has_value());
  CHECK(
      core::parse_class_role_map(core::default_class_role_map_json()).size() ==
      m.size());
}

TEST_CASE("ParseClassRoleMap_InvalidInput_Throws", "[classify_planes]") {
  CHECK_THROWS_AS(core::parse_class_role_map("not json"),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::parse_class_role_map(R"({"version":1})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(
      core::parse_class_role_map(R"({"classes":{"wall":"partition"}})"),
      std::invalid_argument);
  CHECK_THROWS_AS(core::parse_class_role_map(
                      R"({"classes":{"Wall":"wall","wall":"wall"}})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::parse_class_role_map(R"({"version":2,"classes":{}})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::load_class_role_map("/nonexistent/role-map.json"),
                  std::runtime_error);
}

TEST_CASE("ParseClassRoleMap_UserMap_OverridesRoles", "[classify_planes]") {
  const auto m = core::parse_class_role_map(
      R"({"version":1,"classes":{"Column":"wall","table":"clutter"}})");
  CHECK(m.size() == 2);
  CHECK(m.role_of("column") == core::PlaneRole::wall);
  CHECK(m.role_of("TABLE") == core::PlaneRole::clutter);
  CHECK_FALSE(m.role_of("wall").has_value());
  CHECK(core::to_string(core::PlaneRole::furniture) == "furniture");
  CHECK(core::plane_role_from_string("slab") == core::PlaneRole::slab);
  CHECK_FALSE(core::plane_role_from_string("Slab").has_value());
}

// ── rules ─────────────────────────────────────────────────────────────────

TEST_CASE("ClassifyPlanes_UnlabeledRoomNoStoreys_RulesGiveRoles",
          "[classify_planes]") {
  Scene sc;
  const auto ids = sc.room(0.0);
  const auto c = sc.build();
  const auto r = classify_planes(c.cloud, c.planes, c.normals, {});

  REQUIRE(r.planes.size() == 7);
  CHECK(rec(r, ids[0]).role == "floor");
  CHECK(rec(r, ids[1]).role == "ceiling");
  for (size_t w = 2; w <= 5; ++w) {
    CHECK(rec(r, ids[w]).role == "wall");
    CHECK(rec(r, ids[w]).orientation == "vertical");
  }
  CHECK(rec(r, ids[6]).role == "furniture");
  CHECK(rec(r, ids[0]).orientation == "horizontal");

  for (size_t k = 0; k < r.planes.size(); ++k) {
    CHECK(r.planes[k].plane_label == static_cast<int>(k + 1));
    CHECK(r.planes[k].storey == -1);
    CHECK(r.planes[k].semantic_class == -1);
    CHECK(r.planes[k].class_confidence == -1.0);
    CHECK(r.role_source[k] == RoleSource::rules);
  }
  CHECK(r.from_semantic == 0);
  CHECK(r.role_counts.at("wall") == 4);
  CHECK(r.implicit_floor_z == Approx(0.0).margin(1e-6));
  CHECK(r.implicit_ceiling_z == Approx(2.6).margin(1e-6));

  // Area: occupied 0.1 m cells (one extra row/column at the edges).
  CHECK(rec(r, ids[0]).area == Approx(20.0).epsilon(0.1));
  CHECK(rec(r, ids[6]).area == Approx(2.0).epsilon(0.2));
  CHECK(rec(r, ids[2]).z_min == Approx(0.0).margin(1e-6));
  CHECK(rec(r, ids[2]).z_max == Approx(2.6).margin(1e-6));
  CHECK(rec(r, ids[0]).point_count == 101 * 81);
}

TEST_CASE("ClassifyPlanes_ShortVerticalAndSmallShelf_AreClutter",
          "[classify_planes]") {
  Scene sc;
  sc.room(0.0);
  const size_t front = sc.wall_x(2.0, 0.5, 1.5, 0.0, 0.9); // cabinet front
  const size_t shelf = sc.horizontal(4.0, 4.4, 0.2, 0.5, 1.8);
  const size_t roof = sc.slanted(0.5, 0.5, 1.5, 1.4, 0.6);
  const auto c = sc.build();
  const auto r =
      classify_planes(c.cloud, c.planes, c.normals, {storey(0, 0.0, 2.6)});
  CHECK(rec(r, front).role == "clutter");
  CHECK(rec(r, shelf).role == "clutter");
  CHECK(rec(r, roof).orientation == "slanted");
  CHECK(rec(r, roof).role == "unknown");
}

TEST_CASE("ClassifyPlanes_TwoStoreys_AssignsStoreyAndRolesPerStorey",
          "[classify_planes]") {
  Scene sc;
  const auto lower = sc.room(0.0);
  const auto upper = sc.room(3.0); // 0.4 m slab between 2.6 and 3.0
  const auto c = sc.build();
  const auto r = classify_planes(c.cloud, c.planes, c.normals,
                                 {storey(0, 0.0, 2.6), storey(1, 3.0, 5.6)});
  for (size_t k : lower)
    CHECK(rec(r, k).storey == 0);
  for (size_t k : upper)
    CHECK(rec(r, k).storey == 1);
  for (const auto *ids : {&lower, &upper}) {
    CHECK(rec(r, (*ids)[0]).role == "floor");
    CHECK(rec(r, (*ids)[1]).role == "ceiling");
    CHECK(rec(r, (*ids)[2]).role == "wall");
    CHECK(rec(r, (*ids)[6]).role == "furniture");
  }
  CHECK(r.role_counts.at("floor") == 2);
  CHECK(r.role_counts.at("ceiling") == 2);
  CHECK(r.role_counts.at("wall") == 8);
}

TEST_CASE("ClassifyPlanes_TiltedWorld_UsesStoreyLevelFrame",
          "[classify_planes]") {
  Scene sc;
  sc.pose = Eigen::Translation3d(10.0, -4.0, 37.0) *
            Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(12.0 * M_PI / 180.0, Eigen::Vector3d::UnitX());
  const auto ids = sc.room(0.0);
  const auto c = sc.build();
  auto s = storey(0, 0.0, 2.6);
  s.level_transform = LevelFrame{Eigen::Affine3d(sc.pose.inverse())}.to_array();
  const auto r = classify_planes(c.cloud, c.planes, c.normals, {s});
  CHECK(rec(r, ids[0]).role == "floor");
  CHECK(rec(r, ids[1]).role == "ceiling");
  CHECK(rec(r, ids[3]).role == "wall");
  CHECK(rec(r, ids[6]).role == "furniture");
  CHECK(rec(r, ids[6]).z_min == Approx(0.75).margin(1e-4));
}

TEST_CASE("ClassifyPlanes_SplitLevelFloorAndWallOnIt_FloorAndWall",
          "[classify_planes]") {
  // A 1.9 x 3.4 m raised floor 0.75 m up: table height, but far too big for
  // a table. A wall stands on it and reaches the ceiling (1.85 m tall).
  const auto scene = [](bool with_raised_floor) {
    Scene sc;
    sc.room(0.0);
    if (with_raised_floor)
      sc.horizontal(3.1, 5.0, 0.5, 3.9, 0.75);
    sc.wall_y(2.0, 3.2, 4.8, 0.75, 2.6);
    return sc;
  };
  const std::vector<core::StoreyRecord> st{storey(0, 0.0, 2.6)};
  {
    const auto c = scene(true).build();
    const auto r = classify_planes(c.cloud, c.planes, c.normals, st);
    CHECK(rec(r, 7).role == "floor");
    CHECK(rec(r, 8).role == "wall");
    CHECK(rec(r, 6).role == "furniture"); // the 2 m^2 table stays
  }
  {
    // Without the raised floor the wall floats 0.75 m up and is shorter
    // than 0.75 x the storey height: not a wall.
    const auto c = scene(false).build();
    const auto r = classify_planes(c.cloud, c.planes, c.normals, st);
    CHECK(rec(r, 7).role == "clutter");
  }
}

TEST_CASE("ClassifyPlanes_PitchedCeiling_IsCeiling", "[classify_planes]") {
  Scene sc;
  sc.room(0.0);
  const size_t pitch = sc.slanted(0.5, 0.5, 3.5, 1.6, 1.0); // z 1.6..2.6
  const auto c = sc.build();
  const auto r =
      classify_planes(c.cloud, c.planes, c.normals, {storey(0, 0.0, 2.6)});
  CHECK(rec(r, pitch).orientation == "slanted");
  CHECK(rec(r, pitch).role == "ceiling");
}

TEST_CASE("ClassifyPlanes_NormalFacing_DecidesFloorVersusCeiling",
          "[classify_planes]") {
  Scene sc;
  const auto ids = sc.room(0.0);
  // Facing up at ceiling height (seen from above: not a ceiling), and a
  // table-height plane facing down (seen from below: not a table top).
  const size_t up_high = sc.horizontal(0.5, 2.0, 0.5, 2.0, 2.55);
  const size_t down_table = sc.horizontal(3.5, 4.0, 0.5, 1.5, 0.8, -1, true);
  const auto c = sc.build();
  const std::vector<core::StoreyRecord> st{storey(0, 0.0, 2.6)};
  const auto r = classify_planes(c.cloud, c.planes, c.normals, st);
  CHECK(rec(r, up_high).role == "clutter");
  CHECK(rec(r, down_table).role == "clutter");
  CHECK(rec(r, ids[1]).role == "ceiling");

  // Legacy clouds (normals faced the world origin): the sign is ignored.
  ClassifyPlanesOptions legacy;
  legacy.oriented_normals = false;
  const auto rl = classify_planes(c.cloud, c.planes, c.normals, st, {},
                                  core::default_class_role_map(), legacy);
  CHECK(rec(rl, up_high).role == "ceiling");
  CHECK(rec(rl, down_table).role == "furniture");
}

TEST_CASE("ClassifyPlanes_SurfacesEitherSideOfSlab_SplitByFacing",
          "[classify_planes]") {
  Scene sc;
  sc.room(0.0);
  sc.room(3.0);
  // Slab underside 0.25 m above storey 0's ceiling (seen from below) and an
  // upper floor 0.25 m below storey 1's floor (seen from above): by median
  // height alone each is nearer the other storey's interval.
  const size_t under = sc.horizontal(0.5, 2.5, 0.5, 2.5, 2.85, -1, true);
  const size_t over = sc.horizontal(2.5, 4.5, 0.5, 2.5, 2.75);
  const auto c = sc.build();
  const auto r = classify_planes(c.cloud, c.planes, c.normals,
                                 {storey(0, 0.0, 2.6), storey(1, 3.0, 5.6)});
  CHECK(rec(r, under).storey == 0);
  CHECK(rec(r, under).role == "ceiling");
  CHECK(rec(r, over).storey == 1);
  CHECK(rec(r, over).role == "floor");
}

// ── semantics ─────────────────────────────────────────────────────────────

TEST_CASE("ClassifyPlanes_LabelledRoom_MajorityClassWithShare",
          "[classify_planes]") {
  Scene sc;
  const auto ids = sc.room(0.0, /*label=*/true);
  // A full-height wardrobe front: geometry says wall, 70 % "cabinet" says
  // furniture, 30 % "wall".
  const size_t wardrobe = sc.wall_x(1.0, 1.0, 2.0, 0.0, 2.6);
  auto &w = sc.patches[wardrobe];
  for (size_t i = 0; i < w.api.size(); ++i)
    w.api[i] = (i % 10 < 7) ? kCabinet : kWall;
  const auto c = sc.build();
  const auto r = classify_planes(c.cloud, c.planes, c.normals,
                                 {storey(0, 0.0, 2.6)}, semantic_of(c.labels));

  CHECK(rec(r, ids[0]).semantic_class == kFloor);
  CHECK(rec(r, ids[2]).semantic_class == kWall);
  CHECK(rec(r, ids[2]).class_confidence == Approx(1.0));
  CHECK(rec(r, ids[6]).semantic_class == kTable);
  CHECK(rec(r, ids[6]).role == "furniture");
  CHECK(r.role_source[ids[6]] == RoleSource::semantic);
  CHECK(rec(r, ids[1]).role == "ceiling"); // name matched case-insensitively

  CHECK(rec(r, wardrobe).semantic_class == kCabinet);
  CHECK(rec(r, wardrobe).class_confidence == Approx(0.7).margin(0.02));
  CHECK(rec(r, wardrobe).role == "furniture");
  CHECK(r.role_source[wardrobe] == RoleSource::semantic);
  CHECK(r.from_semantic == 8);
}

TEST_CASE("ClassifyPlanes_MisleadingMajority_GeometryOverrules",
          "[classify_planes]") {
  Scene sc;
  const auto ids = sc.room(0.0, /*label=*/true);
  // The floor is mostly labelled "wall" and the ceiling mostly "table":
  // neither fits a large horizontal plane in the floor / ceiling band.
  for (auto &a : sc.patches[ids[0]].api)
    a = kWall;
  for (auto &a : sc.patches[ids[1]].api)
    a = kTable;
  // A vertical wall labelled "floor".
  for (auto &a : sc.patches[ids[3]].api)
    a = kFloor;
  // The table top labelled "floor": a floor-class plane 0.75 m up.
  for (auto &a : sc.patches[ids[6]].api)
    a = kFloor;
  const auto c = sc.build();
  const auto r = classify_planes(c.cloud, c.planes, c.normals,
                                 {storey(0, 0.0, 2.6)}, semantic_of(c.labels));

  CHECK(rec(r, ids[0]).semantic_class == kWall); // evidence kept
  CHECK(rec(r, ids[0]).role == "floor");
  CHECK(r.role_source[ids[0]] == RoleSource::overruled);
  CHECK(rec(r, ids[1]).role == "ceiling");
  CHECK(r.role_source[ids[1]] == RoleSource::overruled);
  CHECK(rec(r, ids[3]).role == "wall");
  CHECK(r.role_source[ids[3]] == RoleSource::overruled);
  CHECK(rec(r, ids[6]).role == "furniture");
  CHECK(r.role_source[ids[6]] == RoleSource::overruled);
  CHECK(r.overruled == 4);
  // The record carries the same verdict as text (plane_classes.role_source).
  for (size_t k = 0; k < r.planes.size(); ++k)
    CHECK(r.planes[k].role_source == to_string(r.role_source[k]));
  CHECK(rec(r, ids[0]).role_source == "overruled");
}

TEST_CASE("ClassifyPlanes_SparseOrSplitLabels_FallsBackToRules",
          "[classify_planes]") {
  Scene sc;
  const auto ids = sc.room(0.0);
  // Table: only 5 % of the points labelled (all "cabinet") -> not trusted.
  auto &t = sc.patches[ids[6]];
  for (size_t i = 0; i < t.api.size(); i += 20)
    t.api[i] = kCabinet;
  // Wall: labels split 40/30/30 -> no clear majority.
  auto &w = sc.patches[ids[2]];
  for (size_t i = 0; i < w.api.size(); ++i)
    w.api[i] = (i % 10 < 4) ? kCabinet : (i % 10 < 7 ? kTable : kFloor);
  const auto c = sc.build();
  const auto r = classify_planes(c.cloud, c.planes, c.normals,
                                 {storey(0, 0.0, 2.6)}, semantic_of(c.labels));
  CHECK(rec(r, ids[6]).semantic_class == kCabinet);
  CHECK(rec(r, ids[6]).class_confidence == Approx(1.0));
  CHECK(rec(r, ids[6]).role == "furniture");
  CHECK(r.role_source[ids[6]] == RoleSource::rules);
  CHECK(rec(r, ids[2]).semantic_class == kCabinet);
  CHECK(rec(r, ids[2]).class_confidence == Approx(0.4).margin(0.02));
  CHECK(rec(r, ids[2]).role == "wall");
  CHECK(r.role_source[ids[2]] == RoleSource::rules);
  // Unlabelled planes have no class.
  CHECK(rec(r, ids[0]).semantic_class == -1);
  CHECK(rec(r, ids[0]).class_confidence == -1.0);
}

TEST_CASE("ClassifyPlanes_ClassNotInRoleMap_RulesDecide", "[classify_planes]") {
  Scene sc;
  const auto ids = sc.room(0.0, /*label=*/true);
  const auto c = sc.build();
  const auto map =
      core::parse_class_role_map(R"({"classes":{"table":"clutter"}})");
  const auto r =
      classify_planes(c.cloud, c.planes, c.normals, {storey(0, 0.0, 2.6)},
                      semantic_of(c.labels), map);
  CHECK(rec(r, ids[6]).role == "clutter");
  CHECK(r.role_source[ids[6]] == RoleSource::semantic);
  CHECK(rec(r, ids[2]).role == "wall");
  CHECK(r.role_source[ids[2]] == RoleSource::rules);
}

// ── determinism and degenerate input ──────────────────────────────────────

TEST_CASE("ClassifyPlanes_PermutedPoints_IdenticalRecords",
          "[classify_planes]") {
  Scene sc;
  sc.room(0.0, /*label=*/true);
  sc.room(3.0);
  const auto c = sc.build();
  const std::vector<core::StoreyRecord> st{storey(0, 0.0, 2.6),
                                           storey(1, 3.0, 5.6)};
  const auto a =
      classify_planes(c.cloud, c.planes, c.normals, st, semantic_of(c.labels));

  std::vector<size_t> perm(c.cloud.size());
  std::iota(perm.begin(), perm.end(), 0);
  std::shuffle(perm.begin(), perm.end(), std::mt19937(42));
  Cloud cloud;
  CloudL planes, labels;
  for (size_t i : perm) {
    cloud.push_back(c.cloud.points[i]);
    planes.push_back(c.planes.points[i]);
    labels.push_back(c.labels.points[i]);
  }
  const auto b =
      classify_planes(cloud, planes, c.normals, st, semantic_of(labels));

  REQUIRE(a.planes.size() == b.planes.size());
  for (size_t k = 0; k < a.planes.size(); ++k) {
    CHECK(a.planes[k].role == b.planes[k].role);
    CHECK(a.planes[k].storey == b.planes[k].storey);
    CHECK(a.planes[k].semantic_class == b.planes[k].semantic_class);
    CHECK(a.planes[k].class_confidence == b.planes[k].class_confidence);
    CHECK(a.planes[k].area == b.planes[k].area);
    CHECK(a.planes[k].z_min == b.planes[k].z_min);
    CHECK(a.planes[k].z_max == b.planes[k].z_max);
  }
}

TEST_CASE("ClassifyPlanes_PlaneWithoutPoints_RoleUnknown",
          "[classify_planes]") {
  Scene sc;
  sc.room(0.0);
  auto c = sc.build();
  NormalT extra;
  extra.normal_z = 1.0f;
  c.normals.push_back(extra); // label 8 has a normal but no points
  const auto r = classify_planes(c.cloud, c.planes, c.normals, {});
  REQUIRE(r.planes.size() == 8);
  CHECK(r.planes[7].plane_label == 8);
  CHECK(r.planes[7].point_count == 0);
  CHECK(r.planes[7].role == "unknown");
  CHECK(r.role_source[7] == RoleSource::none);
}

TEST_CASE("ClassifyPlanes_EmptyInput_ReturnsNoRecords", "[classify_planes]") {
  const auto r = classify_planes(Cloud{}, CloudL{}, CloudN{}, {});
  CHECK(r.planes.empty());
}

TEST_CASE("ClassifyPlanes_MismatchedInput_Throws", "[classify_planes]") {
  Scene sc;
  sc.room(0.0);
  auto c = sc.build();
  CloudL short_labels = c.planes;
  short_labels.points.pop_back();
  short_labels.width = static_cast<uint32_t>(short_labels.points.size());
  CHECK_THROWS_AS(classify_planes(c.cloud, short_labels, c.normals, {}),
                  std::invalid_argument);

  SemanticLabels sem;
  sem.labels = &short_labels;
  CHECK_THROWS_AS(classify_planes(c.cloud, c.planes, c.normals, {}, sem),
                  std::invalid_argument);

  CloudN fewer = c.normals;
  fewer.points.pop_back();
  fewer.width = static_cast<uint32_t>(fewer.points.size());
  CHECK_THROWS_AS(classify_planes(c.cloud, c.planes, fewer, {}),
                  std::invalid_argument);

  ClassifyPlanesOptions bad;
  bad.area_cell_m = 0.0;
  CHECK_THROWS_AS(classify_planes(c.cloud, c.planes, c.normals, {}, {},
                                  core::default_class_role_map(), bad),
                  std::invalid_argument);
}
