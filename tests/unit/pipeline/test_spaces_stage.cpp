// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The `spaces` pipeline stage (reusex-z1v.6.3): the band rooms of `rooms`
// polygonised against the walls table -> the spaces and space_boundaries
// tables, through pipeline::run_stage (the path `rux create spaces` and the
// Qt Pipeline page share).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/space_polygon.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/pipeline/space_parameters.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/types/point_types.hpp>

#include <Eigen/Core>

#include "../../support/temp_path.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using namespace reusex::pipeline;
using reusex::test_support::TempPath;

namespace {

/// The openings stage test's two rooms (copied: tests share no fixtures
/// header yet). The walls stage test's two rooms (floor z = 0, ceiling z = 2.5
/// over [0, 4] x [0, 3], a 10 cm partition at x = 2), with a door (y 1.0
/// .. 1.9, floor to 2.1 m) through the partition and a window (x 2.5 .. 3.5, z
/// 0.9
/// .. 2.0) in the south wall. Every face is its own plane, as `create planes`
/// leaves them; the plane classes are written as `create classify` would.
/// @p door_y0 .. @p door_y1 move the door's jambs and @p door_head its head
/// (RoomsStage_AfterWallsAndOpenings_CutsAtAWideDoor: a doorway wider than
/// the band's constriction cut, its lintel above the wall band).
void seed_rooms(ProjectDB &db, double door_y0 = 1.0, double door_y1 = 1.9,
                double door_head = 2.1) {
  const bool oriented = true;
  Cloud cloud;
  CloudL planes;
  CloudN normals;
  CloudN point_normals; // per point: its plane's normal (the band reads it)
  pcl::PointCloud<pcl::PointXYZ> centroids;
  std::vector<core::PlaneClassRecord> classes;
  uint32_t label = 0;
  const double step = 0.05;
  auto plane = [&](const char *orientation, const char *role, double nx,
                   double ny, double nz, auto &&points) {
    ++label;
    int n = 0;
    NormalT nn;
    nn.normal_x = float(nx);
    nn.normal_y = float(ny);
    nn.normal_z = float(nz);
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    points([&](double x, double y, double z) {
      PointT p;
      p.x = float(x);
      p.y = float(y);
      p.z = float(z);
      cloud.push_back(p);
      LabelT l;
      l.label = label;
      planes.push_back(l);
      point_normals.push_back(nn);
      sum += Eigen::Vector3d(x, y, z);
      ++n;
    });
    normals.push_back(nn);
    const Eigen::Vector3d mean = sum / std::max(n, 1);
    centroids.push_back(
        pcl::PointXYZ(float(mean.x()), float(mean.y()), float(mean.z())));
    core::PlaneClassRecord c;
    c.plane_label = static_cast<int>(label);
    c.guid = core::deterministic_guid(
        db.project_guid(), "plane_classes/planes/" + std::to_string(label));
    c.orientation = orientation;
    c.role = role;
    c.role_source = "rules";
    c.storey = 0;
    c.point_count = n;
    classes.push_back(c);
  };
  auto horizontal = [&](double z) {
    return [=](auto &&emit) {
      for (double x = 0.5 * step; x < 4.0; x += step)
        for (double y = 0.5 * step; y < 3.0; y += step)
          emit(x, y, z);
    };
  };
  // A vertical face from (x0, y0) to (x1, y1).
  auto face = [&](double x0, double y0, double x1, double y1) {
    return [=](auto &&emit) {
      const double len = std::hypot(x1 - x0, y1 - y0);
      for (double s = 0.5 * step; s < len; s += step)
        for (double z = 0.5 * step; z < 2.5; z += step) {
          const double x = x0 + (x1 - x0) * s / len;
          const double y = y0 + (y1 - y0) * s / len;
          const bool door = std::abs(x - 2.0) < 0.1 && y > door_y0 &&
                            y < door_y1 && z < door_head;
          const bool window =
              y < 0.01 && x > 2.5 && x < 3.5 && z > 0.9 && z < 2.0;
          if (!door && !window)
            emit(x, y, z);
        }
    };
  };
  plane("horizontal", "floor", 0, 0, 1, horizontal(0.0));
  plane("horizontal", "ceiling", 0, 0, -1, horizontal(2.5));
  plane("vertical", "wall", 0, 1, 0, face(0, 0, 4, 0));        // south, seen +y
  plane("vertical", "wall", 0, -1, 0, face(0, 3, 4, 3));       // north, seen -y
  plane("vertical", "wall", 1, 0, 0, face(0, 0, 0, 3));        // west, seen +x
  plane("vertical", "wall", -1, 0, 0, face(4, 0, 4, 3));       // east, seen -x
  plane("vertical", "wall", -1, 0, 0, face(1.95, 0, 1.95, 3)); // partition
  plane("vertical", "wall", 1, 0, 0, face(2.05, 0, 2.05, 3));  // partition

  const std::string marker =
      oriented ? R"({"normal_orientation":"sensor"})" : "{}";
  db.save_point_cloud("cloud", cloud, "reconstruct", marker);
  db.save_point_cloud("planes", planes, "segment_planes", "{}");
  db.save_point_cloud("plane_normals", normals, "segment_planes", marker);
  db.save_point_cloud("normals", point_normals, "reconstruct", marker);
  db.save_point_cloud("plane_centroids", centroids, "segment_planes", "{}");

  core::StoreyRecord s;
  s.guid = core::deterministic_guid(db.project_guid(), "storeys/z0");
  s.name = "Etage 0";
  s.z_floor = 0.0;
  s.z_ceiling = 2.5;
  db.save_storeys({s});
  db.save_plane_classes("planes", classes);
}

StageResult run(ProjectDB &db, const TempPath &project, JobStage stage,
                const std::string &params = "{}") {
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = stage;
  ctx.parameters = params;
  return run_stage(db, ctx);
}

/// walls -> openings -> rooms on the two rooms.
void seed_band_rooms(ProjectDB &db, const TempPath &project) {
  seed_rooms(db);
  for (auto stage : {JobStage::walls, JobStage::openings, JobStage::rooms}) {
    const auto r = run(db, project, stage);
    INFO(to_string(stage) << ": " << r.message);
    REQUIRE(r.ok);
  }
}

std::vector<std::string> guids(const ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &s : db.spaces())
    out.push_back(s.guid);
  std::sort(out.begin(), out.end());
  return out;
}

} // namespace

TEST_CASE("SpacesStage_Names_AndContract", "[pipeline][stages][spaces]") {
  CHECK(to_string(JobStage::spaces) == "spaces");
  CHECK(pipeline_log_name(JobStage::spaces) == "spaces");
  CHECK(parse_job_stage("spaces") == JobStage::spaces);
  const auto &c = core::stage_contract(core::PipelineStage::spaces);
  CHECK(c.command == "rux create spaces");
  REQUIRE(c.outputs.size() == 2);
  CHECK(c.outputs[0] == "spaces");
  CHECK(c.outputs[1] == "space_boundaries");
}

TEST_CASE("SpacesStage_Parameters_AreSpacesOptionsDefaults",
          "[pipeline][stages][spaces]") {
  const SpaceStageOptions d{};
  const auto &params = stage_parameters(JobStage::spaces);
  REQUIRE(params.size() == space_parameter_table().size());
  auto value = [&](const char *key) -> ParameterDefault {
    for (const auto &p : params)
      if (p.key == key)
        return p.default_value;
    FAIL("no parameter " << key);
    return {};
  };
  CHECK(std::get<bool>(value("use_walls")) == d.spaces.use_walls);
  CHECK(std::get<bool>(value("axes_only")) == d.spaces.axes_only);
  CHECK(std::get<double>(value("join_extension_m")) ==
        d.spaces.join_extension_m);
  CHECK(std::get<long long>(value("open_plan_min_holes")) ==
        d.spaces.open_plan_min_holes);
  CHECK(std::get<double>(value("guid_match_m")) == d.identity.match_m);
  CHECK(std::get<double>(value("guid_min_area_ratio")) ==
        d.identity.min_area_ratio);

  const auto again =
      space_options_from_parameters(space_parameters_json(SpaceStageOptions{}));
  CHECK(space_parameters_json(again) == space_parameters_json(d));
  const auto o = space_options_from_parameters(
      R"({"join_extension_m": 0.5, "axes_only": true, "use_walls": false})");
  CHECK(o.spaces.join_extension_m == 0.5);
  CHECK(o.spaces.axes_only);
  CHECK_FALSE(o.spaces.use_walls);
  CHECK_THROWS_AS(space_options_from_parameters(R"({"pure_min_share": 2})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(space_options_from_parameters("[1]"), std::invalid_argument);
}

TEST_CASE("SpacesStage_NoRooms_IsRefusedAsInvalidInput",
          "[pipeline][stages][spaces]") {
  TempPath project("test_spaces_stage_norooms");
  ProjectDB db(project.path);
  seed_rooms(db);
  const auto r = run(db, project, JobStage::spaces);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK_FALSE(db.has_spaces());
}

TEST_CASE("SpacesStage_TwoRooms_TwoSpacesBoundedByTheWalls",
          "[pipeline][stages][spaces]") {
  TempPath project("test_spaces_stage_rooms");
  ProjectDB db(project.path);
  seed_band_rooms(db, project);
  const auto r = run(db, project, JobStage::spaces);
  INFO(r.message);
  REQUIRE(r.ok);
  REQUIRE(r.outputs.size() == 2);
  CHECK(r.outputs[0].name == "spaces");
  CHECK(r.outputs[1].name == "space_boundaries");

  const auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  CHECK(r.outputs[0].count == 2);
  const auto labels = db.point_cloud_label("rooms");
  std::set<int> room_ids;
  for (const auto &l : labels->points)
    if (l.label > 0)
      room_ids.insert(static_cast<int>(l.label));
  const auto walls = db.walls();
  std::set<std::string> wall_guids;
  for (const auto &w : walls)
    wall_guids.insert(w.guid);
  for (const auto &s : rows) {
    INFO("space " << s.guid << " area " << s.area);
    CHECK(s.guid.size() == 36);
    CHECK(s.storey == 0);
    CHECK(s.kind == "room");
    CHECK(s.source == "detected");
    CHECK(room_ids.count(s.room_label) == 1);
    // Each half of 4 x 3 minus the 10 cm partition: 1.95 x 3 (faces of
    // the walls, within a cell).
    CHECK_THAT(s.area, WithinAbs(1.95 * 3.0, 0.3));
    CHECK_THAT(core::space_polygon_area(s.polygon), WithinAbs(s.area, 1e-6));
    CHECK_THAT(s.volume, WithinAbs(s.area * 2.5, 1e-6));
    CHECK(s.confidence > 0.5); // mostly wall-bounded faces
  }
  CHECK(rows[0].room_label != rows[1].room_label);
  const auto c0 = core::space_polygon_centroid(rows[0].polygon);
  const auto c1 = core::space_polygon_centroid(rows[1].polygon);
  CHECK(std::abs(c0[0] - c1[0]) > 1.5); // one each side of x = 2

  const auto bounds = db.space_boundaries();
  CHECK(r.outputs[1].count == static_cast<int64_t>(bounds.size()));
  CHECK(bounds.size() >= 6); // 4 walls around each, the partition shared
  for (const auto &b : bounds) {
    CHECK(wall_guids.count(b.wall) == 1);
    CHECK(b.s_end > b.s_start);
  }
  const auto log = db.pipeline_log(1);
  REQUIRE(log.size() == 1);
  CHECK(log[0].stage == "spaces");

  // GUIDs stable across a re-run, and the model reads them.
  const auto first = guids(db);
  REQUIRE(run(db, project, JobStage::spaces).ok);
  CHECK(guids(db) == first);
  const auto m = io::load_solid_model(db);
  CHECK(m.spaces.size() == 2);
  CHECK(m.space_boundaries.size() == bounds.size());
  CHECK(m.has_spaces());
}

TEST_CASE("SpacesStage_LockedRow_SurvivesARerun",
          "[pipeline][stages][spaces]") {
  TempPath project("test_spaces_stage_locked");
  ProjectDB db(project.path);
  seed_band_rooms(db, project);
  REQUIRE(run(db, project, JobStage::spaces).ok);
  auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  rows.front().name = "Køkken"; // a hand fix
  rows.front().source = "manual";
  rows.front().locked = true;
  db.save_spaces(rows, db.space_boundaries());

  const auto r = run(db, project, JobStage::spaces);
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(r.message.find("1 locked") != std::string::npos);
  CHECK(db.spaces().size() == 2);
  const auto kept = db.space_by_guid(rows.front().guid);
  REQUIRE(kept.has_value());
  CHECK(kept->locked);
  CHECK(kept->name == "Køkken");
}

TEST_CASE("SpacesStage_StaleRooms_IsRefused", "[pipeline][stages][spaces]") {
  // The band grid is rebuilt from today's tables: without the walls and
  // openings nothing cuts the wide doorway, so the grid has one room where
  // 'rooms' has two.
  TempPath project("test_spaces_stage_stale");
  ProjectDB db(project.path);
  seed_rooms(db, 0.8, 2.2, 2.35); // a wide doorway only the door cut closes
  for (auto stage : {JobStage::walls, JobStage::openings, JobStage::rooms})
    REQUIRE(run(db, project, stage).ok);
  db.delete_openings();
  db.delete_walls();
  const auto r = run(db, project, JobStage::spaces);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("re-run `rux create rooms`") != std::string::npos);
  CHECK_FALSE(db.has_spaces());
}

TEST_CASE("SpacesStage_LeidenRooms_IsRefused", "[pipeline][stages][spaces]") {
  TempPath project("test_spaces_stage_leiden");
  ProjectDB db(project.path);
  seed_rooms(db);
  const auto rooms =
      run(db, project, JobStage::rooms, R"({"method":"leiden"})");
  INFO(rooms.message);
  REQUIRE(rooms.ok);
  const auto r = run(db, project, JobStage::spaces);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("leiden") != std::string::npos);
}
