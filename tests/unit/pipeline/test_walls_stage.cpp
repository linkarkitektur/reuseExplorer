// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The `walls` pipeline stage (reusex-z1v.5.5): cloud + planes + plane_normals
// + plane_classes + storeys -> the walls and wall_joins tables, through
// pipeline::run_stage (the path `rux create walls` and the Qt Pipeline page
// share).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/pipeline/wall_parameters.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using namespace reusex::pipeline;
using reusex::test_support::TempPath;

namespace {

/// One storey (floor z = 0, ceiling z = 2.5) over [0, 4] x [0, 3]: four
/// outer walls seen from inside, and a 10 cm partition at x = 2 seen from
/// both rooms. Every face is its own plane, as `create planes` leaves them;
/// the plane classes are written as `create classify` would.
void seed_rooms(ProjectDB &db, bool oriented = true) {
  Cloud cloud;
  CloudL planes;
  CloudN normals;
  std::vector<core::PlaneClassRecord> classes;
  uint32_t label = 0;
  const double step = 0.05;
  auto plane = [&](const char *orientation, const char *role, double nx,
                   double ny, double nz, auto &&points) {
    ++label;
    int n = 0;
    points([&](double x, double y, double z) {
      PointT p;
      p.x = float(x);
      p.y = float(y);
      p.z = float(z);
      cloud.push_back(p);
      LabelT l;
      l.label = label;
      planes.push_back(l);
      ++n;
    });
    NormalT nn;
    nn.normal_x = float(nx);
    nn.normal_y = float(ny);
    nn.normal_z = float(nz);
    normals.push_back(nn);
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
        for (double z = 0.5 * step; z < 2.5; z += step)
          emit(x0 + (x1 - x0) * s / len, y0 + (y1 - y0) * s / len, z);
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

  core::StoreyRecord s;
  s.guid = core::deterministic_guid(db.project_guid(), "storeys/z0");
  s.name = "Etage 0";
  s.z_floor = 0.0;
  s.z_ceiling = 2.5;
  db.save_storeys({s});
  db.save_plane_classes("planes", classes);
}

StageResult run(ProjectDB &db, const TempPath &project,
                const std::string &params = "{}") {
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::walls;
  ctx.parameters = params;
  return run_stage(db, ctx);
}

std::vector<std::string> guids(const ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &w : db.walls())
    out.push_back(w.guid);
  std::sort(out.begin(), out.end());
  return out;
}

} // namespace

TEST_CASE("WallsStage_Names_AndContract", "[pipeline][stages][walls]") {
  CHECK(to_string(JobStage::walls) == "walls");
  CHECK(pipeline_log_name(JobStage::walls) == "walls");
  CHECK(parse_job_stage("walls") == JobStage::walls);
  const auto &c = core::stage_contract(core::PipelineStage::walls);
  CHECK(c.command == "rux create walls");
  REQUIRE(c.outputs.size() == 2);
  CHECK(c.outputs[0] == "walls");
  CHECK(c.outputs[1] == "wall_joins");
  CHECK(core::producing_stage("walls") == core::PipelineStage::walls);
  CHECK(core::producing_stage("wall_joins") == core::PipelineStage::walls);
}

TEST_CASE("WallsStage_Parameters_AreWallOptionsDefaults",
          "[pipeline][stages][walls]") {
  const WallStageOptions d{};
  const auto &params = stage_parameters(JobStage::walls);
  REQUIRE(params.size() == wall_parameter_table().size());
  auto value = [&](const char *key) -> ParameterDefault {
    for (const auto &p : params)
      if (p.key == key)
        return p.default_value;
    FAIL("no parameter " << key);
    return {};
  };
  CHECK(std::get<long long>(value("max_families")) == d.walls.max_families);
  CHECK(std::get<double>(value("family_angle_deg")) ==
        d.walls.family_angle_deg);
  CHECK(std::get<bool>(value("wall_role_only")) == d.walls.wall_role_only);
  CHECK(std::get<double>(value("default_thickness_m")) ==
        d.walls.default_thickness_m);
  CHECK(std::get<long long>(value("seed")) == d.walls.seed);
  CHECK(std::get<double>(value("min_seen_share")) == d.walls.min_seen_share);
  CHECK(std::get<double>(value("guid_match_m")) == d.identity.match_m);
  // use_normal_sign comes from the clouds' provenance, never a parameter.
  CHECK(std::none_of(params.begin(), params.end(),
                     [](const auto &p) { return p.key == "use_normal_sign"; }));

  // Every key reaches the options: the defaults round-trip through JSON...
  const auto again =
      wall_options_from_parameters(wall_parameters_json(WallStageOptions{}));
  CHECK(wall_parameters_json(again) == wall_parameters_json(d));
  // ...and a changed value lands in its field.
  const auto o = wall_options_from_parameters(
      R"({"max_gap_m": 0.6, "seed": 7, "wall_role_only": false,
          "guid_match_m": 0.1})");
  CHECK(o.walls.max_gap_m == 0.6);
  CHECK(o.walls.seed == 7u);
  CHECK_FALSE(o.walls.wall_role_only);
  CHECK(o.identity.match_m == 0.1);
  CHECK_THROWS_AS(wall_options_from_parameters(R"({"max_gap_m": -1})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(wall_options_from_parameters(R"({"seed": "x"})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(wall_options_from_parameters("[1]"), std::invalid_argument);
}

TEST_CASE("WallsStage_EmptyProject_IsRefusedAsInvalidInput",
          "[pipeline][stages][walls]") {
  TempPath project("test_walls_stage_empty");
  ProjectDB db(project.path);
  const auto r = run(db, project);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK_FALSE(db.has_walls());
}

TEST_CASE("WallsStage_StaleClasses_AreRefused", "[pipeline][stages][walls]") {
  TempPath project("test_walls_stage_stale");
  ProjectDB db(project.path);
  seed_rooms(db);
  auto classes = db.plane_classes();
  classes[2].point_count += 7; // planes ran again after classify
  db.save_plane_classes("planes", classes);
  const auto r = run(db, project);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("classify") != std::string::npos);
}

TEST_CASE("WallsStage_Rooms_WritesWallsJoinsAndStableGuids",
          "[pipeline][stages][walls]") {
  TempPath project("test_walls_stage_rooms");
  ProjectDB db(project.path);
  seed_rooms(db);
  const auto r = run(db, project);
  INFO(r.message);
  REQUIRE(r.ok);
  REQUIRE(r.outputs.size() == 2);
  CHECK(r.outputs[0].name == "walls");
  CHECK(r.outputs[1].name == "wall_joins");

  const auto walls = db.walls();
  INFO(walls.size() << " walls");
  REQUIRE(walls.size() >= 5);
  CHECK(r.outputs[0].count == static_cast<int64_t>(walls.size()));
  // The partition: measured from its two faces, 10 cm thick, at x = 2.
  const auto partition =
      std::find_if(walls.begin(), walls.end(), [](const auto &w) {
        return !w.assumed && std::abs(w.x0 - 2.0) < 0.05 &&
               std::abs(w.x1 - 2.0) < 0.05;
      });
  REQUIRE(partition != walls.end());
  CHECK_THAT(partition->thickness, WithinAbs(0.10, 0.02));
  CHECK(partition->thickness_ci_lo <= partition->thickness);
  CHECK(partition->thickness_ci_hi >= partition->thickness);
  CHECK_FALSE(partition->is_external);
  // The outer walls: seen from inside only -> assumed and external.
  const auto outer = std::count_if(walls.begin(), walls.end(), [](auto &w) {
    return w.assumed && w.is_external;
  });
  CHECK(outer >= 4);
  for (const auto &w : walls) {
    CHECK(w.storey == 0);
    CHECK(w.source == "detected");
    CHECK(w.guid.size() == 36);
    CHECK_THAT(w.height, WithinAbs(2.5, 1e-9));
  }
  CHECK_FALSE(db.wall_joins().empty());
  const auto log = db.pipeline_log(1);
  REQUIRE(log.size() == 1);
  CHECK(log[0].stage == "walls");
  CHECK(log[0].status == "success");

  // A re-run on the same data: the same GUIDs, no duplicates.
  const auto first = guids(db);
  const auto r2 = run(db, project);
  REQUIRE(r2.ok);
  CHECK(guids(db) == first);
  CHECK(db.walls().size() == walls.size());

  // load_solid_model reads them, so `rux analyze solid` scores the project.
  const auto m = io::load_solid_model(db);
  CHECK(m.walls.size() == walls.size());
  CHECK(m.wall_joins.size() == db.wall_joins().size());
  CHECK(m.has_walls());
}

TEST_CASE("WallsStage_Parameters_ReachDetectWalls",
          "[pipeline][stages][walls]") {
  TempPath project("test_walls_stage_params");
  ProjectDB db(project.path);
  seed_rooms(db);
  REQUIRE(run(db, project, R"({"default_thickness_m": 0.33})").ok);
  const auto walls = db.walls();
  const auto assumed = std::find_if(walls.begin(), walls.end(),
                                    [](const auto &w) { return w.assumed; });
  REQUIRE(assumed != walls.end());
  CHECK_THAT(assumed->thickness, WithinAbs(0.33, 1e-9));

  const auto bad = run(db, project, R"({"max_gap_m": "wide"})");
  CHECK_FALSE(bad.ok);
  CHECK(bad.invalid_input);
}

TEST_CASE("WallsStage_LockedRow_SurvivesARerun", "[pipeline][stages][walls]") {
  TempPath project("test_walls_stage_locked");
  ProjectDB db(project.path);
  seed_rooms(db);
  REQUIRE(run(db, project).ok);

  auto rows = db.walls();
  auto &fixed = rows.front();
  fixed.thickness = 0.37; // a hand fix
  fixed.assumed = false;
  fixed.source = "manual";
  fixed.locked = true;
  db.save_walls(rows, db.wall_joins());
  const auto count = rows.size();

  const auto r = run(db, project);
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(r.message.find("1 locked") != std::string::npos);
  const auto again = db.walls();
  CHECK(again.size() == count); // re-detected, matched, not duplicated
  const auto kept = db.wall_by_guid(fixed.guid);
  REQUIRE(kept.has_value());
  CHECK(kept->locked);
  CHECK_THAT(kept->thickness, WithinAbs(0.37, 1e-12));
}
