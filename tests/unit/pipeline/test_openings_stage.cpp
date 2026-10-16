// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The `openings` pipeline stage (reusex-z1v.7.3): cloud + walls + storeys ->
// the openings table, through pipeline::run_stage (the path `rux create
// openings` and the Qt Pipeline page share).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/pipeline/opening_parameters.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/temp_path.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using namespace reusex::pipeline;
using reusex::test_support::TempPath;

namespace {

/// The walls stage test's two rooms (floor z = 0, ceiling z = 2.5 over
/// [0, 4] x [0, 3], a 10 cm partition at x = 2), with a door (y 1.0 .. 1.9,
/// floor to 2.1 m) through the partition and a window (x 2.5 .. 3.5, z 0.9
/// .. 2.0) in the south wall. Every face is its own plane, as `create planes`
/// leaves them; the plane classes are written as `create classify` would.
void seed_rooms(ProjectDB &db) {
  const bool oriented = true;
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
        for (double z = 0.5 * step; z < 2.5; z += step) {
          const double x = x0 + (x1 - x0) * s / len;
          const double y = y0 + (y1 - y0) * s / len;
          const bool door =
              std::abs(x - 2.0) < 0.1 && y > 1.0 && y < 1.9 && z < 2.1;
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

/// The rooms with their walls detected.
void seed_walls(ProjectDB &db, const TempPath &project) {
  seed_rooms(db);
  const auto r = run(db, project, JobStage::walls);
  INFO(r.message);
  REQUIRE(r.ok);
}

std::vector<std::string> guids(const ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &o : db.openings())
    out.push_back(o.guid);
  std::sort(out.begin(), out.end());
  return out;
}

} // namespace

TEST_CASE("OpeningsStage_Names_AndContract", "[pipeline][stages][openings]") {
  CHECK(to_string(JobStage::openings) == "openings");
  CHECK(pipeline_log_name(JobStage::openings) == "openings");
  CHECK(parse_job_stage("openings") == JobStage::openings);
  const auto &c = core::stage_contract(core::PipelineStage::openings);
  CHECK(c.command == "rux create openings");
  REQUIRE(c.outputs.size() == 1);
  CHECK(c.outputs[0] == "openings");
  CHECK(core::producing_stage("openings") == core::PipelineStage::openings);
}

TEST_CASE("OpeningsStage_Parameters_AreOpeningOptionsDefaults",
          "[pipeline][stages][openings]") {
  const OpeningStageOptions d{};
  const auto &params = stage_parameters(JobStage::openings);
  REQUIRE(params.size() == opening_parameter_table().size());
  auto value = [&](const char *key) -> ParameterDefault {
    for (const auto &p : params)
      if (p.key == key)
        return p.default_value;
    FAIL("no parameter " << key);
    return {};
  };
  CHECK(std::get<double>(value("cell_m")) == d.openings.cell_m);
  CHECK(std::get<double>(value("front_m")) == d.openings.front_m);
  CHECK(std::get<long long>(value("morph_radius")) == d.openings.morph_radius);
  CHECK(std::get<bool>(value("drop_top_bands")) == d.openings.drop_top_bands);
  CHECK(std::get<double>(value("guid_match_m")) == d.identity.match_m);

  const auto again = opening_options_from_parameters(
      opening_parameters_json(OpeningStageOptions{}));
  CHECK(opening_parameters_json(again) == opening_parameters_json(d));
  const auto o = opening_options_from_parameters(
      R"({"front_m": 0.8, "morph_radius": 2, "local_levels": false})");
  CHECK(o.openings.front_m == 0.8);
  CHECK(o.openings.morph_radius == 2);
  CHECK_FALSE(o.openings.local_levels);
  CHECK_THROWS_AS(opening_options_from_parameters(R"({"front_m": -1})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(opening_options_from_parameters(
                      R"({"door_min_height_m": 2.5, "door_max_height_m": 2})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(opening_options_from_parameters("[1]"),
                  std::invalid_argument);
}

TEST_CASE("OpeningsStage_NoWalls_IsRefusedAsInvalidInput",
          "[pipeline][stages][openings]") {
  TempPath project("test_openings_stage_nowalls");
  ProjectDB db(project.path);
  seed_rooms(db);
  const auto r = run(db, project, JobStage::openings);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK_FALSE(db.has_openings());
}

TEST_CASE("OpeningsStage_Rooms_FindsTheDoorAndTheWindow",
          "[pipeline][stages][openings]") {
  TempPath project("test_openings_stage_rooms");
  ProjectDB db(project.path);
  seed_walls(db, project);
  const auto r = run(db, project, JobStage::openings);
  INFO(r.message);
  REQUIRE(r.ok);
  REQUIRE(r.outputs.size() == 1);
  CHECK(r.outputs[0].name == "openings");

  const auto rows = db.openings();
  INFO(rows.size() << " openings");
  CHECK(r.outputs[0].count == static_cast<int64_t>(rows.size()));
  const auto walls = db.walls();
  const auto host = [&](const core::OpeningRecord &o) {
    return *std::find_if(walls.begin(), walls.end(),
                         [&](const auto &w) { return w.guid == o.wall; });
  };
  const auto door = std::find_if(rows.begin(), rows.end(),
                                 [&](auto &o) { return o.kind == "door"; });
  REQUIRE(door != rows.end());
  const auto dw = host(*door);
  CHECK(std::abs(dw.x0 - 2.0) < 0.1); // the partition
  const double y0 = dw.y0 + (dw.y1 > dw.y0 ? 1 : -1) * door->s_start;
  const double y1 = dw.y0 + (dw.y1 > dw.y0 ? 1 : -1) * door->s_end;
  CHECK_THAT(std::min(y0, y1), WithinAbs(1.0, 0.11));
  CHECK_THAT(std::max(y0, y1), WithinAbs(1.9, 0.11));
  CHECK_THAT(door->z_head, WithinAbs(2.1, 0.11));
  CHECK_FALSE(door->is_external);

  const auto window = std::find_if(rows.begin(), rows.end(),
                                   [&](auto &o) { return o.kind == "window"; });
  REQUIRE(window != rows.end());
  CHECK(std::abs(host(*window).y0) < 0.2); // the south wall
  CHECK_THAT(window->z_sill, WithinAbs(0.9, 0.11));
  CHECK_THAT(window->z_head, WithinAbs(2.0, 0.11));
  CHECK(window->is_external);
  CHECK(rows.size() == 2);

  for (const auto &o : rows) {
    CHECK(o.guid.size() == 36);
    CHECK(o.source == "detected");
    CHECK(o.confidence > 0.0);
  }
  const auto log = db.pipeline_log(1);
  REQUIRE(log.size() == 1);
  CHECK(log[0].stage == "openings");

  // A re-run gives the same GUIDs; so does a walls re-run in between (the
  // walls keep theirs, and the openings name them by GUID).
  const auto first = guids(db);
  REQUIRE(run(db, project, JobStage::openings).ok);
  CHECK(guids(db) == first);
  REQUIRE(run(db, project, JobStage::walls).ok);
  CHECK(db.openings().size() == rows.size());
  REQUIRE(run(db, project, JobStage::openings).ok);
  CHECK(guids(db) == first);

  const auto m = io::load_solid_model(db);
  CHECK(m.openings.size() == rows.size());
  CHECK(m.has_openings());
}

TEST_CASE("OpeningsStage_LockedRow_SurvivesARerun",
          "[pipeline][stages][openings]") {
  TempPath project("test_openings_stage_locked");
  ProjectDB db(project.path);
  seed_walls(db, project);
  REQUIRE(run(db, project, JobStage::openings).ok);
  auto rows = db.openings();
  REQUIRE_FALSE(rows.empty());
  auto &fixed = rows.front();
  fixed.z_head = 2.2; // a hand fix
  fixed.source = "manual";
  fixed.locked = true;
  db.save_openings(rows);

  const auto r = run(db, project, JobStage::openings);
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(r.message.find("1 locked") != std::string::npos);
  CHECK(db.openings().size() == rows.size());
  const auto kept = db.opening_by_guid(fixed.guid);
  REQUIRE(kept.has_value());
  CHECK(kept->locked);
  CHECK_THAT(kept->z_head, WithinAbs(2.2, 1e-12));
}
