// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The `solids` pipeline stage (W6.1, reusex-z1v.8.1): storeys + walls +
// openings + spaces -> the meshes solid_walls / solid_slabs / solid_spaces,
// through pipeline::run_stage (the path `rux create solids` and the Qt
// Pipeline page share).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/pipeline/solid_parameters.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/pipeline/stages.hpp>

#include <nlohmann/json.hpp>
#include <pcl/PolygonMesh.h>

#include "../../support/temp_path.hpp"

#include <string>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using namespace reusex::pipeline;
using reusex::test_support::TempPath;
using json = nlohmann::json;

namespace {

StageResult run(ProjectDB &db, const TempPath &project,
                const std::string &params = "{}") {
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::solids;
  ctx.parameters = params;
  return run_stage(db, ctx);
}

/// One storey, two rooms either side of a partition with a door in it, and
/// a window in the south wall.
core::SolidModel two_rooms() {
  core::SolidModel m;
  core::StoreyRecord st;
  st.guid = "st0";
  st.z_floor = 0.0;
  st.z_ceiling = 2.5;
  m.storeys = {st};
  const auto wall = [](const char *g, double x0, double y0, double x1,
                       double y1) {
    core::WallRecord w;
    w.guid = g;
    w.x0 = x0;
    w.y0 = y0;
    w.x1 = x1;
    w.y1 = y1;
    w.thickness = 0.2;
    w.height = 2.5;
    return w;
  };
  m.walls = {wall("s", 0, 0, 8, 0), wall("e", 8, 0, 8, 4),
             wall("n", 8, 4, 0, 4), wall("w", 0, 4, 0, 0),
             wall("p", 4, 0, 4, 4)};
  core::OpeningRecord door;
  door.guid = "d";
  door.wall = "p";
  door.kind = "door";
  door.s_start = 1.0;
  door.s_end = 1.9;
  door.z_sill = 0.0;
  door.z_head = 2.1;
  core::OpeningRecord window = door;
  window.guid = "o";
  window.wall = "s";
  window.kind = "window";
  window.s_start = 5.0;
  window.s_end = 6.5;
  window.z_sill = 0.9;
  window.z_head = 2.0;
  m.openings = {door, window};
  const auto space = [](const char *g, double x0, double x1) {
    core::SpaceRecord s;
    s.guid = g;
    s.kind = "room";
    s.polygon.outer = {{x0, 0.1}, {x1, 0.1}, {x1, 3.9}, {x0, 3.9}};
    s.z_floor = 0.0;
    s.z_ceiling = 2.5;
    return s;
  };
  m.spaces = {space("a", 0.1, 3.9), space("b", 4.1, 7.9)};
  return m;
}

/// The model's tables, as the storeys / walls / openings / spaces stages
/// would leave them.
void store(ProjectDB &db, const core::SolidModel &m) {
  db.save_storeys(m.storeys);
  db.save_walls(m.walls, m.wall_joins);
  db.save_openings(m.openings);
  db.save_spaces(m.spaces, {});
}

} // namespace

TEST_CASE("SolidsStage_Identity", "[pipeline][solids]") {
  CHECK(to_string(JobStage::solids) == "solids");
  CHECK(pipeline_log_name(JobStage::solids) == "solids");
  CHECK(parse_job_stage("solids") == JobStage::solids);
  const auto &params = stage_parameters(JobStage::solids);
  REQUIRE(params.size() == solid_parameter_table().size());
  const geometry::SolidsOptions d{};
  for (const auto &p : params)
    if (p.key == "slab_thickness_m")
      CHECK(std::get<double>(p.default_value) == d.slab_thickness_m);
}

TEST_CASE("SolidsStage_Parameters_RoundTripAndBounds", "[pipeline][solids]") {
  SolidStageOptions o;
  o.solids.slab_thickness_m = 0.3;
  o.solids.roof = false;
  const auto back = solid_options_from_parameters(solid_parameters_json(o));
  CHECK(back.solids.slab_thickness_m == 0.3);
  CHECK_FALSE(back.solids.roof);
  CHECK_THROWS_AS(solid_options_from_parameters(R"({"slab_thickness_m": -1})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(solid_options_from_parameters(R"({"roof": 3})"),
                  std::invalid_argument);
}

TEST_CASE("SolidsStage_NoWalls_IsRefused", "[pipeline][solids]") {
  TempPath project("test_solids_stage_refused");
  ProjectDB db(project.path);
  const auto r = run(db, project);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
}

TEST_CASE("SolidsStage_TwoRooms_WritesThreeMeshesWithAnIndex",
          "[pipeline][solids]") {
  TempPath project("test_solids_stage_two_rooms");
  ProjectDB db(project.path);
  store(db, two_rooms());
  const auto r = run(db, project);
  INFO(r.message);
  REQUIRE(r.ok);
  REQUIRE(r.outputs.size() == 3);
  CHECK(r.outputs[0].kind == "mesh");
  CHECK(r.outputs[0].name == kSolidWallsMesh);
  for (auto name : {kSolidWallsMesh, kSolidSlabsMesh, kSolidSpacesMesh})
    CHECK(db.has_mesh(name));

  // The element index in the wall mesh's parameters.
  const auto meta = db.mesh_metadata(kSolidWallsMesh);
  CHECK(meta.stage == "solids");
  const json j = json::parse(meta.parameters);
  CHECK(j.at("solids_version") == 1);
  CHECK(j.at("type") == "wall");
  const auto &elements = j.at("elements");
  REQUIRE(elements.size() == 5);
  std::size_t faces = 0;
  int voids = 0;
  for (const auto &e : elements) {
    CHECK(e.at("closed").get<bool>());
    CHECK(e.at("faces").at(0).get<std::size_t>() == faces);
    faces += e.at("faces").at(1).get<std::size_t>();
    voids += static_cast<int>(e.at("voids").size());
    CHECK(e.at("profile").at("placement").size() == 16);
  }
  CHECK(voids == 2);
  CHECK(faces == static_cast<std::size_t>(meta.face_count));
  // The partition carries the door.
  bool door = false;
  for (const auto &e : elements)
    if (e.at("guid") == "p") {
      REQUIRE(e.at("voids").size() == 1);
      door = e.at("voids").at(0).at("guid") == "d";
    }
  CHECK(door);

  const json slabs = json::parse(db.mesh_metadata(kSolidSlabsMesh).parameters);
  REQUIRE(slabs.at("elements").size() == 2); // floor + roof
  CHECK(slabs.at("elements").at(0).at("guid") == "st0");
  CHECK(slabs.at("elements").at(0).at("predefined") == "FLOOR");
  CHECK(slabs.at("elements").at(1).at("predefined") == "ROOF");
  const json spaces =
      json::parse(db.mesh_metadata(kSolidSpacesMesh).parameters);
  REQUIRE(spaces.at("elements").size() == 2);
  CHECK_THAT(spaces.at("elements").at(0).at("volume").get<double>(),
             WithinAbs(3.8 * 3.8 * 2.5, 1e-6));
}

TEST_CASE("SolidsStage_Rerun_WithoutSpaces_DropsTheStaleSpacesMesh",
          "[pipeline][solids]") {
  TempPath project("test_solids_stage_rerun");
  ProjectDB db(project.path);
  store(db, two_rooms());
  REQUIRE(run(db, project).ok);
  REQUIRE(db.has_mesh(kSolidSpacesMesh));
  db.delete_spaces();
  const auto r = run(db, project, R"({"roof": false})");
  REQUIRE(r.ok);
  CHECK_FALSE(db.has_mesh(kSolidSpacesMesh));
  CHECK(r.outputs.size() == 2);
  const json slabs = json::parse(db.mesh_metadata(kSolidSlabsMesh).parameters);
  CHECK(slabs.at("elements").size() == 1); // the walls' hull, no roof
  CHECK(slabs.at("parameters").at("roof") == false);
}
