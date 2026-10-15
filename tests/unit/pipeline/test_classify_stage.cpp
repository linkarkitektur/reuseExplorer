// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The `classify` pipeline stage (reusex-z1v.4.2): cloud + planes + storeys
// (+ labels) -> the plane_classes table, through pipeline::run_stage (the
// path `rux create classify` and the Qt Pipeline page share).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/normal_orientation.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/segmentation/classify_planes.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::pipeline;
using reusex::test_support::TempPath;

namespace {

/// One storey (floor z = 0, ceiling z = 2.7) with a wall and a table, as the
/// planes and storeys stages would leave it. Plane labels: 1 floor,
/// 2 ceiling, 3 wall, 4 table. @p oriented stamps the reusex-0se marker.
/// @p table_class labels the table's points with that class (point label 2;
/// "wall" is point label 1 on the wall).
void seed_room(ProjectDB &db, bool oriented,
               const char *table_class = nullptr) {
  Cloud cloud;
  CloudL planes;
  CloudL labels;
  CloudN normals;
  uint32_t label = 0;
  auto patch = [&](auto &&point_at, int nu, int nv, double nx, double ny,
                   double nz, uint32_t semantic) {
    ++label;
    for (int u = 0; u <= nu; ++u)
      for (int v = 0; v <= nv; ++v) {
        const auto [x, y, z] = point_at(u, v);
        PointT p;
        p.x = float(x);
        p.y = float(y);
        p.z = float(z);
        cloud.push_back(p);
        LabelT l;
        l.label = label;
        planes.push_back(l);
        LabelT s;
        s.label = semantic;
        labels.push_back(s);
      }
    NormalT n;
    n.normal_x = float(nx);
    n.normal_y = float(ny);
    n.normal_z = float(nz);
    normals.push_back(n);
  };
  using P = std::array<double, 3>;
  patch([](int u, int v) { return P{0.1 * u, 0.1 * v, 0.0}; }, 40, 40, 0, 0, 1,
        0); // floor, seen from above
  patch([](int u, int v) { return P{0.1 * u, 0.1 * v, 2.7}; }, 40, 40, 0, 0, -1,
        0); // ceiling, seen from below
  patch([](int u, int v) { return P{0.0, 0.1 * u, 0.1 * v}; }, 40, 27, 1, 0, 0,
        1); // wall
  patch([](int u, int v) { return P{1.0 + 0.1 * u, 1.0 + 0.1 * v, 0.75}; }, 10,
        8, 0, 0, 1, table_class ? 2u : 0u); // table top

  const std::string marker =
      oriented ? R"({"normal_orientation":"sensor"})" : "{}";
  db.save_point_cloud("cloud", cloud, "reconstruct", marker);
  db.save_point_cloud("planes", planes, "segment_planes", "{}");
  db.save_point_cloud("plane_normals", normals, "segment_planes", marker);
  if (table_class) {
    db.save_point_cloud("labels", labels, "reconstruct",
                        R"({"label_encoding":"api_plus_one"})");
    db.save_label_definitions("labels", {{1, "wall"}, {2, table_class}});
  }

  core::StoreyRecord s;
  s.guid = core::deterministic_guid(db.project_guid(), "storeys/z0");
  s.name = "Etage 0";
  s.z_floor = 0.0;
  s.z_ceiling = 2.7;
  db.save_storeys({s});
}

StageResult run(ProjectDB &db, const TempPath &project,
                const std::string &params = "{}") {
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::classify;
  ctx.parameters = params;
  return run_stage(db, ctx);
}

std::map<int, std::string> roles(const ProjectDB &db) {
  std::map<int, std::string> out;
  for (const auto &p : db.plane_classes())
    out[p.plane_label] = p.role;
  return out;
}

} // namespace

TEST_CASE("ClassifyStage_Names_AndContract", "[pipeline][stages][classify]") {
  CHECK(to_string(JobStage::classify) == "classify");
  CHECK(pipeline_log_name(JobStage::classify) == "classify");
  CHECK(parse_job_stage("classify") == JobStage::classify);
  const auto &c = core::stage_contract(core::PipelineStage::classify);
  CHECK(c.command == "rux create classify");
  REQUIRE(c.outputs.size() == 1);
  CHECK(c.outputs[0] == "plane_classes");
  CHECK(core::producing_stage("plane_classes") ==
        core::PipelineStage::classify);
}

TEST_CASE("ClassifyStage_Parameters_AreClassifyPlanesOptionsDefaults",
          "[pipeline][stages][classify]") {
  const geometry::ClassifyPlanesOptions d{};
  const auto &params = stage_parameters(JobStage::classify);
  auto value = [&](const char *key) -> ParameterDefault {
    for (const auto &p : params)
      if (p.key == key)
        return p.default_value;
    FAIL("no parameter " << key);
    return {};
  };
  CHECK(params.size() == 15);
  CHECK(std::get<double>(value("horizontal_cos")) == d.horizontal_cos);
  CHECK(std::get<double>(value("vertical_cos")) == d.vertical_cos);
  CHECK(std::get<double>(value("min_large_area_m2")) == d.min_large_area_m2);
  CHECK(std::get<double>(value("band_m")) == d.band_m);
  CHECK(std::get<double>(value("min_wall_height_m")) == d.min_wall_height_m);
  CHECK(std::get<double>(value("min_wall_height_share")) ==
        d.min_wall_height_share);
  CHECK(std::get<double>(value("max_furniture_area_m2")) ==
        d.max_furniture_area_m2);
  CHECK(std::get<double>(value("table_min_m")) == d.table_min_m);
  CHECK(std::get<double>(value("table_max_m")) == d.table_max_m);
  CHECK(std::get<double>(value("min_storey_height_m")) ==
        d.min_storey_height_m);
  CHECK(std::get<double>(value("area_cell_m")) == d.area_cell_m);
  CHECK(std::get<double>(value("extent_quantile")) == d.extent_quantile);
  CHECK(std::get<double>(value("min_labelled_share")) == d.min_labelled_share);
  CHECK(std::get<double>(value("min_class_share")) == d.min_class_share);
  CHECK(std::get<std::string>(value("role_map")).empty());
}

TEST_CASE("ClassifyStage_EmptyProject_IsRefusedAsInvalidInput",
          "[pipeline][stages][classify]") {
  TempPath project("test_classify_stage_empty");
  ProjectDB db(project.path);
  const auto r = run(db, project);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK_FALSE(db.has_plane_classes());
}

TEST_CASE("ClassifyStage_NoStoreys_IsRefusedAsInvalidInput",
          "[pipeline][stages][classify]") {
  TempPath project("test_classify_stage_no_storeys");
  ProjectDB db(project.path);
  seed_room(db, /*oriented=*/true);
  db.delete_storeys();
  const auto r = run(db, project);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("storeys") != std::string::npos);
}

TEST_CASE("ClassifyStage_Room_WritesRolesWithStableGuids",
          "[pipeline][stages][classify]") {
  TempPath project("test_classify_stage_room");
  ProjectDB db(project.path);
  seed_room(db, /*oriented=*/true);

  const auto r = run(db, project);
  INFO(r.message);
  REQUIRE(r.ok);
  REQUIRE(r.outputs.size() == 1);
  CHECK(r.outputs[0].kind == "table");
  CHECK(r.outputs[0].name == "plane_classes");
  CHECK(r.outputs[0].count == 4);
  CHECK(r.message.find("sensor-facing") != std::string::npos);

  CHECK(roles(db) ==
        std::map<int, std::string>{
            {1, "floor"}, {2, "ceiling"}, {3, "wall"}, {4, "furniture"}});
  const auto rows = db.plane_classes();
  const auto log = db.pipeline_log(1);
  REQUIRE(log.size() == 1);
  CHECK(log[0].stage == "classify");
  for (const auto &p : rows) {
    CHECK(p.guid == core::deterministic_guid(
                        db.project_guid(), "plane_classes/planes/" +
                                               std::to_string(p.plane_label)));
    CHECK(p.source == "detected");
    CHECK(p.role_source == "rules"); // no labels cloud
    CHECK(p.storey == 0);
    const auto meta = nlohmann::json::parse(p.metadata);
    CHECK(meta.at("oriented_normals") == true);
  }

  // A re-run keeps the GUIDs and the row count.
  REQUIRE(run(db, project).ok);
  const auto again = db.plane_classes();
  REQUIRE(again.size() == rows.size());
  for (size_t i = 0; i < rows.size(); ++i)
    CHECK(again[i].guid == rows[i].guid);
}

TEST_CASE("ClassifyStage_LegacyNormals_RunWithoutFacing",
          "[pipeline][stages][classify]") {
  TempPath project("test_classify_stage_legacy");
  ProjectDB db(project.path);
  seed_room(db, /*oriented=*/false);
  const auto r = run(db, project);
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(r.message.find("legacy") != std::string::npos);
  const auto rows = db.plane_classes();
  REQUIRE(rows.size() == 4);
  CHECK(nlohmann::json::parse(rows[0].metadata).at("oriented_normals") ==
        false);
}

TEST_CASE("ClassifyStage_Labels_AndRoleMapFile",
          "[pipeline][stages][classify]") {
  TempPath project("test_classify_stage_labels");
  ProjectDB db(project.path);
  seed_room(db, /*oriented=*/true, "desk");

  REQUIRE(run(db, project).ok);
  auto rows = db.plane_classes();
  REQUIRE(rows.size() == 4);
  CHECK(rows[2].role == "wall");
  CHECK(rows[2].role_source == "semantic");
  CHECK(rows[2].semantic_class == 0); // API class of point label 1
  CHECK(rows[3].role == "furniture"); // built-in map: desk -> furniture
  CHECK(rows[3].role_source == "semantic");

  TempPath map_file("test_classify_stage_role_map.json");
  {
    std::ofstream out(map_file.path);
    out << R"({"version":1,"classes":{"wall":"wall","desk":"clutter"}})";
  }
  const auto r = run(
      db, project, nlohmann::json{{"role_map", map_file.path.string()}}.dump());
  INFO(r.message);
  REQUIRE(r.ok);
  rows = db.plane_classes();
  CHECK(rows[3].role == "clutter");

  // A role map that does not exist is the caller's mistake.
  const auto bad = run(db, project, R"({"role_map":"/no/such/map.json"})");
  CHECK_FALSE(bad.ok);
  CHECK(bad.invalid_input);
}

TEST_CASE("ClassifyStage_LockedRow_SurvivesARerun",
          "[pipeline][stages][classify]") {
  TempPath project("test_classify_stage_locked");
  ProjectDB db(project.path);
  seed_room(db, /*oriented=*/true);
  REQUIRE(run(db, project).ok);

  auto rows = db.plane_classes();
  rows[3].role = "slab"; // a hand fix of the table
  rows[3].role_source = "manual";
  rows[3].source = "manual";
  rows[3].locked = true;
  db.save_plane_classes("planes", rows);

  const auto r = run(db, project);
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(r.message.find("1 locked") != std::string::npos);
  const auto again = db.plane_classes();
  REQUIRE(again.size() == 4);
  CHECK(again[3].role == "slab");
  CHECK(again[3].locked);
}
