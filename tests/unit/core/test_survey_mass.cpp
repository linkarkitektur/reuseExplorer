// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Survey quantities and mass grounded in the model (schema v37,
// reusex-uprf): the build-up library, measure_survey (walls, slabs, roofs,
// openings, counts), the mass calculation with the user override, value
// provenance, and the v36 -> v37 migration of the "Opbygning" user column.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/buildups.hpp>
#include <core/resources.hpp>
#include <core/survey_mass.hpp>
#include <core/survey_measure.hpp>
#include <core/survey_service.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <sqlite3.h>

#include <algorithm>
#include <filesystem>
#include <initializer_list>
#include <string>

using Catch::Approx;
using reusex::ProjectDB;
using reusex::test_support::TempPath;
using namespace reusex::core;
namespace fs = std::filesystem;

namespace {

void exec_raw(const fs::path &path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  INFO((err ? err : ""));
  sqlite3_free(err);
  sqlite3_close(raw);
  REQUIRE(rc == SQLITE_OK);
}

reusex::CloudL label_cloud(std::initializer_list<std::uint32_t> labels) {
  reusex::CloudL c;
  for (auto l : labels) {
    pcl::Label p;
    p.label = l;
    c.push_back(p);
  }
  return c;
}

pcl::PolygonMesh triangle() {
  pcl::PointCloud<pcl::PointXYZ> pts;
  pts.push_back({0, 0, 0});
  pts.push_back({1, 0, 0});
  pts.push_back({0, 1, 0});
  pcl::PolygonMesh m;
  pcl::toPCLPointCloud2(pts, m.cloud);
  pcl::Vertices v;
  v.vertices = {0, 1, 2};
  m.polygons.push_back(v);
  return m;
}

constexpr const char *kStoreyGuid = "0b6c7a8e-5d2f-5c41-9e1a-3f7d2b9c4a10";

/// Storey 0; an external wall 10 m × 3 m (measured 0.35 m) with a 1 × 2 m
/// door detected from instance 1; an interior wall 4 m × 3 m, thickness
/// assumed; a floor slab of 20 m³ and a roof of 10 m³ at 0.2 m.
void seed_model(ProjectDB &db) {
  StoreyRecord s;
  s.guid = kStoreyGuid;
  s.idx = 0;
  s.name = "Stueetage";
  s.z_ceiling = 3.0;
  db.save_storeys({s});

  WallRecord ext;
  ext.guid = "wall-ext";
  ext.x1 = 10.0;
  ext.height = 3.0;
  ext.thickness = 0.35;
  ext.is_external = true;
  WallRecord in;
  in.guid = "wall-in";
  in.y1 = 4.0;
  in.height = 3.0;
  in.thickness = -1.0;
  in.assumed = true;
  db.save_walls({ext, in}, {});

  OpeningRecord door;
  door.guid = "door-1";
  door.wall = "wall-ext";
  door.kind = "door";
  door.s_start = 2.0;
  door.s_end = 3.0;
  door.z_sill = 0.0;
  door.z_head = 2.0;
  door.source_instance_guid = "g-door";
  db.save_openings({door});

  db.save_point_cloud("instances", label_cloud({1, 1, 2}), "test", "{}");
  db.save_instances("instances", {{1, "g-door", 6, 2}, {2, "g-pipe", 13, 1}});

  using json = nlohmann::json;
  const json walls = {{"solids_version", 1},
                      {"type", "wall"},
                      {"elements",
                       {{{"guid", "wall-ext"},
                         {"type", "wall"},
                         {"predefined", "STANDARD"},
                         {"part", 0},
                         {"storey", 0},
                         {"volume", 9.8},
                         {"closed", true},
                         {"profile", {{"depth", 3.0}}}}}}};
  db.save_mesh("solid_walls", triangle(), "solids", walls.dump());
  const json slabs = {{"solids_version", 1},
                      {"type", "slab"},
                      {"elements",
                       {{{"guid", kStoreyGuid},
                         {"type", "slab"},
                         {"predefined", "FLOOR"},
                         {"part", 0},
                         {"storey", 0},
                         {"volume", 20.0},
                         {"closed", true},
                         {"profile", {{"depth", 0.2}}}},
                        {{"guid", kStoreyGuid},
                         {"type", "slab"},
                         {"predefined", "ROOF"},
                         {"part", 0},
                         {"storey", 0},
                         {"volume", 10.0},
                         {"closed", true},
                         {"profile", {{"depth", 0.2}}}}}}};
  db.save_mesh("solid_slabs", triangle(), "solids", slabs.dump());

  ProjectDB::SurveyTypeRecord doors;
  doors.name = "door";
  doors.semantic_class = 6;
  const auto door_type = db.add_survey_type(doors).id;
  ProjectDB::SurveyTypeRecord pipes;
  pipes.name = "pipe";
  pipes.semantic_class = 13;
  const auto pipe_type = db.add_survey_type(pipes).id;
  ProjectDB::SurveyPartRecord p1;
  p1.code = "RX-001";
  p1.type_id = door_type;
  p1.cloud_name = "instances";
  p1.instance_id = 1;
  db.add_survey_part(p1);
  ProjectDB::SurveyPartRecord p2;
  p2.code = "RX-002";
  p2.type_id = pipe_type;
  p2.cloud_name = "instances";
  p2.instance_id = 2;
  db.add_survey_part(p2);
}

std::optional<ProjectDB::SurveyTypeRecord> type_named(const ProjectDB &db,
                                                      const std::string &n) {
  for (const auto &t : db.survey_types())
    if (t.name == n)
      return t;
  return std::nullopt;
}

std::vector<ProjectDB::SurveyPartRecord> parts_of(const ProjectDB &db,
                                                  int64_t type_id) {
  std::vector<ProjectDB::SurveyPartRecord> out;
  for (const auto &p : db.survey_parts())
    if (p.type_id == type_id)
      out.push_back(p);
  return out;
}

} // namespace

TEST_CASE("Buildups_DefaultLibrary_ParsesAndMatchesTheFrontend",
          "[survey][buildups]") {
  const auto &lib = default_buildups();
  REQUIRE(lib.size() >= 8);
  const auto b1 = find_buildup(lib, "b1");
  REQUIRE(b1);
  CHECK(b1->category == BuildupCategory::ydervaeg);
  CHECK(b1->main.t == Approx(0.216));
  CHECK(layered_thickness(*b1) == Approx(0.291));
  CHECK(find_buildup_by_name(lib, " Betondæk, armeret ")->id == "b5");
  // area × t × rho, area × kg/m², volume × rho
  CHECK(layer_tonnes(b1->main, 100.0, kUnitArea) == Approx(38.88));
  CHECK(layer_tonnes(find_buildup(lib, "b7")->main, 100.0, kUnitArea) ==
        Approx(0.5));
  CHECK(layer_tonnes(find_buildup(lib, "b5")->main, 10.0, kUnitVolume) ==
        Approx(24.0));
  // Round trip.
  CHECK(parse_buildup(to_json(*b1)).others.size() == 1);
  CHECK_THROWS_AS(parse_buildup(R"({"id":"x","name":"x","category":"loft",
                    "main":{"rho":1}})"),
                  std::invalid_argument);
  CHECK_THROWS_AS(parse_buildup(R"({"id":"x","name":"x","category":"tag",
                    "main":{"material":"luft"}})"),
                  std::invalid_argument);
}

TEST_CASE("Buildups_CategoryOf_NameThenBim7aa", "[survey][buildups]") {
  CHECK(category_of("Etagedæk", "") == BuildupCategory::daek);
  CHECK(category_of("Tagpap", "") == BuildupCategory::tag);
  CHECK(category_of("Ydervæg, tegl", "") == BuildupCategory::ydervaeg);
  CHECK(category_of("Let skillevæg", "") == BuildupCategory::indervaeg);
  CHECK(category_of("Noget", "22.1") == BuildupCategory::indervaeg);
  CHECK_FALSE(category_of("Radiator", "").has_value());
}

TEST_CASE("Buildups_ProjectOverrideReplacesAndAdds", "[survey][buildups]") {
  TempPath tmp("test_survey_mass_buildup_override");
  ProjectDB db(tmp.path);
  db.set_project_buildup("b5",
                         R"({"id":"b5","category":"daek","name":"Betondæk, let",
               "main":{"material":"beton","t":1,"rho":2000,"eak":"17.01.01"}})");
  db.set_project_buildup("p1", R"({"id":"p1","category":"tag","name":"Skifer",
               "main":{"material":"skifer","kg_m2":30,"eak":"17.09.04"}})");
  db.set_project_buildup("bad", "{not json");
  const auto lib = buildup_library(db);
  CHECK(find_buildup(lib, "b5")->main.rho == Approx(2000));
  CHECK(find_buildup(lib, "b5")->origin == "project");
  CHECK(find_buildup(lib, "p1")->main.kg_m2 == Approx(30));
  CHECK_FALSE(find_buildup(lib, "bad"));
  CHECK(lib.size() == default_buildups().size() + 1);
  CHECK(db.delete_project_buildup("b5"));
  CHECK(find_buildup(buildup_library(db), "b5")->main.rho == Approx(2400));
}

TEST_CASE("SurveyMeasure_GroundsPartsInTheModel", "[survey][measure]") {
  TempPath tmp("test_survey_measure_model");
  ProjectDB db(tmp.path);
  seed_model(db);

  const auto r = measure_survey(db);
  CHECK(r.types_created == 4); // Ydervæg, Indervæg, Etagedæk, Tag
  CHECK(r.parts_created == 4); // 2 walls, 1 floor, 1 roof
  CHECK(r.wall_parts == 2);
  CHECK(r.slab_parts == 1);
  CHECK(r.roof_parts == 1);
  CHECK(r.opening_parts == 1);
  CHECK(r.count_parts == 1);

  // The door part is its opening: 1 m × 2 m. The type is in stk, so the
  // quantity stays a count.
  const auto door = db.survey_part("RX-001");
  REQUIRE(door);
  CHECK(door->element_guid == "door-1");
  CHECK(door->geom_source == "opening_area");
  CHECK(*door->geom_quantity == Approx(2.0));
  CHECK(door->geom_unit == "m²");
  CHECK(door->quantity == 1.0);
  const auto pipe = db.survey_part("RX-002");
  CHECK(pipe->geom_source == "count");
  CHECK_FALSE(pipe->element_guid);

  // External wall: 10 × 3 − the door's 2 m² = 28 m²; height from its solid.
  const auto ext = db.survey_part_for_element("wall-ext");
  REQUIRE(ext);
  CHECK(ext->geom_source == "wall_area");
  CHECK(*ext->geom_quantity == Approx(28.0));
  CHECK(ext->quantity == Approx(28.0)); // seeded: same unit, not entered
  CHECK(ext->room_name == "Stueetage");
  const auto yt = type_named(db, "Ydervæg");
  REQUIRE(yt);
  CHECK(yt->unit == "m²");
  CHECK(yt->element_category == "ydervaeg");
  CHECK(yt->semantic_class == kManualSemanticClass);
  // Measured 0.35 m: closest layered thickness among the ydervæg build-ups
  // is b2 (348 mm), and its EAK fills the empty code.
  CHECK(yt->buildup_id == "b2");
  CHECK(yt->eak_code == "17.01.02");
  // Interior wall: thickness assumed, so the category's first build-up.
  const auto it = type_named(db, "Indervæg");
  CHECK(it->buildup_id == "b4");
  CHECK(*db.survey_part_for_element("wall-in")->geom_quantity == Approx(12.0));

  const auto floor = parts_of(db, type_named(db, "Etagedæk")->id);
  REQUIRE(floor.size() == 1);
  CHECK(floor[0].geom_source == "solid_volume");
  CHECK(floor[0].quantity == Approx(20.0));
  const auto roof = parts_of(db, type_named(db, "Tag")->id);
  REQUIRE(roof.size() == 1);
  CHECK(roof[0].geom_source == "slab_area");
  CHECK(roof[0].quantity == Approx(50.0)); // 10 m³ / 0.2 m

  // Provenance: measured quantity, suggested build-up.
  const auto qp = part_provenance(db, ext->code);
  REQUIRE(qp.size() == 1);
  CHECK(qp[0].field == "quantity");
  CHECK(qp[0].kind == "measured");
  const auto tp = db.value_provenance("type", std::to_string(yt->id));
  CHECK(std::any_of(tp.begin(), tp.end(), [](const auto &row) {
    return row.field == "buildup_id" && row.kind == "suggested";
  }));

  // Idempotent: nothing new on a re-run.
  const auto again = measure_survey(db);
  CHECK(again.parts_created == 0);
  CHECK(again.types_created == 0);
  CHECK(again.quantities_seeded == 0);
  CHECK(again.buildups_suggested == 0);
}

TEST_CASE("SurveyMeasure_RespectsEnteredQuantityAndDeletedElements",
          "[survey][measure]") {
  TempPath tmp("test_survey_measure_entered");
  ProjectDB db(tmp.path);
  seed_model(db);
  measure_survey(db);
  const auto ext = *db.survey_part_for_element("wall-ext");

  ProjectDB::SurveyPartPatch q;
  q.quantity = 25.0;
  update_survey_part_entered(db, ext.code, q);
  measure_survey(db);
  CHECK(db.survey_part(ext.code)->quantity == 25.0); // kept
  CHECK(*db.survey_part(ext.code)->geom_quantity == Approx(28.0));
  CHECK(part_provenance(db, ext.code)[0].kind == "entered");

  // A deleted element part is not re-created.
  const auto in = *db.survey_part_for_element("wall-in");
  delete_resource(db, in.code);
  CHECK(measure_survey(db).parts_created == 0);
  CHECK_FALSE(db.survey_part_for_element("wall-in"));
  CHECK(part_provenance(db, in.code).empty()); // went with the part

  // A wall that disappears from the model: the part stays, its geometry is
  // cleared.
  auto walls = db.walls();
  walls.erase(
      std::remove_if(walls.begin(), walls.end(),
                     [](const auto &w) { return w.guid == "wall-ext"; }),
      walls.end());
  db.save_walls(walls, {});
  const auto r = measure_survey(db);
  CHECK(r.elements_missing >= 1);
  const auto gone = db.survey_part(ext.code);
  REQUIRE(gone);
  CHECK_FALSE(gone->geom_quantity);
  CHECK(gone->element_guid == "wall-ext");
}

TEST_CASE("SurveyMass_BuildupTimesQuantity_OverrideWins", "[survey][mass]") {
  TempPath tmp("test_survey_mass_calc");
  ProjectDB db(tmp.path);
  seed_model(db);
  measure_survey(db);

  // Ydervæg b2 is one layer, and the wall's thickness was measured: 28 m² ×
  // 0.35 m × 1800 kg/m³ = 17.64 t.
  const auto yt = *type_named(db, "Ydervæg");
  auto m = type_mass(db, yt.id);
  REQUIRE(m.calc_t);
  CHECK(*m.calc_t == Approx(17.64));
  CHECK(m.method == "areal × målt tykkelse × densitet");
  CHECK(m.source == "calculated");
  CHECK(*m.effective_t == Approx(17.64));

  // Etagedæk b5: 20 m³ × 2400 kg/m³ = 48 t. Tag b7: 50 m² × 5 kg/m² = 0.25 t.
  CHECK(*type_mass(db, type_named(db, "Etagedæk")->id).calc_t == Approx(48.0));
  CHECK(*type_mass(db, type_named(db, "Tag")->id).calc_t == Approx(0.25));
  // Indervæg b4 (gips, 26 mm, two layers): 12 m² × 0.026 × 900 = 0.2808 t.
  CHECK(*type_mass(db, type_named(db, "Indervæg")->id).calc_t ==
        Approx(0.2808));
  // stk types have no calculation.
  CHECK_FALSE(type_mass(db, type_named(db, "door")->id).calc_t);

  // The override wins; clearing it brings the calculation back.
  ProjectDB::SurveyTypePatch o;
  o.mass_t = 20.0;
  update_survey_type_entered(db, yt.id, o);
  m = type_mass(db, yt.id);
  CHECK(*m.effective_t == 20.0);
  CHECK(m.source == "override");
  CHECK(*m.calc_t == Approx(17.64));
  auto prov = type_provenance(db, m);
  auto mass_row = std::find_if(prov.begin(), prov.end(), [](const auto &r) {
    return r.field == "mass_t";
  });
  REQUIRE(mass_row != prov.end());
  CHECK(mass_row->kind == "entered");
  o.mass_t = std::optional<double>{};
  update_survey_type_entered(db, yt.id, o);
  m = type_mass(db, yt.id);
  CHECK(m.source == "calculated");
  prov = type_provenance(db, m);
  mass_row = std::find_if(prov.begin(), prov.end(),
                          [](const auto &r) { return r.field == "mass_t"; });
  REQUIRE(mass_row != prov.end());
  CHECK(mass_row->kind == "calculated");

  // A dismissed part is not counted.
  const auto ext = *db.survey_part_for_element("wall-ext");
  ProjectDB::SurveyPartPatch d;
  d.dismissed = true;
  db.update_survey_part(ext.code, d);
  CHECK_FALSE(type_mass(db, yt.id).calc_t);

  // type_totals (and so the report) see the effective mass.
  bool seen = false;
  for (const auto &t : type_totals(db))
    if (t.type_id == type_named(db, "Etagedæk")->id) {
      CHECK(*t.mass_t == Approx(48.0));
      seen = true;
    }
  CHECK(seen);
}

TEST_CASE("SurveyMass_MultiLayerKeepsLayersAndWarnsOnThickness",
          "[survey][mass]") {
  TempPath tmp("test_survey_mass_layers");
  ProjectDB db(tmp.path);
  seed_model(db);
  measure_survey(db);
  const auto yt = *type_named(db, "Ydervæg");
  ProjectDB::SurveyTypePatch b;
  b.buildup_id = std::string("b3"); // 108 tegl + 75 + 100 = 283 mm
  update_survey_type_entered(db, yt.id, b);
  const auto m = type_mass(db, yt.id);
  // Main layer only: 28 × 0.108 × 1800 = 5.4432 t; 350 vs 283 mm is 24 %.
  CHECK(*m.calc_t == Approx(5.4432));
  CHECK(m.method == "areal × tykkelse × densitet");
  CHECK(m.warnings.empty());
  b.buildup_id = std::string("b9"); // a single 108 mm layer
  update_survey_type_entered(db, yt.id, b);
  CHECK(type_mass(db, yt.id).method == "areal × målt tykkelse × densitet");
  // An unknown id is a warning, not a crash.
  b.buildup_id = std::string("nope");
  update_survey_type_entered(db, yt.id, b);
  const auto u = type_mass(db, yt.id);
  CHECK_FALSE(u.calc_t);
  CHECK(u.warnings.size() == 1);
  // A user's choice (even clearing it) is never re-suggested.
  b.buildup_id = std::optional<std::string>{};
  update_survey_type_entered(db, yt.id, b);
  CHECK(measure_survey(db).buildups_suggested == 0);
  CHECK_FALSE(db.survey_type(yt.id)->buildup_id);
}

TEST_CASE("SurveyMass_ResourceMassReadsEffectiveAndRecordsEntered",
          "[survey][mass][resources]") {
  TempPath tmp("test_survey_mass_resource");
  ProjectDB db(tmp.path);
  seed_model(db);
  measure_survey(db);
  const auto floor = parts_of(db, type_named(db, "Etagedæk")->id)[0];
  CHECK(
      value_of(resource(db, floor.code, std::vector<std::string>{"sys:mass_t"}),
               "sys:mass_t") == "48");
  patch_resource(db, floor.code, {{"sys:quantity", "10"}});
  CHECK(part_provenance(db, floor.code)[0].kind == "entered");
  CHECK(
      value_of(resource(db, floor.code, std::vector<std::string>{"sys:mass_t"}),
               "sys:mass_t") == "24");
}

TEST_CASE("SurveyMass_MigrationFromV36_TakesTheOpbygningColumn",
          "[projectdb][migration][survey]") {
  TempPath tmp("test_migration_v37_buildups");
  std::string type_id;
  {
    ProjectDB db(tmp.path);
    ProjectDB::SurveyTypeRecord t;
    t.name = "Etagedæk";
    t.unit = "m³";
    t.mass_t = 12.0;
    type_id = std::to_string(db.add_survey_type(t).id);
    for (const char *code : {"RX-001", "RX-002", "RX-003"}) {
      ProjectDB::SurveyPartRecord p;
      p.code = code;
      p.type_id = std::stoll(type_id);
      db.add_survey_part(p);
    }
    ProjectDB::Transaction tx(db);
    for (const char *code : {"RX-001", "RX-002"})
      db.set_passport_property(db.ensure_resource_passport(code), "Opbygning",
                               "Betondæk, armeret");
    db.set_passport_property(db.ensure_resource_passport("RX-003"), "Opbygning",
                             "Huldæk, beton");
    tx.commit();
  }
  exec_raw(tmp.path, R"(
    DROP INDEX idx_survey_parts_element;
    ALTER TABLE survey_parts DROP COLUMN element_guid;
    ALTER TABLE survey_parts DROP COLUMN geom_quantity;
    ALTER TABLE survey_parts DROP COLUMN geom_unit;
    ALTER TABLE survey_parts DROP COLUMN geom_source;
    ALTER TABLE survey_types DROP COLUMN buildup_id;
    ALTER TABLE survey_types DROP COLUMN element_category;
    DROP TABLE buildups;
    DROP TABLE value_provenance;
    DELETE FROM schema_version WHERE version >= 37;
  )");
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 36);
    CHECK_FALSE(ro.has_survey_mass_columns());
    REQUIRE(ro.survey_types().size() == 1);
    CHECK_FALSE(ro.survey_types()[0].buildup_id);
    CHECK(ro.value_provenance("type", type_id).empty());
    // A v36 read still computes: no build-up, the stored mass is the
    // override.
    const auto m = type_mass(ro, std::stoll(type_id));
    CHECK(*m.effective_t == 12.0);
  }
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    const auto t = db.survey_type(std::stoll(type_id));
    CHECK(t->buildup_id == "b5"); // two parts of three
    CHECK(t->mass_t == 12.0);     // kept, now an override
    const auto prov = db.value_provenance("type", type_id);
    REQUIRE(prov.size() == 1);
    CHECK(prov[0].kind == "entered");
    CHECK(prov[0].source == "migrated:Opbygning");
  }
}
