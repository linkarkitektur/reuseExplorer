// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Resources (spec §4): values by key id, writes routed by scope (type,
// part, passport), lazy passports, add/delete, and user columns whose
// rename carries their stored values.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/logging.hpp>
#include <core/materialepas_json_export.hpp>
#include <core/resource_keys.hpp>
#include <core/resources.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <sqlite3.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_resources") {}
};
std::string lex(const char *field) {
  for (const auto &f : leksikon_fields())
    if (f.field_name == field)
      return "lex:" + f.guid;
  FAIL("no leksikon field " << field);
  return {};
}
const ResourceKey &lex_key(const char *field) {
  static const auto keys = leksikon_keys();
  for (const auto &k : keys)
    if (k.field == field)
      return k;
  FAIL("no leksikon key " << field);
  return keys.front();
}
/// Every passport through the path `rux export materialepas` takes.
void export_all(const ProjectDB &db) {
  for (const auto &p : db.all_material_passports())
    (void)json_export::to_json_with_defaults(p);
}
/// A value @p k accepts.
std::string valid_value(const ResourceKey &k) {
  if (k.data_type == "number")
    return "3";
  if (k.data_type == "boolean")
    return "true";
  if (k.data_type == "enum")
    return k.options.front();
  if (k.data_type == "multiselect")
    return nlohmann::json::array({k.options.front(), k.options.back()}).dump();
  if (k.data_type == "date")
    return "2026-10-02";
  return "tekst";
}
void raw_exec(const reusex::test_support::TempPath &t, const char *sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(t.path.string().c_str(), &raw) == SQLITE_OK);
  REQUIRE(sqlite3_exec(raw, sql, nullptr, nullptr, nullptr) == SQLITE_OK);
  sqlite3_close(raw);
}
/// Captures library log lines for the scope of one test.
struct LogCapture {
  std::vector<std::pair<LogLevel, std::string>> lines;
  LogLevel saved = get_log_level();
  LogCapture() {
    set_log_handler([this](LogLevel l, std::string_view m) {
      lines.emplace_back(l, std::string(m));
    });
    set_log_level(LogLevel::info);
  }
  ~LogCapture() {
    reset_log_handler();
    set_log_level(saved);
  }
  std::size_t count(LogLevel l, std::string_view needle) const {
    return static_cast<std::size_t>(
        std::count_if(lines.begin(), lines.end(), [&](const auto &e) {
          return e.first == l && e.second.find(needle) != std::string::npos;
        }));
  }
};
} // namespace

TEST_CASE("Resources_List_NoTemplate_BuiltinsPlusStored", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-002", t, std::nullopt, 2.5);
  make_part(db, "RX-001", t);
  const auto list = list_resources(db);
  REQUIRE(list.size() == 2);
  CHECK(list[0].code == "RX-001");
  CHECK(list[0].manual);
  CHECK(list[0].values.size() == builtin_keys().size());
  CHECK(value_of(list[1], "sys:quantity") == "2.5");
  CHECK(value_of(list[1], "sys:name") == "Døre");
  CHECK(value_of(list[1], "sys:treatment") == "genbrug");
  CHECK(value_of(list[1], "sys:environment") == "ren_screening");
  CHECK(value_of(list[1], "sys:starred") == "false");
  CHECK_FALSE(value_of(list[1], "sys:mass_t").has_value());
  patch_resource(db, "RX-001", {{lex("width_mm"), "600"}});
  const auto r = resource(db, "RX-001");
  CHECK(r.values.size() == builtin_keys().size() + 1);
  CHECK(value_of(r, lex("width_mm")) == "600");
}

TEST_CASE("Resources_List_WithKeys_OrderAndNullForMissing", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  const auto r = resource(
      db, "RX-001",
      std::vector<std::string>{lex("width_mm"), "sys:name", "col:gone"});
  REQUIRE(r.values.size() == 3);
  CHECK(r.values[0].key == lex("width_mm"));
  CHECK_FALSE(r.values[0].value.has_value());
  CHECK(r.values[1].value == "Døre");
  CHECK_FALSE(r.values[2].value.has_value());
  CHECK_THROWS_AS(resource(db, "RX-404"), std::out_of_range);
}

TEST_CASE("Resources_Patch_RoutesByScope_ReturnsSiblings", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  make_part(db, "RX-002", t);
  const auto res = patch_resource(db, "RX-001",
                                  {{"sys:eak", "17.02.01"},
                                   {"sys:mass_t", "0,9"},
                                   {"sys:note", "ved trappen"},
                                   {"sys:starred", "true"},
                                   {"sys:quantity", "4"}});
  CHECK(db.survey_type(t)->eak_code == "17.02.01");
  CHECK(db.survey_type(t)->mass_t == 0.9);
  CHECK(db.survey_part("RX-001")->note == "ved trappen");
  CHECK(db.survey_part("RX-001")->starred);
  CHECK(db.survey_part("RX-001")->quantity == 4.0);
  CHECK(db.survey_part("RX-002")->note.empty());
  REQUIRE(res.siblings.size() == 1);
  CHECK(res.siblings[0].code == "RX-002");
  CHECK(value_of(res.siblings[0], "sys:eak") == "17.02.01");
  // Part-only writes have no siblings to refresh.
  CHECK(
      patch_resource(db, "RX-001", {{"sys:room", "Kælder"}}).siblings.empty());
  // Clearing: mass_t to NULL, eak to "".
  patch_resource(db, "RX-001", {{"sys:mass_t", std::nullopt}, {"sys:eak", ""}});
  CHECK_FALSE(db.survey_type(t)->mass_t.has_value());
  CHECK(db.survey_type(t)->eak_code.empty());
  // A null clear of a NOT NULL text column writes "".
  patch_resource(db, "RX-001",
                 {{"sys:note", std::nullopt},
                  {"sys:room", std::nullopt},
                  {"sys:bim7aa", std::nullopt},
                  {"sys:eak", std::nullopt}});
  CHECK(db.survey_part("RX-001")->note.empty());
  CHECK(db.survey_part("RX-001")->room_name.empty());
  CHECK(db.survey_type(t)->bim7aa_code.empty());
  CHECK(value_of(resource(db, "RX-001"), "sys:note") == "");
}

TEST_CASE("Resources_Patch_LazyPassport_InstanceLinkFollows", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  make_part(db, "RX-002", t);
  // Clearing an absent value succeeds and creates no passport.
  patch_resource(db, "RX-002", {{lex("width_mm"), std::nullopt}});
  CHECK_FALSE(db.survey_part("RX-002")->material_guid.has_value());
  patch_resource(db, "RX-001", {{lex("width_mm"), "600"}});
  const auto guid = db.survey_part("RX-001")->material_guid;
  REQUIRE(guid.has_value());
  CHECK(db.instance_material_guid("instances", 1) == guid);
  patch_resource(db, "RX-001", {{lex("width_mm"), std::nullopt}});
  CHECK(db.passport_stored_properties(*guid).count("width_mm") == 0);
}

TEST_CASE("Resources_Patch_ValidatesAllBeforeWriting", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  for (const std::vector<ResourceWrite> &w :
       std::vector<std::vector<ResourceWrite>>{
           {{"sys:note", "x"}, {"sys:environment", "forurenet"}},
           {{"sys:note", "x"}, {"sys:nope", "1"}},
           {{"sys:note", "x"}, {"sys:treatment", "smid ud"}},
           {{"sys:note", "x"}, {"sys:quantity", "mange"}}}) {
    CHECK_THROWS_AS(patch_resource(db, "RX-001", w), KeyValueError);
  }
  CHECK(db.survey_part("RX-001")->note.empty());
  CHECK_THROWS_AS(patch_resource(db, "RX-404", {{"sys:note", "x"}}),
                  std::out_of_range);
}

TEST_CASE("Resources_CreateAndDelete", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  const auto r = create_resource(db, t, std::string("Branddør"));
  CHECK(r.code == "RX-002");
  CHECK(r.manual);
  CHECK(value_of(r, lex("designation")) == "Branddør");
  const auto guid = db.survey_part("RX-002")->material_guid;
  REQUIRE(guid.has_value());
  CHECK(create_resource(db, t).code == "RX-003");
  CHECK_THROWS_AS(create_resource(db, 999), std::out_of_range);
  CHECK_THROWS_AS(create_resource(db, t, std::string("")),
                  std::invalid_argument);
  delete_resource(db, "RX-002");
  CHECK_FALSE(db.survey_part("RX-002").has_value());
  const auto guids = db.list_passport_guids();
  CHECK(std::find(guids.begin(), guids.end(), *guid) == guids.end());
  CHECK_THROWS_AS(delete_resource(db, "RX-404"), std::out_of_range);
}

TEST_CASE("Resources_RenameColumn_CarriesValues", "[resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::PropertyDefinition def;
  def.name = "Stand";
  def.type = "select";
  def.options = {"God", "Dårlig"};
  const auto col = create_column(db, def);
  CHECK_FALSE(col.id.empty());
  patch_resource(db, "RX-001", {{"col:" + col.id, "God"}});
  ColumnPatch p;
  p.name = "Tilstand";
  CHECK(update_column(db, col.id, p).name == "Tilstand");
  CHECK(value_of(resource(db, "RX-001"), "col:" + col.id) == "God");
  ProjectDB::PropertyDefinition dup;
  dup.name = "Tilstand";
  dup.type = "text";
  CHECK_THROWS_AS(create_column(db, dup), reusex::core::NameConflictError);
  ProjectDB::PropertyDefinition lexname;
  lexname.name = "width_mm";
  lexname.type = "text";
  CHECK_THROWS_AS(create_column(db, lexname), reusex::core::NameConflictError);
  CHECK_THROWS_AS(update_column(db, "nope", p), std::out_of_range);
}

TEST_CASE("Resources_RenameColumn_RefusesNameWithStoredValues",
          "[resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::PropertyDefinition a;
  a.name = "Gammel";
  a.type = "text";
  const auto old_col = create_column(db, a);
  patch_resource(db, "RX-001", {{"col:" + old_col.id, "rest"}});
  db.delete_property_definition(old_col.id); // values stay, column gone
  ProjectDB::PropertyDefinition b;
  b.name = "Ny";
  b.type = "text";
  const auto new_col = create_column(db, b);
  patch_resource(db, "RX-001", {{"col:" + new_col.id, "frisk"}});
  ColumnPatch p;
  p.name = "Gammel";
  CHECK_THROWS_AS(update_column(db, new_col.id, p),
                  reusex::core::NameConflictError);
  CHECK(value_of(resource(db, "RX-001"), "col:" + new_col.id) == "frisk");
  p.name = "Ny"; // same name: no-op
  CHECK(update_column(db, new_col.id, p).name == "Ny");
  p.name = "Ny 2";
  update_column(db, new_col.id, p);
  p.name = "Ny"; // and back again
  update_column(db, new_col.id, p);
  CHECK(value_of(resource(db, "RX-001"), "col:" + new_col.id) == "frisk");
}

TEST_CASE("Resources_Column_RefusesNameWithStaleValues",
          "[resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::PropertyDefinition a;
  a.name = "Gammel";
  a.type = "text";
  const auto old_col = create_column(db, a);
  patch_resource(db, "RX-001", {{"col:" + old_col.id, "rest"}});
  db.delete_property_definition(old_col.id); // values stay, column gone
  // Neither a new column nor a value-less rename may adopt "Gammel"'s values.
  CHECK_THROWS_AS(create_column(db, a), reusex::core::NameConflictError);
  ProjectDB::PropertyDefinition b;
  b.name = "Tom";
  b.type = "text";
  const auto empty_col = create_column(db, b);
  ColumnPatch p;
  p.name = "Gammel";
  CHECK_THROWS_AS(update_column(db, empty_col.id, p),
                  reusex::core::NameConflictError);
  const auto defs = db.list_property_definitions();
  REQUIRE(defs.size() == 1);
  CHECK(defs[0].name == "Tom");
}

TEST_CASE("Resources_Patch_DuplicateKey_RefusedTypeUnchanged", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  CHECK_THROWS_AS(
      patch_resource(db, "RX-001",
                     {{"sys:eak", "17.02.01"}, {"sys:eak", std::nullopt}}),
      KeyValueError);
  // A refused patch leaves the type untouched too.
  CHECK_THROWS_AS(patch_resource(db, "RX-001",
                                 {{"sys:eak", "17.02.01"},
                                  {"sys:mass_t", "0,9"},
                                  {"sys:treatment", "smid ud"}}),
                  KeyValueError);
  const auto type = db.survey_type(t);
  CHECK(type->eak_code.empty());
  CHECK_FALSE(type->mass_t.has_value());
  CHECK(type->treatment == reusex::core::Treatment::genbrug);
}

TEST_CASE("Resources_Delete_KeepsPassportLinkedElsewhere", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  const auto r = create_resource(db, t, std::string("Branddør"));
  const auto guid = db.survey_part(r.code)->material_guid;
  REQUIRE(guid.has_value());
  db.set_instance_material("instances", 1, *guid); // linked elsewhere
  delete_resource(db, r.code);
  CHECK_FALSE(db.survey_part(r.code).has_value());
  const auto guids = db.list_passport_guids();
  CHECK(std::find(guids.begin(), guids.end(), *guid) != guids.end());
  CHECK(db.instance_material_guid("instances", 1) == guid);
}

TEST_CASE("Resources_LazyPassport_MaterialepasExportReadsIt",
          "[resources][materialepas]") {
  // A passport created on the first write (PATCH, POST {name}) must not
  // leave NULL metadata the passport readers copy into std::string.
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  patch_resource(db, "RX-001", {{lex("width_mm"), "600"}});
  create_resource(db, t, std::string("Branddør"));
  const auto passports = db.all_material_passports();
  REQUIRE(passports.size() == 2);
  for (const auto &p : passports) {
    CHECK_FALSE(p.metadata.creation_date.empty());
    CHECK(p.metadata.revision_date.empty());
    CHECK(p.metadata.version_date.empty());
    CHECK_FALSE(p.metadata.version_number.empty());
  }
  export_all(db);
}

TEST_CASE("Resources_PassportReader_NullMetadataReadsEmpty",
          "[resources][materialepas]") {
  // Rows written before the fix (or by hand) can still hold NULLs.
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    const auto t = make_type(db, "Døre");
    make_part(db, "RX-001", t);
    patch_resource(db, "RX-001", {{lex("width_mm"), "600"}});
  }
  raw_exec(tmp, "UPDATE material_passports SET created_at = NULL, "
                "revised_at = NULL, version_number = NULL, "
                "version_date = NULL;");
  ProjectDB db(tmp.path);
  const auto passports = db.all_material_passports();
  REQUIRE(passports.size() == 1);
  CHECK(passports[0].metadata.creation_date.empty());
  CHECK(passports[0].metadata.version_number.empty());
  CHECK(passports[0].dimensions.width_mm == 600.0);
  export_all(db);
}

TEST_CASE("Resources_EveryEditableLeksikonKey_SurvivesTheMaterialepasExport",
          "[resources][materialepas]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  std::vector<ResourceWrite> writes;
  for (const auto &k : leksikon_keys())
    if (k.editable)
      writes.push_back({k.id, valid_value(k)});
  REQUIRE(writes.size() > 40);
  patch_resource(db, "RX-001", writes);
  const auto passports = db.all_material_passports();
  REQUIRE(passports.size() == 1);
  CHECK(passports[0].description.materials.size() == 2);
  export_all(db);
  const auto r = resource(db, "RX-001");
  for (const auto &w : writes) {
    INFO(w.key);
    CHECK(value_of(r, w.key) == w.value);
  }
}

TEST_CASE("Resources_LeksikonKeys_RefuseWhatTheExportCannotParse",
          "[resources][materialepas]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  const auto &materials = lex_key("materials");
  CHECK(materials.data_type == "multiselect");
  CHECK(materials.editable);
  CHECK(materials.options.size() == 43);
  CHECK(materials.options.front() == "natural_stone");
  for (const char *bad : {"træ", R"(["træ"])", "[1]", R"({"a":1})",
                          R"("concrete")", "[\"concrete\""}) {
    INFO(bad);
    CHECK_THROWS_AS(patch_resource(db, "RX-001", {{materials.id, bad}}),
                    KeyValueError);
  }
  std::size_t read_only = 0;
  for (const auto &k : leksikon_keys()) {
    INFO(k.id << " " << k.field);
    if (!k.editable) {
      ++read_only;
      CHECK_THROWS_AS(patch_resource(db, "RX-001", {{k.id, R"(["x"])"}}),
                      KeyValueError);
    } else if (k.data_type != "text") {
      CHECK_THROWS_AS(patch_resource(db, "RX-001", {{k.id, "ikke gyldig"}}),
                      KeyValueError);
    }
  }
  CHECK_FALSE(lex_key("images").editable); // string arrays: read-only
  CHECK(read_only > 0);
  CHECK_THROWS_AS(
      patch_resource(db, "RX-001",
                     {{lex("year_of_installation"), "99999999999"}}),
      KeyValueError);
  // Nothing was written: no passport was even created.
  CHECK_FALSE(db.survey_part("RX-001")->material_guid.has_value());
  // A valid array is stored compact; an empty one clears.
  patch_resource(db, "RX-001",
                 {{materials.id, R"( [ "steel" , "concrete" ] )"}});
  CHECK(value_of(resource(db, "RX-001"), materials.id) ==
        R"(["steel","concrete"])");
  export_all(db);
  patch_resource(db, "RX-001", {{materials.id, "[]"}});
  const auto guid = db.survey_part("RX-001")->material_guid;
  REQUIRE(guid.has_value());
  CHECK(db.passport_stored_properties(*guid).count("materials") == 0);
  export_all(db);
}

TEST_CASE("Resources_BlankStoredValues_ReadAsUnset", "[resources]") {
  // add_material_passport stores every field, blank ones as "", "[]" or the
  // TriState default "unknown"; a read must not report them as values.
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  make_passport(db, "guid-p");
  db.set_instance_material("instances", 1, "guid-p");
  REQUIRE(db.survey_part("RX-001")->material_guid ==
          std::optional<std::string>("guid-p"));
  const auto stored = db.passport_stored_properties("guid-p");
  REQUIRE(stored.at("designation").empty());
  REQUIRE(stored.at("materials") == "[]");
  const auto all = resource(db, "RX-001");
  CHECK(all.values.size() == builtin_keys().size());
  const auto picked =
      resource(db, "RX-001",
               std::vector<std::string>{lex("designation"), lex("materials"),
                                        lex("contains_reach_substances")});
  for (const auto &v : picked.values) {
    INFO(v.key);
    CHECK_FALSE(v.value.has_value());
  }
}

TEST_CASE("Resources_EmptyStringClearsTextLeksikonAndColumnKeys",
          "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::PropertyDefinition def;
  def.name = "Farve";
  def.type = "text";
  const auto col = create_column(db, def);
  patch_resource(db, "RX-001",
                 {{lex("designation"), "Branddør"}, {"col:" + col.id, "rød"}});
  const auto guid = *db.survey_part("RX-001")->material_guid;
  patch_resource(db, "RX-001",
                 {{lex("designation"), ""}, {"col:" + col.id, ""}});
  const auto stored = db.passport_stored_properties(guid);
  CHECK(stored.count("designation") == 0);
  CHECK(stored.count("Farve") == 0);
  const auto r = resource(db, "RX-001");
  CHECK_FALSE(value_of(r, lex("designation")).has_value());
  CHECK_FALSE(value_of(r, "col:" + col.id).has_value());
  // Built-in text keys keep "" (their columns are NOT NULL).
  CHECK(value_of(r, "sys:note") == "");
}

TEST_CASE("Resources_DeleteColumn_PurgesItsValues_NameReusable",
          "[resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  make_part(db, "RX-002", t);
  ProjectDB::PropertyDefinition def;
  def.name = "Farve";
  def.type = "text";
  const auto col = create_column(db, def);
  patch_resource(db, "RX-001", {{"col:" + col.id, "rød"}});
  patch_resource(db, "RX-002", {{"col:" + col.id, "blå"}});
  {
    LogCapture log;
    delete_column(db, col.id);
    CHECK(log.count(LogLevel::warn, "2 stored value") == 1);
    CHECK(log.count(LogLevel::warn, "Farve") == 1);
  }
  CHECK(db.list_property_definitions().empty());
  CHECK_FALSE(db.has_passport_field_values("Farve"));
  const auto again = create_column(db, def); // the name is free again
  CHECK_FALSE(value_of(resource(db, "RX-001"), "col:" + again.id).has_value());
  {
    LogCapture log;
    delete_column(db, again.id); // nothing stored: no warning
    CHECK(log.count(LogLevel::warn, "") == 0);
  }
  CHECK_THROWS_AS(delete_column(db, "nope"), std::out_of_range);
}

TEST_CASE("Resources_Patch_View_RoundTripsChecksSourceAndClears",
          "[resources][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  db.save_sensor_frame(7, cv::Mat(8, 8, CV_8UC3, cv::Scalar(1, 2, 3)));
  // Unset: the key is listed (null) and Resource::view is empty.
  const auto before = resource(db, "RX-001");
  CHECK_FALSE(before.view);
  CHECK(std::any_of(
      before.values.begin(), before.values.end(),
      [](const auto &v) { return v.key == "sys:view" && !v.value; }));

  const std::string in =
      R"({"region":[0.1,0.2,0.3,0.4],"kind":"frame","id":7})";
  const auto out = patch_resource(db, "RX-001", {{"sys:view", in}});
  CHECK(value_of(out.resource, "sys:view") ==
        R"({"kind":"frame","id":7,"region":[0.1,0.2,0.3,0.4]})");
  REQUIRE(out.resource.view);
  CHECK(out.resource.view->id == 7);
  CHECK(out.siblings.empty()); // part-scoped
  CHECK(db.survey_part("RX-001")->view == out.resource.view);
  // A keys filter without sys:view still carries Resource::view.
  CHECK(resource(db, "RX-001", std::vector<std::string>{"sys:name"}).view);

  // No such frame / bad shape: KeyValueError, nothing written.
  CHECK_THROWS_AS(patch_resource(db, "RX-001",
                                 {{"sys:note", "x"},
                                  {"sys:view", R"({"kind":"frame","id":8})"}}),
                  KeyValueError);
  CHECK_THROWS_AS(
      patch_resource(db, "RX-001",
                     {{"sys:view", R"({"kind":"panorama","id":1})"}}),
      KeyValueError);
  CHECK_THROWS_AS(patch_resource(db, "RX-001", {{"sys:view", "nope"}}),
                  KeyValueError);
  CHECK(db.survey_part("RX-001")->note.empty());
  CHECK(db.survey_part("RX-001")->view->id == 7);

  patch_resource(db, "RX-001", {{"sys:view", std::nullopt}});
  CHECK_FALSE(db.survey_part("RX-001")->view);
}

TEST_CASE("Resources_Patch_FrameViewWithoutRegion_StoresInstanceRegion",
          "[resources][view]") {
  using Catch::Matchers::WithinAbs;
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  reusex::Cloud positions; // instance 1's one point, on the optical axis
  pcl::PointXYZRGB p;
  p.x = p.y = 0.0F;
  p.z = 2.0F;
  positions.push_back(p);
  db.save_point_cloud("cloud", positions, "test", "{}");
  SensorIntrinsics k; // fx = fy = 100, principal point (64, 64), 128 x 128
  k.fx = k.fy = 100.0;
  k.cx = k.cy = 64.0;
  k.width = k.height = 128;
  db.save_sensor_frame(
      1, cv::Mat(128, 128, CV_8UC3, cv::Scalar(1, 2, 3)), cv::Mat(), cv::Mat(),
      {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, k, 1.0, -1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  make_part(db, "RX-002", t);

  // Instance-backed: (64, 64), one pixel, padded 15 % per side.
  const auto out = patch_resource(db, "RX-001",
                                  {{"sys:view", R"({"kind":"frame","id":1})"}});
  const auto stored = db.survey_part("RX-001")->view;
  REQUIRE(stored);
  REQUIRE(stored->region);
  CHECK_THAT((*stored->region)[0], WithinAbs(63.85 / 128.0, 1e-9));
  CHECK_THAT((*stored->region)[1], WithinAbs(63.85 / 128.0, 1e-9));
  CHECK_THAT((*stored->region)[2], WithinAbs(1.3 / 128.0, 1e-9));
  CHECK_THAT((*stored->region)[3], WithinAbs(1.3 / 128.0, 1e-9));
  CHECK(out.resource.view == stored);
  CHECK(value_of(out.resource, "sys:view") == view_to_json(*stored));

  // Manual part: nothing to place, the whole image.
  const auto manual = patch_resource(
      db, "RX-002", {{"sys:view", R"({"kind":"frame","id":1})"}});
  REQUIRE(manual.resource.view);
  CHECK_FALSE(manual.resource.view->region);
  CHECK(value_of(manual.resource, "sys:view") ==
        R"({"kind":"frame","id":1,"region":null})");
}
