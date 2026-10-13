// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// The resource key catalogue (resources/templates spec §4.3): built-in keys,
// leksikon keys from the compiled MaterialEPAS traits, user columns, and the
// one place values are validated and normalised.

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/materialepas_json_export.hpp>
#include <core/resource_keys.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

using namespace reusex::core;
using reusex::ProjectDB;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_resource_keys") {}
};
const ResourceKey &key_named(const std::vector<ResourceKey> &keys,
                             const std::string &field) {
  const auto it = std::find_if(keys.begin(), keys.end(),
                               [&](const auto &k) { return k.field == field; });
  REQUIRE(it != keys.end());
  return *it;
}
} // namespace

TEST_CASE("ResourceKeys_Builtins_MatchTheSpecTable", "[resources][keys]") {
  const auto keys = builtin_keys();
  std::vector<std::string> ids;
  for (const auto &k : keys)
    ids.push_back(k.id);
  CHECK(ids == std::vector<std::string>{
                   "sys:name", "sys:quantity", "sys:unit", "sys:eak",
                   "sys:bim7aa", "sys:treatment", "sys:environment", "sys:room",
                   "sys:mass_t", "sys:note", "sys:starred", "sys:view"});
  for (const auto &k : keys) {
    CHECK(k.category == "Kortlægning");
    CHECK(k.source == KeySource::builtin);
    CHECK(k.editable == (k.id != "sys:environment"));
  }
  CHECK(keys[0].label == "Betegnelse");
  CHECK(keys[0].scope == KeyScope::type);
  CHECK(keys[1].scope == KeyScope::part);
  CHECK(keys[1].data_type == "number");
  CHECK(keys[5].data_type == "enum");
  CHECK(keys[5].options ==
        std::vector<std::string>{"bevaring", "genbrug", "genanvendelse",
                                 "nyttiggoerelse", "bortskaffelse"});
  CHECK(keys[6].options == std::vector<std::string>{"ren_screening", "afventer",
                                                    "forurenet",
                                                    "ren_proevesvar"});
  CHECK(keys[8].unit == "t");
  CHECK(keys[10].data_type == "boolean");
  CHECK(keys[11].label == "Ressourcebillede");
  CHECK(keys[11].scope == KeyScope::part);
  CHECK(keys[11].data_type == "view");
  CHECK(keys[11].editable);
  CHECK(to_string(KeyScope::type) == "type");
}

TEST_CASE("ResourceKeys_Leksikon_EveryTopLevelPropertyInLeksikonOrder",
          "[resources][keys]") {
  std::size_t expected = 0;
  for (const auto &sd : json_export::section_descriptors())
    for (std::size_t i = 0; i < sd.property_count; ++i)
      if (sd.properties[i].type != traits::PropertyType::ObjectArray)
        ++expected;
  const auto keys = leksikon_keys();
  REQUIRE(keys.size() == expected);
  CHECK(keys.front().id == "lex:0Bwj05D$55V931bq9VaBE5");
  CHECK(keys.front().field == "contact_email");
  CHECK(keys.front().label == "Contact email");
  CHECK(keys.front().category == "Owner");
  CHECK(keys.front().scope == KeyScope::part);
  const auto &width = key_named(keys, "width_mm");
  CHECK(width.data_type == "number");
  CHECK(width.unit == "mm");
  CHECK_FALSE(width.integer);
  CHECK(key_named(keys, "year_of_installation").integer);
  CHECK(key_named(keys, "volume_m3").unit == "m³");
  const auto &reach = key_named(keys, "contains_reach_substances");
  CHECK(reach.data_type == "enum");
  CHECK(reach.options == std::vector<std::string>{"yes", "no", "unknown"});
  CHECK(key_named(keys, "has_epd").data_type == "boolean");
  CHECK(leksikon_categories() ==
        std::vector<std::string>{
            "Owner", "Description", "Product", "Certifications", "Dimensions",
            "History", "Condition", "Pollution", "Environmental", "Fire"});
}

TEST_CASE("ResourceKeys_LeksikonCategories_MatchPropertyDefinitions",
          "[resources][keys]") {
  // ProjectDB files each leksikon field under a category string; the
  // catalogue must use the same one, or a category template member would
  // miss fields.
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    MaterialPassport p;
    p.metadata.document_guid = "guid-p";
    db.add_material_passport(p, "");
  }
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  REQUIRE(sqlite3_prepare_v2(raw,
                             "SELECT leksikon_guid, name_en, category FROM "
                             "property_definitions;",
                             -1, &s, nullptr) == SQLITE_OK);
  std::map<std::string, std::pair<std::string, std::string>> stored;
  while (sqlite3_step(s) == SQLITE_ROW)
    stored[reinterpret_cast<const char *>(sqlite3_column_text(s, 0))] = {
        reinterpret_cast<const char *>(sqlite3_column_text(s, 1)),
        reinterpret_cast<const char *>(sqlite3_column_text(s, 2))};
  sqlite3_finalize(s);
  sqlite3_close(raw);
  for (const auto &f : leksikon_fields()) {
    INFO(f.field_name);
    REQUIRE(stored.count(f.guid) == 1);
    CHECK(stored[f.guid].first == f.field_name);
    CHECK(stored[f.guid].second == f.category);
  }
}

TEST_CASE("ResourceKeys_Catalogue_BuiltinLeksikonThenColumns",
          "[resources][keys]") {
  ProjectDB::PropertyDefinition sel;
  sel.id = "c1";
  sel.name = "Stand";
  sel.type = "select";
  sel.options = {"God", "Dårlig"};
  ProjectDB::PropertyDefinition multi;
  multi.id = "c2";
  multi.name = "Mærker";
  multi.type = "multiselect";
  const auto cat = key_catalogue({sel, multi});
  REQUIRE(cat.size() == builtin_keys().size() + leksikon_keys().size() + 2);
  CHECK(cat.front().id == "sys:name");
  const auto &a = cat[cat.size() - 2];
  CHECK(a.id == "col:c1");
  CHECK(a.label == "Stand");
  CHECK(a.category == "Egne felter");
  CHECK(a.data_type == "enum");
  CHECK(a.options == std::vector<std::string>{"God", "Dårlig"});
  CHECK(a.field == "Stand");
  CHECK(a.source == KeySource::column);
  CHECK(cat.back().data_type == "text");
  CHECK(find_key(cat, "col:c2") == &cat.back());
  CHECK(find_key(cat, "col:nope") == nullptr);
}

TEST_CASE("ResourceKeys_Normalise_NumbersEnumsBooleansDates",
          "[resources][keys]") {
  const auto cat = key_catalogue(std::vector<ProjectDB::PropertyDefinition>{});
  const auto &qty = *find_key(cat, "sys:quantity");
  CHECK(normalise_value(qty, std::string("12,5")) == "12.5");
  CHECK(normalise_value(qty, std::string(" 3 ")) == "3");
  CHECK_THROWS_AS(normalise_value(qty, std::string("-1")), KeyValueError);
  CHECK_THROWS_AS(normalise_value(qty, std::string("1,2,3")), KeyValueError);
  CHECK_THROWS_AS(normalise_value(qty, std::string("nan")), KeyValueError);
  const auto &year = key_named(cat, "year_of_installation");
  CHECK(normalise_value(year, std::string("1968")) == "1968");
  CHECK_THROWS_AS(normalise_value(year, std::string("1968.5")), KeyValueError);
  const auto &tr = *find_key(cat, "sys:treatment");
  CHECK(normalise_value(tr, std::string("genbrug")) == "genbrug");
  try {
    normalise_value(tr, std::string("genbrugt"));
    FAIL("expected KeyValueError");
  } catch (const KeyValueError &e) {
    CHECK(e.key() == "sys:treatment");
    CHECK(std::string(e.what()).find("sys:treatment") != std::string::npos);
  }
  const auto &star = *find_key(cat, "sys:starred");
  CHECK(normalise_value(star, std::string("true")) == "true");
  CHECK_THROWS_AS(normalise_value(star, std::string("ja")), KeyValueError);
  ProjectDB::PropertyDefinition d;
  d.id = "d";
  d.name = "Dato";
  d.type = "date";
  const auto date = column_key(d);
  CHECK(normalise_value(date, std::string("2026-10-02")) == "2026-10-02");
  CHECK_THROWS_AS(normalise_value(date, std::string("2026-13-02")),
                  KeyValueError);
  CHECK_THROWS_AS(normalise_value(date, std::string("02-10-2026")),
                  KeyValueError);
  CHECK(normalise_value(*find_key(cat, "sys:note"), std::string("")) == "");
}

TEST_CASE("ResourceKeys_Normalise_BlankClears_RequiredRefuses",
          "[resources][keys]") {
  const auto cat = key_catalogue(std::vector<ProjectDB::PropertyDefinition>{});
  // "" on a typed key is a clear, not a parse error.
  CHECK_FALSE(normalise_value(*find_key(cat, "sys:mass_t"), std::string(""))
                  .has_value());
  CHECK_FALSE(
      normalise_value(*find_key(cat, "sys:mass_t"), std::nullopt).has_value());
  for (const char *id : {"sys:name", "sys:quantity", "sys:unit",
                         "sys:treatment", "sys:starred"}) {
    INFO(id);
    CHECK_THROWS_AS(normalise_value(*find_key(cat, id), std::nullopt),
                    KeyValueError);
  }
  CHECK_THROWS_AS(
      normalise_value(*find_key(cat, "sys:quantity"), std::string("")),
      KeyValueError);
  CHECK_THROWS_AS(normalise_value(*find_key(cat, "sys:name"), std::string("")),
                  KeyValueError);
  // R-P7: "" on sys:unit is a 400-class rejection too (non-clearable), same
  // as sys:name — not a silent clear.
  CHECK_THROWS_AS(normalise_value(*find_key(cat, "sys:unit"), std::string("")),
                  KeyValueError);
  CHECK_THROWS_AS(normalise_value(*find_key(cat, "sys:environment"),
                                  std::string("forurenet")),
                  KeyValueError);
  CHECK(format_number(290.0) == "290");
  CHECK(format_number(0.1) == "0.1");
  CHECK(parse_number("+4") == 4.0);
  CHECK_FALSE(parse_number("").has_value());
}

TEST_CASE("NormaliseValue_ViewKey_CanonicalisesAndRefusesBadShapes",
          "[resources][keys]") {
  const auto keys = builtin_keys();
  const auto &k = *find_key(keys, "sys:view");
  CHECK(normalise_value(k, std::string(R"({"id":3,"kind":"frame"})")) ==
        R"({"kind":"frame","id":3,"region":null})");
  CHECK_FALSE(normalise_value(k, std::string("")).has_value());
  CHECK_FALSE(normalise_value(k, std::nullopt).has_value());
  CHECK_THROWS_AS(normalise_value(k, std::string("Foto #3")), KeyValueError);
  CHECK_THROWS_AS(
      normalise_value(
          k,
          std::string(R"({"kind":"frame","id":3,"region":[0,0.9,0.5,0.5]})")),
      KeyValueError);
}
