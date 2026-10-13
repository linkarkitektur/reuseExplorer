// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// The template service (spec §5.5): CRUD with unique names, duplicate as
// "(kopi)", restore seeds, and views resolved against the live catalogue.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_templates.hpp>
#include <core/survey.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using json = nlohmann::json;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_template_service") {}
};
TemplateInput named(const char *name) {
  TemplateInput in;
  in.name = name;
  return in;
}
} // namespace

TEST_CASE("TemplateService_Create_ValidatesAndResolves", "[templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  auto in = named("Mit valg");
  in.members = json::parse(R"([{"key":"sys:note"},{"key":"col:gone"}])");
  in.csv = json::parse(R"({"delimiter":","})");
  const auto v = create_template(db, in);
  CHECK(v.record.name == "Mit valg");
  CHECK_FALSE(v.record.seed.has_value());
  CHECK(v.resolved.keys == std::vector<std::string>{"sys:note"});
  CHECK(v.resolved.missing.size() == 1);
  CHECK(v.csv.delimiter == ",");
  CHECK_THROWS_AS(create_template(db, named("Mit valg")), NameConflictError);
  CHECK_THROWS_AS(create_template(db, TemplateInput{}), std::invalid_argument);
  CHECK_THROWS_AS(create_template(db, named("")), std::invalid_argument);
  auto bad = named("Andet");
  bad.members = json::parse(R"([{"key":1}])");
  CHECK_THROWS_AS(create_template(db, bad), std::invalid_argument);
  bad.members.reset();
  bad.csv = json::parse(R"({"header":"kolonne"})");
  CHECK_THROWS_AS(create_template(db, bad), std::invalid_argument);
  CHECK(template_views(db).size() == 3);
}

TEST_CASE("TemplateService_Update_Delete", "[templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = create_template(db, named("A")).record.id;
  TemplateInput in;
  in.members = json::parse(R"([{"category":"Kortlægning"}])");
  const auto v = update_template(db, id, in);
  CHECK(v.record.name == "A");
  CHECK(v.resolved.keys.size() == 12);
  in.members.reset();
  in.name = "Hurtig genbrugsscreening";
  CHECK_THROWS_AS(update_template(db, id, in), NameConflictError);
  CHECK_THROWS_AS(update_template(db, 999, named("x")), std::out_of_range);
  // A csv-only edit changes the options and nothing else; a bad one is
  // refused and changes nothing.
  TemplateInput csv;
  csv.csv = json::parse(R"({"delimiter":"\t","header":"key"})");
  const auto edited = update_template(db, id, csv);
  CHECK(edited.csv.delimiter == "\t");
  CHECK(edited.csv.header == "key");
  CHECK(edited.csv.encoding == "utf-8-bom");
  CHECK(edited.record.name == "A");
  CHECK(edited.resolved.keys.size() == 12);
  csv.csv = json::parse(R"({"delimiter":"|"})");
  CHECK_THROWS_AS(update_template(db, id, csv), std::invalid_argument);
  CHECK(template_view(db, id).csv.delimiter == "\t");
  delete_template(db, id);
  CHECK_THROWS_AS(delete_template(db, id), std::out_of_range);
  CHECK_THROWS_AS(template_view(db, id), std::out_of_range);
}

TEST_CASE("TemplateService_Duplicate_NumbersTheCopy", "[templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto seed = template_views(db)[1];
  const auto a = duplicate_template(db, seed.record.id);
  CHECK(a.record.name == "Hurtig genbrugsscreening (kopi)");
  CHECK_FALSE(a.record.seed.has_value());
  CHECK(a.record.members_json == seed.record.members_json);
  CHECK(duplicate_template(db, seed.record.id).record.name ==
        "Hurtig genbrugsscreening (kopi 2)");
  CHECK_THROWS_AS(duplicate_template(db, 999), std::out_of_range);
}

TEST_CASE("TemplateService_RestoreSeeds_SuffixOnNameClash_Idempotent",
          "[templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  for (const auto &v : template_views(db))
    delete_template(db, v.record.id);
  create_template(db, named("Materialepas (fuld)")); // user's own
  const auto restored = restore_seed_templates(db);
  REQUIRE(restored.size() == 2);
  CHECK(restored[0].record.name == "Materialepas (fuld) (standard)");
  CHECK(restored[0].record.seed == std::optional<std::string>("materialepas"));
  CHECK(restored[1].record.name == "Hurtig genbrugsscreening");
  CHECK(restore_seed_templates(db).empty());
  CHECK(template_views(db).size() == 3);
}
