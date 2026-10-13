// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Template members and resolution (spec §5.2), seeds (§5.3), CSV options,
// and the export_templates column mapping (§5.4) — pure, no project.

#include <catch2/catch_test_macros.hpp>

#include <core/resource_keys.hpp>
#include <core/resource_templates.hpp>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using namespace reusex::core;
using reusex::ProjectDB;
using json = nlohmann::json;

namespace {
ProjectDB::PropertyDefinition column(const char *id, const char *name) {
  ProjectDB::PropertyDefinition d;
  d.id = id;
  d.name = name;
  d.type = "text";
  return d;
}
TemplateMember cat(const char *c) { return {MemberKind::category, c}; }
TemplateMember key(const std::string &k) { return {MemberKind::key, k}; }
} // namespace

TEST_CASE("ResourceTemplates_Resolve_ExpandsDedupsAndReportsMissing",
          "[resources][templates]") {
  const auto catalogue = key_catalogue({column("a", "Bredde")});
  const auto r =
      resolve_template({key("sys:note"), cat("Kortlægning"), key("col:a"),
                        key("col:gone"), cat("Ingen"), key("sys:name")},
                       catalogue);
  REQUIRE(r.keys.size() == 13);
  CHECK(r.keys[0] == "sys:note"); // first position wins
  CHECK(r.keys[1] == "sys:name");
  CHECK(r.keys[10] == "sys:starred");
  CHECK(r.keys[11] == "sys:view");
  CHECK(r.keys[12] == "col:a");
  CHECK(r.missing ==
        std::vector<TemplateMember>{key("col:gone"), cat("Ingen")});
}

TEST_CASE("ResourceTemplates_Resolve_NewKeyAppearsUnderCategoryMember",
          "[resources][templates]") {
  const std::vector<TemplateMember> members{cat("Egne felter")};
  CHECK(resolve_template(members, key_catalogue({column("a", "A")})).keys ==
        std::vector<std::string>{"col:a"});
  CHECK(resolve_template(members,
                         key_catalogue({column("a", "A"), column("b", "B")}))
            .keys == std::vector<std::string>{"col:a", "col:b"});
  CHECK(resolve_template(members, key_catalogue({})).missing.size() == 1);
}

TEST_CASE("ResourceTemplates_Members_ParseValidatesShape",
          "[resources][templates]") {
  const auto m = parse_members(
      json::parse(R"([{"category":"Owner"},{"key":"sys:name"}])"));
  CHECK(m == std::vector<TemplateMember>{cat("Owner"), key("sys:name")});
  CHECK(members_json(m) ==
        json::parse(R"([{"category":"Owner"},{"key":"sys:name"}])"));
  for (const char *bad :
       {R"({"key":"x"})", R"([{"key":""}])", R"([{"key":1}])",
        R"([{"category":"a","key":"b"}])", R"([{"other":"x"}])", R"(["x"])"}) {
    INFO(bad);
    CHECK_THROWS_AS(parse_members(json::parse(bad)), std::invalid_argument);
  }
  // Stored JSON that is broken reads as no members instead of failing.
  CHECK(read_members("not json", "T").empty());
  CHECK(read_members(R"([{"key":"sys:name"}])", "T") ==
        std::vector<TemplateMember>{key("sys:name")});
}

TEST_CASE("ResourceTemplates_Seeds_MaterialepasAndScreening",
          "[resources][templates]") {
  const auto seeds = seed_templates();
  REQUIRE(seeds.size() == 2);
  CHECK(seeds[0].tag == "materialepas");
  CHECK(seeds[0].name == "Materialepas (fuld)");
  REQUIRE(seeds[0].members.size() == leksikon_categories().size());
  CHECK(seeds[0].members.front() == cat("Owner"));
  CHECK(seeds[1].tag == "screening");
  CHECK(seeds[1].name == "Hurtig genbrugsscreening");
  std::vector<TemplateMember> screening;
  for (const auto &k : builtin_keys())
    if (k.id != kViewKey)
      screening.push_back(key(k.id));
  CHECK(seeds[1].members == screening);
  // Every leksikon key resolves through the full seed.
  const auto catalogue = key_catalogue({});
  CHECK(resolve_template(seeds[0].members, catalogue).keys.size() ==
        leksikon_keys().size());
}

TEST_CASE("ResourceTemplates_CsvOptions_DefaultsExtrasAndValidation",
          "[resources][templates]") {
  const auto d = parse_csv_options(json::object());
  CHECK(d.delimiter == ";");
  CHECK(d.encoding == "utf-8-bom");
  CHECK(d.header == "label");
  const auto o = parse_csv_options(
      json::parse(R"({"delimiter":",","header":"key","sort":"code"})"));
  CHECK(o.delimiter == ",");
  CHECK(o.header == "key");
  CHECK(o.extra == json::parse(R"({"sort":"code"})"));
  CHECK(csv_options_json(o) ==
        json::parse(R"({"delimiter":",","encoding":"utf-8-bom",)"
                    R"("header":"key","sort":"code"})"));
  CHECK(parse_csv_options(json::parse(R"({"delimiter":"\t"})")).delimiter ==
        "\t");
  CHECK_THROWS_AS(parse_csv_options(json::parse(R"({"delimiter":"|"})")),
                  std::invalid_argument);
  CHECK_THROWS_AS(parse_csv_options(json::parse(R"({"encoding":"latin1"})")),
                  std::invalid_argument);
  CHECK_THROWS_AS(parse_csv_options(json::parse("[]")), std::invalid_argument);
  // Stored options are read tolerantly: a bad field falls back to default.
  const auto r = read_csv_options(R"({"delimiter":"|","header":"key"})", "T");
  CHECK(r.delimiter == ";");
  CHECK(r.header == "key");
  CHECK(read_csv_options("garbage", "T").delimiter == ";");
}

TEST_CASE("ResourceTemplates_LegacyColumnMember_MapsToKeys",
          "[resources][templates]") {
  const auto catalogue = key_catalogue({column("c1", "Bredde")});
  CHECK(legacy_column_member("Bredde", catalogue) == key("col:c1"));
  CHECK(legacy_column_member("Note", catalogue) == key("sys:note"));
  CHECK(legacy_column_member("width_mm", catalogue).ref.rfind("lex:", 0) == 0);
  CHECK(legacy_column_member("kind", catalogue) == key("legacy:kind"));
}

TEST_CASE("ResourceTemplates_UniqueName_NumbersUntilFree",
          "[resources][templates]") {
  CHECK(unique_name("A", "kopi", {"A"}) == "A (kopi)");
  CHECK(unique_name("A", "kopi", {"A", "A (kopi)"}) == "A (kopi 2)");
  CHECK(unique_name("A", "kopi", {"A", "A (kopi)", "A (kopi 2)"}) ==
        "A (kopi 3)");
  CHECK(unique_name("Mine", "eksport", {}) == "Mine (eksport)");
}
