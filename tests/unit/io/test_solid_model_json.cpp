// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The solid-model JSON document (reusex-z1v.2.5): one format for authored
// reference plans and detected models.

#include "../../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using Catch::Matchers::ContainsSubstring;
using reusex::core::SolidModel;
using reusex::io::solid_model_from_json;
using reusex::io::solid_model_to_json;

namespace {

std::string read_file(const std::filesystem::path &p) {
  std::ifstream in(p);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::filesystem::path fixture(const char *name) {
  return std::filesystem::path(__FILE__)
             .parent_path()
             .parent_path()
             .parent_path() /
         "fixtures" / "solid_model" / name;
}

/// A minimal valid document; tests mutate it to provoke one error each.
nlohmann::json minimal_doc() {
  return nlohmann::json::parse(R"({
    "model_version": 1,
    "storeys": [{"idx": 0, "name": "Ground", "z_floor": 0, "z_ceiling": 2.6}],
    "walls": [
      {"guid": "w1", "storey": 0, "axis": [[0, 0], [4, 0]], "thickness": 0.2},
      {"guid": "w2", "storey": 0, "axis": [[4, 0], [4, 3]], "thickness": 0.1}
    ],
    "spaces": [
      {"guid": "s1", "storey": 0, "name": "Kitchen",
       "polygon": {"outer": [[0, 0], [4, 0], [4, 3], [0, 3]]}}
    ],
    "openings": [
      {"guid": "o1", "wall": "w1", "kind": "door", "s_start": 1.0,
       "s_end": 1.9, "z_sill": 0.0, "z_head": 2.1}
    ]
  })");
}

} // namespace

TEST_CASE("SolidModelFromJson_MinimalDocument_ParsesEveryElement",
          "[io][solid_model]") {
  const SolidModel m = solid_model_from_json(minimal_doc().dump());
  REQUIRE(m.storeys.size() == 1);
  CHECK(m.storeys[0].name == "Ground");
  CHECK(m.storeys[0].z_ceiling == 2.6);
  REQUIRE(m.walls.size() == 2);
  CHECK(m.walls[0].guid == "w1");
  CHECK(m.walls[0].x1 == 4.0);
  CHECK(m.walls[0].thickness == 0.2);
  CHECK(m.walls[1].y1 == 3.0);
  REQUIRE(m.spaces.size() == 1);
  CHECK(m.spaces[0].polygon.outer.size() == 4);
  CHECK(m.spaces[0].polygon.holes.empty());
  REQUIRE(m.openings.size() == 1);
  CHECK(m.openings[0].wall == "w1");
  CHECK(m.openings[0].kind == "door");
  CHECK(m.openings[0].z_head == 2.1);
  CHECK(m.has_walls());
  CHECK(m.has_spaces());
  CHECK(m.has_openings());
}

TEST_CASE("SolidModelFromJson_OmittedOptionalFields_TakeRecordDefaults",
          "[io][solid_model]") {
  const SolidModel m = solid_model_from_json(minimal_doc().dump());
  const auto &w = m.walls[0];
  CHECK(w.source == "detected");
  CHECK_FALSE(w.locked);
  CHECK(w.confidence == -1.0);
  CHECK(w.seen_share == -1.0);
  CHECK_FALSE(w.assumed);
  CHECK(m.storeys[0].level_transform[0] == 1.0);
  CHECK(m.storeys[0].level_transform[1] == 0.0);
}

TEST_CASE("SolidModelJson_RoundTrip_IsLossless", "[io][solid_model]") {
  const std::string first = solid_model_to_json(
      solid_model_from_json(read_file(fixture("two_rooms.plan.json"))));
  const std::string second = solid_model_to_json(solid_model_from_json(first));
  CHECK(first == second);

  const auto j = nlohmann::json::parse(first);
  CHECK(j.at("model_version") == reusex::io::kSolidModelJsonVersion);
  CHECK(j.at("walls").size() == 7);
  CHECK(j.at("spaces").size() == 2);
  CHECK(j.at("openings").size() == 3);
}

TEST_CASE("SolidModelJson_ProvidedButEmpty_SurvivesRoundTrip",
          "[io][solid_model]") {
  // "walls": [] means "this model has no walls" (scored as a miss), which is
  // not the same as omitting the key ("this model says nothing about walls").
  const SolidModel m =
      solid_model_from_json(R"({"model_version": 1, "walls": []})");
  CHECK(m.has_walls());
  CHECK_FALSE(m.has_spaces());
  const auto j = nlohmann::json::parse(solid_model_to_json(m));
  CHECK(j.contains("walls"));
  CHECK(j.at("walls").empty());
  CHECK_FALSE(j.contains("spaces"));
}

TEST_CASE("SolidModelFromJson_MetadataObject_RoundTripsAsJson",
          "[io][solid_model]") {
  auto doc = minimal_doc();
  doc["walls"][0]["metadata"] = {{"author", "agent"}, {"pass", 2}};
  const SolidModel m = solid_model_from_json(doc.dump());
  CHECK(nlohmann::json::parse(m.walls[0].metadata) ==
        doc["walls"][0]["metadata"]);
  const auto out = nlohmann::json::parse(solid_model_to_json(m));
  CHECK(out["walls"][0]["metadata"] == doc["walls"][0]["metadata"]);
}

TEST_CASE("SolidModelFromJson_InvalidDocuments_ThrowNamingTheElement",
          "[io][solid_model]") {
  SECTION("not JSON") {
    CHECK_THROWS_AS(solid_model_from_json("{nope"), std::invalid_argument);
  }
  SECTION("missing model_version") {
    auto doc = minimal_doc();
    doc.erase("model_version");
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("model_version"));
  }
  SECTION("future model_version") {
    auto doc = minimal_doc();
    doc["model_version"] = 99;
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("99"));
  }
  SECTION("unknown key is a typo, not silently ignored") {
    auto doc = minimal_doc();
    doc["walls"][1]["thicknes"] = 0.3;
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("walls[1]") &&
                          ContainsSubstring("thicknes"));
  }
  SECTION("zero-length wall axis") {
    auto doc = minimal_doc();
    doc["walls"][0]["axis"] = {{1, 1}, {1, 1}};
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("walls[0]") && ContainsSubstring("w1"));
  }
  SECTION("wrong type") {
    auto doc = minimal_doc();
    doc["walls"][0]["thickness"] = "thick";
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("walls[0]") &&
                          ContainsSubstring("thickness"));
  }
  SECTION("polygon with two points") {
    auto doc = minimal_doc();
    doc["spaces"][0]["polygon"]["outer"] = {{0, 0}, {1, 0}};
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("spaces[0]"));
  }
  SECTION("opening on a wall that does not exist") {
    auto doc = minimal_doc();
    doc["openings"][0]["wall"] = "w9";
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("openings[0]") &&
                          ContainsSubstring("w9"));
  }
  SECTION("opening with s_end before s_start") {
    auto doc = minimal_doc();
    doc["openings"][0]["s_end"] = 0.5;
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("openings[0]"));
  }
  SECTION("duplicate wall guid") {
    auto doc = minimal_doc();
    doc["walls"][1]["guid"] = "w1";
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("w1"));
  }
  SECTION("wall on a storey that does not exist") {
    auto doc = minimal_doc();
    doc["walls"][1]["storey"] = 3;
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("walls[1]") &&
                          ContainsSubstring("storey"));
  }
  SECTION("level_transform with the wrong length") {
    auto doc = minimal_doc();
    doc["storeys"][0]["level_transform"] = {1, 0, 0};
    CHECK_THROWS_WITH(solid_model_from_json(doc.dump()),
                      ContainsSubstring("storeys[0]") &&
                          ContainsSubstring("level_transform"));
  }
}

TEST_CASE("SolidModelFromJson_EmptyDocument_ProvidesNothing",
          "[io][solid_model]") {
  const SolidModel m = solid_model_from_json(R"({"model_version": 1})");
  CHECK_FALSE(m.has_storeys());
  CHECK_FALSE(m.has_walls());
  CHECK_FALSE(m.has_spaces());
  CHECK_FALSE(m.has_openings());
}

TEST_CASE("LoadSolidModel_ProjectWithoutSolidModelTables_ProvidesNothing",
          "[io][solid_model]") {
  reusex::test_support::TempPath tmp("test_load_solid_model");
  reusex::ProjectDB db(tmp.path);
  const SolidModel m = reusex::io::load_solid_model(db);
  CHECK_FALSE(m.has_storeys());
  CHECK_FALSE(m.has_walls());
  CHECK_FALSE(m.has_spaces());
  CHECK_FALSE(m.has_openings());
}
