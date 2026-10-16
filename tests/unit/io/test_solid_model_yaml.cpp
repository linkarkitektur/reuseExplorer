// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The YAML front-end for the solid-model document (reusex-yf4.1): a
// human-friendly, comment-allowing way to author the same document
// `solid_model_from_json` / `solid_model_to_json` read and write, lossless
// with the JSON text and the `core::SolidModel` records.

#include "../../support/source_dir.hpp"
#include "../../support/temp_path.hpp"

#include <reusex/core/solid_model.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

using Catch::Matchers::ContainsSubstring;
using reusex::core::SolidModel;
using reusex::io::solid_model_from_file;
using reusex::io::solid_model_from_json;
using reusex::io::solid_model_from_yaml;
using reusex::io::solid_model_to_file;
using reusex::io::solid_model_to_json;
using reusex::io::solid_model_to_yaml;

namespace {

std::filesystem::path fixture(const char *name) {
  return reusex::test_support::source_dir() / "tests" / "fixtures" /
         "solid_model" / name;
}

/// A minimal valid document, the YAML twin of test_solid_model_json.cpp's
/// minimal_doc(), with a comment and mixed quoting thrown in.
const char *kMinimalYaml = R"yaml(
# A minimal reference plan (this comment must not survive parsing, since
# there is nowhere in core::SolidModel to keep it).
model_version: 1
storeys:
  - idx: 0
    name: Ground
    z_floor: 0
    z_ceiling: 2.6
walls:
  - guid: w1
    storey: 0
    axis: [[0, 0], [4, 0]]
    thickness: 0.2
    is_external: true   # a real plain boolean
  - guid: "w2"           # a quoted guid -- still just a string
    storey: 0
    axis: [[4, 0], [4, 3]]
    thickness: 0.1
spaces:
  - guid: s1
    storey: 0
    name: Kitchen
    polygon:
      outer: [[0, 0], [4, 0], [4, 3], [0, 3]]
openings:
  - guid: o1
    wall: w1
    kind: door
    s_start: 1.0
    s_end: 1.9
    z_sill: 0.0
    z_head: 2.1
    metadata: null   # explicit null: same as omitting the key
)yaml";

} // namespace

TEST_CASE("SolidModelFromYaml_MinimalDocument_ParsesEveryElement",
          "[io][solid_model][yaml]") {
  const SolidModel m = solid_model_from_yaml(kMinimalYaml);
  REQUIRE(m.storeys.size() == 1);
  CHECK(m.storeys[0].name == "Ground");
  CHECK(m.storeys[0].z_ceiling == 2.6);
  REQUIRE(m.walls.size() == 2);
  CHECK(m.walls[0].guid == "w1");
  CHECK(m.walls[0].x1 == 4.0);
  CHECK(m.walls[0].thickness == 0.2);
  CHECK(m.walls[0].is_external);  // a genuine plain `true`, not a string
  CHECK(m.walls[1].guid == "w2"); // quoted guid, same string either way
  CHECK(m.walls[1].y1 == 3.0);
  REQUIRE(m.spaces.size() == 1);
  CHECK(m.spaces[0].name == "Kitchen");
  REQUIRE(m.openings.size() == 1);
  CHECK(m.openings[0].metadata.empty()); // explicit `null` == absent
}

TEST_CASE("SolidModelFromYaml_SameAsJson_ForTheSameDocument",
          "[io][solid_model][yaml]") {
  // The YAML and JSON forms of the same document must parse to records that
  // write back out identically.
  const SolidModel from_yaml = solid_model_from_yaml(kMinimalYaml);
  const SolidModel from_json = solid_model_from_json(R"json({
    "model_version": 1,
    "storeys": [{"idx": 0, "name": "Ground", "z_floor": 0, "z_ceiling": 2.6}],
    "walls": [
      {"guid": "w1", "storey": 0, "axis": [[0, 0], [4, 0]], "thickness": 0.2,
       "is_external": true},
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
  })json");
  CHECK(solid_model_to_json(from_yaml) == solid_model_to_json(from_json));
}

TEST_CASE("SolidModelYaml_RoundTrip_ThroughJsonAndRecords_IsLossless",
          "[io][solid_model][yaml]") {
  // JSON fixture -> records -> YAML -> records -> JSON must be a fixed point.
  const SolidModel original =
      solid_model_from_json(R"json({"model_version": 1})json");
  const SolidModel fixture_model =
      solid_model_from_file(fixture("two_rooms.plan.json"));
  for (const SolidModel *m : {&original, &fixture_model}) {
    const std::string yaml = solid_model_to_yaml(*m);
    const SolidModel back = solid_model_from_yaml(yaml);
    CHECK(solid_model_to_json(back) == solid_model_to_json(*m));
  }
}

TEST_CASE("SolidModelFile_YamlExtension_DispatchesToTheYamlFrontEnd",
          "[io][solid_model][yaml]") {
  const SolidModel m = solid_model_from_file(fixture("two_rooms.plan.json"));
  reusex::test_support::TempPath tmp("test_solid_model_yaml", ".yaml");
  solid_model_to_file(m, tmp.path);
  // It really is YAML, not a `.yaml`-named copy of the JSON text: a flow-style
  // JSON object line would start with '{'.
  std::ifstream in(tmp.path);
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string text = ss.str();
  CHECK_THAT(text, !ContainsSubstring("{"));
  CHECK_THAT(text, ContainsSubstring("model_version: 1"));

  const SolidModel back = solid_model_from_file(tmp.path);
  CHECK(solid_model_to_json(back) == solid_model_to_json(m));
}

TEST_CASE("SolidModelFile_YmlExtension_IsAlsoYaml", "[io][solid_model][yaml]") {
  reusex::test_support::TempPath tmp("test_solid_model_yml", ".yml");
  std::ofstream out(tmp.path);
  out << "model_version: 1\n";
  out.close();
  const SolidModel m = solid_model_from_file(tmp.path);
  CHECK_FALSE(m.has_walls());
}

TEST_CASE("SolidModelToYaml_KeyOrder_MatchesTheJsonWriter",
          "[io][solid_model][yaml]") {
  const SolidModel m = solid_model_from_file(fixture("two_rooms.plan.json"));
  const std::string yaml = solid_model_to_yaml(m);
  // Block style throughout: no flow collections.
  CHECK_THAT(yaml, !ContainsSubstring("{"));
  CHECK_THAT(yaml, !ContainsSubstring("["));
  const auto model_version_pos = yaml.find("model_version:");
  const auto units_pos = yaml.find("units:");
  const auto storeys_pos = yaml.find("storeys:");
  const auto walls_pos = yaml.find("walls:");
  REQUIRE(model_version_pos != std::string::npos);
  REQUIRE(units_pos != std::string::npos);
  REQUIRE(storeys_pos != std::string::npos);
  REQUIRE(walls_pos != std::string::npos);
  CHECK(model_version_pos < units_pos);
  CHECK(units_pos < storeys_pos);
  CHECK(storeys_pos < walls_pos);
}

TEST_CASE("SolidModelToYaml_AmbiguousGuid_RoundTripsAsAString",
          "[io][solid_model][yaml]") {
  // A guid that reads as a number or a bool if written unquoted must come
  // back as the exact same string.
  SolidModel m;
  m.provided.walls = true;
  reusex::core::WallRecord w;
  w.guid = "123"; // would be read back as an integer if left unquoted
  w.storey = 0;
  w.x0 = 0.0;
  w.y0 = 0.0;
  w.x1 = 4.0;
  w.y1 = 0.0;
  w.thickness = 0.2;
  m.walls.push_back(w);
  reusex::core::WallRecord w2;
  w2.guid = "true"; // would be read back as a bool if left unquoted
  w2.storey = 0;
  w2.x0 = 0.0;
  w2.y0 = 0.0;
  w2.x1 = 0.0;
  w2.y1 = 3.0;
  w2.thickness = 0.1;
  m.walls.push_back(w2);

  const std::string yaml = solid_model_to_yaml(m);
  CHECK_THAT(yaml, ContainsSubstring("\"123\""));
  CHECK_THAT(yaml, ContainsSubstring("\"true\""));
  const SolidModel back = solid_model_from_yaml(yaml);
  REQUIRE(back.walls.size() == 2);
  CHECK(back.walls[0].guid == "123");
  CHECK(back.walls[1].guid == "true");
  CHECK(solid_model_to_json(back) == solid_model_to_json(m));
}

TEST_CASE("SolidModelFromYaml_TypeError_SameMessageAsJson",
          "[io][solid_model][yaml]") {
  // The same malformed value (a string where a number belongs), expressed in
  // each format, must throw the exact same element-naming message: both go
  // through the one validator.
  constexpr const char *yaml_doc = R"yaml(
model_version: 1
walls:
  - guid: w1
    storey: 0
    axis: [[0, 0], [4, 0]]
    thickness: "thick"
)yaml";
  constexpr const char *json_doc = R"json({
    "model_version": 1,
    "walls": [
      {"guid": "w1", "storey": 0, "axis": [[0, 0], [4, 0]],
       "thickness": "thick"}
    ]
  })json";

  std::string yaml_error, json_error;
  try {
    (void)solid_model_from_yaml(yaml_doc);
    FAIL("expected std::invalid_argument");
  } catch (const std::invalid_argument &e) {
    yaml_error = e.what();
  }
  try {
    (void)solid_model_from_json(json_doc);
    FAIL("expected std::invalid_argument");
  } catch (const std::invalid_argument &e) {
    json_error = e.what();
  }
  CHECK(yaml_error == json_error);
  CHECK_THAT(yaml_error, ContainsSubstring("walls[0] (guid 'w1')"));
  CHECK_THAT(yaml_error, ContainsSubstring("'thickness' must be a number"));
}

TEST_CASE("SolidModelFromYaml_InvalidYamlSyntax_ThrowsInvalidArgument",
          "[io][solid_model][yaml]") {
  // Not an element-naming error (there is no element yet): a document-level
  // one, same exception type as a JSON parse failure.
  CHECK_THROWS_AS(solid_model_from_yaml("model_version: [1\n"),
                  std::invalid_argument);
}

TEST_CASE("SolidModelFromYaml_NotAMapAtTopLevel_ThrowsInvalidArgument",
          "[io][solid_model][yaml]") {
  // A bare scalar or a sequence at the document root converts to a non-object
  // JSON value, which the shared Reader rejects the same way an empty/odd
  // JSON document would.
  CHECK_THROWS_AS(solid_model_from_yaml("- 1\n- 2\n"), std::invalid_argument);
  CHECK_THROWS_AS(solid_model_from_yaml("just a string\n"),
                  std::invalid_argument);
  CHECK_THROWS_AS(solid_model_from_yaml(""), std::invalid_argument);
}
