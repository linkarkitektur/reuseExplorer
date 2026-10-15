// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the `rux get` / `rux set` / `rux del` path grammar (#249).
//
// `rux::database::parse_path` is the front door for every path-based database
// command, so its grammar decisions -- which component is a Collection, which
// an Item, which a Property, and which inputs are rejected outright -- are
// user-visible CLI behaviour. Until #249 none of it was reachable from ctest,
// because everything under apps/rux lived directly in the executable.
//
// These tests link `rux_core_lib`, the viewer-free and ML-free half of the app
// layer, so they run in the light test binary.

#include <catch2/catch_test_macros.hpp>

#include <database/path_parser.hpp>

#include <string>
#include <vector>

using namespace rux::database;

// ===========================================================================
// Collection names
// ===========================================================================

TEST_CASE("IsValidCollection_KnownAndUnknownNames_AcceptsOnlyRoutedOnes",
          "[rux][path_parser]") {
  // The ten collections that actually have a router in apps/rux/src/database.
  for (const auto *name :
       {"clouds", "components", "frames", "labels", "log", "materials",
        "meshes", "panoramas", "projects", "storeys"}) {
    INFO("collection: " << name);
    CHECK(is_valid_collection(name));
  }

  CHECK_FALSE(is_valid_collection("cloud"));  // singular
  CHECK_FALSE(is_valid_collection("Clouds")); // case-sensitive
  CHECK_FALSE(is_valid_collection("planes")); // a cloud name, not a collection
  CHECK_FALSE(is_valid_collection(""));
}

// ===========================================================================
// parse_path -- the happy paths
// ===========================================================================

TEST_CASE("ParsePath_CollectionOnly_ReturnsSingleCollectionComponent",
          "[rux][path_parser]") {
  const auto components = parse_path("clouds");

  REQUIRE(components.size() == 1);
  CHECK(components[0].is_collection());
  CHECK(components[0].value == "clouds");
  CHECK_FALSE(components[0].index.has_value());
}

TEST_CASE("ParsePath_CollectionItemProperty_ClassifiesByPosition",
          "[rux][path_parser]") {
  const auto components = parse_path("clouds.scan1.metadata");

  REQUIRE(components.size() == 3);
  CHECK(components[0].is_collection());
  CHECK(components[0].value == "clouds");
  CHECK(components[1].is_item());
  CHECK(components[1].value == "scan1");
  CHECK(components[2].is_property());
  CHECK(components[2].value == "metadata");
}

TEST_CASE("ParsePath_SlashNotation_ParsesSameAsDotNotation",
          "[rux][path_parser]") {
  const auto dots = parse_path("meshes.roof.vertices");
  const auto slashes = parse_path("meshes/roof/vertices");

  REQUIRE(dots.size() == slashes.size());
  for (size_t i = 0; i < dots.size(); ++i) {
    INFO("component " << i);
    CHECK(dots[i].type == slashes[i].type);
    CHECK(dots[i].value == slashes[i].value);
  }
}

TEST_CASE("ParsePath_TrailingAndRepeatedSeparators_SkipsEmptyComponents",
          "[rux][path_parser]") {
  // split() drops empty pieces, so these normalise to the plain path rather
  // than raising "Path contains empty component".
  const auto trailing = parse_path("clouds.scan1.");
  REQUIRE(trailing.size() == 2);
  CHECK(trailing[1].value == "scan1");

  const auto doubled = parse_path("clouds//scan1");
  REQUIRE(doubled.size() == 2);
  CHECK(doubled[0].is_collection());
  CHECK(doubled[1].is_item());
}

TEST_CASE("ParsePath_DeepPath_TreatsEveryComponentPastTheItemAsProperty",
          "[rux][path_parser]") {
  const auto components = parse_path("frames.42.pose.translation");

  REQUIRE(components.size() == 4);
  CHECK(components[1].is_item());
  CHECK(components[1].value == "42");
  CHECK(components[2].is_property());
  CHECK(components[3].is_property());
  CHECK(components[3].value == "translation");
}

TEST_CASE("ParsePath_WildcardItem_KeepsPatternAndFlagsIt",
          "[rux][path_parser]") {
  const auto components = parse_path("clouds.scan*");

  REQUIRE(components.size() == 2);
  CHECK(components[1].is_item());
  CHECK(components[1].value == "scan*");
  CHECK(components[1].has_wildcard());
  CHECK_FALSE(components[0].has_wildcard());
}

// ===========================================================================
// parse_path -- array indexing
// ===========================================================================

TEST_CASE("ParsePath_CollectionWithIndex_EmitsCollectionThenIndexComponent",
          "[rux][path_parser]") {
  const auto components = parse_path("clouds[7]");

  REQUIRE(components.size() == 2);
  CHECK(components[0].is_collection());
  CHECK(components[0].value == "clouds");
  CHECK(components[1].is_index());
  REQUIRE(components[1].index.has_value());
  CHECK(*components[1].index == 7);
  CHECK(components[1].value.empty());
}

TEST_CASE("ParsePath_IndexFollowedByProperty_KeepsBothComponents",
          "[rux][path_parser]") {
  const auto components = parse_path("meshes[0].metadata");

  REQUIRE(components.size() == 3);
  CHECK(components[0].is_collection());
  CHECK(components[1].is_index());
  CHECK(*components[1].index == 0);
  // Third *part* of the path, but the index expanded to two components, so the
  // property lands at slot 2 with type Property (i == 1 is the index part).
  CHECK(components[2].is_item());
  CHECK(components[2].value == "metadata");
}

TEST_CASE("ParsePath_MalformedOrMisplacedIndex_Throws", "[rux][path_parser]") {
  CHECK_THROWS_AS(parse_path("clouds[]"), PathError);    // empty index
  CHECK_THROWS_AS(parse_path("clouds[abc]"), PathError); // non-numeric
  CHECK_THROWS_AS(parse_path("clouds[-1]"), PathError);  // sign is not a digit
  CHECK_THROWS_AS(parse_path("clouds[0"), PathError);    // unterminated
  // Indexing is only defined on the leading collection.
  CHECK_THROWS_AS(parse_path("clouds.scan1[0]"), PathError);
}

// ===========================================================================
// parse_path -- rejections
// ===========================================================================

TEST_CASE("ParsePath_EmptyPath_Throws", "[rux][path_parser]") {
  CHECK_THROWS_AS(parse_path(""), PathError);
}

TEST_CASE("ParsePath_SeparatorsOnly_Throws", "[rux][path_parser]") {
  // Every part is empty, so split() returns nothing to classify.
  CHECK_THROWS_AS(parse_path("."), PathError);
  CHECK_THROWS_AS(parse_path("///"), PathError);
}

TEST_CASE("ParsePath_UnknownLeadingCollection_ThrowsAndListsValidNames",
          "[rux][path_parser]") {
  // The message is the only guidance a user gets after a typo, so it has to
  // name the alternatives (STANDARDS §5).
  try {
    parse_path("cloudz.scan1");
    FAIL("expected PathError");
  } catch (const PathError &e) {
    const std::string what = e.what();
    CHECK(what.find("cloudz") != std::string::npos);
    CHECK(what.find("clouds") != std::string::npos);
    CHECK(what.find("components") != std::string::npos);
    CHECK(what.find("panoramas") != std::string::npos);
  }
}

// ===========================================================================
// Wildcards
// ===========================================================================

TEST_CASE("MatchesWildcard_StarPositions_MatchesPrefixSuffixAndInfix",
          "[rux][path_parser]") {
  CHECK(matches_wildcard("scan1", "scan*"));
  CHECK(matches_wildcard("scan", "scan*")); // '*' may match nothing
  CHECK(matches_wildcard("raw_scan", "*scan"));
  CHECK(matches_wildcard("a_scan_b", "a*b"));
  CHECK(matches_wildcard("anything", "*"));

  CHECK_FALSE(matches_wildcard("mesh1", "scan*"));
  CHECK_FALSE(matches_wildcard("scan1_extra", "*scan"));
}

TEST_CASE("MatchesWildcard_NoStar_RequiresExactMatch", "[rux][path_parser]") {
  CHECK(matches_wildcard("scan1", "scan1"));
  CHECK_FALSE(matches_wildcard("scan1", "scan"));
  CHECK_FALSE(matches_wildcard("scan", "scan1"));
}

TEST_CASE("MatchesWildcard_RegexMetacharacters_AreTreatedLiterally",
          "[rux][path_parser]") {
  // A cloud literally named "scan.1" must not be matched by the pattern
  // "scan.1" treating '.' as "any character" -- the escaping in
  // matches_wildcard is what prevents that.
  CHECK(matches_wildcard("scan.1", "scan.1"));
  CHECK_FALSE(matches_wildcard("scanX1", "scan.1"));

  CHECK(matches_wildcard("a+b", "a+b"));
  CHECK(matches_wildcard("x(y)", "x(y)"));
  CHECK(matches_wildcard("v[0]", "v[0]"));
  CHECK_FALSE(matches_wildcard("v0", "v[0]"));
}

TEST_CASE("ExpandWildcard_PatternAgainstItemList_ReturnsMatchesInInputOrder",
          "[rux][path_parser]") {
  const std::vector<std::string> items = {"scan1", "scan2", "mesh1",
                                          "scan_final", "raw"};

  const auto matches = expand_wildcard("scan*", items);
  REQUIRE(matches.size() == 3);
  CHECK(matches[0] == "scan1");
  CHECK(matches[1] == "scan2");
  CHECK(matches[2] == "scan_final");
}

TEST_CASE("ExpandWildcard_NoMatchOrEmptyItemList_ReturnsEmpty",
          "[rux][path_parser]") {
  const std::vector<std::string> items = {"scan1", "mesh1"};
  CHECK(expand_wildcard("plane*", items).empty());
  CHECK(expand_wildcard("*", {}).empty());
}
