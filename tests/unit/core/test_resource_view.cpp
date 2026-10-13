// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The resource view record (spec 2026-10-13-resource-view-record §2): shape
// validation, the canonical sys:view JSON, its display text and padding.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/resource_view.hpp>

#include <stdexcept>
#include <string>

using namespace reusex::core;
using Catch::Matchers::WithinAbs;

TEST_CASE("ViewToJson_FrameWithRegion_IsCompactKindIdRegion", "[core][view]") {
  const ResourceView v{ViewKind::frame, 123, ViewRegion{0.1, 0.2, 0.3, 0.4}};
  CHECK(view_to_json(v) ==
        R"({"kind":"frame","id":123,"region":[0.1,0.2,0.3,0.4]})");
  const ResourceView whole{ViewKind::panorama, 4, std::nullopt};
  CHECK(view_to_json(whole) == R"({"kind":"panorama","id":4,"region":null})");
}

TEST_CASE("ParseView_RoundTrip_IsIdentity", "[core][view]") {
  const ResourceView v{ViewKind::frame, 7, ViewRegion{0.25, 0.5, 0.5, 0.25}};
  CHECK(parse_view(view_to_json(v)) == v);
  // Field order and a missing region do not matter on input.
  CHECK(parse_view(R"({"id":3,"kind":"panorama"})") ==
        ResourceView{ViewKind::panorama, 3, std::nullopt});
}

TEST_CASE("ParseView_BadShapes_Throw", "[core][view]") {
  for (const char *bad :
       {"not json", "[]", R"({"kind":"photo","id":1})", R"({"kind":"frame"})",
        R"({"kind":"frame","id":"1"})", R"({"kind":"frame","id":1.5})",
        R"({"kind":"frame","id":-1})", R"({"kind":"frame","id":1,"extra":0})",
        R"({"kind":"frame","id":1,"region":[0,0,1]})",
        R"({"kind":"frame","id":1,"region":[0,0,"1",1]})",
        R"({"kind":"frame","id":1,"region":[0,0,0,0.5]})",        // w = 0
        R"({"kind":"frame","id":1,"region":[-0.1,0,0.5,0.5]})",   // x < 0
        R"({"kind":"frame","id":1,"region":[0,0.6,0.5,0.5]})",    // y + h > 1
        R"({"kind":"frame","id":1,"region":[0,0,1.5,0.5]})",      // w > 1
        R"({"kind":"panorama","id":1,"region":[1.0,0,0.2,0.5]})", // x >= 1
        R"({"kind":"frame","id":99999999999,"region":null})"}) {
    INFO(bad);
    CHECK_THROWS_AS(parse_view(bad), std::invalid_argument);
  }
}

TEST_CASE("ParseView_PanoramaWrap_AcceptedFrameWrap_Throws", "[core][view]") {
  // A panorama region may cross the equirect seam; a frame region may not.
  CHECK(parse_view(R"({"kind":"panorama","id":2,"region":[0.9,0.4,0.2,0.25]})")
            .region.has_value());
  CHECK_THROWS_AS(
      parse_view(R"({"kind":"frame","id":2,"region":[0.9,0.4,0.2,0.25]})"),
      std::invalid_argument);
}

TEST_CASE("ValidateViewShape_EpsilonOvershoot_IsAccepted", "[core][view]") {
  // y + h one ulp above 1 is rounding, not an error.
  const double y = 0.3, h = 0.7000000000000001;
  CHECK_NOTHROW(
      validate_view_shape({ViewKind::frame, 1, ViewRegion{0, y, 1.0, h}}));
}

TEST_CASE("ViewDisplay_KindAndId_DanishLabel", "[core][view]") {
  CHECK(view_display({ViewKind::frame, 123, std::nullopt}) == "Foto #123");
  CHECK(view_display({ViewKind::panorama, 4, std::nullopt}) == "360° #4");
}

TEST_CASE("PadRegion_FifteenPercentPerSide_ClampedToImage", "[core][view]") {
  const auto r = pad_region({0.4, 0.4, 0.2, 0.1});
  CHECK_THAT(r[0], WithinAbs(0.37, 1e-12));
  CHECK_THAT(r[1], WithinAbs(0.385, 1e-12));
  CHECK_THAT(r[2], WithinAbs(0.26, 1e-12));
  CHECK_THAT(r[3], WithinAbs(0.13, 1e-12));
  const auto edge = pad_region({0.0, 0.9, 1.0, 0.1});
  CHECK(edge[0] == 0.0);
  CHECK(edge[2] == 1.0);
  CHECK(edge[1] + edge[3] <= 1.0 + kViewRegionEpsilon);
}

TEST_CASE("ViewKindFromString_Unknown_ReturnsNullopt", "[core][view]") {
  CHECK(view_kind_from_string("frame") == ViewKind::frame);
  CHECK(view_kind_from_string("panorama") == ViewKind::panorama);
  CHECK_FALSE(view_kind_from_string("Frame"));
  CHECK(to_string(ViewKind::panorama) == "panorama");
}
