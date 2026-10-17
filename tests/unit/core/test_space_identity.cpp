// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core::assign_space_guids (reusex-z1v.6.3): a space's natural key is its
// storey and its polygon's centroid rounded to 5 cm; a re-detected space
// keeps the GUID of the stored one it lies on.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/guid.hpp>
#include <reusex/core/space_identity.hpp>

#include <stdexcept>
#include <string>
#include <vector>

using namespace reusex::core;

namespace {

constexpr const char *kProject = "6f1c2a8e-4b1d-4c5e-9a7f-0d3e2b1a4c5d";

SpaceRecord sp(const std::string &guid, int storey, double x0, double y0,
               double w, double h) {
  SpaceRecord s;
  s.guid = guid;
  s.storey = storey;
  s.polygon.outer = {{x0, y0}, {x0 + w, y0}, {x0 + w, y0 + h}, {x0, y0 + h}};
  s.area = w * h;
  return s;
}

} // namespace

TEST_CASE("SpaceIdentity_NaturalKey_StoreyAndCentroidAt5cm",
          "[core][spaces][identity]") {
  // Centroid (2.012, 1.537) -> 2.00, 1.55
  CHECK(space_natural_key(sp("x", 1, 0.012, 0.037, 4.0, 3.0)) ==
        "spaces/1/2.00,1.55");
  // -0.024 rounds to -0.00 -> printed as 0.00
  CHECK(space_natural_key(sp("x", 0, -1.024, -1.0, 2.0, 2.0)) ==
        "spaces/0/0.00,0.00");
}

TEST_CASE("SpaceIdentity_FreshSpaces_MintFromTheirKey",
          "[core][spaces][identity]") {
  std::vector<SpaceRecord> fresh{sp("space-0-1", 0, 0, 0, 4, 4)};
  std::vector<SpaceBoundaryRecord> bounds{{"space-0-1", "w", "a", 0, 4}};
  const auto r = assign_space_guids(fresh, bounds, {}, kProject);
  CHECK(r.minted == 1);
  CHECK(r.matched == 0);
  CHECK(fresh[0].guid == deterministic_guid(kProject, "spaces/0/2.00,2.00"));
  CHECK(bounds[0].space == fresh[0].guid);

  // Deterministic: the same input gives the same GUID.
  std::vector<SpaceRecord> again{sp("space-0-7", 0, 0, 0, 4, 4)};
  std::vector<SpaceBoundaryRecord> none;
  assign_space_guids(again, none, {}, kProject);
  CHECK(again[0].guid == fresh[0].guid);
}

TEST_CASE("SpaceIdentity_MovedOutline_KeepsTheStoredGuid",
          "[core][spaces][identity]") {
  auto stored = sp("stored-guid", 0, 0, 0, 4, 4);
  stored.locked = true;
  std::vector<SpaceRecord> fresh{
      sp("p1", 0, 0.1, -0.05, 4.1, 4.0), // same room, outline moved
      sp("p2", 1, 0, 0, 4, 4),           // same place, other storey: new
      sp("p3", 0, 6, 0, 4, 4)};          // elsewhere: new
  std::vector<SpaceBoundaryRecord> bounds{{"p1", "w", "a", 0, 4},
                                          {"p3", "w", "b", 6, 10}};
  const auto r = assign_space_guids(fresh, bounds, {stored}, kProject);
  CHECK(r.matched == 1);
  CHECK(r.locked_matched == 1);
  CHECK(r.minted == 2);
  CHECK(fresh[0].guid == "stored-guid");
  CHECK(fresh[1].guid != "stored-guid");
  CHECK(fresh[2].guid != "stored-guid");
  CHECK(bounds[0].space == "stored-guid");
  CHECK(bounds[1].space == fresh[2].guid);
}

TEST_CASE("SpaceIdentity_SplitRoom_OnlyOneHalfInherits",
          "[core][spaces][identity]") {
  // A stored 8 x 4 room is now two 4 x 4 rooms. With a generous match_m
  // both centroids are close enough, but each half has half the stored
  // area, below min_area_ratio, so neither inherits the GUID.
  const auto stored = sp("stored-guid", 0, 0, 0, 8, 4);
  std::vector<SpaceRecord> fresh{sp("p1", 0, 0, 0, 4, 4),
                                 sp("p2", 0, 4, 0, 4, 4)};
  std::vector<SpaceBoundaryRecord> none;
  SpaceIdentityOptions opt;
  opt.match_m = 3.0;
  opt.min_area_ratio = 0.6; // 16 / 32 = 0.5 < 0.6
  const auto r = assign_space_guids(fresh, none, {stored}, kProject, opt);
  CHECK(r.matched == 0);
  CHECK(r.minted == 2);
  CHECK(fresh[0].guid != fresh[1].guid);
}

TEST_CASE("SpaceIdentity_RejectsDuplicatePlaceholdersAndABadNamespace",
          "[core][spaces][identity]") {
  std::vector<SpaceRecord> twice{sp("k", 0, 0, 0, 1, 1),
                                 sp("k", 0, 5, 0, 1, 1)};
  std::vector<SpaceBoundaryRecord> none;
  CHECK_THROWS_AS(assign_space_guids(twice, none, {}, kProject),
                  std::invalid_argument);
  std::vector<SpaceRecord> one{sp("k", 0, 0, 0, 1, 1)};
  CHECK_THROWS_AS(assign_space_guids(one, none, {}, "not-a-uuid"),
                  std::invalid_argument);
}

TEST_CASE("SpaceIdentity_SameKeyTwice_GetsASuffix",
          "[core][spaces][identity]") {
  // Two spaces whose centroids round to the same 5 cm cell (a ring room and
  // the room it encloses) still get distinct GUIDs.
  std::vector<SpaceRecord> fresh{sp("p1", 0, 0, 0, 4, 4),
                                 sp("p2", 0, 1.99, 1.99, 0.03, 0.03)};
  std::vector<SpaceBoundaryRecord> none;
  assign_space_guids(fresh, none, {}, kProject);
  CHECK(fresh[0].guid == deterministic_guid(kProject, "spaces/0/2.00,2.00"));
  CHECK(fresh[1].guid == deterministic_guid(kProject, "spaces/0/2.00,2.00#2"));
}
