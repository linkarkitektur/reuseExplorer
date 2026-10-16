// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core::assign_opening_guids (reusex-z1v.7.3): a re-detected opening keeps
// the GUID of the stored one in the same wall.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/guid.hpp>
#include <reusex/core/opening_identity.hpp>

#include <stdexcept>
#include <string>
#include <vector>

using namespace reusex::core;

namespace {

constexpr const char *kProject = "6f1c2a8e-4b1d-4c5e-9a7f-0d3e2b1a4c5d";

OpeningRecord op(const std::string &guid, const std::string &wall, double s0,
                 double s1) {
  OpeningRecord o;
  o.guid = guid;
  o.wall = wall;
  o.s_start = s0;
  o.s_end = s1;
  o.z_head = 2.0;
  return o;
}

} // namespace

TEST_CASE("OpeningIdentity_FreshOpenings_MintFromTheirKey",
          "[core][openings][identity]") {
  std::vector<OpeningRecord> fresh{op("openings/w1/1.00", "w1", 1.0, 1.9)};
  const auto r = assign_opening_guids(fresh, {}, kProject);
  CHECK(r.minted == 1);
  CHECK(fresh[0].guid == deterministic_guid(kProject, "openings/w1/1.00"));
}

TEST_CASE("OpeningIdentity_MovedJamb_KeepsTheStoredGuid",
          "[core][openings][identity]") {
  auto stored = op("stored-guid", "w1", 1.0, 1.9);
  stored.locked = true;
  std::vector<OpeningRecord> fresh{
      op("openings/w1/1.05", "w1", 1.04, 1.92), // same opening
      op("openings/w2/1.05", "w2", 1.04, 1.92), // other wall: new
      op("openings/w1/3.00", "w1", 3.0, 3.8)};  // too far: new
  const auto r = assign_opening_guids(fresh, {stored}, kProject);
  CHECK(r.matched == 1);
  CHECK(r.locked_matched == 1);
  CHECK(r.minted == 2);
  CHECK(fresh[0].guid == "stored-guid");
  CHECK(fresh[1].guid != "stored-guid");
  CHECK(fresh[2].guid != "stored-guid");
}

TEST_CASE("OpeningIdentity_RejectsDuplicateKeysAndABadNamespace",
          "[core][openings][identity]") {
  std::vector<OpeningRecord> twice{op("k", "w", 1, 2), op("k", "w", 3, 4)};
  CHECK_THROWS_AS(assign_opening_guids(twice, {}, kProject),
                  std::invalid_argument);
  std::vector<OpeningRecord> one{op("k", "w", 1, 2)};
  CHECK_THROWS_AS(assign_opening_guids(one, {}, "not-a-uuid"),
                  std::invalid_argument);
}
