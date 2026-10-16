// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Stable wall GUIDs across re-runs (reusex-z1v.5.5, core/wall_identity.hpp).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/guid.hpp>
#include <reusex/core/wall_identity.hpp>

#include <stdexcept>
#include <string>
#include <vector>

using reusex::core::assign_wall_guids;
using reusex::core::deterministic_guid;
using reusex::core::WallJoinRecord;
using reusex::core::WallRecord;

namespace {

const std::string kProject = "6f1c8e9a-3b2d-4c5e-8f7a-1b2c3d4e5f60";

WallRecord wall(const std::string &key, int storey, double x0, double y0,
                double x1, double y1) {
  WallRecord w;
  w.guid = key;
  w.storey = storey;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  return w;
}

} // namespace

TEST_CASE("WallIdentity_FirstRun_MintsFromTheKey", "[core][walls][guid]") {
  std::vector<WallRecord> fresh{wall("walls/0/0,0,4,0", 0, 0, 0, 4, 0),
                                wall("walls/0/4,0,4,3", 0, 4, 0, 4, 3)};
  std::vector<WallJoinRecord> joins{
      {"walls/0/0,0,4,0", "walls/0/4,0,4,3", "corner", "end", "start"}};
  const auto r = assign_wall_guids(fresh, joins, {}, kProject);
  CHECK(r.matched == 0);
  CHECK(r.minted == 2);
  CHECK(fresh[0].guid == deterministic_guid(kProject, "walls/0/0,0,4,0"));
  CHECK(fresh[1].guid == deterministic_guid(kProject, "walls/0/4,0,4,3"));
  CHECK(joins[0].wall_a == fresh[0].guid);
  CHECK(joins[0].wall_b == fresh[1].guid);
}

TEST_CASE("WallIdentity_ReDetectedWall_KeepsItsGuid", "[core][walls][guid]") {
  std::vector<WallRecord> first{wall("walls/0/0,0,4,0", 0, 0, 0, 4, 0)};
  std::vector<WallJoinRecord> none;
  assign_wall_guids(first, none, {}, kProject);

  // The re-run moved the wall 3 cm and its end across a 5 cm rounding
  // boundary: another key, the same wall.
  std::vector<WallRecord> again{
      wall("walls/0/0,0.05,4.05,0.05", 0, 0.01, 0.03, 4.03, 0.03),
      wall("walls/0/9,9,9,12", 0, 9, 9, 9, 12)};
  const auto r = assign_wall_guids(again, none, first, kProject);
  CHECK(r.matched == 1);
  CHECK(r.minted == 1);
  CHECK(again[0].guid == first[0].guid);
  CHECK(again[1].guid == deterministic_guid(kProject, "walls/0/9,9,9,12"));
}

TEST_CASE("WallIdentity_SameData_SameGuids", "[core][walls][guid]") {
  const std::vector<WallRecord> run{
      wall("walls/0/0,0,4,0", 0, 0, 0, 4, 0),
      wall("walls/0/0,0.2,4,0.2", 0, 0, 0.2, 4, 0.2),
      wall("walls/1/0,0,4,0", 1, 0, 0, 4, 0)};
  auto a = run;
  std::vector<WallJoinRecord> none;
  assign_wall_guids(a, none, {}, kProject);
  auto b = run;
  const auto r = assign_wall_guids(b, none, a, kProject);
  CHECK(r.matched == 3);
  for (size_t i = 0; i < a.size(); ++i)
    CHECK(a[i].guid == b[i].guid);
}

TEST_CASE("WallIdentity_DoesNotMatchAcrossStoreysOrAngles",
          "[core][walls][guid]") {
  std::vector<WallRecord> stored{wall("walls/0/0,0,4,0", 0, 0, 0, 4, 0)};
  std::vector<WallJoinRecord> none;
  assign_wall_guids(stored, none, {}, kProject);
  std::vector<WallRecord> fresh{
      wall("walls/1/0,0,4,0", 1, 0, 0, 4, 0),         // other storey
      wall("walls/0/0,0,0,4", 0, 0, 0, 0, 4),         // perpendicular
      wall("walls/0/0,0.6,4,0.6", 0, 0, 0.6, 4, 0.6), // too far off
      wall("walls/0/6,0,9,0", 0, 6, 0, 9, 0)};        // no overlap
  const auto r = assign_wall_guids(fresh, none, stored, kProject);
  CHECK(r.matched == 0);
  CHECK(r.minted == 4);
}

TEST_CASE("WallIdentity_MintedGuid_AvoidsAStoredOne", "[core][walls][guid]") {
  // A stored wall holds the GUID the new key would mint (it moved away).
  std::vector<WallRecord> stored{wall("walls/0/0,0,4,0", 0, 20, 20, 24, 20)};
  stored[0].guid = deterministic_guid(kProject, "walls/0/0,0,4,0");
  std::vector<WallRecord> fresh{wall("walls/0/0,0,4,0", 0, 0, 0, 4, 0)};
  std::vector<WallJoinRecord> none;
  const auto r = assign_wall_guids(fresh, none, stored, kProject);
  CHECK(r.minted == 1);
  CHECK(fresh[0].guid != stored[0].guid);
  CHECK(fresh[0].guid == deterministic_guid(kProject, "walls/0/0,0,4,0#2"));
}

TEST_CASE("WallIdentity_RejectsBadInput", "[core][walls][guid]") {
  std::vector<WallJoinRecord> none;
  std::vector<WallRecord> dup{wall("k", 0, 0, 0, 1, 0),
                              wall("k", 0, 0, 1, 1, 1)};
  CHECK_THROWS_AS(assign_wall_guids(dup, none, {}, kProject),
                  std::invalid_argument);
  std::vector<WallRecord> one{wall("k", 0, 0, 0, 1, 0)};
  CHECK_THROWS_AS(assign_wall_guids(one, none, {}, "not-a-uuid"),
                  std::invalid_argument);
}
