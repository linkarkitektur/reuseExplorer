// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Storey identity across re-runs (reusex-z1v.3.3 review): a re-detected
// storey keeps the GUID of the stored storey whose floor it matches in the
// new level frame, not the GUID of whatever row had the same idx.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/guid.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/core/storey_identity.hpp>

#include <array>
#include <cmath>
#include <set>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex::core;

namespace {

const std::string kProject = "12345678-1234-5678-1234-567812345678";

/// World -> level transform that only lifts by @p dz (level z = world z - dz).
std::array<double, 16> lift(double dz) {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -dz, 0, 0, 0, 1};
}

StoreyRecord storey(int idx, double z_floor, double z_ceiling,
                    std::array<double, 16> t, std::string guid = {}) {
  StoreyRecord s;
  s.idx = idx;
  s.z_floor = z_floor;
  s.z_ceiling = z_ceiling;
  s.level_transform = t;
  s.guid = std::move(guid);
  return s;
}

} // namespace

TEST_CASE("FloorInFrame_ConvertsBetweenLevelFrames", "[core][storeys]") {
  // Stored in a frame whose origin is world z = 10; floor 3 m up = world 13.
  const auto s = storey(1, 3.0, 5.6, lift(10.0));
  CHECK_THAT(floor_in_frame(s, lift(7.0)), WithinAbs(6.0, 1e-12));
  CHECK_THAT(floor_in_frame(s, lift(10.0)), WithinAbs(3.0, 1e-12));
}

TEST_CASE("AssignStoreyGuids_FirstRun_MintsUniqueDeterministicGuids",
          "[core][storeys]") {
  std::vector<StoreyRecord> fresh{storey(0, 0.0, 2.6, lift(0)),
                                  storey(1, 3.0, 5.6, lift(0))};
  auto again = fresh;
  const auto r = assign_storey_guids(fresh, {}, kProject);
  CHECK(r.matched == 0);
  CHECK(r.minted == 2);
  CHECK(fresh[0].guid != fresh[1].guid);
  assign_storey_guids(again, {}, kProject);
  CHECK(again[0].guid == fresh[0].guid);
  CHECK(again[1].guid == fresh[1].guid);
}

TEST_CASE("AssignStoreyGuids_NewLowerStorey_OldStoreysKeepTheirGuids",
          "[core][storeys]") {
  // First run: two storeys, frame origin at world z = 0.
  std::vector<StoreyRecord> first{storey(0, 0.0, 2.6, lift(0)),
                                  storey(1, 3.0, 5.6, lift(0))};
  assign_storey_guids(first, {}, kProject);

  // Re-run finds a basement: the frame origin moves to world z = -3 and
  // every idx shifts up by one.
  std::vector<StoreyRecord> second{storey(0, 0.0, 2.6, lift(-3)),
                                   storey(1, 3.0, 5.6, lift(-3)),
                                   storey(2, 6.0, 8.6, lift(-3))};
  const auto r = assign_storey_guids(second, first, kProject);
  CHECK(r.matched == 2);
  CHECK(r.minted == 1);
  CHECK(second[1].guid == first[0].guid);
  CHECK(second[2].guid == first[1].guid);
  const std::set<std::string> all{second[0].guid, second[1].guid,
                                  second[2].guid};
  CHECK(all.size() == 3); // the basement's minted GUID collides with nothing
}

TEST_CASE("AssignStoreyGuids_BeyondTolerance_IsNotAMatch", "[core][storeys]") {
  std::vector<StoreyRecord> first{storey(0, 0.0, 2.6, lift(0))};
  assign_storey_guids(first, {}, kProject);
  std::vector<StoreyRecord> moved{storey(0, 0.0, 2.6, lift(0.8))};
  const auto r =
      assign_storey_guids(moved, first, kProject, StoreyIdentityOptions{0.5});
  CHECK(r.matched == 0);
  CHECK(moved[0].guid != first[0].guid);
}

TEST_CASE("LockedStoreyConflicts_ReportFrameAndOverlap", "[core][storeys]") {
  auto locked = storey(0, 0.0, 2.6, lift(0), deterministic_guid(kProject, "x"));
  locked.locked = true;
  locked.source = "manual";
  auto unlocked =
      storey(1, 3.0, 5.6, lift(0), deterministic_guid(kProject, "y"));

  SECTION("same frame, no overlap: nothing to report") {
    const std::vector<StoreyRecord> fresh{storey(1, 3.0, 5.6, lift(0))};
    CHECK(locked_storey_conflicts({locked, unlocked}, fresh).empty());
  }
  SECTION("different frame and an overlapping new row") {
    // New frame 0.2 m lower: the locked range [0, 2.6] sits at [0.2, 2.8].
    const std::vector<StoreyRecord> fresh{storey(0, 0.0, 2.7, lift(-0.2))};
    const auto c = locked_storey_conflicts({locked, unlocked}, fresh);
    REQUIRE(c.size() == 1);
    CHECK(c[0].guid == locked.guid);
    CHECK(c[0].frame_differs);
    CHECK_THAT(c[0].frame_shift_m, WithinAbs(0.2, 1e-9));
    CHECK(c[0].overlapping_idx == std::vector<int>{0});
  }
}
