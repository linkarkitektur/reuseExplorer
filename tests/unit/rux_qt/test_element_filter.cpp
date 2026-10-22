// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The 3D workspace's "Bygningsdele" toggles: which building-element
// categories and which storey are shown (rux_qt/element_filter.hpp).

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/element_filter.hpp>

#include <string>
#include <vector>

using namespace rux::qt;

namespace {

/// Two storeys of walls, a roof on the top one, doors and windows, and a
/// door component on no storey.
std::vector<ElementTally> tallies() {
  return {{"wall", 0, 106}, {"wall", 1, 94}, {"roof", 1, 1},
          {"door", 0, 60},  {"door", 1, 40}, {"window", 1, 85},
          {"door", -1, 2},  {"space", 0, 0}};
}

} // namespace

TEST_CASE("ElementFilter_Rows_ListPresentCategoriesInOrderWithCounts",
          "[rux_qt][elements]") {
  ElementFilter f;
  f.set_tallies(tallies());
  const auto rows = f.rows();
  // wall, slab, roof, space, door, window, opening -- only those present
  // (a zero count is not present).
  REQUIRE(rows.size() == 4);
  CHECK(rows[0].key == "wall");
  CHECK(rows[0].label == "Vægge");
  CHECK(rows[0].count == 200);
  CHECK(rows[1].key == "roof");
  CHECK(rows[1].label == "Tag");
  CHECK(rows[2].key == "door");
  CHECK(rows[2].label == "Døre");
  CHECK(rows[2].count == 102);
  CHECK(rows[3].key == "window");
  CHECK(rows[3].label == "Vinduer");
  CHECK(f.storeys() == std::vector<int>{0, 1});
  CHECK(f.has_elements());
  ElementFilter none;
  CHECK_FALSE(none.has_elements());
  CHECK(none.all_state() == CheckState::off);
}

TEST_CASE("ElementFilter_Labels_AreDanish", "[rux_qt][elements]") {
  CHECK(element_category_label("wall") == "Vægge");
  CHECK(element_category_label("slab") == "Dæk");
  CHECK(element_category_label("roof") == "Tag");
  CHECK(element_category_label("space") == "Rum");
  CHECK(element_category_label("door") == "Døre");
  CHECK(element_category_label("window") == "Vinduer");
  CHECK(element_category_label("opening") == "Andre åbninger");
  CHECK(element_category_label("chimney") == "chimney");
}

TEST_CASE("ElementFilter_Master_SwitchesEveryCategoryAndReportsPartial",
          "[rux_qt][elements]") {
  ElementFilter f;
  f.set_tallies(tallies());
  CHECK(f.all_state() == CheckState::on);
  f.set_shown("wall", false);
  CHECK_FALSE(f.shown("wall"));
  CHECK(f.shown("door"));
  CHECK(f.all_state() == CheckState::partial);
  f.set_all(false);
  CHECK(f.all_state() == CheckState::off);
  CHECK_FALSE(f.shown("window"));
  f.set_all(true);
  CHECK(f.all_state() == CheckState::on);
  CHECK(f.shown("wall"));
  // A category the project lacks does not make the master partial.
  f.set_shown("slab", false);
  CHECK(f.all_state() == CheckState::on);
}

TEST_CASE("ElementFilter_Only_ShowsJustTheGivenCategories",
          "[rux_qt][elements]") {
  ElementFilter f;
  f.set_tallies(tallies());
  f.show_only({"door", "window"});
  CHECK(f.shown("door"));
  CHECK(f.shown("window"));
  CHECK_FALSE(f.shown("wall"));
  CHECK_FALSE(f.shown("roof"));
  CHECK(f.all_state() == CheckState::partial);
}

TEST_CASE("ElementFilter_ActorVisible_FollowsCategoryAndStorey",
          "[rux_qt][elements]") {
  ElementFilter f;
  f.set_tallies(tallies());
  CHECK(f.actor_visible("wall", 0));
  CHECK(f.actor_visible("", 3)); // untagged actors are not filtered
  f.set_shown("wall", false);
  CHECK_FALSE(f.actor_visible("wall", 0));
  CHECK(f.actor_visible("door", 0));

  f.set_storey(1);
  CHECK(f.storey() == 1);
  CHECK_FALSE(f.actor_visible("door", 0));
  CHECK(f.actor_visible("door", 1));
  CHECK(f.actor_visible("door", -1)); // not on a storey: always shown
  // Counts follow the storey filter.
  const auto rows = f.rows();
  REQUIRE(rows.size() == 4);
  CHECK(rows[0].count == 94); // walls on storey 1
  CHECK(rows[2].count == 42); // doors on storey 1 + the unplaced ones
  f.set_storey(std::nullopt);
  CHECK(f.actor_visible("door", 0));
}

TEST_CASE("ElementFilter_SetTallies_KeepsChoicesAndDropsAGoneStorey",
          "[rux_qt][elements]") {
  ElementFilter f;
  f.set_tallies(tallies());
  f.set_shown("wall", false);
  f.set_storey(1);
  f.set_tallies({{"wall", 0, 3}, {"door", 0, 1}});
  CHECK_FALSE(f.shown("wall"));        // a reload keeps what was hidden
  CHECK_FALSE(f.storey().has_value()); // storey 1 is gone
  f.reset();
  CHECK(f.shown("wall"));
}

TEST_CASE("OpeningFrames_ShowWhenTheirBoxIsOnOrTheSolidWallsAreHidden",
          "[rux_qt][elements]") {
  ElementFilter f;
  f.set_tallies(tallies());
  // The solids draw the openings as reveals in the walls: no frames on top.
  CHECK_FALSE(opening_frames_shown(false, true, f));
  CHECK(opening_frames_shown(true, true, f));
  // Walls hidden: the reveals went with them, so the frames stand in.
  f.set_shown("wall", false);
  CHECK(opening_frames_shown(false, true, f));
  // No solids drawn: only the box decides.
  CHECK_FALSE(opening_frames_shown(false, false, f));
  CHECK(opening_frames_shown(true, false, f));
}
