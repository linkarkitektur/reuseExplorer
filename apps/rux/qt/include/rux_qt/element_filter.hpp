// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Qt-free state of the 3D workspace's "Bygningsdele" group: which
// building-element categories (walls, slabs, roofs, spaces, doors, windows,
// other openings) and which storey are shown, with the counts its rows show.
//
// The category keys are visualize::kElementCategories' (the tags the scene
// builder puts on each actor of the solids, walls, openings, spaces and
// components layers, reusex/visualize/scene.hpp); the counts come from
// visualize::count_building_elements(). Standard library only, so the light
// test binary covers it (tests/unit/rux_qt/test_element_filter.cpp).

#include <cstddef>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

/// How many elements of one category one storey holds (-1 = no storey).
struct ElementTally {
  std::string category;
  int storey = -1;
  std::size_t count = 0;
};

/// One row of the group: a category the project has.
struct ElementCategoryRow {
  std::string key;
  std::string label;     ///< Danish, UTF-8
  std::size_t count = 0; ///< under the storey filter
};

/// A tri-state check box's state.
enum class CheckState { off, partial, on };

/// The Danish row label of category @p key ("wall" -> "Vægge"); an unknown
/// key is returned as it is.
std::string element_category_label(std::string_view key);

class ElementFilter {
    public:
  /// The project's elements. Keeps which categories are hidden; drops a
  /// storey filter whose storey is no longer there.
  void set_tallies(std::vector<ElementTally> tallies);
  /// Everything shown, no storey filter (a project was opened).
  void reset();

  /// The categories present (count > 0), in category order, counted under
  /// the storey filter (elements on no storey always count).
  std::vector<ElementCategoryRow> rows() const;
  /// The storeys (>= 0) that hold an element, ascending.
  std::vector<int> storeys() const;
  bool has_elements() const;

  bool shown(std::string_view category) const;
  void set_shown(std::string_view category, bool on);
  void set_all(bool on);
  /// Show exactly @p categories, hide every other.
  void show_only(const std::vector<std::string> &categories);
  /// Over the categories present: on (all shown), off, or partial.
  CheckState all_state() const;

  std::optional<int> storey() const { return storey_; }
  void set_storey(std::optional<int> storey) { storey_ = storey; }

  /// Whether an actor tagged with @p category (empty = untagged, never
  /// filtered) on @p storey (-1 = none, never filtered by storey) is drawn.
  bool actor_visible(std::string_view category, int storey) const;

    private:
  std::vector<ElementTally> tallies_;
  std::set<std::string, std::less<>> hidden_;
  std::optional<int> storey_;
};

/// Whether the `openings` layer's door and window frames are drawn: when
/// their own layer box is on, or when the solids are drawn but their walls
/// are hidden -- the solids show openings as reveals in the walls, which go
/// with the walls, so the frames stand in for them ("only doors and
/// windows").
bool opening_frames_shown(bool openings_box, bool solids_shown,
                          const ElementFilter &filter);

} // namespace rux::qt
