// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/element_filter.hpp>

#include <algorithm>
#include <array>
#include <utility>

namespace rux::qt {

namespace {

struct Category {
  const char *key;
  const char *label;
};

/// visualize::kElementCategories, in its order, with the rows' labels.
constexpr std::array<Category, 7> kCategories{{
    {"wall", "Vægge"},
    {"slab", "Dæk"},
    {"roof", "Tag"},
    {"space", "Rum"},
    {"door", "Døre"},
    {"window", "Vinduer"},
    {"opening", "Andre åbninger"},
}};

} // namespace

std::string element_category_label(std::string_view key) {
  for (const auto &c : kCategories)
    if (key == c.key)
      return c.label;
  return std::string(key);
}

void ElementFilter::set_tallies(std::vector<ElementTally> tallies) {
  tallies_ = std::move(tallies);
  if (storey_) {
    const auto s = storeys();
    if (std::find(s.begin(), s.end(), *storey_) == s.end())
      storey_.reset();
  }
}

void ElementFilter::reset() {
  hidden_.clear();
  storey_.reset();
}

std::vector<ElementCategoryRow> ElementFilter::rows() const {
  std::vector<ElementCategoryRow> out;
  const auto add = [&](std::string_view key, std::string label) {
    std::size_t total = 0, under = 0;
    for (const auto &t : tallies_)
      if (t.category == key) {
        total += t.count;
        if (!storey_ || t.storey < 0 || t.storey == *storey_)
          under += t.count;
      }
    if (total > 0)
      out.push_back({std::string(key), std::move(label), under});
  };
  for (const auto &c : kCategories)
    add(c.key, c.label);
  // Categories this list does not know yet, after the known ones.
  std::vector<std::string> extra;
  for (const auto &t : tallies_)
    if (std::none_of(kCategories.begin(), kCategories.end(),
                     [&](const Category &c) { return t.category == c.key; }) &&
        std::find(extra.begin(), extra.end(), t.category) == extra.end())
      extra.push_back(t.category);
  for (const auto &k : extra)
    add(k, k);
  return out;
}

std::vector<int> ElementFilter::storeys() const {
  std::set<int> s;
  for (const auto &t : tallies_)
    if (t.storey >= 0 && t.count > 0)
      s.insert(t.storey);
  return {s.begin(), s.end()};
}

bool ElementFilter::has_elements() const { return !rows().empty(); }

bool ElementFilter::shown(std::string_view category) const {
  return hidden_.find(category) == hidden_.end();
}

void ElementFilter::set_shown(std::string_view category, bool on) {
  if (on) {
    if (const auto it = hidden_.find(category); it != hidden_.end())
      hidden_.erase(it);
  } else {
    hidden_.emplace(category);
  }
}

void ElementFilter::set_all(bool on) {
  hidden_.clear();
  if (!on) {
    for (const auto &c : kCategories)
      hidden_.emplace(c.key);
    for (const auto &t : tallies_)
      hidden_.insert(t.category);
  }
}

void ElementFilter::show_only(const std::vector<std::string> &categories) {
  set_all(false);
  for (const auto &c : categories)
    set_shown(c, true);
}

CheckState ElementFilter::all_state() const {
  std::size_t on = 0, n = 0;
  for (const auto &r : rows()) {
    ++n;
    on += shown(r.key) ? 1 : 0;
  }
  if (n == 0 || on == 0)
    return CheckState::off;
  return on == n ? CheckState::on : CheckState::partial;
}

bool ElementFilter::actor_visible(std::string_view category, int storey) const {
  if (category.empty())
    return true;
  if (!shown(category))
    return false;
  return !storey_ || storey < 0 || storey == *storey_;
}

bool opening_frames_shown(bool openings_box, bool solids_shown,
                          const ElementFilter &filter) {
  return openings_box || (solids_shown && !filter.shown("wall"));
}

} // namespace rux::qt
