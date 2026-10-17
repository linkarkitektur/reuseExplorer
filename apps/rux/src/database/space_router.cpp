// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/space_router.hpp"

#include <algorithm>
#include <charconv>
#include <stdexcept>
#include <string>

namespace rux::database {

namespace {

std::optional<int64_t> parse_int(std::string_view s) {
  int64_t value = 0;
  auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec != std::errc{} || ptr != s.data() + s.size())
    return std::nullopt;
  return value;
}

nlohmann::json metadata_json(const std::string &text) {
  if (text.empty())
    return nullptr;
  try {
    return nlohmann::json::parse(text);
  } catch (const nlohmann::json::parse_error &) {
    return text;
  }
}

nlohmann::json ring_json(const std::vector<reusex::core::PlanPoint> &ring) {
  nlohmann::json arr = nlohmann::json::array();
  for (const auto &p : ring)
    arr.push_back({p[0], p[1]});
  return arr;
}

} // namespace

nlohmann::json
space_to_json(const reusex::core::SpaceRecord &s,
              const std::vector<reusex::core::SpaceBoundaryRecord> &bounds) {
  nlohmann::json j;
  j["id"] = s.id;
  j["guid"] = s.guid;
  j["storey"] = s.storey;
  j["name"] = s.name;
  j["kind"] = s.kind;
  nlohmann::json holes = nlohmann::json::array();
  for (const auto &h : s.polygon.holes)
    holes.push_back(ring_json(h));
  j["polygon"] = {{"outer", ring_json(s.polygon.outer)}, {"holes", holes}};
  j["z_floor"] = s.z_floor;
  j["z_ceiling"] = s.z_ceiling;
  j["area"] = s.area;
  j["perimeter"] = s.perimeter;
  j["volume"] = s.volume;
  j["compactness"] = s.compactness;
  j["room_label"] =
      s.room_label >= 0 ? nlohmann::json(s.room_label) : nlohmann::json();
  j["source"] = s.source;
  j["locked"] = s.locked;
  j["confidence"] = s.confidence;
  j["metadata"] = metadata_json(s.metadata);
  nlohmann::json b = nlohmann::json::array();
  for (const auto &r : bounds)
    if (r.space == s.guid)
      b.push_back({{"wall", r.wall},
                   {"side", r.side},
                   {"s_start", r.s_start},
                   {"s_end", r.s_end}});
  j["boundaries"] = std::move(b);
  return j;
}

nlohmann::json spaces_json(const reusex::ProjectDB &db,
                           std::optional<int> storey) {
  const auto bounds = db.space_boundaries();
  nlohmann::json arr = nlohmann::json::array();
  for (const auto &s : db.spaces())
    if (!storey || s.storey == *storey)
      arr.push_back(space_to_json(s, bounds));
  return arr;
}

std::vector<std::string> SpaceRouter::list() const {
  std::vector<std::string> ids;
  for (const auto &s : db_->spaces())
    ids.push_back(std::to_string(s.id));
  return ids;
}

DataPayload SpaceRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty())
    return spaces_json(*db_);

  const auto rows = db_->spaces();
  const reusex::core::SpaceRecord *match = nullptr;
  if (components[0].is_index()) {
    const int i = *components[0].index;
    if (i < 0 || i >= static_cast<int>(rows.size()))
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(i));
    match = &rows[static_cast<size_t>(i)];
  } else if (components[0].is_item()) {
    const auto &key = components[0].value;
    const auto id = parse_int(key);
    const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &s) {
      return id ? s.id == *id : s.guid == key;
    });
    if (it == rows.end())
      throw std::runtime_error("Space not found: " + key);
    match = &*it;
  } else {
    throw std::runtime_error("Expected a space id, guid or index after "
                             "collection");
  }

  const auto j = space_to_json(*match, db_->space_boundaries());
  if (components.size() == 1)
    return j;
  const auto &prop = components[1].value;
  if (j.contains(prop))
    return j.at(prop);
  std::string available;
  for (const auto &[k, _] : j.items())
    available += (available.empty() ? "" : ", ") + k;
  throw std::runtime_error("Unknown property: " + prop +
                           "\nAvailable properties: " + available);
}

void SpaceRouter::set(const std::vector<PathComponent> & /*components*/,
                      const DataPayload & /*data*/) {
  throw std::runtime_error("Cannot set spaces via 'rux set'. They are "
                           "written by 'rux create spaces'.");
}

std::optional<std::string> SpaceRouter::del_requires_force(
    const std::vector<PathComponent> & /*components*/) const {
  const auto rows = db_->spaces();
  const auto locked = std::count_if(rows.begin(), rows.end(),
                                    [](const auto &s) { return s.locked; });
  if (locked == 0)
    return std::nullopt;
  return std::to_string(locked) + " of " + std::to_string(rows.size()) +
         " space(s) are locked (edited by hand); deleting them needs --force";
}

void SpaceRouter::del(const std::vector<PathComponent> &components) {
  if (!components.empty())
    throw std::runtime_error("Spaces are deleted all at once: 'rux del "
                             "spaces' (locked rows and boundaries included)");
  db_->delete_spaces();
}

} // namespace rux::database
