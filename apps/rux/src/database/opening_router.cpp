// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/opening_router.hpp"

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

} // namespace

nlohmann::json opening_to_json(const reusex::core::OpeningRecord &o) {
  nlohmann::json j;
  j["id"] = o.id;
  j["guid"] = o.guid;
  j["wall"] = o.wall;
  j["kind"] = o.kind;
  j["s_start"] = o.s_start;
  j["s_end"] = o.s_end;
  j["width"] = o.s_end - o.s_start;
  j["z_sill"] = o.z_sill;
  j["z_head"] = o.z_head;
  j["height"] = o.z_head - o.z_sill;
  j["state"] = o.state;
  j["is_external"] = o.is_external;
  j["source_instance_guid"] = o.source_instance_guid.empty()
                                  ? nlohmann::json(nullptr)
                                  : nlohmann::json(o.source_instance_guid);
  j["source"] = o.source;
  j["locked"] = o.locked;
  j["confidence"] = o.confidence;
  j["metadata"] = metadata_json(o.metadata);
  return j;
}

nlohmann::json openings_json(const reusex::ProjectDB &db,
                             const std::string &wall) {
  nlohmann::json arr = nlohmann::json::array();
  for (const auto &o : db.openings())
    if (wall.empty() || o.wall == wall)
      arr.push_back(opening_to_json(o));
  return arr;
}

std::vector<std::string> OpeningRouter::list() const {
  std::vector<std::string> ids;
  for (const auto &o : db_->openings())
    ids.push_back(std::to_string(o.id));
  return ids;
}

DataPayload OpeningRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty())
    return openings_json(*db_);

  const auto rows = db_->openings();
  const reusex::core::OpeningRecord *match = nullptr;
  if (components[0].is_index()) {
    const int i = *components[0].index;
    if (i < 0 || i >= static_cast<int>(rows.size()))
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(i));
    match = &rows[static_cast<size_t>(i)];
  } else if (components[0].is_item()) {
    const auto &key = components[0].value;
    const auto id = parse_int(key);
    const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &o) {
      return id ? o.id == *id : o.guid == key;
    });
    if (it == rows.end())
      throw std::runtime_error("Opening not found: " + key);
    match = &*it;
  } else {
    throw std::runtime_error("Expected an opening id, guid or index after "
                             "collection");
  }

  const auto j = opening_to_json(*match);
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

void OpeningRouter::set(const std::vector<PathComponent> & /*components*/,
                        const DataPayload & /*data*/) {
  throw std::runtime_error("Cannot set openings via 'rux set'. They are "
                           "written by 'rux create openings'.");
}

std::optional<std::string> OpeningRouter::del_requires_force(
    const std::vector<PathComponent> & /*components*/) const {
  const auto rows = db_->openings();
  const auto locked = std::count_if(rows.begin(), rows.end(),
                                    [](const auto &o) { return o.locked; });
  if (locked == 0)
    return std::nullopt;
  return std::to_string(locked) + " of " + std::to_string(rows.size()) +
         " opening(s) are locked (edited by hand); deleting them needs --force";
}

void OpeningRouter::del(const std::vector<PathComponent> &components) {
  if (!components.empty())
    throw std::runtime_error("Openings are deleted all at once: 'rux del "
                             "openings' (locked rows included)");
  db_->delete_openings();
}

} // namespace rux::database
