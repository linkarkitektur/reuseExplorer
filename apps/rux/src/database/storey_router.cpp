// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/storey_router.hpp"

#include <algorithm>
#include <charconv>
#include <stdexcept>
#include <string>

namespace rux::database {

namespace {

std::optional<int> parse_int(std::string_view s) {
  int value = 0;
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

nlohmann::json storey_to_json(const reusex::core::StoreyRecord &s) {
  nlohmann::json j;
  j["guid"] = s.guid;
  j["idx"] = s.idx;
  j["name"] = s.name;
  j["z_floor"] = s.z_floor;
  j["z_ceiling"] = s.z_ceiling;
  j["height"] = s.z_ceiling - s.z_floor;
  j["level_transform"] = s.level_transform;
  j["source"] = s.source;
  j["locked"] = s.locked;
  j["confidence"] = s.confidence;
  j["metadata"] = metadata_json(s.metadata);
  return j;
}

std::vector<std::string> StoreyRouter::list() const {
  std::vector<std::string> ids;
  for (const auto &s : db_->storeys())
    ids.push_back(std::to_string(s.idx));
  return ids; // idx ascending
}

DataPayload StoreyRouter::get(const std::vector<PathComponent> &components) {
  const auto rows = db_->storeys();
  if (components.empty()) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto &s : rows)
      arr.push_back(storey_to_json(s));
    return arr;
  }

  const reusex::core::StoreyRecord *match = nullptr;
  if (components[0].is_index()) {
    const int i = *components[0].index;
    if (i < 0 || i >= static_cast<int>(rows.size()))
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(i));
    match = &rows[i];
  } else if (components[0].is_item()) {
    const auto &key = components[0].value;
    const auto idx = parse_int(key);
    const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &s) {
      return idx ? s.idx == *idx : s.guid == key;
    });
    if (it == rows.end())
      throw std::runtime_error("Storey not found: " + key);
    match = &*it;
  } else {
    throw std::runtime_error("Expected a storey idx, guid or index after "
                             "collection");
  }

  const auto j = storey_to_json(*match);
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

void StoreyRouter::set(const std::vector<PathComponent> & /*components*/,
                       const DataPayload & /*data*/) {
  throw std::runtime_error("Cannot set storeys via 'rux set'. They are "
                           "written by 'rux create storeys'.");
}

std::optional<std::string> StoreyRouter::del_requires_force(
    const std::vector<PathComponent> & /*components*/) const {
  const auto rows = db_->storeys();
  const auto locked = std::count_if(rows.begin(), rows.end(),
                                    [](const auto &s) { return s.locked; });
  if (locked == 0)
    return std::nullopt;
  return std::to_string(locked) + " of " + std::to_string(rows.size()) +
         " storey(s) are locked (edited by hand); deleting them needs --force";
}

void StoreyRouter::del(const std::vector<PathComponent> &components) {
  if (!components.empty())
    throw std::runtime_error("Storeys are deleted all at once: "
                             "'rux del storeys' (locked rows included)");
  db_->delete_storeys();
}

} // namespace rux::database
