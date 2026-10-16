// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/wall_router.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
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

nlohmann::json join_to_json(const reusex::core::WallJoinRecord &j) {
  return {{"wall_a", j.wall_a},
          {"wall_b", j.wall_b},
          {"kind", j.kind},
          {"at_a", j.at_a},
          {"at_b", j.at_b}};
}

} // namespace

nlohmann::json
wall_to_json(const reusex::core::WallRecord &w,
             const std::vector<reusex::core::WallJoinRecord> &joins) {
  nlohmann::json j;
  j["id"] = w.id;
  j["guid"] = w.guid;
  j["storey"] = w.storey;
  j["x0"] = w.x0;
  j["y0"] = w.y0;
  j["x1"] = w.x1;
  j["y1"] = w.y1;
  j["length"] = std::hypot(w.x1 - w.x0, w.y1 - w.y0);
  j["z_base"] = w.z_base;
  j["height"] = w.height;
  j["thickness"] = w.thickness;
  j["thickness_ci_lo"] = w.thickness_ci_lo;
  j["thickness_ci_hi"] = w.thickness_ci_hi;
  j["lean_deg"] = w.lean_deg;
  j["is_external"] = w.is_external;
  j["assumed"] = w.assumed;
  j["seen_share"] = w.seen_share;
  j["face_a_points"] = w.face_a_points;
  j["face_b_points"] = w.face_b_points;
  j["co_visibility"] = w.co_visibility;
  j["axis_family"] = w.axis_family;
  j["source"] = w.source;
  j["locked"] = w.locked;
  j["confidence"] = w.confidence;
  j["metadata"] = metadata_json(w.metadata);
  nlohmann::json mine = nlohmann::json::array();
  for (const auto &jn : joins) {
    if (jn.wall_a == w.guid)
      mine.push_back({{"wall", jn.wall_b},
                      {"kind", jn.kind},
                      {"at", jn.at_a},
                      {"at_other", jn.at_b}});
    else if (jn.wall_b == w.guid)
      mine.push_back({{"wall", jn.wall_a},
                      {"kind", jn.kind},
                      {"at", jn.at_b},
                      {"at_other", jn.at_a}});
  }
  j["joins"] = std::move(mine);
  return j;
}

nlohmann::json walls_json(const reusex::ProjectDB &db, int storey) {
  const auto joins = db.wall_joins();
  nlohmann::json arr = nlohmann::json::array();
  for (const auto &w : db.walls())
    if (storey < 0 || w.storey == storey)
      arr.push_back(wall_to_json(w, joins));
  return arr;
}

std::vector<std::string> WallRouter::list() const {
  std::vector<std::string> ids;
  for (const auto &w : db_->walls())
    ids.push_back(std::to_string(w.id));
  return ids;
}

DataPayload WallRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty())
    return walls_json(*db_);

  const auto rows = db_->walls();
  const reusex::core::WallRecord *match = nullptr;
  if (components[0].is_index()) {
    const int i = *components[0].index;
    if (i < 0 || i >= static_cast<int>(rows.size()))
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(i));
    match = &rows[i];
  } else if (components[0].is_item()) {
    const auto &key = components[0].value;
    const auto id = parse_int(key);
    const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &w) {
      return id ? w.id == *id : w.guid == key;
    });
    if (it == rows.end())
      throw std::runtime_error("Wall not found: " + key);
    match = &*it;
  } else {
    throw std::runtime_error("Expected a wall id, guid or index after "
                             "collection");
  }

  const auto j = wall_to_json(*match, db_->wall_joins());
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

void WallRouter::set(const std::vector<PathComponent> & /*components*/,
                     const DataPayload & /*data*/) {
  throw std::runtime_error("Cannot set walls via 'rux set'. They are written "
                           "by 'rux create walls'.");
}

std::optional<std::string> WallRouter::del_requires_force(
    const std::vector<PathComponent> & /*components*/) const {
  const auto rows = db_->walls();
  const auto locked = std::count_if(rows.begin(), rows.end(),
                                    [](const auto &w) { return w.locked; });
  if (locked == 0)
    return std::nullopt;
  return std::to_string(locked) + " of " + std::to_string(rows.size()) +
         " wall(s) are locked (edited by hand); deleting them needs --force";
}

void WallRouter::del(const std::vector<PathComponent> &components) {
  if (!components.empty())
    throw std::runtime_error("Walls are deleted all at once: 'rux del walls' "
                             "(locked rows and every join included)");
  db_->delete_walls();
}

std::vector<std::string> WallJoinRouter::list() const {
  std::vector<std::string> ids;
  const auto joins = db_->wall_joins();
  for (size_t i = 0; i < joins.size(); ++i)
    ids.push_back(std::to_string(i));
  return ids;
}

DataPayload WallJoinRouter::get(const std::vector<PathComponent> &components) {
  const auto joins = db_->wall_joins();
  if (components.empty()) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto &j : joins)
      arr.push_back(join_to_json(j));
    return arr;
  }
  if (!components[0].is_index())
    throw std::runtime_error("Expected an index after collection: "
                             "wall_joins[i]");
  const int i = *components[0].index;
  if (i < 0 || i >= static_cast<int>(joins.size()))
    throw std::runtime_error("Array index out of range: " + std::to_string(i));
  const auto j = join_to_json(joins[static_cast<size_t>(i)]);
  if (components.size() == 1)
    return j;
  const auto &prop = components[1].value;
  if (j.contains(prop))
    return j.at(prop);
  throw std::runtime_error("Unknown property: " + prop +
                           "\nAvailable properties: wall_a, wall_b, kind, "
                           "at_a, at_b");
}

void WallJoinRouter::set(const std::vector<PathComponent> & /*components*/,
                         const DataPayload & /*data*/) {
  throw std::runtime_error("Cannot set wall joins via 'rux set'. They are "
                           "written by 'rux create walls'.");
}

void WallJoinRouter::del(const std::vector<PathComponent> & /*components*/) {
  throw std::runtime_error("Wall joins go with their walls: 'rux del walls'");
}

} // namespace rux::database
