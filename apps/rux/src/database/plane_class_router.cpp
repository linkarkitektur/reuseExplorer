// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/plane_class_router.hpp"

#include <algorithm>
#include <charconv>
#include <stdexcept>
#include <string>

namespace rux::database {

namespace {

constexpr std::string_view kCloud = "planes";

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

nlohmann::json plane_class_to_json(const reusex::core::PlaneClassRecord &p) {
  nlohmann::json j;
  j["plane_label"] = p.plane_label;
  j["guid"] = p.guid;
  j["orientation"] = p.orientation;
  j["semantic_class"] =
      p.semantic_class >= 0 ? nlohmann::json(p.semantic_class) : nullptr;
  j["class_confidence"] =
      p.class_confidence >= 0.0 ? nlohmann::json(p.class_confidence) : nullptr;
  j["role"] = p.role;
  j["role_source"] = p.role_source;
  j["storey"] = p.storey;
  j["point_count"] = p.point_count;
  j["area"] = p.area;
  j["z_min"] = p.z_min;
  j["z_max"] = p.z_max;
  j["source"] = p.source;
  j["locked"] = p.locked;
  j["metadata"] = metadata_json(p.metadata);
  return j;
}

nlohmann::json plane_classes_json(const reusex::ProjectDB &db,
                                  std::string_view cloud) {
  nlohmann::json arr = nlohmann::json::array();
  for (const auto &p : db.plane_classes(cloud))
    arr.push_back(plane_class_to_json(p));
  return arr;
}

std::vector<std::string> PlaneClassRouter::list() const {
  std::vector<std::string> ids;
  for (const auto &p : db_->plane_classes(kCloud))
    ids.push_back(std::to_string(p.plane_label));
  return ids; // label ascending
}

DataPayload
PlaneClassRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty())
    return plane_classes_json(*db_, kCloud);

  const auto rows = db_->plane_classes(kCloud);
  const reusex::core::PlaneClassRecord *match = nullptr;
  if (components[0].is_index()) {
    const int i = *components[0].index;
    if (i < 0 || i >= static_cast<int>(rows.size()))
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(i));
    match = &rows[i];
  } else if (components[0].is_item()) {
    const auto &key = components[0].value;
    const auto label = parse_int(key);
    const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto &p) {
      return label ? p.plane_label == *label : p.guid == key;
    });
    if (it == rows.end())
      throw std::runtime_error("Plane class not found: " + key);
    match = &*it;
  } else {
    throw std::runtime_error("Expected a plane label, guid or index after "
                             "collection");
  }

  const auto j = plane_class_to_json(*match);
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

void PlaneClassRouter::set(const std::vector<PathComponent> & /*components*/,
                           const DataPayload & /*data*/) {
  throw std::runtime_error("Cannot set plane classes via 'rux set'. They are "
                           "written by 'rux create classify'.");
}

std::optional<std::string> PlaneClassRouter::del_requires_force(
    const std::vector<PathComponent> & /*components*/) const {
  const auto rows = db_->plane_classes(kCloud);
  const auto locked = std::count_if(rows.begin(), rows.end(),
                                    [](const auto &p) { return p.locked; });
  if (locked == 0)
    return std::nullopt;
  return std::to_string(locked) + " of " + std::to_string(rows.size()) +
         " plane class(es) are locked (edited by hand); deleting them needs "
         "--force";
}

void PlaneClassRouter::del(const std::vector<PathComponent> &components) {
  if (!components.empty())
    throw std::runtime_error("Plane classes are deleted all at once: "
                             "'rux del plane_classes' (locked rows included)");
  db_->delete_plane_classes(kCloud);
}

} // namespace rux::database
