// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/class_role_map.hpp"
#include "reusex/core/class_role_map_default.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace reusex::core {
namespace {

constexpr std::array<std::pair<PlaneRole, std::string_view>, 7> kRoleNames{{
    {PlaneRole::wall, "wall"},
    {PlaneRole::floor, "floor"},
    {PlaneRole::ceiling, "ceiling"},
    {PlaneRole::slab, "slab"},
    {PlaneRole::furniture, "furniture"},
    {PlaneRole::clutter, "clutter"},
    {PlaneRole::unknown, "unknown"},
}};

std::string normalise(std::string_view name) {
  const auto blank = [](unsigned char c) { return std::isspace(c) != 0; };
  while (!name.empty() && blank(static_cast<unsigned char>(name.front())))
    name.remove_prefix(1);
  while (!name.empty() && blank(static_cast<unsigned char>(name.back())))
    name.remove_suffix(1);
  std::string out(name);
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

} // namespace

std::string_view to_string(PlaneRole role) noexcept {
  for (const auto &[r, name] : kRoleNames)
    if (r == role)
      return name;
  return "unknown";
}

std::optional<PlaneRole> plane_role_from_string(std::string_view text) {
  for (const auto &[r, name] : kRoleNames)
    if (name == text)
      return r;
  return std::nullopt;
}

void ClassRoleMap::set(std::string_view class_name, PlaneRole role) {
  roles_[normalise(class_name)] = role;
}

std::optional<PlaneRole>
ClassRoleMap::role_of(std::string_view class_name) const {
  const auto it = roles_.find(normalise(class_name));
  if (it == roles_.end())
    return std::nullopt;
  return it->second;
}

ClassRoleMap parse_class_role_map(std::string_view json_text) {
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(json_text);
  } catch (const nlohmann::json::parse_error &e) {
    throw std::invalid_argument(
        fmt::format("class role map: not valid JSON ({})", e.what()));
  }
  if (!j.is_object())
    throw std::invalid_argument("class role map: top level must be an object");
  if (j.contains("version") &&
      (!j["version"].is_number_integer() || j["version"].get<int>() != 1))
    throw std::invalid_argument(
        fmt::format("class role map: unsupported version {} (expected 1)",
                    j["version"].dump()));
  if (!j.contains("classes") || !j["classes"].is_object())
    throw std::invalid_argument(
        "class role map: missing \"classes\" object (class name -> role)");

  ClassRoleMap map;
  for (const auto &[name, value] : j["classes"].items()) {
    const std::string key = normalise(name);
    if (key.empty())
      throw std::invalid_argument("class role map: empty class name");
    if (!value.is_string())
      throw std::invalid_argument(fmt::format(
          "class role map: role of class \"{}\" must be a string", name));
    const auto role = plane_role_from_string(value.get<std::string>());
    if (!role)
      throw std::invalid_argument(fmt::format(
          "class role map: class \"{}\" has unknown role \"{}\" (expected "
          "wall, floor, ceiling, slab, furniture, clutter or unknown)",
          name, value.get<std::string>()));
    if (map.role_of(key))
      throw std::invalid_argument(fmt::format(
          "class role map: class \"{}\" is listed twice (names match "
          "case-insensitively)",
          name));
    map.set(key, *role);
  }
  return map;
}

ClassRoleMap load_class_role_map(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error(
        fmt::format("class role map: cannot read {}", path.string()));
  std::ostringstream ss;
  ss << in.rdbuf();
  try {
    return parse_class_role_map(ss.str());
  } catch (const std::invalid_argument &e) {
    throw std::invalid_argument(fmt::format("{}: {}", path.string(), e.what()));
  }
}

std::string_view default_class_role_map_json() noexcept {
  return detail::kDefaultClassRoleMapJson;
}

const ClassRoleMap &default_class_role_map() {
  static const ClassRoleMap map =
      parse_class_role_map(default_class_role_map_json());
  return map;
}

} // namespace reusex::core
