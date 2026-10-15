// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file class_role_map.hpp
/// @brief Semantic class name -> structural role of a plane (solid-model WP2).
///
/// Plane classification (segmentation/classify_planes.hpp) turns the majority
/// semantic class of a plane's inliers into a role through this map. The
/// default lives in `core/class_role_map.json` next to label_semantics.hpp
/// (embedded at build time); a user copies and edits it and passes it as
/// `rux create classify --role-map file.json`.
///
/// File format:
/// @code{.json}
/// { "version": 1, "classes": { "wall": "wall", "table": "furniture" } }
/// @endcode
/// Class names match case-insensitively with surrounding blanks ignored. A
/// class that is not listed has no role: the geometric rules decide.

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace reusex::core {

/// What a plane is in the building (plane_classes.role).
enum class PlaneRole {
  wall,
  floor,
  ceiling,
  slab,
  furniture,
  clutter,
  unknown
};

/// "wall", "floor", ... — the plane_classes.role text.
std::string_view to_string(PlaneRole role) noexcept;

/// Inverse of to_string(); std::nullopt for any other text.
std::optional<PlaneRole> plane_role_from_string(std::string_view text);

/// Class name (normalised: lower case, blanks trimmed) -> role.
class ClassRoleMap {
    public:
  ClassRoleMap() = default;

  /// Map @p class_name (any case) to @p role.
  void set(std::string_view class_name, PlaneRole role);

  /// The role of @p class_name, std::nullopt when it is not listed.
  [[nodiscard]] std::optional<PlaneRole>
  role_of(std::string_view class_name) const;

  [[nodiscard]] size_t size() const noexcept { return roles_.size(); }
  [[nodiscard]] const std::map<std::string, PlaneRole> &entries() const {
    return roles_;
  }

    private:
  std::map<std::string, PlaneRole> roles_;
};

/// Parse the role-map JSON text (format above).
/// @throws std::invalid_argument on malformed JSON, a missing or non-object
///         "classes", a non-string role, an unknown role, an empty class
///         name, two entries that normalise to the same name, or a
///         "version" other than 1.
ClassRoleMap parse_class_role_map(std::string_view json_text);

/// Read and parse a role-map file.
/// @throws std::runtime_error if @p path cannot be read;
///         std::invalid_argument as parse_class_role_map().
ClassRoleMap load_class_role_map(const std::filesystem::path &path);

/// The built-in map (core/class_role_map.json).
const ClassRoleMap &default_class_role_map();

/// The built-in map's JSON text, verbatim (to write a starting file).
std::string_view default_class_role_map_json() noexcept;

} // namespace reusex::core
