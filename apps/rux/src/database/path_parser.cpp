// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/path_parser.hpp"

#include <algorithm>
#include <cctype>
#include <regex>
#include <spdlog/spdlog.h>

namespace rux::database {

namespace {
// Valid collection names (only implemented routers)
const std::vector<std::string> kValidCollections = {
    "clouds",    "components", "frames",     "labels",    "log",
    "materials", "meshes",     "openings",   "panoramas", "plane_classes",
    "projects",  "storeys",    "wall_joins", "walls"};

/**
 * @brief Split a string by a delimiter
 */
std::vector<std::string> split(std::string_view str, char delimiter) {
  std::vector<std::string> result;
  size_t start = 0;
  size_t end = str.find(delimiter);

  while (end != std::string_view::npos) {
    if (end > start) { // Skip empty components
      result.emplace_back(str.substr(start, end - start));
    }
    start = end + 1;
    end = str.find(delimiter, start);
  }

  if (start < str.length()) {
    result.emplace_back(str.substr(start));
  }

  return result;
}

/**
 * @brief Parse array index from string like "[0]"
 * @return Index value if valid, nullopt otherwise
 */
std::optional<int> parse_array_index(std::string_view str) {
  // Check format: starts with '[', ends with ']', contains only digits
  if (str.length() < 3 || str.front() != '[' || str.back() != ']') {
    return std::nullopt;
  }

  auto digits = str.substr(1, str.length() - 2);
  if (digits.empty() || !std::all_of(digits.begin(), digits.end(), ::isdigit)) {
    return std::nullopt;
  }

  try {
    return std::stoi(std::string(digits));
  } catch (...) {
    return std::nullopt;
  }
}

} // namespace

bool is_valid_collection(std::string_view name) {
  return std::find(kValidCollections.begin(), kValidCollections.end(), name) !=
         kValidCollections.end();
}

std::vector<PathComponent> parse_path(std::string_view path) {
  if (path.empty()) {
    throw PathError("Path cannot be empty");
  }

  // Normalize slashes to dots for backward compatibility
  std::string normalized_path(path);
  std::replace(normalized_path.begin(), normalized_path.end(), '/', '.');

  // Split by dots
  auto parts = split(normalized_path, '.');

  if (parts.empty()) {
    throw PathError("Path must contain at least one component");
  }

  std::vector<PathComponent> components;
  components.reserve(parts.size());

  for (size_t i = 0; i < parts.size(); ++i) {
    const auto &part = parts[i];

    if (part.empty()) {
      throw PathError("Path contains empty component");
    }

    // Check for array indexing (e.g., "clouds[0]")
    auto bracket_pos = part.find('[');
    if (bracket_pos != std::string::npos) {
      // Extract base name and index
      std::string base_name = part.substr(0, bracket_pos);
      std::string index_part = part.substr(bracket_pos);

      auto index = parse_array_index(index_part);
      if (!index) {
        throw PathError("Invalid array index format: " + part);
      }

      // First component must be collection
      if (i == 0) {
        if (!is_valid_collection(base_name)) {
          throw PathError("Invalid collection name: " + base_name);
        }
        components.emplace_back(ComponentType::Collection, base_name);
        components.emplace_back(ComponentType::Index, "", *index);
      } else {
        throw PathError(
            "Array indexing only allowed on collection (first component)");
      }
    } else {
      // Regular component (no array indexing)
      if (i == 0) {
        // First component is always a collection
        if (!is_valid_collection(part)) {
          throw PathError(
              "Invalid collection name: " + part +
              "\nValid collections: clouds, components, frames, "
              "labels, log, materials, meshes, panoramas, projects");
        }
        components.emplace_back(ComponentType::Collection, part);
      } else if (i == 1) {
        // Second component is an item (name or id)
        components.emplace_back(ComponentType::Item, part);
      } else {
        // Subsequent components are properties
        components.emplace_back(ComponentType::Property, part);
      }
    }
  }

  spdlog::trace("Parsed path '{}' into {} components", path, components.size());
  return components;
}

bool matches_wildcard(std::string_view str, std::string_view pattern) {
  // Convert wildcard pattern to regex
  // Escape all special regex chars except *
  std::string regex_pattern;
  regex_pattern.reserve(pattern.size() * 2);

  for (char ch : pattern) {
    if (ch == '*') {
      regex_pattern += ".*";
    } else if (ch == '.' || ch == '+' || ch == '?' || ch == '[' || ch == ']' ||
               ch == '(' || ch == ')' || ch == '{' || ch == '}' || ch == '^' ||
               ch == '$' || ch == '|' || ch == '\\') {
      regex_pattern += '\\';
      regex_pattern += ch;
    } else {
      regex_pattern += ch;
    }
  }

  std::regex re("^" + regex_pattern + "$");
  return std::regex_match(std::string(str), re);
}

std::vector<std::string>
expand_wildcard(std::string_view pattern,
                const std::vector<std::string> &items) {
  std::vector<std::string> matches;

  for (const auto &item : items) {
    if (matches_wildcard(item, pattern)) {
      matches.push_back(item);
    }
  }

  spdlog::trace("Wildcard pattern '{}' matched {} items", pattern,
                matches.size());
  return matches;
}

} // namespace rux::database
