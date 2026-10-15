// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "format_handler.hpp"
#include "path_parser.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::database {

/**
 * @brief Base interface for resource routers
 *
 * Each resource type (clouds, meshes, frames, etc.) implements this interface
 * to handle get/set/del operations for its specific resource type.
 */
class ResourceRouter {
    public:
  explicit ResourceRouter(std::shared_ptr<reusex::ProjectDB> db) : db_(db) {}
  virtual ~ResourceRouter() = default;

  /**
   * @brief Get resource data at the given path
   *
   * @param components Parsed path components (after collection)
   * @return Data payload
   * @throws std::runtime_error if path is invalid or resource not found
   */
  virtual DataPayload get(const std::vector<PathComponent> &components) = 0;

  /**
   * @brief Set resource data at the given path
   *
   * @param components Parsed path components (after collection)
   * @param data Input data
   * @throws std::runtime_error if operation fails
   */
  virtual void set(const std::vector<PathComponent> &components,
                   const DataPayload &data) = 0;

  /**
   * @brief Delete resource at the given path
   *
   * @param components Parsed path components (after collection)
   * @throws std::runtime_error if operation fails
   */
  virtual void del(const std::vector<PathComponent> &components) = 0;

  /**
   * @brief Why deleting @p components needs `rux del --force`, if it does
   *
   * `rux del` refuses such a path without --force, the way it refuses a
   * wildcard. Default: never (nullopt).
   */
  virtual std::optional<std::string>
  del_requires_force(const std::vector<PathComponent> & /*components*/) const {
    return std::nullopt;
  }

  /**
   * @brief List all items in this collection
   *
   * Used for wildcard expansion and array indexing.
   * Returns items in deterministic order (see MEMORY.md).
   *
   * @return Ordered list of item names/ids
   */
  virtual std::vector<std::string> list() const = 0;

  /**
   * @brief Resolve array index to item name/id
   *
   * @param index Array index
   * @return Item name/id if valid, nullopt otherwise
   */
  std::optional<std::string> resolve_index(int index) const {
    auto items = list();
    if (index < 0 || index >= static_cast<int>(items.size())) {
      return std::nullopt;
    }
    return items[index];
  }

    protected:
  std::shared_ptr<reusex::ProjectDB> db_;
};

/**
 * @brief Router registry for dispatching paths to appropriate routers
 */
class RouterRegistry {
    public:
  /**
   * @brief Create routers for all resource types
   */
  explicit RouterRegistry(std::shared_ptr<reusex::ProjectDB> db);

  /**
   * @brief Get router for a collection
   *
   * @param collection Collection name
   * @return Router for that collection
   * @throws std::runtime_error if collection not found
   */
  ResourceRouter &get_router(std::string_view collection);

  /// Sorted list of registered collection names — use this instead of a
  /// hardcoded array so the help text and runtime output cannot drift.
  std::vector<std::string> collection_names() const;

    private:
  std::shared_ptr<reusex::ProjectDB> db_;
  std::map<std::string, std::unique_ptr<ResourceRouter>> routers_;
};

/**
 * @brief Helper to expand wildcards in path components
 *
 * @param components Path components (may contain wildcards)
 * @param router Router for the collection
 * @return Expanded paths (one per matching item)
 */
std::vector<std::vector<PathComponent>>
expand_wildcards(const std::vector<PathComponent> &components,
                 const ResourceRouter &router);

} // namespace rux::database
