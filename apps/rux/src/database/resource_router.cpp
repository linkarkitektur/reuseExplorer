// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/resource_router.hpp"

#include "database/cloud_router.hpp"
#include "database/component_router.hpp"
#include "database/frame_router.hpp"
#include "database/label_router.hpp"
#include "database/log_router.hpp"
#include "database/mesh_router.hpp"
#include "database/opening_router.hpp"
#include "database/panorama_router.hpp"
#include "database/passport_router.hpp"
#include "database/plane_class_router.hpp"
#include "database/project_router.hpp"
#include "database/space_router.hpp"
#include "database/storey_router.hpp"
#include "database/wall_router.hpp"

#include <spdlog/spdlog.h>

namespace rux::database {

RouterRegistry::RouterRegistry(std::shared_ptr<reusex::ProjectDB> db)
    : db_(db) {
  // Register routers for each collection type.
  // The map is ordered alphabetically; collection_names() and the
  // error message below reflect that order automatically.
  routers_["clouds"] = std::make_unique<CloudRouter>(db);
  routers_["components"] = std::make_unique<ComponentRouter>(db);
  routers_["frames"] = std::make_unique<FrameRouter>(db);
  routers_["labels"] = std::make_unique<LabelRouter>(db);
  routers_["log"] = std::make_unique<LogRouter>(db);
  routers_["materials"] = std::make_unique<PassportRouter>(db);
  routers_["meshes"] = std::make_unique<MeshRouter>(db);
  routers_["panoramas"] = std::make_unique<PanoramaRouter>(db);
  routers_["plane_classes"] = std::make_unique<PlaneClassRouter>(db);
  routers_["projects"] = std::make_unique<ProjectRouter>(db);
  routers_["storeys"] = std::make_unique<StoreyRouter>(db);
  routers_["walls"] = std::make_unique<WallRouter>(db);
  routers_["openings"] = std::make_unique<OpeningRouter>(db);
  routers_["spaces"] = std::make_unique<SpaceRouter>(db);
  routers_["wall_joins"] = std::make_unique<WallJoinRouter>(db);

  spdlog::trace("Initialized router registry with {} routers", routers_.size());
}

std::vector<std::string> RouterRegistry::collection_names() const {
  std::vector<std::string> names;
  names.reserve(routers_.size());
  for (const auto &[name, _] : routers_) {
    names.push_back(name);
  }
  return names; // std::map iterates in sorted key order
}

ResourceRouter &RouterRegistry::get_router(std::string_view collection) {
  auto it = routers_.find(std::string(collection));
  if (it == routers_.end()) {
    // Build the list from the live registry so it cannot drift.
    std::string supported;
    for (const auto &[name, _] : routers_) {
      if (!supported.empty())
        supported += ", ";
      supported += name;
    }
    throw std::runtime_error(
        "No router for collection: " + std::string(collection) +
        "\nSupported collections: " + supported);
  }
  return *it->second;
}

std::vector<std::vector<PathComponent>>
expand_wildcards(const std::vector<PathComponent> &components,
                 const ResourceRouter &router) {
  // Check if there are wildcards to expand
  bool has_wildcard = false;
  for (const auto &comp : components) {
    if (comp.has_wildcard()) {
      has_wildcard = true;
      break;
    }
  }

  if (!has_wildcard) {
    // No wildcards, return as-is
    return {components};
  }

  // Expand wildcards (only in item position)
  if (components.empty() || !components[0].is_item() ||
      !components[0].has_wildcard()) {
    // Wildcard not in expected position
    return {components};
  }

  // Get all items and expand
  auto items = router.list();
  auto matches = expand_wildcard(components[0].value, items);

  if (matches.empty()) {
    spdlog::warn("Wildcard pattern '{}' matched no items", components[0].value);
    return {};
  }

  spdlog::debug("Wildcard pattern '{}' matched {} items", components[0].value,
                matches.size());

  // Create expanded paths
  std::vector<std::vector<PathComponent>> expanded_paths;
  expanded_paths.reserve(matches.size());

  for (const auto &match : matches) {
    std::vector<PathComponent> path = components;
    path[0].value = match; // Replace wildcard with actual name
    expanded_paths.push_back(std::move(path));
  }

  return expanded_paths;
}

} // namespace rux::database
