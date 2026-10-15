// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "del.hpp"

#include "database/path_parser.hpp"
#include "database/resource_router.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <fmt/format.h>
#include <iostream>
#include <spdlog/spdlog.h>

namespace {
constexpr int kMaxWildcardExpansion = 1000; // Safety limit

/**
 * @brief Prompt user for confirmation
 * @return true if user confirms (y/Y)
 */
bool prompt_confirmation() {
  std::cout << "Continue? [y/N] " << std::flush;
  std::string response;
  std::getline(std::cin, response);

  return !response.empty() && (response[0] == 'y' || response[0] == 'Y');
}

} // namespace

void setup_subcommand_del(CLI::App &app,
                          std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<DatabaseDelOptions>();

  auto *sub = app.add_subcommand("del", "Delete data from project database");

  sub->footer(R"(
DESCRIPTION:
  Deletes point clouds, meshes, or other data from ReUseX project database
  using hierarchical path notation. Supports wildcards for batch deletion
  with safety confirmation prompts. Destructive operation - use with care.

EXAMPLES:
  rux del clouds.test                  # Delete specific cloud
  rux del meshes.old_mesh              # Delete specific mesh
  rux del clouds.test_* --force        # Delete with wildcard (preview + confirm)
  rux del clouds.scan* --force --yes   # Scripting: skip confirmation

WILDCARD PATTERNS:
  clouds.test_*        Match all clouds starting with "test_"
  meshes.*_backup      Match all meshes ending with "_backup"
  clouds.scan[0-9]     Match clouds like "scan1", "scan2"

SAFETY:
  --force required     Wildcards require explicit --force flag
  Confirmation prompt  Shows preview of items to be deleted
  --yes to skip        Use for scripting (with --force)
  Max 1000 items       Safety limit on wildcard expansion

NOTES:
  - Deletion is immediate and cannot be undone
  - Use 'rux get' to list items before deleting
  - Wildcard deletion shows preview before confirmation
  - Non-existent paths return error (use get to verify first)
)");

  sub->add_option("path", opt->path,
                  "Resource path (e.g., clouds.oldcloud, clouds.test_*)")
      ->required();

  sub->add_flag("-f,--force", opt->force,
                "Force deletion (required for wildcards, shows preview + "
                "confirmation)")
      ->default_val(false);

  sub->add_flag("-y,--yes", opt->yes,
                "Skip confirmation prompt (use with --force for scripts)")
      ->default_val(false);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_del");
    rux::finish(run_subcommand_del(*opt, *global_opt));
  });
}

int run_subcommand_del(const DatabaseDelOptions &opt,
                       const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project: {}", project_path.string());
    auto db = std::make_shared<reusex::ProjectDB>(project_path,
                                                  /* readOnly */ false);

    // Parse path
    spdlog::debug("Parsing path: {}", opt.path);
    auto components = rux::database::parse_path(opt.path);

    if (components.empty()) {
      spdlog::error("Invalid empty path");
      return 1;
    }

    // Get collection name
    const auto &collection = components[0].value;
    spdlog::debug("Collection: {}", collection);

    // Create router registry
    rux::database::RouterRegistry registry(db);
    auto &router = registry.get_router(collection);

    // Remove collection from components
    std::vector<rux::database::PathComponent> relative_components(
        components.begin() + 1, components.end());

    // Check for wildcards
    bool has_wildcard = false;
    for (const auto &comp : relative_components) {
      if (comp.has_wildcard()) {
        has_wildcard = true;
        break;
      }
    }

    if (has_wildcard && !opt.force) {
      spdlog::error(
          "Wildcard deletion requires --force flag to prevent accidents");
      fmt::print("Hint: Use 'rux del {} --force' to proceed\n", opt.path);
      return 1;
    }

    // Expand wildcards
    auto expanded_paths =
        rux::database::expand_wildcards(relative_components, router);

    if (expanded_paths.empty()) {
      spdlog::warn("Path matched no resources to delete");
      return 0;
    }

    // Safety check: limit wildcard expansion
    if (expanded_paths.size() > kMaxWildcardExpansion) {
      spdlog::error("Wildcard expansion exceeds safety limit ({} > {})",
                    expanded_paths.size(), kMaxWildcardExpansion);
      return 1;
    }

    // Show preview for multiple items
    if (expanded_paths.size() > 1) {
      fmt::print("This will delete {} item(s) from {}:\n",
                 expanded_paths.size(), project_path.string());

      for (const auto &path : expanded_paths) {
        if (!path.empty() && path[0].is_item()) {
          fmt::print("  - {}\n", path[0].value);
        }
      }

      // Prompt for confirmation unless --yes
      if (!opt.yes) {
        if (!prompt_confirmation()) {
          spdlog::info("Deletion cancelled by user");
          return 0;
        }
      }
    }

    // Paths a router protects (e.g. storeys with locked rows) need --force,
    // like wildcards.
    if (!opt.force)
      for (const auto &path : expanded_paths)
        if (auto reason = router.del_requires_force(path)) {
          spdlog::error("{}", *reason);
          fmt::print("Hint: Use 'rux del {} --force' to proceed\n", opt.path);
          return 1;
        }

    // Perform deletions
    int deleted_count = 0;
    int failed_count = 0;

    for (const auto &path : expanded_paths) {
      try {
        router.del(path);
        deleted_count++;

        if (!path.empty() && path[0].is_item()) {
          spdlog::info("Deleted: {}", path[0].value);
        }
      } catch (const std::exception &e) {
        failed_count++;
        if (!path.empty() && path[0].is_item()) {
          spdlog::error("Failed to delete {}: {}", path[0].value, e.what());
        }
      }
    }

    if (deleted_count > 0) {
      fmt::print("Successfully deleted {} item(s)\n", deleted_count);
    }

    if (failed_count > 0) {
      spdlog::error("Failed to delete {} item(s)", failed_count);
      return 1;
    }

    return 0;

  } catch (const rux::database::PathError &e) {
    spdlog::error("Path error: {}", e.what());
    return 1;
  } catch (const std::exception &e) {
    spdlog::error("Failed to delete data: {}", e.what());
    return 1;
  }
}
