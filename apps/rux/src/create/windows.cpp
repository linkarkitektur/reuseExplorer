// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/windows.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/component_persistence.hpp>
#include <reusex/reconstruction/create_windows.hpp>

#include <spdlog/spdlog.h>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <regex>
#include <set>

void setup_subcommand_create_windows(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandWindowOptions>();
  auto *sub =
      app.add_subcommand("windows", "Create window building components");

  sub->footer(R"(
DESCRIPTION:
  Generates window building components from instance-segmented point clouds
  and wall geometry from the mesh. For each window instance, projects points
  onto the nearest wall, computes a boundary polygon (rectangle or concave
  hull), and offsets it along the outward wall normal. Detects windows on all
  surface orientations (vertical walls, horizontal skylights, tilted roofs).

EXAMPLES:
  rux create windows                        # Defaults: rect mode, 0.5m offset
  rux create windows --mode poly --alpha 0.3
  rux create windows -l 5,8 --offset 0.3   # Only labels 5,8; 30cm offset
  rux -p scan.rux create windows            # Custom project path

WORKFLOW:
  1. rux create annotate --net model        # Semantic segmentation
  2. rux create clouds                      # Reconstruct labeled cloud
  3. rux create instances -l 5              # Instance segment windows (label 5)
  4. rux create mesh                        # Generate wall mesh
  5. rux create windows -l 5               # Generate window components
  6. rux get components                     # View component data

NOTES:
  - Requires 'instances' label cloud, 'labels' semantic cloud, and 'mesh'
  - Use --mode rect for axis-aligned bounding boxes (default)
  - Use --mode poly for concave hull boundaries (--alpha controls tightness)
  - Wall offset moves the polygon outward from the wall surface
  - --wall-normal-z restricts which surfaces may host windows; it defaults to
    off (all orientations, so skylights are kept), 0.3 keeps near-vertical
    walls only
  - Use --clear to replace all existing windows (default: update/append via UPSERT)
  - Output stored in project database building_components table
)");

  sub->add_option("--mesh", opt->mesh_name,
                  "Name of the wall mesh in ProjectDB")
      ->default_val(opt->mesh_name);

  sub->add_option("--instances", opt->instance_cloud_name,
                  "Name of instance label cloud")
      ->default_val(opt->instance_cloud_name);

  sub->add_option("-s,--semantic", opt->semantic_cloud_name,
                  "Name of semantic label cloud")
      ->default_val(opt->semantic_cloud_name);

  sub->add_option("--mode", opt->mode,
                  "Boundary mode: 'rect' (AABB) or 'poly' (concave hull)")
      ->default_val(opt->mode)
      ->check(CLI::IsMember({"rect", "poly"}));

  sub->add_option("--offset", opt->wall_offset,
                  "Offset along outward wall normal (meters)")
      ->default_val(opt->wall_offset)
      ->check(CLI::Range(0.0, 5.0));

  sub->add_option("--alpha", opt->alpha, "ConcaveHull alpha for polyline mode")
      ->default_val(opt->alpha)
      ->check(CLI::Range(0.01, 10.0));

  sub->add_option("--wall-normal-z", opt->wall_normal_z_threshold,
                  "Verticality gate on wall candidates: a mesh region hosts "
                  "windows only when |normal.z| is below this. The default "
                  "turns the gate off, accepting every orientation (keeps "
                  "skylights and tilted roof glazing); use e.g. 0.3 for "
                  "near-vertical walls only")
      ->default_val(opt->wall_normal_z_threshold)
      ->check(CLI::Range(0.0f, reusex::geometry::kWallVerticalityGateOff));

  sub->add_option("-l,--labels", opt->labels_to_process,
                  "Semantic point labels to treat as windows (comma-separated; "
                  "point label = API class + 1)")
      ->delimiter(',');

  sub->add_flag(
         "--clear", opt->clear_existing,
         "Delete all existing window components before creating new ones")
      ->default_val(false);

  sub->add_flag(
         "--include-internal", opt->include_internal,
         "Include windows fully inside mesh volume (default: only exterior)")
      ->default_val(false);

  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_create_windows(*opt, *global_opt));
  });
}

int run_subcommand_create_windows(SubcommandWindowOptions const &opt,
                                  const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Create windows in project: {}", project_path.string());

    reusex::ProjectDB db(project_path);

    // Pre-flight validation. The mesh / instances / semantic cloud existence
    // checks that used to be open-coded here are all part of the `windows`
    // stage contract now (#246), retargeted to whatever the flags named.
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::windows,
            {{"labels", opt.semantic_cloud_name},
             {"instances", opt.instance_cloud_name},
             {"mesh", opt.mesh_name}});
        rc != RuxError::SUCCESS)
      return rc;

    // Log pipeline start
    int logId = db.log_pipeline_start(
        "create_windows",
        fmt::format(R"({{"mode":"{}","offset":{},"alpha":{},"mesh":"{}"}})",
                    opt.mode, opt.wall_offset, opt.alpha, opt.mesh_name));

    // Load data
    spdlog::info("Loading point cloud, instance labels, and mesh...");
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto instance_labels = db.point_cloud_label(opt.instance_cloud_name);
    auto mesh = db.mesh(opt.mesh_name);

    if (!cloud || !instance_labels || !mesh) {
      spdlog::error("Failed to load required data");
      db.log_pipeline_end(logId, false);
      return RuxError::IO;
    }

    spdlog::info("Loaded {} points, {} instance labels, mesh with {} faces",
                 cloud->size(), instance_labels->size(), mesh->polygons.size());

    // Reconstruct instance_to_semantic map from label definitions
    auto label_defs = db.label_definitions(opt.instance_cloud_name);
    std::map<uint32_t, uint32_t> instance_to_semantic;

    // Parse label definitions: "semantic_class_N_instance_M (P points)"
    std::regex def_regex(R"(SM(\d+)-(\d+))");
    for (const auto &[inst_id, def_str] : label_defs) {
      std::smatch match;
      if (std::regex_search(def_str, match, def_regex)) {
        uint32_t semantic_class =
            static_cast<uint32_t>(std::stoul(match[1].str()));
        instance_to_semantic[static_cast<uint32_t>(inst_id)] = semantic_class;
      }
    }

    spdlog::info("Parsed {} instance-to-semantic mappings",
                 instance_to_semantic.size());

    if (instance_to_semantic.empty()) {
      spdlog::error(
          "No instance-to-semantic mappings found in label definitions");
      spdlog::info("Resolution: Run 'rux create instances' first");
      db.log_pipeline_end(logId, false);
      return RuxError::INVALID_ARGUMENT;
    }

    // Determine which semantic labels to treat as windows
    std::vector<uint32_t> window_labels = opt.labels_to_process;
    if (window_labels.empty()) {
      // Collect all unique semantic labels
      std::set<uint32_t> unique_labels;
      for (const auto &[_, sem] : instance_to_semantic)
        unique_labels.insert(sem);
      window_labels.assign(unique_labels.begin(), unique_labels.end());
      spdlog::info(
          "No --labels specified, processing all semantic labels: [{}]",
          fmt::join(window_labels, ", "));
    } else {
      spdlog::info("Processing semantic labels: [{}]",
                   fmt::join(window_labels, ", "));
    }

    // Clear existing windows if requested
    if (opt.clear_existing) {
      auto existing_windows = reusex::geometry::list_building_components(
          db, reusex::geometry::ComponentType::window);

      if (!existing_windows.empty()) {
        spdlog::info("Clearing {} existing window components",
                     existing_windows.size());
        for (const auto &name : existing_windows) {
          db.delete_building_component(name);
          spdlog::debug("Deleted component '{}'", name);
        }
      } else {
        spdlog::debug("No existing windows to clear");
      }
    }

    // Configure options
    reusex::geometry::CreateWindowsOptions create_opts;
    create_opts.mode = (opt.mode == "poly")
                           ? reusex::geometry::WindowBoundaryMode::polyline
                           : reusex::geometry::WindowBoundaryMode::rectangle;
    create_opts.wall_offset = opt.wall_offset;
    create_opts.alpha = opt.alpha;
    create_opts.wall_normal_z_threshold = opt.wall_normal_z_threshold;
    create_opts.include_internal = opt.include_internal;

    // Provenance resolver: map an instance label id to its stable GUID so each
    // created component records the instance it came from (issue #211).
    // Guarded by schema_version >= 10 (the instances table) so populated
    // projects work and older projects keep an empty source_instance_guid.
    const std::string instance_cloud = opt.instance_cloud_name;
    reusex::geometry::ResolveInstanceGuidFn resolve_instance_guid =
        [&db, instance_cloud](uint32_t inst_id) -> std::string {
      if (db.schema_version() < 10) {
        spdlog::debug("schema v{} < 10: instance provenance unavailable, "
                      "leaving source_instance_guid empty for instance {}",
                      db.schema_version(), inst_id);
        return {};
      }
      try {
        return db.instance_guid(instance_cloud, inst_id);
      } catch (const std::exception &e) {
        spdlog::debug("No instance guid for {}#{}: {}", instance_cloud, inst_id,
                      e.what());
        return {};
      }
    };

    // Create windows
    spdlog::info("Creating window components...");
    auto result = reusex::geometry::create_windows(
        cloud, instance_labels, instance_to_semantic, *mesh, window_labels,
        create_opts, resolve_instance_guid);

    // Save components
    for (const auto &comp : result.components) {
      reusex::geometry::save_building_component(db, comp);
      spdlog::debug("Saved component '{}' ({} vertices)", comp.name,
                    comp.boundary.vertices.size());
    }

    // Summary
    spdlog::info("Summary:");
    if (opt.clear_existing) {
      spdlog::info("  Existing windows cleared");
    }
    spdlog::info("  Window components created: {}", result.components.size());
    spdlog::info("  Unmatched instances: {}",
                 result.unmatched_instances.size());
    if (!result.unmatched_instances.empty()) {
      spdlog::warn("  Unmatched instance IDs: [{}]",
                   fmt::join(result.unmatched_instances, ", "));
    }

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Window creation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
