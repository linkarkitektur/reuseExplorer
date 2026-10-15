// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/project.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <fmt/format.h>
#include <fmt/std.h>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/label_semantics.hpp>
#include <reusex/vision/project.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <range/v3/to_container.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>

namespace fs = std::filesystem;

void setup_subcommand_create_project(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandProjectOptions>();
  auto *sub =
      app.add_subcommand("project", "Project 2D labels onto 3D point cloud");

  sub->footer(R"(
DESCRIPTION:
  Projects 2D semantic segmentation labels from sensor frame images onto
  the 3D point cloud using camera poses and intrinsics. Maps per-pixel
  labels to corresponding 3D points, enabling semantic analysis of the
  reconstructed geometry. Essential bridge between 2D ML inference and 3D.

EXAMPLES:
  rux create project                   # Project all segmentation labels
  rux -p scan.rux create project       # Custom project path
  rux -vv create project               # Debug with verbose output

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor frames
  2. rux create annotate --net model   # Run ML inference on frames
  3. rux create clouds                 # Reconstruct point clouds
  4. rux create project                # Project labels to 3D
  5. rux view                          # Visualize labeled cloud

NOTES:
  - Requires point cloud, sensor frames, and segmentation images
  - Uses camera intrinsics and poses for accurate 3D-2D mapping
  - Handles occlusions and multi-view label fusion
  - Output saved as 'labels' label cloud in project database
  - Run after 'rux create annotate' and 'rux create clouds'
)");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_project");
    rux::finish(run_subcommand_project(*opt, *global_opt));
  });
};

int run_subcommand_project([[maybe_unused]] SubcommandProjectOptions const &opt,
                           const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Projecting labels in project: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Pre-flight validation: check for cloud and segmentation images
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::project);
        rc != RuxError::SUCCESS)
      return rc;

    int logId = db.log_pipeline_start("project_labels");

    spdlog::trace("Loading point cloud from ProjectDB");
    auto cloud = db.point_cloud_xyzrgb("cloud");

    spdlog::trace("Projecting labels from sensor frames");
    CloudLPtr labels = reusex::vision::project(db, cloud);

    spdlog::trace("Saving projected labels to ProjectDB");
    // Point label = API class + 1 (STANDARDS §3.1); recorded so the schema
    // v29 migration never re-encodes this cloud.
    db.save_point_cloud("labels", *labels, "project_labels",
                        fmt::format(R"({{"{}":"{}"}})",
                                    reusex::core::kLabelEncodingKey,
                                    reusex::core::kLabelEncodingApiPlusOne));

    spdlog::info("Label projection complete");

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Label projection failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
