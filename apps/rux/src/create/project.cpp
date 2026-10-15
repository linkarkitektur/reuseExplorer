// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/project.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <fmt/format.h>
#include <fmt/std.h>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/annotate_class_map.hpp>
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

  sub->add_option("--max-depth", opt->labels.max_depth_m,
                  "Points farther than this from a frame's camera (metres) "
                  "take no label from that frame")
      ->check(CLI::PositiveNumber)
      ->default_val(opt->labels.max_depth_m);

  sub->add_option("--depth-tolerance", opt->labels.depth_tolerance_m,
                  "Z-buffer agreement (metres) for a point to count as "
                  "visible in a frame")
      ->check(CLI::NonNegativeNumber)
      ->default_val(opt->labels.depth_tolerance_m);

  sub->footer(R"(
DESCRIPTION:
  Projects 2D semantic segmentation labels from sensor frame images onto
  the 3D point cloud using camera poses and intrinsics. Every frame that
  sees a point (within --max-depth, and within --depth-tolerance of the
  frame's z-buffer) casts one vote for the class at its pixel; the point
  takes the majority class. Background never votes, so it never overwrites
  a class, and an exact tie goes to the lowest class id.

EXAMPLES:
  rux create project                   # Project all segmentation labels
  rux create project --max-depth 4     # Ignore points beyond 4 m per frame
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
  - Handles occlusions and multi-view label fusion (majority vote)
  - Output saved as 'labels' label cloud in project database, with the
    class names of the latest 'rux create annotate' run as its label
    definitions
  - Run after 'rux create annotate' and 'rux create clouds'
)");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_project");
    if (rux::capture_stage_params(
            "project", reusex::vision::project_labels_parameters(opt->labels)))
      return;
    rux::finish(run_subcommand_project(*opt, *global_opt));
  });
};

int run_subcommand_project(SubcommandProjectOptions const &opt,
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

    const std::string params =
        reusex::vision::project_labels_parameters(opt.labels);
    int logId = db.log_pipeline_start("project_labels", params);

    spdlog::trace("Loading point cloud from ProjectDB");
    auto cloud = db.point_cloud_xyzrgb("cloud");

    spdlog::trace("Projecting labels from sensor frames");
    CloudLPtr labels = reusex::vision::project(db, cloud, opt.labels);

    spdlog::trace("Saving projected labels to ProjectDB");
    // Point label = API class + 1 (STANDARDS §3.1); the parameters record it
    // (label_encoding) so the schema v29 migration never re-encodes this cloud.
    db.save_point_cloud("labels", *labels, "project_labels", params);

    // Class names of the latest annotate run, keyed by point label.
    if (auto definitions = reusex::core::annotated_label_definitions(db);
        !definitions.empty())
      db.save_label_definitions("labels", definitions);
    else
      spdlog::warn("No annotate class map in the pipeline log: the 'labels' "
                   "cloud's label definitions were not updated");

    spdlog::info("Label projection complete");

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Label projection failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
