// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/instances.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "global-params.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>
#include <spdlog/spdlog.h>

#include <fmt/ranges.h>

void setup_subcommand_create_instances(CLI::App &app,
                                       std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandSegInstancesOptions>();

  auto *sub =
      app.add_subcommand("instances", "Separate labels into spatial instances");

  sub->footer(R"(
DESCRIPTION:
  Separates semantic segmentation labels into distinct spatial instances
  using Euclidean distance clustering. For example, multiple 'window'
  labels become 'window_1', 'window_2', etc. based on spatial separation.
  Essential for distinguishing individual objects of the same class.

EXAMPLES:
  rux create instances                 # Process all labels
  rux create instances -t 0.3          # 30cm clustering tolerance
  rux create instances -l 3,5,7        # Only labels 3, 5, 7
  rux create instances -s labels -o inst  # Custom input/output

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create annotate --net model   # Semantic segmentation
  3. rux create clouds                 # Reconstruct labeled cloud
  4. rux create instances              # Instance segmentation
  5. rux view                          # Visualize instances

NOTES:
  - Requires 'cloud' and semantic label cloud (default: 'labels')
  - Tolerance: Euclidean distance threshold for clustering (meters)
  - Min/max size filters noise and oversized clusters
  - Use --labels to process specific semantic classes only. Values are point
    labels (API class + 1: the first annotate prompt class is 1), the keys
    of the semantic cloud's label definitions
  - Output includes per-instance metadata and statistics
  - Saved as new label cloud with instance IDs
)");

  sub->add_option("-t,--tolerance", opt->cluster_tolerance,
                  "Euclidean distance threshold for clustering (meters)")
      ->default_val(opt->cluster_tolerance)
      ->check(CLI::Range(0.01, 5.0));

  sub->add_option("-m,--min-size", opt->min_cluster_size,
                  "Minimum points per instance cluster")
      ->default_val(opt->min_cluster_size)
      ->check(CLI::Range(1, 100000));

  sub->add_option("-M,--max-size", opt->max_cluster_size,
                  "Maximum points per instance cluster")
      ->default_val(opt->max_cluster_size)
      ->check(CLI::Range(10, 10000000));

  sub->add_option("-s,--semantic", opt->semantic_cloud_name,
                  "Input semantic labels cloud name")
      ->default_val("labels");

  sub->add_option("-o,--output", opt->output_cloud_name,
                  "Output instance labels cloud name")
      ->default_val("instances");

  sub->add_option(
         "-l,--labels", opt->labels_to_process,
         "Comma-separated semantic point labels to process (point label = "
         "API class + 1; empty = all)")
      ->delimiter(',');

  sub->callback([opt, global_opt]() {
    if (rux::capture_stage_params("instances",
                                  instances_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_segment_instances(*opt, *global_opt));
  });
}

std::string
instances_stage_parameters(const SubcommandSegInstancesOptions &opt) {
  return rux::StageParams()
      .set("cluster_tolerance", opt.cluster_tolerance)
      .set("min_cluster_size", opt.min_cluster_size)
      .set("max_cluster_size", opt.max_cluster_size)
      .set("semantic_cloud", opt.semantic_cloud_name)
      .set("output_cloud", opt.output_cloud_name)
      .set_if(!opt.labels_to_process.empty(), "labels", opt.labels_to_process)
      .dump();
}

int run_subcommand_segment_instances(const SubcommandSegInstancesOptions &opt,
                                     const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project database: {}", project_path.string());
    reusex::ProjectDB db(project_path);

    // Validate prerequisites
    spdlog::info("Validating prerequisites...");
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::instances,
            {{"labels", opt.semantic_cloud_name}});
        rc != RuxError::SUCCESS)
      return rc;

    if (!opt.labels_to_process.empty())
      spdlog::info("Processing only labels: {}",
                   fmt::join(opt.labels_to_process, ", "));

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::instances;
    ctx.parameters = instances_stage_parameters(opt);

    // Everything the old body did here — clustering, GUID reconciliation
    // across regeneration (#207), material re-linking, and the per-class
    // report — now lives once in reusex::pipeline::run_stage.
    const auto result = reusex::pipeline::run_stage(db, ctx);
    return rux::exit_code_for(result);

  } catch (const std::exception &e) {
    spdlog::error("Instance segmentation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
