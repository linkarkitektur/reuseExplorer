// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "graph/init.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/stage_contract.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <exception>

void setup_subcommand_graph_init(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandGraphInitOptions>();
  auto *sub = parent.add_subcommand(
      "init", "Snapshot initial poses and create odometry edges");
  sub->footer(R"(
DESCRIPTION:
  Copies every frame's current pose into sensor_frames.initial_transform (the
  pose 'rux optimize' seeds from) and stores one odometry edge per pair of
  consecutive frames of each scan, measured from those poses. 'rux optimize'
  runs this automatically when the graph has no odometry edges.

  On a graph that already has odometry it only fills the gaps: frames
  without an initial pose are snapshotted and consecutive pairs without
  odometry get an edge (e.g. after importing another scan). Existing
  snapshots and edges are kept; it refuses only when there is nothing to add.

  --force replaces the auto odometry_init edges and re-snapshots every initial
  pose from the current poses; confirmed odometry_init edges, loop edges,
  RTABMap edges and rejected edges are kept. Use it after changing poses
  outside optimize (e.g. 'rux register') to optimise on top of that result.
)");
  sub->add_flag("--force", opt->force,
                "Re-derive the odometry_init edges and re-snapshot every "
                "initial pose from the current poses");
  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_graph_init(*opt, *global_opt));
  });
}

int run_subcommand_graph_init(const SubcommandGraphInitOptions &opt,
                              const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::graph_init);
        rc != RuxError::SUCCESS)
      return rc;
    const int logId = db.log_pipeline_start(
        "graph_init", fmt::format(R"({{"force":{}}})", opt.force));
    try {
      const auto r = reusex::slam::init_pose_graph(db, {.force = opt.force});
      db.log_pipeline_end(logId, true);
      spdlog::info("Pose graph: {} frames in {} scan(s); {} initial poses "
                   "snapshotted; {} odometry edges written ({} pairs kept "
                   "their existing edge)",
                   r.frames, r.scans, r.snapshots_written,
                   r.odometry_edges_written, r.pairs_with_existing_odometry);
      return RuxError::SUCCESS;
    } catch (const reusex::slam::PoseGraphExistsError &e) {
      db.log_pipeline_end(logId, false, e.what());
      spdlog::error("{}", e.what());
      return RuxError::INVALID_ARGUMENT;
    } catch (const std::exception &e) {
      db.log_pipeline_end(logId, false, e.what());
      throw;
    }
  } catch (const std::exception &e) {
    spdlog::error("graph init failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
