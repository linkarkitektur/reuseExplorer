// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The in-process `optimize` stage and the executor that adds it to
// default_stage_executor(), shared by every in-process job runner: the Qt
// client (rux) and the web GUI (ruxd).
//
// LAYERING: reusex_pipeline does not link reusex_slam (GTSAM, #464), so this
// header is deliberately HEADER-ONLY and compiled into its consumers, which
// must link both modules (the `reusex` umbrella does). It is the same
// arrangement as geometry/component_persistence.hpp. Including it from
// reusex_pipeline itself, or from ruxd_api_lib, would be a link error.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/optimize_parameters.hpp>

#include <fmt/format.h>

#include <exception>
#include <utility>

namespace reusex::pipeline {

/// Run `optimize` on an open project: the library defaults (which the
/// `rux optimize` flags mirror) plus the job's parameters through
/// geometry::apply_optimize_parameters(), the reader `rux optimize` uses too.
inline StageResult run_optimize_stage(ProjectDB &db, const StageContext &ctx) {
  geometry::PlaneGraphOptions options;
  bool dry_run = false;
  try {
    geometry::apply_optimize_parameters(options, dry_run, ctx.parameters);
  } catch (const std::exception &e) {
    return StageResult::invalid(e.what());
  }

  try {
    auto result = geometry::optimize_sensor_poses(db, options, dry_run);

    if (result.landmarks == 0 && result.loop_edges == 0)
      return StageResult::success(
          fmt::format("no plane landmarks reached min_observations={} and no "
                      "loop edges present; poses left unchanged",
                      options.min_landmark_observations));

    if (!result.converged)
      return StageResult::failure(
          "factor-graph optimizer failed to produce a solution; poses left "
          "unchanged");

    if (dry_run)
      return StageResult::success(
          fmt::format("dry run: {} frames, {} landmarks, {:.4f} -> {:.4f} "
                      "error, max shift {:.4f} m (poses not written)",
                      result.frames, result.landmarks, result.initial_error,
                      result.final_error, result.max_pose_shift));

    return StageResult::success(
        fmt::format("{} frames, {} landmarks, {:.4f} -> {:.4f} error, max "
                    "shift {:.4f} m",
                    result.frames, result.landmarks, result.initial_error,
                    result.final_error, result.max_pose_shift),
        // `sensor_frames` holds the optimized poses; `pose_graph_edges` the
        // per-edge metrics (and the odometry, when this run initialised it).
        {{"table", "sensor_frames", static_cast<int64_t>(result.frames)},
         {"table", "pose_graph_edges",
          static_cast<int64_t>(result.edges_with_metrics)}});
  } catch (const std::exception &e) {
    return StageResult::failure(
        fmt::format("pose optimization failed: {}", e.what()));
  }
}

/// default_stage_executor() plus `optimize` (run_optimize_stage on a
/// read-write open of the job's project).
inline StageExecutor stage_executor_with_optimize() {
  auto default_exec = default_stage_executor();
  return [default_exec =
              std::move(default_exec)](const StageContext &ctx) -> StageResult {
    if (ctx.stage == JobStage::optimize) {
      try {
        ProjectDB db(ctx.project, /*readOnly=*/false);
        return run_optimize_stage(db, ctx);
      } catch (const std::exception &e) {
        return StageResult::failure(fmt::format(
            "could not open project '{}': {}", ctx.project.string(), e.what()));
      }
    }
    return default_exec(ctx);
  };
}

} // namespace reusex::pipeline
