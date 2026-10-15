// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The in-process `optimize` and `graph_discover` stages and the executor that
// adds them to default_stage_executor(), shared by every in-process job
// runner: the Qt client (rux) and the web GUI (ruxd).
//
// LAYERING: reusex_pipeline does not link reusex_slam (GTSAM, #464), so this
// header is deliberately HEADER-ONLY and compiled into its consumers, which
// must link both modules (the `reusex` umbrella does). It is the same
// arrangement as geometry/component_persistence.hpp. Including it from
// reusex_pipeline itself, or from ruxd_api_lib, would be a link error.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/discover_parameters.hpp>
#include <reusex/slam/graph_discover.hpp>
#include <reusex/slam/optimize_parameters.hpp>

#include <fmt/format.h>

#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
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

/// Run `graph_discover` on an open project through the same JSON reader
/// `rux graph discover` uses (slam::apply_discover_parameters).
inline StageResult run_graph_discover_stage(ProjectDB &db,
                                            const StageContext &ctx) {
  slam::GraphDiscoverOptions options;
  try {
    slam::apply_discover_parameters(options, ctx.parameters);
  } catch (const std::exception &e) {
    return StageResult::invalid(e.what());
  }
  try {
    const auto r = slam::discover_loop_edges(db, options);
    // Contract: the Qt client's rux_qt::parse_discover_summary reads the
    // counts back out of this exact text for Rediger's status line (pinned
    // by test_discover_summary). Change both together.
    const std::string summary = fmt::format(
        "{}{}{} proposed, {} matched, {} PCM-rejected, {} over the PCM cap, {} "
        "already stored, {} rejected before, {} added, {} stored rows given "
        "a measurement",
        options.dry_run ? "dry run: " : "",
        r.initialised_graph
            ? fmt::format("no odometry stored: initialised {} odometry edges "
                          "first; ",
                          r.odometry_edges_written)
            : std::string(),
        r.proposed, r.candidates, r.pcm_rejected, r.pcm_capped,
        r.skipped_existing, r.skipped_rejected, r.added, r.filled);
    if (options.dry_run)
      return StageResult::success(summary);
    return StageResult::success(
        summary, {{"table", "pose_graph_edges",
                   static_cast<int64_t>(r.added + r.filled +
                                        r.odometry_edges_written)}});
  } catch (const std::invalid_argument &e) {
    return StageResult::invalid(e.what());
  } catch (const std::exception &e) {
    return StageResult::failure(
        fmt::format("loop-edge discovery failed: {}", e.what()));
  }
}

/// default_stage_executor() plus `optimize` and `graph_discover`
/// (run_optimize_stage / run_graph_discover_stage on a read-write open of
/// the job's project).
///
/// Both stages are wrapped in run_logged_stage() (reusex::pipeline::
/// run_stage()'s shared pipeline_log open/close protocol), so a Qt
/// Pipeline/Rediger or ruxd run of either leaves the same kind of
/// `pipeline_log` row `rux optimize` / `rux graph discover` have always
/// written, under the same log name (reusex-xlg.5) — instead of the silent
/// gap stage_executor_with_optimize() left by calling the two stage bodies
/// directly.
inline StageExecutor stage_executor_with_optimize() {
  auto default_exec = default_stage_executor();
  return [default_exec =
              std::move(default_exec)](const StageContext &ctx) -> StageResult {
    if (ctx.stage == JobStage::optimize ||
        ctx.stage == JobStage::graph_discover) {
      try {
        ProjectDB db(ctx.project, /*readOnly=*/false);
        return run_logged_stage(
            db, ctx, to_string(ctx.stage), pipeline_log_name(ctx.stage),
            [&db, &ctx]() -> StageResult {
              return ctx.stage == JobStage::optimize
                         ? run_optimize_stage(db, ctx)
                         : run_graph_discover_stage(db, ctx);
            });
      } catch (const std::exception &e) {
        return StageResult::failure(fmt::format(
            "could not open project '{}': {}", ctx.project.string(), e.what()));
      }
    }
    return default_exec(ctx);
  };
}

} // namespace reusex::pipeline
