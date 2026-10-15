// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Database-level pipeline stage execution (#265, Phase 1).
//
// The algorithms in segmentation/ and reconstruction/ are pure functions over
// PCL clouds; the "load the named clouds out of a ProjectDB, run the algorithm,
// write the results back, record it in pipeline_log" wrapper used to live only
// inside apps/rux/src/create/*.cpp. This header lifts that wrapper DOWN into
// the library so both `rux create <stage>` and the GUI/daemon job runners drive
// the exact same code path (issue #265: "shared logic goes DOWN into the
// library, not sideways between apps").
//
// Layering: this is the `pipeline` module, which sits ABOVE the Layer-3 peers
// (segmentation / reconstruction / slam / io) — it is the one place allowed to
// link several of them at once. See docs/STANDARDS.md §1.

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::pipeline {

/// A pipeline stage that can be executed as a job.
///
/// Deliberately narrower than core::PipelineStage (which also covers stages
/// with no runner yet) and than core::Stage (which is a progress-reporting
/// phase, not a unit of work). Extending this enum is how a stage becomes
/// GUI-runnable; see `ruxd`'s `docs/gui/openapi.yaml` (in its own repo),
/// which lists the same names.
enum class JobStage {
  clouds,    ///< back-project sensor frames into "cloud" + "normals"
  planes,    ///< detect planar surfaces -> "planes"/"plane_*"
  rooms,     ///< partition planes into rooms -> "rooms"
  instances, ///< split semantic labels into spatial instances
  mesh,      ///< MIP-solve the cell complex into a watertight mesh
  optimize,  ///< refine per-frame sensor poses via plane-landmark pose graph
  annotate,  ///< ML semantic labelling of the stored sensor frames
  graph_discover, ///< find loop edges and store them in the pose graph
};

/// Canonical lower-case stage name (the token used on the wire and the CLI).
std::string_view to_string(JobStage stage);

/// The name this stage writes into `pipeline_log.stage`.
///
/// Deliberately NOT the same as to_string(): the CLI has been writing
/// `segment_planes` / `cloud_reconstruction` into that column since long
/// before the GUI existed, and existing project databases are full of those
/// values. The wire token stays short (`planes`); the log name stays
/// compatible, so one table does not end up with two names for one operation.
std::string_view pipeline_log_name(JobStage stage);

/// Parse a canonical stage name. Returns nullopt for an unknown name.
std::optional<JobStage> parse_job_stage(std::string_view name);

/// Every name accepted by parse_job_stage(), in pipeline order.
std::vector<std::string> job_stage_names();

/// True when the stage threads the cancel token into its inner loop, i.e.
/// cancellation takes effect mid-run rather than only between jobs.
bool stage_supports_cancellation(JobStage stage);

/// One artifact a stage wrote, so a client can refresh exactly what changed
/// instead of re-fetching everything.
struct StageArtifact {
  /// "cloud" | "mesh" | "table".
  std::string kind;
  /// Stored name, e.g. "planes". For a `table` artifact this is the key the
  /// table is scoped by — the instances table of cloud "instances" is
  /// `{kind: "table", name: "instances"}`, reachable at
  /// `GET /api/v1/instances/instances`.
  std::string name;
  /// Points/rows written; -1 when not meaningful, or when the stage does not
  /// have the number in hand and counting it would mean re-reading the
  /// artifact purely to report it. Never a guess.
  int64_t count = -1;
};

/// Outcome of one stage execution.
struct StageResult {
  bool ok = false;        ///< The stage completed and wrote its outputs.
  bool cancelled = false; ///< The stage stopped early on a cancel request.
  /// The request was refused before/without doing the work because an input or
  /// a parameter was invalid, as opposed to the stage failing part-way through.
  /// Front ends need the distinction: `rux` returns INVALID_ARGUMENT rather
  /// than a generic error, and an HTTP front end would answer 400, not 500.
  /// Always accompanied by `ok == false`.
  bool invalid_input = false;
  std::string message; ///< Human-readable summary or failure reason.
  /// What this run actually wrote. Only populated on `ok == true`, and only
  /// with artifacts the stage can attest to having written on *this* run — a
  /// name that merely exists in the project afterwards is not reported.
  std::vector<StageArtifact> outputs;

  static StageResult success(std::string message = {});
  static StageResult success(std::string message,
                             std::vector<StageArtifact> outputs);
  static StageResult failure(std::string message);
  static StageResult invalid(std::string message);
  static StageResult cancel(std::string message = "cancelled");
};

/// Everything a stage execution needs, independent of any HTTP/CLI front end.
struct StageContext {
  /// Path to the `.rux` project the stage reads and writes.
  std::filesystem::path project;
  /// Which stage to run.
  JobStage stage = JobStage::clouds;
  /// Stage parameters as a JSON object (empty string = all defaults). Keys
  /// mirror the option-struct fields of the underlying algorithm; unknown keys
  /// are ignored. The same string is persisted into `pipeline_log.parameters`.
  std::string parameters;
  /// Cooperative cancellation flag, owned by the caller. May be null.
  const std::atomic_bool *cancel_token = nullptr;
  /// Identifier of the job driving this run, or empty for a direct call.
  /// Recorded into the stage's `pipeline_log.parameters` under `"job_id"` so
  /// the durable history can be joined back to the job that caused it (#265).
  std::string job_id;

  /// True when a cancel token is present and set.
  bool is_cancelled() const noexcept;
};

/// Run a stage against an already-open, writable ProjectDB.
///
/// Never throws: algorithm/IO failures are converted into a failed StageResult
/// and, when a pipeline_log row was opened, closed out with the error message
/// (STANDARDS §5 — no silent failure).
StageResult run_stage(ProjectDB &db, const StageContext &ctx);

/// Run `body` wrapped in the same `pipeline_log` open/close protocol
/// run_stage() uses for the stages it dispatches directly: opens a row under
/// `log_name` (`ctx.job_id` folded into the recorded parameters exactly as
/// run_stage() does), evaluates `body`, and closes the row according to the
/// StageResult it returns — or, if `body` throws, the same way run_stage()'s
/// catch clause does (distinguishing a cancel from a failure via
/// `ctx.is_cancelled()`). `name` is the stage name used in log/error messages
/// (`to_string(stage)`); `log_name` is the value written to
/// `pipeline_log.stage` (`pipeline_log_name(stage)`).
///
/// Exists for a stage whose algorithm needs a module reusex_pipeline must not
/// link (optimize/graph_discover need reusex_slam, #464): their header-only
/// executor (pipeline/optimize_stage.hpp) calls this instead of
/// re-implementing the open/close bookkeeping, so the CLI and in-process
/// paths cannot drift on what a pipeline_log row looks like.
StageResult run_logged_stage(ProjectDB &db, const StageContext &ctx,
                             std::string_view name, std::string_view log_name,
                             const std::function<StageResult()> &body);

/// Open `ctx.project` read-write and run the stage against it.
StageResult run_stage(const StageContext &ctx);

/// Stage execution strategy. Exists so JobRunner's state machine can be tested
/// against a fake executor without touching real scan data.
using StageExecutor = std::function<StageResult(const StageContext &)>;

/// The executor that actually runs the pipeline (a thin wrap of run_stage).
StageExecutor default_stage_executor();

} // namespace reusex::pipeline
