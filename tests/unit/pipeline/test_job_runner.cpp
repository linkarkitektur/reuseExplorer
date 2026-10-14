// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// JobRunner state-machine tests (#265, Phase 1).
//
// The runner is constructed with a fake StageExecutor throughout, so these
// exercise queueing, status transitions, cancellation and event emission
// without touching a real project or running a real pipeline stage.

#include <catch2/catch_test_macros.hpp>

#include <core/processing_observer.hpp>
#include <core/stages.hpp>
#include <pipeline/JobRunner.hpp>
#include <pipeline/stages.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace reusex::pipeline;

namespace {

/// A gate the fake executor blocks on, so a test can hold a job in `running`.
class Gate {
    public:
  void wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return open_; });
  }
  void open() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      open_ = true;
    }
    cv_.notify_all();
  }

    private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool open_ = false;
};

/// Poll until @p predicate holds, or fail the test after a generous timeout.
/// Nothing here sleeps for a fixed duration — a timing-dependent test would be
/// flaky on a loaded CI machine (STANDARDS §6).
template <typename Predicate> bool wait_for(Predicate predicate) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return predicate();
}

StageExecutor always_succeeds(std::string message = "ok") {
  return
      [message](const StageContext &) { return StageResult::success(message); };
}

} // namespace

TEST_CASE("ParseJobStage_ValidAndInvalidNames_RoundTripsOrRejects",
          "[pipeline][jobs]") {
  for (const auto &name : job_stage_names()) {
    auto stage = parse_job_stage(name);
    REQUIRE(stage.has_value());
    REQUIRE(to_string(*stage) == name);
  }

  REQUIRE_FALSE(parse_job_stage("not-a-stage").has_value());
  REQUIRE_FALSE(parse_job_stage("").has_value());
  // `mesh` now has a runner (#265 Phase 3), so it parses and round-trips.
  REQUIRE(parse_job_stage("mesh").has_value());
  CHECK(to_string(*parse_job_stage("mesh")) == "mesh");
}

TEST_CASE("JobStatus_AllValues_RoundTripsAndTerminalityMatches",
          "[pipeline][jobs]") {
  const JobStatus all[] = {JobStatus::queued, JobStatus::running,
                           JobStatus::succeeded, JobStatus::failed,
                           JobStatus::cancelled};
  for (JobStatus status : all) {
    auto parsed = parse_job_status(to_string(status));
    REQUIRE(parsed.has_value());
    REQUIRE(*parsed == status);
  }
  REQUIRE_FALSE(parse_job_status("pending").has_value());

  REQUIRE_FALSE(is_terminal(JobStatus::queued));
  REQUIRE_FALSE(is_terminal(JobStatus::running));
  REQUIRE(is_terminal(JobStatus::succeeded));
  REQUIRE(is_terminal(JobStatus::failed));
  REQUIRE(is_terminal(JobStatus::cancelled));
}

TEST_CASE("Iso8601UtcNow_Called_ProducesParseableUtcTimestamp",
          "[pipeline][jobs]") {
  const auto stamp = iso8601_utc_now();
  REQUIRE(stamp.size() == 20);
  REQUIRE(stamp[4] == '-');
  REQUIRE(stamp[7] == '-');
  REQUIRE(stamp[10] == 'T');
  REQUIRE(stamp[13] == ':');
  REQUIRE(stamp[16] == ':');
  REQUIRE(stamp.back() == 'Z');
}

TEST_CASE("JobRunner_SubmittedJob_RunsToSuccess", "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", always_succeeds("done"));

  const auto id = runner.submit(JobStage::planes);
  REQUIRE_FALSE(id.empty());

  runner.wait_idle();

  auto record = runner.job(id);
  REQUIRE(record.has_value());
  CHECK(record->status == JobStatus::succeeded);
  CHECK(record->stage == JobStage::planes);
  CHECK(record->error.empty());
  CHECK_FALSE(record->submitted_at.empty());
  CHECK_FALSE(record->started_at.empty());
  CHECK_FALSE(record->finished_at.empty());
}

TEST_CASE("JobRunner_FailingStage_RecordsFailureReason", "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", [](const StageContext &) {
    return StageResult::failure("inputs not satisfied");
  });

  const auto id = runner.submit(JobStage::rooms);
  runner.wait_idle();

  auto record = runner.job(id);
  REQUIRE(record.has_value());
  CHECK(record->status == JobStatus::failed);
  CHECK(record->error == "inputs not satisfied");
}

TEST_CASE("JobRunner_StageThrows_ReportsFailureWithoutPropagating",
          "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux",
                   [](const StageContext &) -> StageResult {
                     throw std::runtime_error("boom");
                   });

  const auto id = runner.submit(JobStage::clouds);
  runner.wait_idle();

  auto record = runner.job(id);
  REQUIRE(record.has_value());
  CHECK(record->status == JobStatus::failed);
  CHECK(record->error == "boom");
}

TEST_CASE("JobRunner_SubmitWithParameters_PassesParametersVerbatim",
          "[pipeline][jobs]") {
  std::string seen;
  JobRunner runner("/nonexistent/project.rux",
                   [&seen](const StageContext &ctx) {
                     seen = ctx.parameters;
                     return StageResult::success();
                   });

  runner.submit(JobStage::planes, R"({"radius":0.5})");
  runner.wait_idle();
  CHECK(seen == R"({"radius":0.5})");
}

TEST_CASE("JobRunner_NonObjectParameters_RejectedAtSubmit",
          "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", always_succeeds());

  REQUIRE_THROWS_AS(runner.submit(JobStage::planes, "not json"),
                    std::runtime_error);
  REQUIRE_THROWS_AS(runner.submit(JobStage::planes, "[1,2,3]"),
                    std::runtime_error);
  // A rejected submission must leave no trace.
  CHECK(runner.jobs().empty());
}

TEST_CASE("JobRunner_UnknownJobId_ReturnsNulloptNotInvented",
          "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", always_succeeds());
  CHECK_FALSE(runner.job("no-such-id").has_value());
  CHECK_FALSE(runner.cancel("no-such-id"));
}

TEST_CASE("JobRunner_MultipleSubmittedJobs_ExecuteSeriallyInSubmissionOrder",
          "[pipeline][jobs]") {
  Gate gate;
  std::atomic<int> concurrent{0};
  std::atomic<int> max_concurrent{0};
  std::mutex order_mutex;
  std::vector<std::string> order;

  JobRunner runner("/nonexistent/project.rux", [&](const StageContext &ctx) {
    const int now = ++concurrent;
    int previous = max_concurrent.load();
    while (now > previous &&
           !max_concurrent.compare_exchange_weak(previous, now))
      ;
    {
      std::lock_guard<std::mutex> lock(order_mutex);
      order.emplace_back(to_string(ctx.stage));
    }
    gate.wait();
    --concurrent;
    return StageResult::success();
  });

  const auto first = runner.submit(JobStage::clouds);
  const auto second = runner.submit(JobStage::planes);
  const auto third = runner.submit(JobStage::rooms);

  REQUIRE(wait_for([&] { return runner.is_busy(); }));
  // With the gate shut, exactly one job is running and the rest are queued.
  CHECK(runner.queued_count() == 2);
  CHECK(runner.job(second)->status == JobStatus::queued);
  CHECK(runner.job(third)->status == JobStatus::queued);

  gate.open();
  runner.wait_idle();

  CHECK(max_concurrent.load() == 1);
  CHECK(order == std::vector<std::string>{"clouds", "planes", "rooms"});
  CHECK(runner.job(first)->status == JobStatus::succeeded);
  CHECK(runner.job(third)->status == JobStatus::succeeded);
}

TEST_CASE("JobRunner_CancelQueuedJob_StopsBeforeItRuns", "[pipeline][jobs]") {
  Gate gate;
  std::atomic<int> executions{0};

  JobRunner runner("/nonexistent/project.rux", [&](const StageContext &) {
    ++executions;
    gate.wait();
    return StageResult::success();
  });

  const auto running = runner.submit(JobStage::clouds);
  const auto queued = runner.submit(JobStage::planes);

  REQUIRE(wait_for([&] { return runner.is_busy(); }));
  REQUIRE(runner.cancel(queued));

  auto record = runner.job(queued);
  REQUIRE(record.has_value());
  CHECK(record->status == JobStatus::cancelled);
  CHECK(record->cancel_requested);
  CHECK_FALSE(record->finished_at.empty());
  CHECK(record->started_at.empty()); // It never started.

  gate.open();
  runner.wait_idle();

  CHECK(executions.load() == 1); // Only the first job ever executed.
  CHECK(runner.job(running)->status == JobStatus::succeeded);
}

TEST_CASE("JobRunner_CancelRunningJob_SetsCancelTokenStageObserves",
          "[pipeline][jobs]") {
  std::atomic_bool entered{false};
  std::atomic_bool saw_cancel{false};

  JobRunner runner("/nonexistent/project.rux", [&](const StageContext &ctx) {
    entered = true;
    // Cooperative cancellation: poll the token exactly like a real stage's
    // inner loop does.
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      if (ctx.is_cancelled()) {
        saw_cancel = true;
        return StageResult::cancel("stopped early");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return StageResult::success();
  });

  const auto id = runner.submit(JobStage::planes);
  REQUIRE(wait_for([&] { return entered.load(); }));
  REQUIRE(runner.cancel(id));
  runner.wait_idle();

  CHECK(saw_cancel.load());
  auto record = runner.job(id);
  REQUIRE(record.has_value());
  CHECK(record->status == JobStatus::cancelled);
  CHECK(record->error == "stopped early");
}

TEST_CASE("JobRunner_CancelTerminalJob_IsNoOpAndStaysSuccessful",
          "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", always_succeeds());

  const auto id = runner.submit(JobStage::clouds);
  runner.wait_idle();
  REQUIRE(runner.job(id)->status == JobStatus::succeeded);

  // Idempotent: cancelling a finished job must report success (the id exists)
  // without rewriting its outcome.
  CHECK(runner.cancel(id));
  CHECK(runner.job(id)->status == JobStatus::succeeded);
}

TEST_CASE("JobRunner_Jobs_ListsNewestFirst", "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", always_succeeds());

  const auto first = runner.submit(JobStage::clouds);
  const auto second = runner.submit(JobStage::planes);
  runner.wait_idle();

  const auto listed = runner.jobs();
  REQUIRE(listed.size() == 2);
  CHECK(listed[0].id == second);
  CHECK(listed[1].id == first);
}

TEST_CASE("JobRunner_Listener_ObservesFullLifecycle", "[pipeline][jobs]") {
  std::mutex mutex;
  std::vector<JobEvent> events;

  JobRunner runner("/nonexistent/project.rux", always_succeeds());
  const auto token = runner.add_listener([&](const JobEvent &event) {
    std::lock_guard<std::mutex> lock(mutex);
    events.push_back(event);
  });

  const auto id = runner.submit(JobStage::rooms);
  runner.wait_idle();

  REQUIRE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return events.size() >= 3;
  }));

  std::lock_guard<std::mutex> lock(mutex);
  CHECK(events.front().type == JobEvent::Type::submitted);
  CHECK(events.back().type == JobEvent::Type::finished);
  CHECK(events.back().job.status == JobStatus::succeeded);
  for (const auto &event : events) {
    CHECK(event.job.id == id);
    CHECK_FALSE(event.timestamp.empty());
  }

  runner.remove_listener(token);
  const auto count_before = events.size();
  runner.submit(JobStage::clouds);
  runner.wait_idle();
  CHECK(events.size() == count_before); // Removed listener stays silent.
}

TEST_CASE("JobRunner_ListenerThrows_DoesNotCrashRunner", "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", always_succeeds());
  runner.add_listener(
      [](const JobEvent &) { throw std::runtime_error("listener exploded"); });

  const auto id = runner.submit(JobStage::planes);
  runner.wait_idle();
  CHECK(runner.job(id)->status == JobStatus::succeeded);
}

TEST_CASE("JobRunner_ProgressObserver_UpdatesJobRecord", "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", [](const StageContext &) {
    // The runner installs itself as the global progress observer for the
    // duration of a job; a real stage reaches it via core::ProgressObserver.
    reusex::core::ProgressObserver progress(reusex::core::Stage::region_growing,
                                            100);
    progress.update(40);
    progress.update(35);
    return StageResult::success();
  });

  const auto id = runner.submit(JobStage::planes);
  runner.wait_idle();

  auto record = runner.job(id);
  REQUIRE(record.has_value());
  // on_process_finished snaps current to total once a total was declared.
  CHECK(record->progress_total == 100);
  CHECK(record->progress_current == 100);
  CHECK(record->progress_stage == reusex::core::Stage::region_growing);
}

TEST_CASE("JobRunner_PreexistingProgressObserver_ChainedThenRestored",
          "[pipeline][jobs]") {
  struct Recorder : reusex::core::IProgressObserver {
    std::atomic<size_t> started{0};
    void on_process_started(reusex::core::Stage, size_t) override { ++started; }
  } recorder;

  reusex::core::set_progress_observer(&recorder);

  {
    JobRunner runner("/nonexistent/project.rux", [](const StageContext &) {
      reusex::core::ProgressObserver progress(reusex::core::Stage::ray_tracing,
                                              10);
      progress.update(10);
      return StageResult::success();
    });
    runner.submit(JobStage::planes);
    runner.wait_idle();
  }

  // Chained, not replaced: the pre-existing observer still saw the event...
  CHECK(recorder.started.load() == 1);
  // ...and is still the installed one after the job finished.
  CHECK(reusex::core::get_progress_observer() ==
        static_cast<reusex::core::IProgressObserver *>(&recorder));

  reusex::core::reset_progress_observer();
}

TEST_CASE("JobRunner_Destruction_CancelsStillQueuedJobs", "[pipeline][jobs]") {
  Gate gate;
  std::atomic<int> executions{0};
  std::mutex mutex;
  std::vector<JobEvent> finished;

  {
    JobRunner runner("/nonexistent/project.rux", [&](const StageContext &) {
      ++executions;
      gate.wait();
      return StageResult::success();
    });
    runner.add_listener([&](const JobEvent &event) {
      if (event.type != JobEvent::Type::finished)
        return;
      std::lock_guard<std::mutex> lock(mutex);
      finished.push_back(event);
    });

    runner.submit(JobStage::clouds);
    runner.submit(JobStage::planes);
    REQUIRE(wait_for([&] { return runner.is_busy(); }));

    gate.open(); // Let the running job complete so the destructor can join.
  }

  // Both jobs reached a terminal state, and nothing was silently dropped.
  std::lock_guard<std::mutex> lock(mutex);
  CHECK(finished.size() == 2);
  CHECK(executions.load() <= 2);
}

TEST_CASE("StageSupportsCancellation_KnownStages_MatchesRealPlumbing",
          "[pipeline][jobs]") {
  // planes/rooms/instances thread a cancel token into the algorithm;
  // reconstruct_point_clouds does not, and the contract must say so rather
  // than pretend.
  CHECK(stage_supports_cancellation(JobStage::planes));
  CHECK(stage_supports_cancellation(JobStage::rooms));
  CHECK(stage_supports_cancellation(JobStage::instances));
  CHECK_FALSE(stage_supports_cancellation(JobStage::clouds));
  // A batch annotate run iterates frames, so a cancel request can take effect
  // between frames the way it does for planes/rooms/instances (#gpu-worker
  // W0) — even though the default executor never gets that far (see
  // stages.cpp's dispatch()).
  CHECK(stage_supports_cancellation(JobStage::annotate));
  CHECK_FALSE(stage_supports_cancellation(JobStage::graph_discover));
}

TEST_CASE("StageContext_IsCancelled_ReadsTokenSafely", "[pipeline][jobs]") {
  StageContext ctx;
  CHECK_FALSE(ctx.is_cancelled()); // No token attached.

  std::atomic_bool token{false};
  ctx.cancel_token = &token;
  CHECK_FALSE(ctx.is_cancelled());
  token = true;
  CHECK(ctx.is_cancelled());
}

// ===========================================================================
// Review follow-ups (#274)
// ===========================================================================

TEST_CASE("JobRunner_EventSequenceNumbers_AreMonotonicAndGapless",
          "[pipeline][jobs]") {
  // Events are published without the runner lock (a listener must never run
  // under it), so arrival order is not emission order. The sequence number is
  // assigned under the lock at the moment the state changed and is what a
  // client must order by.
  std::mutex mutex;
  std::vector<JobEvent> events;

  JobRunner runner("/nonexistent/project.rux", always_succeeds());
  runner.add_listener([&](const JobEvent &event) {
    std::lock_guard<std::mutex> lock(mutex);
    events.push_back(event);
  });

  runner.submit(JobStage::clouds);
  runner.submit(JobStage::planes);
  runner.wait_idle();

  REQUIRE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return events.size() >= 6; // submitted+started+finished, twice
  }));

  std::lock_guard<std::mutex> lock(mutex);
  std::vector<uint64_t> sequences;
  for (const auto &event : events) {
    CHECK(event.sequence > 0);
    sequences.push_back(event.sequence);
  }

  // Sorting by seq must recover the true order, and no number is reused.
  std::sort(sequences.begin(), sequences.end());
  CHECK(std::adjacent_find(sequences.begin(), sequences.end()) ==
        sequences.end());
  for (size_t i = 1; i < sequences.size(); ++i)
    CHECK(sequences[i] == sequences[i - 1] + 1);

  // Ordering by seq puts each job's lifecycle in the right order, whatever
  // order the callbacks happened to arrive in.
  std::map<std::string, std::vector<std::pair<uint64_t, JobEvent::Type>>>
      by_job;
  for (const auto &event : events)
    by_job[event.job.id].emplace_back(event.sequence, event.type);
  for (auto &[id, timeline] : by_job) {
    std::sort(timeline.begin(), timeline.end());
    REQUIRE(timeline.size() >= 3);
    CHECK(timeline.front().second == JobEvent::Type::submitted);
    CHECK(timeline.back().second == JobEvent::Type::finished);
  }
}

TEST_CASE("JobRunner_CancelUnhonoredByStage_ReportsSuccessNotCancelled",
          "[pipeline][jobs]") {
  // The clouds stage cannot be interrupted: it writes its output and returns
  // success. Marking the job "cancelled" because a cancel was *requested*
  // would send the user looking for results that are already in the project.
  std::atomic_bool entered{false};
  std::atomic_bool release{false};

  JobRunner runner("/nonexistent/project.rux", [&](const StageContext &) {
    entered = true;
    while (!release.load())
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    // Ignores the cancel token entirely, exactly like reconstruct_point_clouds.
    return StageResult::success("wrote its output anyway");
  });

  const auto id = runner.submit(JobStage::clouds);
  REQUIRE(wait_for([&] { return entered.load(); }));
  REQUIRE(runner.cancel(id));
  release = true;
  runner.wait_idle();

  auto record = runner.job(id);
  REQUIRE(record.has_value());
  CHECK(record->status == JobStatus::succeeded);
  CHECK(record->error.empty());
  // The request is still visible, so a UI can explain why nothing stopped.
  CHECK(record->cancel_requested);
}

TEST_CASE("JobRunner_CancelHonoredByStage_ReportsCancelled",
          "[pipeline][jobs]") {
  std::atomic_bool entered{false};

  JobRunner runner("/nonexistent/project.rux", [&](const StageContext &ctx) {
    entered = true;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      if (ctx.is_cancelled())
        return StageResult::cancel("stopped early");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return StageResult::success();
  });

  const auto id = runner.submit(JobStage::planes);
  REQUIRE(wait_for([&] { return entered.load(); }));
  REQUIRE(runner.cancel(id));
  runner.wait_idle();

  CHECK(runner.job(id)->status == JobStatus::cancelled);
}

// ===========================================================================
// Bounded job store (#286)
// ===========================================================================

TEST_CASE("JobRunner_TerminalJobsBeyondCap_EvictsOldestFirst",
          "[pipeline][jobs]") {
  JobRunnerOptions options;
  options.max_terminal_jobs = 3;
  JobRunner runner("/nonexistent/project.rux", always_succeeds(), options);

  std::vector<std::string> ids;
  for (int i = 0; i < 6; ++i) {
    ids.push_back(runner.submit(JobStage::planes));
    runner.wait_idle();
  }

  // Exactly the cap is retained, and it is the most recent submissions —
  // eviction follows submission order, not the hash order of the record map.
  const auto listed = runner.jobs();
  REQUIRE(listed.size() == 3);
  CHECK(listed[0].id == ids[5]); // jobs() is newest-first.
  CHECK(listed[1].id == ids[4]);
  CHECK(listed[2].id == ids[3]);

  for (size_t i = 0; i < 3; ++i)
    CHECK_FALSE(runner.job(ids[i]).has_value());
  for (size_t i = 3; i < 6; ++i)
    CHECK(runner.job(ids[i]).has_value());
}

TEST_CASE("JobRunner_EvictedJobId_ReturnsNullopt", "[pipeline][jobs]") {
  JobRunnerOptions options;
  options.max_terminal_jobs = 1;
  JobRunner runner("/nonexistent/project.rux", always_succeeds(), options);

  const auto dropped = runner.submit(JobStage::clouds);
  runner.wait_idle();
  REQUIRE(runner.job(dropped).has_value());

  const auto kept = runner.submit(JobStage::planes);
  runner.wait_idle();

  // Indistinguishable from an id that never existed — by design; the durable
  // record is pipeline_log, joined via pipeline_log.parameters.job_id.
  CHECK_FALSE(runner.job(dropped).has_value());
  CHECK(runner.job(kept).has_value());
}

TEST_CASE("JobRunner_QueuedAndRunningJobs_SurviveTerminalCap",
          "[pipeline][jobs]") {
  // A cap that could forget a submitted job would be worse than the unbounded
  // memory it exists to bound, so only terminal jobs are ever candidates.
  Gate gate;
  JobRunnerOptions options;
  options.max_terminal_jobs = 1;

  JobRunner runner(
      "/nonexistent/project.rux",
      [&](const StageContext &) {
        gate.wait();
        return StageResult::success();
      },
      options);

  const auto running = runner.submit(JobStage::clouds);
  REQUIRE(wait_for([&] { return runner.is_busy(); }));

  const auto queued = runner.submit(JobStage::planes);
  const auto first = runner.submit(JobStage::rooms);
  const auto second = runner.submit(JobStage::instances);
  const auto third = runner.submit(JobStage::planes);

  // Cancelling before execution is the other way the terminal set grows; three
  // cancellations against a cap of one force two evictions.
  REQUIRE(runner.cancel(first));
  REQUIRE(runner.cancel(second));
  REQUIRE(runner.cancel(third));

  auto live = runner.job(running);
  REQUIRE(live.has_value());
  CHECK(live->status == JobStatus::running);
  auto waiting = runner.job(queued);
  REQUIRE(waiting.has_value());
  CHECK(waiting->status == JobStatus::queued);

  // Of the three terminal jobs only the most recently submitted survives.
  CHECK_FALSE(runner.job(first).has_value());
  CHECK_FALSE(runner.job(second).has_value());
  CHECK(runner.job(third).has_value());

  const auto listed = runner.jobs();
  REQUIRE(listed.size() == 3);
  CHECK(listed[0].id == third);
  CHECK(listed[1].id == queued);
  CHECK(listed[2].id == running);

  gate.open();
  runner.wait_idle();
}

// ===========================================================================
// Stage results carried onto the job record (#286)
// ===========================================================================

TEST_CASE("JobRunner_SucceededJob_CarriesStageResultSummaryAndOutputs",
          "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", [](const StageContext &) {
    return StageResult::success(
        "detected 3 plane(s)",
        {{"cloud", "planes", 42}, {"cloud", "plane_centroids", 3}});
  });

  const auto id = runner.submit(JobStage::planes);
  runner.wait_idle();

  auto record = runner.job(id);
  REQUIRE(record.has_value());
  REQUIRE(record->status == JobStatus::succeeded);
  CHECK(record->result_summary == "detected 3 plane(s)");
  CHECK(record->error.empty()); // Success never fills the failure channel.

  REQUIRE(record->result_outputs.size() == 2);
  CHECK(record->result_outputs[0].kind == "cloud");
  CHECK(record->result_outputs[0].name == "planes");
  CHECK(record->result_outputs[0].count == 42);
  CHECK(record->result_outputs[1].name == "plane_centroids");
  CHECK(record->result_outputs[1].count == 3);
}

TEST_CASE("JobRunner_FailedJob_LeavesResultFieldsEmptyAndReportsViaError",
          "[pipeline][jobs]") {
  JobRunner runner("/nonexistent/project.rux", [](const StageContext &) {
    // A stage that fails part-way may still have written something, but the
    // job must not advertise a result: the message belongs in `error` only.
    StageResult result = StageResult::failure("plane fitting diverged");
    result.outputs.push_back({"cloud", "planes", 7});
    return result;
  });

  const auto id = runner.submit(JobStage::planes);
  runner.wait_idle();

  auto record = runner.job(id);
  REQUIRE(record.has_value());
  CHECK(record->status == JobStatus::failed);
  CHECK(record->error == "plane fitting diverged");
  CHECK(record->result_summary.empty());
  CHECK(record->result_outputs.empty());
}

TEST_CASE("JobRunner_CancelledJob_LeavesResultFieldsEmpty",
          "[pipeline][jobs]") {
  std::atomic_bool entered{false};

  JobRunner runner("/nonexistent/project.rux", [&](const StageContext &ctx) {
    entered = true;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      if (ctx.is_cancelled())
        return StageResult::cancel("stopped early");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return StageResult::success("should not get here");
  });

  const auto id = runner.submit(JobStage::rooms);
  REQUIRE(wait_for([&] { return entered.load(); }));
  REQUIRE(runner.cancel(id));
  runner.wait_idle();

  auto record = runner.job(id);
  REQUIRE(record.has_value());
  CHECK(record->status == JobStatus::cancelled);
  CHECK(record->error == "stopped early");
  CHECK(record->result_summary.empty());
  CHECK(record->result_outputs.empty());
}
