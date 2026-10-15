// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "render_watch.hpp"
#include "analyze/solid.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/reconstruction/plan_support.hpp>
#include <reusex/reconstruction/solid_metrics.hpp>
#include <reusex/utils/file_watcher.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <utility>

namespace fs = std::filesystem;

namespace rux {

namespace {

std::atomic<bool> g_stop{false};

extern "C" void render_watch_signal_handler(int /*sig*/) {
  g_stop.store(true, std::memory_order_release);
}

/// SIGINT / SIGTERM stop the loop for as long as this lives.
class StopOnSignal {
    public:
  StopOnSignal() {
    g_stop.store(false, std::memory_order_release);
    previous_int_ = std::signal(SIGINT, render_watch_signal_handler);
    previous_term_ = std::signal(SIGTERM, render_watch_signal_handler);
  }
  ~StopOnSignal() {
    std::signal(SIGINT, previous_int_);
    std::signal(SIGTERM, previous_term_);
  }
  StopOnSignal(const StopOnSignal &) = delete;
  StopOnSignal &operator=(const StopOnSignal &) = delete;

    private:
  using Handler = void (*)(int);
  Handler previous_int_ = SIG_DFL;
  Handler previous_term_ = SIG_DFL;
};

/// "a.png", or "a.png (+9 more)".
std::string images_line(const std::vector<fs::path> &written) {
  if (written.empty())
    return "none";
  if (written.size() == 1)
    return written.front().string();
  return fmt::format("{} (+{} more)", written.front().string(),
                     written.size() - 1);
}

} // namespace

RenderWatchSession::RenderWatchSession(SubcommandRenderOptions opt,
                                       RuxOptions global_opt)
    : opt_(std::move(opt)), global_opt_(std::move(global_opt)),
      plan_(opt_.plan) {}

std::vector<fs::path> RenderWatchSession::watched_paths() const {
  std::vector<fs::path> paths{opt_.plan};
  if (opt_.watch_project) {
    paths.push_back(global_opt_.project_db);
    paths.emplace_back(global_opt_.project_db.string() + "-wal");
  }
  return paths;
}

std::string RenderWatchSession::scores(const reusex::core::SolidModel &plan) {
  using namespace reusex;
  std::string out;
  try {
    ProjectDB db(global_opt_.project_db, /*readOnly=*/true);

    // GT self-check: do the plan's walls sit on the cloud?
    if (!cloud_loaded_) {
      cloud_loaded_ = true;
      cloud_.reset();
      cloud_note_.clear();
      if (db.has_point_cloud(opt_.cloud_name))
        cloud_ = db.point_cloud_xyzrgb(opt_.cloud_name);
      else
        cloud_note_ =
            fmt::format("the project has no cloud '{}'", opt_.cloud_name);
    }
    if (cloud_) {
      const geometry::PlanSupportOptions support_opt;
      out += format_plan_support(
          geometry::plan_support(*cloud_, plan, support_opt), support_opt);
    } else {
      out += fmt::format("wall support          n/a ({})\n", cloud_note_);
    }

    // The project's model against the plan, as `rux analyze solid` scores
    // it (without the fit metrics, which do not depend on the plan).
    const core::SolidModel detected = io::load_solid_model(db);
    const auto report = geometry::compare_solid_models(detected, plan);
    out += "model vs plan         (rux analyze solid; fit metrics: run it "
           "with --reference)\n";
    out += format_solid_summary(report);
    std::vector<std::string> missing;
    if (!detected.has_walls())
      missing.emplace_back("walls");
    if (!detected.has_spaces())
      missing.emplace_back("spaces");
    if (!detected.has_openings())
      missing.emplace_back("openings");
    if (!missing.empty())
      out += fmt::format("n/a because           the project provides no {} "
                         "yet\n",
                         fmt::join(missing, ", "));
  } catch (const std::exception &e) {
    out += fmt::format("scores                failed: {}\n", e.what());
  }
  return out;
}

RenderWatchPass RenderWatchSession::pass(bool plan_changed,
                                         bool project_changed) {
  RenderWatchPass r;
  r.index = passes_++;
  if (project_changed)
    cloud_loaded_ = false;

  std::string out = fmt::format("── pass {}: {} ──\n", r.index,
                                opt_.plan.filename().string());
  bool rerender = project_changed;
  if (plan_changed) {
    r.plan_read = true;
    r.plan_ok = plan_.reload();
    if (r.plan_ok) {
      good_pass_ = r.index;
      rerender = true;
      out += fmt::format("plan                  ok: {}\n",
                         plan_counts(*plan_.model()));
    } else {
      r.plan_error = plan_.error();
      out += fmt::format("plan                  INVALID: {}\n", r.plan_error);
      out += plan_.model()
                 ? fmt::format("                      keeping the last good "
                               "plan (pass {})\n",
                               good_pass_)
                 : std::string("                      no good plan yet: "
                               "fix the file and save again\n");
    }
  }

  if (rerender) {
    try {
      r.written = render_to_files(opt_, global_opt_, plan_.model());
      r.rendered = true;
      out += fmt::format("images                {}\n", images_line(r.written));
    } catch (const std::exception &e) {
      r.render_error = e.what();
      out += fmt::format("images                FAILED: {}\n", r.render_error);
    }
    if (plan_.model())
      out += scores(*plan_.model());
  } else {
    out += "images                unchanged\n";
  }
  r.report = std::move(out);
  return r;
}

void request_render_watch_stop() {
  g_stop.store(true, std::memory_order_release);
}

int run_render_watch(SubcommandRenderOptions const &opt,
                     const RuxOptions &global_opt) {
  if (opt.plan.empty()) {
    spdlog::error("render --watch needs --plan <file>: the plan is what it "
                  "watches");
    return RuxError::INVALID_ARGUMENT;
  }
  StopOnSignal stop_on_signal;
  RenderWatchSession session(opt, global_opt);
  reusex::utils::FileWatcher watcher(session.watched_paths());

  const auto print = [](const RenderWatchPass &p) {
    std::cout << p.report << std::flush;
  };
  RenderWatchPass first = session.pass(true, true);
  print(first);
  if (!first.plan_ok && !session.plan().model())
    spdlog::warn("Plan '{}' does not parse yet; waiting for a good save",
                 opt.plan.string());

  const auto interval = std::chrono::milliseconds(opt.watch_interval_ms);
  int cycles = 0;
  if (opt.watch_cycles != 0)
    spdlog::info("Watching {} (Ctrl-C to stop)", opt.plan.string());
  while (opt.watch_cycles < 0 || cycles < opt.watch_cycles) {
    const auto changed = watcher.wait(interval, g_stop);
    if (changed.empty())
      break; // stopped
    const bool plan_changed =
        std::find(changed.begin(), changed.end(), opt.plan) != changed.end();
    print(
        session.pass(plan_changed, changed.size() > (plan_changed ? 1U : 0U)));
    ++cycles;
  }
  spdlog::info("Watch stopped after {} re-render(s)", cycles);
  return RuxError::SUCCESS;
}

} // namespace rux
