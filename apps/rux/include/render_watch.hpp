// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `rux render --watch` (reusex-yf4.2): the headless half of the ground-truth
// authoring loop. An agent (or a person without a display) edits a plan file
// and gets, on every save, a fresh render (one view or the --qa contact
// sheet) and a block of scores on stdout.
//
// The loop is a thin shell around RenderWatchSession::pass(), which tests
// drive directly, without sleeping.

#include "global-params.hpp"
#include "plan_watch.hpp"
#include "render.hpp"

#include <reusex/types/point_types.hpp>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace rux {

/// What one pass of the watch did.
struct RenderWatchPass {
  int index = 0;            ///< 0 = the initial pass
  bool plan_read = false;   ///< the plan file was (re)read this pass
  bool plan_ok = false;     ///< ... and accepted
  std::string plan_error;   ///< the reader's error when it was not
  bool rendered = false;    ///< images were (re)written
  std::string render_error; ///< why rendering failed, if it did
  std::vector<std::filesystem::path> written;
  std::string report; ///< the block printed to stdout
};

class RenderWatchSession {
    public:
  RenderWatchSession(SubcommandRenderOptions opt, RuxOptions global_opt);

  /// One pass: re-read the plan when @p plan_changed (a file that does not
  /// parse keeps the last good plan and nothing is re-rendered), forget what
  /// was cached from the project when @p project_changed, render, score.
  /// Never throws: every failure is reported in the result.
  RenderWatchPass pass(bool plan_changed, bool project_changed);

  /// The files the loop watches: the plan, and with --watch-project the
  /// project and its sqlite -wal.
  [[nodiscard]] std::vector<std::filesystem::path> watched_paths() const;

  [[nodiscard]] const PlanFile &plan() const { return plan_; }

    private:
  std::string scores(const reusex::core::SolidModel &plan);

  SubcommandRenderOptions opt_;
  RuxOptions global_opt_;
  PlanFile plan_;
  int passes_ = 0;
  int good_pass_ = -1; ///< the pass that read the plan on screen
  CloudPtr cloud_;     ///< cached until the project changes
  bool cloud_loaded_ = false;
  std::string cloud_note_;
};

/// The `--watch` loop: pass(true, true), then one pass per change until
/// SIGINT / SIGTERM (or request_render_watch_stop()) or --watch-cycles.
int run_render_watch(SubcommandRenderOptions const &opt,
                     const RuxOptions &global_opt);

/// Ask a running watch loop to stop, as SIGINT does.
void request_render_watch_stop();

} // namespace rux

/// Re-exported for render.cpp, which dispatches to it.
using rux::run_render_watch;
