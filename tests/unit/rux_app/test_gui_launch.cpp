// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Plain `rux` (no subcommand) hands the desktop app a GuiLaunch through the
// launcher main() injects (rux_app.hpp). The seam keeps Qt out of rux_lib —
// and out of this binary — and lets the hand-off be tested with a fake.
//
// rux::run mutates process-global state (spdlog, signal handlers), so each
// TEST_CASE calls it once; ctest runs every case in its own process.

#include <catch2/catch_test_macros.hpp>

#include <rux_app.hpp>
#include <rux_qt/background.hpp>

#include <reusex/core/logging.hpp>

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include "../../support/temp_path.hpp"

#include <cstdlib>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Call {
  bool called = false;
  rux::GuiLaunch request;
};

int run_with(std::vector<std::string> args, Call &call,
             bool with_launcher = true) {
  // A display the launch decision accepts without a real X/Wayland session.
  ::setenv("QT_QPA_PLATFORM", "offscreen", 1);
  std::vector<char *> argv;
  for (auto &a : args)
    argv.push_back(a.data());
  argv.push_back(nullptr);
  rux::GuiLauncher fake;
  if (with_launcher)
    fake = [&](int, char **, const rux::GuiLaunch &r) {
      call.called = true;
      call.request = r;
      return 7;
    };
  return rux::run(static_cast<int>(args.size()), argv.data(), fake);
}

} // namespace

TEST_CASE("GuiLaunch_ExplicitProject_IsHandedToTheLauncher",
          "[rux_app][gui_launch]") {
  reusex::test_support::TempPath tmp("test_gui_launch");
  Call call;
  const int rc =
      run_with({"rux", "-p", tmp.path.string(), "--quit-after-ms", "5"}, call);
  CHECK(rc == 7); // the launcher's exit code is rux's
  REQUIRE(call.called);
  REQUIRE(call.request.project.has_value());
  CHECK(*call.request.project == tmp.path);
  CHECK(call.request.quit_after_ms == 5);
  CHECK_FALSE(std::filesystem::exists(tmp.path)); // rux itself never opens it
}

TEST_CASE("GuiLaunch_NoProjectFlag_NeverPassesTheDefault",
          "[rux_app][gui_launch]") {
  Call call;
  run_with({"rux"}, call);
  REQUIRE(call.called);
  // The ./project.rux default is not a request: opening it would create it.
  CHECK_FALSE(call.request.project.has_value());
  CHECK(call.request.quit_after_ms == -1);
}

TEST_CASE("GuiLaunch_NoLauncher_PrintsHelpAndSucceeds",
          "[rux_app][gui_launch]") {
  Call call;
  CHECK(run_with({"rux"}, call, /*with_launcher=*/false) == 0);
  CHECK_FALSE(call.called);
}

TEST_CASE("GuiLaunch_DetachedWorkStillRunning_LeavesTheLoggerUp",
          "[rux_app][gui_launch]") {
  // A launcher that returns while detached work (a locked open, an ICP run)
  // is still in flight: rux::run must not shut spdlog down under it.
  ::setenv("QT_QPA_PLATFORM", "offscreen", 1);
  std::vector<std::string> args = {"rux"};
  std::vector<char *> argv = {args[0].data(), nullptr};
  std::promise<void> release;
  std::thread worker;
  rux::GuiLauncher fake = [&](int, char **, const rux::GuiLaunch &) {
    std::promise<void> started;
    worker = std::thread([&, s = &started] {
      rux::qt::BackgroundWork work;
      s->set_value();
      release.get_future().wait();
    });
    started.get_future().wait();
    return 0;
  };
  CHECK(rux::run(1, argv.data(), fake) == 0);
  CHECK(rux::qt::background_work_in_flight() == 1);
  CHECK(spdlog::default_logger() != nullptr); // not shut down
  CHECK(rux::logging_left_up());
  release.set_value();
  worker.join();
  CHECK(rux::qt::background_work_in_flight() == 0);
  // The work finished after run() returned: main() tears the logger down.
  rux::finish_logging();
  CHECK_FALSE(rux::logging_left_up());
  CHECK(spdlog::default_logger() == nullptr);
}

TEST_CASE("GuiLaunch_NoDetachedWork_ShutsTheLoggerDown",
          "[rux_app][gui_launch]") {
  Call call;
  run_with({"rux"}, call);
  REQUIRE(call.called);
  CHECK(spdlog::default_logger() == nullptr); // spdlog::shutdown() ran
  CHECK_FALSE(rux::logging_left_up());
}

TEST_CASE("GuiLaunch_Teardown_DetachesTheLibraryLogHandler",
          "[rux_app][gui_launch]") {
  // run() routes ReUseX library logs into spdlog. Its teardown shuts spdlog
  // down (the default logger becomes null), so it must also take that handler
  // down: a library log after run() — from a lingering thread, a static
  // destructor or, in this binary, the next test — would otherwise call
  // spdlog::warn() on a null default logger and crash (reusex-m8o).
  Call call;
  run_with({"rux"}, call);
  REQUIRE(call.called);
  REQUIRE(spdlog::default_logger() == nullptr);

  std::ostringstream out;
  spdlog::set_default_logger(std::make_shared<spdlog::logger>(
      "test_gui_launch",
      std::make_shared<spdlog::sinks::ostream_sink_mt>(out)));
  reusex::warn("after run");
  spdlog::default_logger()->flush();
  const std::string logged = out.str();
  spdlog::set_default_logger(nullptr); // `out` dies with this scope
  CHECK(logged.find("after run") == std::string::npos);
}
