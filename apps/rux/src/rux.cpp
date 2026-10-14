// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <align.hpp>
#include <analyze.hpp>
#include <assemble.hpp>
#include <create.hpp>
#include <del.hpp>
#include <edit.hpp>
#include <export.hpp>
#include <get.hpp>
#include <graph.hpp>
#include <import.hpp>
#include <info.hpp>
#include <log.hpp>
#include <optimize.hpp>
#include <processing_observer.hpp>
#include <register.hpp>
#include <render.hpp>
#include <rux_app.hpp>
#include <set.hpp>
#include <validate.hpp>
#include <view.hpp>

#include <reusex/core/logging.hpp>
#include <reusex/core/version.hpp>
#include <reusex/pipeline/optimize_stage.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/launch.hpp>
#include <rux_qt/workspace_logic.hpp>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <execinfo.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

namespace {
constexpr int kMaxVerbosity = 3;

// Signal handler that dumps a backtrace to stderr on fatal signals
// (SIGABRT / SIGSEGV / SIGBUS / SIGFPE). Async-signal-safe: only uses
// write(), backtrace(), and backtrace_symbols_fd(). The default handler
// re-raises the signal afterwards so we still get the usual crash exit
// code + a core file if ulimit permits.
void fatal_signal_handler(int sig) {
  const char *msg = "\n\n=== rux fatal signal — backtrace follows ===\n";
  ssize_t r = write(STDERR_FILENO, msg, std::strlen(msg));
  (void)r;
  void *frames[128];
  const int n = backtrace(frames, 128);
  backtrace_symbols_fd(frames, n, STDERR_FILENO);
  const char *tail = "=== end backtrace ===\n";
  r = write(STDERR_FILENO, tail, std::strlen(tail));
  (void)r;
  // Re-raise with the default handler so the process actually dies with
  // the right signal (and produces a core file if enabled).
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}

void install_fatal_signal_handlers() {
  std::signal(SIGABRT, fatal_signal_handler);
  std::signal(SIGSEGV, fatal_signal_handler);
  std::signal(SIGBUS, fatal_signal_handler);
  std::signal(SIGFPE, fatal_signal_handler);
}
} // namespace

namespace rux {

namespace {
bool g_logging_left_up = false;
} // namespace

bool logging_left_up() { return g_logging_left_up; }

void finish_logging() {
  g_logging_left_up = false;
  spdlog::shutdown();
}

int run(int argc, char **argv, GuiLauncher launch_gui) {
  install_fatal_signal_handlers();

  // Initialize async logger thread pool (lock-free queue, background writer
  // thread) Queue size: 8192 messages, 1 background thread
  spdlog::init_thread_pool(8192, 1);

  // Create async logger with lock-free queue (no mutex contention)
  auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
  auto console_logger = std::make_shared<spdlog::async_logger>(
      "rux", console_sink, spdlog::thread_pool(),
      spdlog::async_overflow_policy::block);

  // Replace the default logger with our async logger
  spdlog::set_default_logger(console_logger);

  // Set the global processing observer to enable progress reporting and
  // visualization
  rux::setup_processing_observer();

  spdlog::set_level(spdlog::level::warn); // Default level
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%=7l%$] %v");

  reusex::core::set_log_handler(
      [](reusex::core::LogLevel level, std::string_view message) {
        // The Qt client's Pipeline workspace tails a running stage through
        // this tap; without a listener it costs a lock and an empty loop.
        rux::qt::publish_log(static_cast<int>(level), message);
        switch (level) {
        case reusex::core::LogLevel::trace:
          spdlog::trace("{}", message);
          break;
        case reusex::core::LogLevel::debug:
          spdlog::debug("{}", message);
          break;
        case reusex::core::LogLevel::info:
          spdlog::info("{}", message);
          break;
        case reusex::core::LogLevel::warn:
          spdlog::warn("{}", message);
          break;
        case reusex::core::LogLevel::error:
          spdlog::error("{}", message);
          break;
        case reusex::core::LogLevel::critical:
          spdlog::critical("{}", message);
          break;
        case reusex::core::LogLevel::off:
          break;
        }
      });
  reusex::core::set_log_level(reusex::core::LogLevel::warn);

  CLI::App app{"rux: ReUseX a tool for processing "
               "interior lidar scans of buildings."};

  auto opt = std::make_shared<RuxOptions>();

  app.get_formatter()->column_width(40);

  app.add_flag(
         "-v, --verbose",
         [](int count) {
           const int safe_count = std::clamp(count, 0, kMaxVerbosity);
           const auto level = static_cast<spdlog::level::level_enum>(
               kMaxVerbosity - safe_count);
           spdlog::set_level(level);
           reusex::core::set_log_level(
               static_cast<reusex::core::LogLevel>(kMaxVerbosity - safe_count));
           spdlog::info("Verbosity level: {}",
                        spdlog::level::to_string_view(spdlog::get_level()));
         },
         "Increase verbosity, use -vv & -vvv for more details.")
      ->multi_option_policy(CLI::MultiOptionPolicy::Sum)
      ->check(CLI::Range(0, 3));
  app.add_flag(
      "-V, --version",
      [](bool /*count*/) {
        std::cout << "rux version: " << reusex::core::VERSION << std::endl;
        exit(0);
      },
      "Show version information");
  app.add_flag(
      "-L, --license",
      [](bool /*count*/) {
        std::cout << reusex::core::LICENSE_TEXT << std::endl;
        exit(0);
      },
      "Show license information");
  app.add_flag(
         "-D, --visualize", [](bool) { rux::start_viewer(); },
         "Enable visualization of processing steps")
      ->default_val(false);

  // Global project file flag (applied before subcommands)
  // Note: No file existence check here - commands will validate when needed
  app.add_option("-p,--project", opt->project_db,
                 "Path to .rux project file (applies to all subcommands)")
      ->default_val(fs::current_path() / "project.rux");

  setup_subcommand_align(app, opt);
  setup_subcommand_analyze(app, opt);
  setup_subcommand_assemble(app, opt);
  setup_subcommand_create(app, opt);
  setup_subcommand_edit(app, opt);
  setup_subcommand_import(app, opt);
  setup_subcommand_export(app, opt);
  setup_subcommand_register(app, opt);
  setup_subcommand_render(app, opt);
  setup_subcommand_optimize(app, opt);
  setup_subcommand_graph(app, opt);

  // Unified path-based database commands (replaces old get/add/set/remove)
  setup_subcommand_get(app, opt);
  setup_subcommand_set(app, opt);
  setup_subcommand_del(app, opt);

  setup_subcommand_info(app, opt);
  setup_subcommand_log(app, opt);
  setup_subcommand_validate(app, opt);
  setup_subcommand_view(app, opt);

  // Dev/test flag of the Qt client (plain `rux`): quit after N ms and print
  // a one-line state report. Hidden from --help (empty group).
  int quit_after_ms = -1;
  app.add_option("--quit-after-ms", quit_after_ms,
                 "Qt client: quit after N ms (smoke tests)")
      ->group("");

  app.require_subcommand(/* min */ 0, /* max */ 2);

  argv = app.ensure_utf8(argv);

  // Teardown shared by the success path and the runtime-failure path, so that
  // a failing subcommand differs from a succeeding one only in its exit code.
  auto teardown = [] {
    rux::wait_for_viewer();
    reusex::core::reset_visual_observer();
    reusex::core::reset_progress_observer();
    // g_processing_observer.stop();

    // Flush async queue to ensure all logs are written before exit
    spdlog::shutdown();
  };

  try {
    app.parse(argc, argv);
  } catch (const CLI::RuntimeError &e) {
    // A subcommand ran and failed (issue #299). rux::finish() turned its
    // RuxError into this exception; CLI::App::exit() prints nothing for a
    // RuntimeError, so the subcommand's own spdlog::error line stays the only
    // diagnostic and only the process exit code changes.
    teardown();
    return app.exit(e);
  } catch (const CLI::ParseError &e) {
    // Argument parsing failed before any subcommand body ran, so there is no
    // viewer to wind down — keep the original early-out.
    spdlog::shutdown(); // Flush async queue before exit
    return app.exit(e);
  }

  // No subcommand: plain `rux` (or `rux -p x.rux`) is the Qt client when
  // there is a display and a launcher was linked (main.cpp), and the help
  // text otherwise (rux_qt/launch.hpp).
  {
    auto env = [](const char *name) {
      const char *v = std::getenv(name);
      return std::string_view(v ? v : "");
    };
    rux::qt::LaunchInputs in;
    in.has_subcommand = !app.get_subcommands().empty();
    in.qt_client_built = static_cast<bool>(launch_gui);
    in.display = env("DISPLAY");
    in.wayland_display = env("WAYLAND_DISPLAY");
    in.xdg_runtime_dir = env("XDG_RUNTIME_DIR");
    in.qpa_platform = env("QT_QPA_PLATFORM");
    in.path_exists = [](const std::string &p) {
      std::error_code ec;
      return fs::exists(p, ec);
    };
    switch (rux::qt::decide_launch(in)) {
    case rux::qt::LaunchAction::run_subcommand:
      break;
    case rux::qt::LaunchAction::print_help:
      std::cout << app.help();
      if (!in.has_subcommand && in.qt_client_built)
        std::cout << "\nNo display: the desktop app (plain `rux`) needs X11 "
                     "or Wayland. Run a subcommand instead.\n";
      break;
    case rux::qt::LaunchAction::open_app: {
      GuiLaunch request;
      // Only an explicit -p opens a project; the ./project.rux default
      // would otherwise be created by the open.
      if (app.count("--project") > 0)
        request.project = opt->project_db;
      request.quit_after_ms = quit_after_ms;
      request.stage_executor = reusex::pipeline::stage_executor_with_optimize();
      const int rc = launch_gui(argc, argv, request);
      if (rux::qt::background_work_in_flight() > 0) {
        // run_app's bounded wait ran out while a detached thread (a project
        // open stuck on a lock or a long migration, an ICP refine) is still
        // going. It may log at any moment: spdlog::shutdown() or resetting
        // the ReUseX log handler under it would be a use-after-free. Flush
        // what was logged and leave both alive; main() then ends the process
        // with std::quick_exit, so no static destructor runs under the
        // thread either (rux_qt/background.hpp).
        if (auto logger = spdlog::default_logger())
          logger->flush();
        g_logging_left_up = true;
        return rc;
      }
      teardown();
      return rc;
    }
    }
  }

  teardown();
  return 0;
}

} // namespace rux
