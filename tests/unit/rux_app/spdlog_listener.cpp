// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Restores spdlog's default logger before every test case (reusex-m8o).
//
// rux::run() ends the way the `rux` process does: spdlog::shutdown(), which
// leaves spdlog::default_logger() null (rux_app.hpp: "Call it once, from
// main()"). The GuiLaunch tests call it inside this binary, and every rux
// subcommand logs through spdlog::info()/error(), which dereference the
// default logger unchecked. So whichever test ran after a GuiLaunch case and
// logged from a subcommand (GraphCli_*, CliCommandRoundTrip_*, ...) crashed
// with SIGSEGV when the binary ran in one process; ctest's one-process-per-
// case runs never saw it. A fresh default logger per test case stands in for
// the new process each case would get under ctest.

#include <catch2/catch_test_case_info.hpp>
#include <catch2/reporters/catch_reporter_event_listener.hpp>
#include <catch2/reporters/catch_reporter_registrars.hpp>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>

namespace {

class SpdlogDefaultLoggerListener : public Catch::EventListenerBase {
    public:
  using Catch::EventListenerBase::EventListenerBase;

  void testCaseStarting(const Catch::TestCaseInfo &) override {
    if (spdlog::default_logger())
      return;
    // What spdlog itself starts with: an unnamed colour logger on stdout.
    spdlog::set_default_logger(std::make_shared<spdlog::logger>(
        "", std::make_shared<spdlog::sinks::stdout_color_sink_mt>()));
  }
};

} // namespace

CATCH_REGISTER_LISTENER(SpdlogDefaultLoggerListener)
