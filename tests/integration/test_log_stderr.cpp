// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Verify that rux routes log output to stderr, not stdout (#353).
//
// rux previously installed spdlog on stdout, so a schema-migration warning
// (or any -v/-vv log line) was prepended to the JSON payload of a --json
// subcommand, breaking every `rux ... --json | parser` pipeline.
//
// This test runs `rux -vv info --json` against an empty project — at -vv the
// logger is at debug level, guaranteeing at least the "Verbosity level: debug"
// line is produced — then asserts:
//   a) stdout contains only valid JSON (no log lines),
//   b) stderr contains at least one spdlog-shaped line.

#include "../support/rux_executable.hpp"
#include "../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include <sys/wait.h>

namespace fs = std::filesystem;
using reusex::test_support::TempPath;

namespace {

struct SplitResult {
  int exit_code = -1;
  std::string out;
  std::string err;
};

/// Run `rux <args>` capturing stdout and stderr into separate temp files.
SplitResult run_rux_split(const std::string &args) {
  const TempPath out_file("rux_log_stderr_out", ".txt");
  const TempPath err_file("rux_log_stderr_err", ".txt");

  std::ostringstream cmd;
  cmd << reusex::test_support::rux_executable() << ' ' << args << " >"
      << out_file.path.string() << " 2>" << err_file.path.string();

  const int status = std::system(cmd.str().c_str()); // NOLINT

  SplitResult result;
  if (status == -1) {
    result.exit_code = -1;
  } else if (WIFEXITED(status)) {
    result.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    result.exit_code = 128 + WTERMSIG(status);
  }

  auto read_file = [](const fs::path &p) -> std::string {
    std::ifstream in(p);
    std::ostringstream buf;
    buf << in.rdbuf();
    return buf.str();
  };
  result.out = read_file(out_file.path);
  result.err = read_file(err_file.path);
  return result;
}

fs::path make_empty_project(const TempPath &tmp) {
  reusex::ProjectDB db(tmp.path);
  return tmp.path;
}

// spdlog pattern: [2026-09-21 12:34:56.789] [rux] [ info  ] …
const std::regex kLogLine{
    R"(\[\d{4}-\d{2}-\d{2}[ T][\d:.]+\]\s*\[[^\]]*\]\s*\[[^\]]*\])"};

bool has_log_line(const std::string &text) {
  return std::regex_search(text, kLogLine);
}

} // namespace

TEST_CASE("RuxCli_LogsGoToStderr_NotStdout", "[integration][cli][logging]") {
  const TempPath project("rux_log_stderr", ".rux");
  make_empty_project(project);

  // -vv raises the level to debug, guaranteeing log output is produced.
  const auto r =
      run_rux_split("-vv -p " + project.path.string() + " info --json");

  INFO("stdout:\n" << r.out);
  INFO("stderr:\n" << r.err);

  REQUIRE(r.exit_code == 0);

  // stdout must be parseable as JSON with no leading/trailing log lines.
  std::string trimmed = r.out;
  // Strip ANSI escapes before parsing (defensive; should not be on stdout).
  trimmed =
      std::regex_replace(trimmed, std::regex(R"(\x1b\[[0-9;]*[A-Za-z])"), "");
  CHECK_FALSE(has_log_line(r.out));
  nlohmann::json parsed;
  REQUIRE_NOTHROW(parsed = nlohmann::json::parse(trimmed));
  REQUIRE(parsed.is_object());

  // At -vv at least the "Verbosity level: …" info line must appear on stderr.
  CHECK(has_log_line(r.err));
}
