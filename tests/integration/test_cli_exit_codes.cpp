// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Process-level exit-code contract for the `rux` CLI (#299).
//
// Every other test in this tree calls a library entry point directly. This one
// deliberately does not: the bug it pins lives entirely in the seam between
// `run_subcommand_*` and `main()`. CLI11 takes subcommand callbacks as
// `std::function<void()>`, so a callback written as
//
//     sub->callback([opt, g]() { return run_subcommand_render(*opt, *g); });
//
// compiles happily and throws the status away. The subcommand logged its
// error, the stage did not run, and `rux` still exited 0. Nothing observable
// from inside the library distinguishes that from success — only the process
// exit code does — so the assertions here have to come from an actual fork and
// wait, against the actual binary.
//
// The cases are chosen to cover the two distinct failure routes plus both
// success routes, since a fix that makes *everything* non-zero would be just
// as broken as the bug:
//
//   - a stage refused by its input contract  (empty project -> create mesh)
//   - a subcommand that fails opening the DB (missing project -> info)
//   - an argument the library rejects at run time, after CLI11 has already
//     accepted the parse (render --layers bogus)
//   - a command that does no work at all     (--version)
//   - a command that does real work and succeeds (info on a real project)

#include "../support/rux_executable.hpp"
#include "../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <sys/wait.h>

namespace fs = std::filesystem;
using reusex::test_support::TempPath;

namespace {

struct CommandResult {
  int exit_code = -1;
  std::string output;
};

/// Run `rux <args>`, returning its exit status and combined stdout+stderr.
///
/// Output is captured to a temp file rather than inherited so that a failing
/// assertion can report exactly what the binary said, without the passing
/// cases spraying log lines through the ctest transcript.
CommandResult run_rux(const std::string &args) {
  const TempPath log("rux_exit_codes", ".log");

  std::ostringstream cmd;
  cmd << reusex::test_support::rux_executable() << ' ' << args << " > "
      << log.path.string() << " 2>&1";

  const int status = std::system(cmd.str().c_str());

  CommandResult result;
  if (status == -1) {
    result.exit_code = -1; // fork/exec never happened
  } else if (WIFEXITED(status)) {
    result.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    // Report the shell convention so a crash is legible in the failure text
    // rather than showing up as some arbitrary small integer.
    result.exit_code = 128 + WTERMSIG(status);
  }

  std::ifstream in(log.path);
  std::ostringstream buffer;
  buffer << in.rdbuf();
  result.output = buffer.str();
  return result;
}

/// Create an empty but valid project database on disk.
fs::path make_empty_project(const TempPath &tmp) {
  reusex::ProjectDB db(tmp.path);
  return tmp.path;
}

} // namespace

TEST_CASE("RuxCli_StageRefusedByInputContract_ExitsNonZero",
          "[integration][cli][exit_code]") {
  const TempPath project("rux_exit_codes_refused", ".rux");
  make_empty_project(project);

  // An empty project has no planes and no rooms, so `create mesh` must refuse
  // before doing any work. Before #299 this logged an error and exited 0.
  const auto result = run_rux("-p " + project.path.string() + " create mesh");

  INFO("rux output:\n" << result.output);
  CHECK(result.exit_code != 0);
  // A refusal is a clean failure, not a crash: anything >= 128 means we were
  // killed by a signal and the exit code is incidental.
  CHECK(result.exit_code < 128);
}

TEST_CASE("RuxCli_MissingProjectFile_ExitsNonZero",
          "[integration][cli][exit_code]") {
  // Deliberately a path that cannot exist rather than an unlinked temp file:
  // the point is that ProjectDB fails to open and `info` reports RuxError::IO.
  const auto result =
      run_rux("-p /nonexistent/reusex-issue-299/missing.rux info");

  INFO("rux output:\n" << result.output);
  CHECK(result.exit_code != 0);
  CHECK(result.exit_code < 128);
}

TEST_CASE("RuxCli_InvalidRenderLayer_ExitsNonZero",
          "[integration][cli][exit_code]") {
  const TempPath project("rux_exit_codes_render", ".rux");
  make_empty_project(project);
  const TempPath image("rux_exit_codes_render", ".png");

  // `--layers` is a free-form string as far as CLI11 is concerned, so the
  // parse succeeds and the vocabulary check happens inside
  // run_subcommand_render(). That check runs before the ProjectDB is opened
  // and before any rendering, so this case needs neither fixture data nor a
  // GPU.
  const auto result = run_rux("-p " + project.path.string() + " render -o " +
                              image.path.string() + " --layers bogus");

  INFO("rux output:\n" << result.output);
  CHECK(result.exit_code != 0);
  CHECK(result.exit_code < 128);
}

TEST_CASE("RuxCli_SuccessfulCommand_ExitsZero",
          "[integration][cli][exit_code]") {
  SECTION("--version does no work and succeeds") {
    const auto result = run_rux("--version");
    INFO("rux output:\n" << result.output);
    CHECK(result.exit_code == 0);
  }

  SECTION("info on a readable project succeeds") {
    const TempPath project("rux_exit_codes_info", ".rux");
    make_empty_project(project);

    const auto result = run_rux("-p " + project.path.string() + " info");
    INFO("rux output:\n" << result.output);
    CHECK(result.exit_code == 0);
  }
}
