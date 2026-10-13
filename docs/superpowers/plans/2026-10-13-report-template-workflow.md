# Report Template Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `apps/rux/resources/report.typ` the single, live-editable source of the Ressourcekortlægning PDF template: embedded at build time, overridable at run time, previewable in under a second against committed sample data or a real project's dumped data, with no C++ build in the loop.

**Architecture:** CMake reads `report.typ` into a generated header (same `file(READ)` + `configure_file` pattern as `engine-build.json`), replacing the pasted `kTypstTemplate`. `report_generator` gains a `ReportOptions` overload (template override + `$REUSEX_REPORT_TEMPLATE`) and `dump_report_data()`, which writes exactly the `data.json` + `images/` the PDF run sees. The template reads its data directory from `sys.inputs` so the devshell scripts can compile the repo's own template file against any data directory (`--root /`), while the generator keeps compiling in a temp dir with the default `"."`. A light `nix develop .#report` shell wraps `typst watch/compile` and `pdftoppm`; `rux export pdf` exposes both library calls.

**Tech Stack:** C++20, CMake, Catch2 v3, nlohmann_json, CLI11, spdlog, Typst 0.15.1, poppler-utils (pdftoppm), ImageMagick 7 (sample images only), Nix flakes.

**Spec:** `docs/superpowers/specs/2026-10-13-report-template-workflow-design.md`

## Global Constraints

- Work only in the worktree `/home/mephisto/repos/ReUseX__worktrees/pdf-template` (branch `pdf-template`). All paths below are relative to it.
- All builds/tests run inside `nix develop` (default shell). If `build/` does not exist yet: `cmake -B build -DCMAKE_BUILD_TYPE=Release` (the first build is long). Always run ctest with `--parallel`.
- `apps/rux/resources/report.typ` is the only copy of the template. No pasted copy in C++, ever again.
- The existing call `generate_ressourcekortlaegning_pdf(ProjectDB &, std::optional<std::int64_t>)` must keep compiling unchanged — ruxd (`/home/mephisto/repos/ruxd/src/api/edits.cpp:247`) calls it with an `std::optional<std::int64_t>` lvalue.
- New API, verbatim from the spec: `struct ReportOptions { std::optional<std::int64_t> resource_template_id; std::filesystem::path template_path; };`, `generate_ressourcekortlaegning_pdf(ProjectDB &, const ReportOptions &)`, `dump_report_data(ProjectDB &, const ReportOptions &, const std::filesystem::path &dir)`.
- "A template path that does not exist throws `std::runtime_error` naming it."
- Data layout is fixed: `<dir>/data.json`, `<dir>/images/<name>.jpg`. B keeps today's JSON field names unchanged (C extends them). No visual change to the report (that is sub-project C).
- Sample data: `apps/rux/resources/report-sample/` — `data.json` + ~8 JPEGs ≤ 60 kB each, synthetic (no third-party photos); ~15 resources over ~5 types, a few without an image, at least one very long name, one template with ≥ 15 fields, one draft state (`blocking > 0`), Danish characters everywhere.
- Devshell `devshells/report.nix`: light (no `inputsFrom` the C++ package); commands `report-watch`, `report-build`, `report-shots` (`writeShellScriptBin`), `menu` + motd like `devshells/completions.nix`; outputs under `shots/report/` (already gitignored by `/shots/` in `.gitignore:203` — verified, no change needed); fonts via `TYPST_FONT_PATHS`, no system fonts.
- Flakes only see git-tracked files: `git add devshells/report.nix` before `nix develop .#report`.
- `rux export pdf` is thin (STANDARDS §1): parse, call one library entry point, report.
- Every new source file carries an SPDX header (`// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `GPL-3.0-or-later`); the sample dir gets a `REUSE.toml` annotation.
- Test names follow `Subject_Scenario_ExpectedBehavior` (STANDARDS §7.1); temp paths only via `tests/support/temp_path.hpp`; tests that need `typst` `SKIP()` when it is not on PATH.
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **Re-dumping, or dumping into a non-empty directory** (`--dump-data .`, or the same DIR twice): a previous dump's `images/` must be replaced, never mixed with stale files; an unrelated non-empty directory must be refused and left untouched. → tests in Task 3.
2. **A stale `$REUSEX_REPORT_TEMPLATE` in ruxd's environment**: every PDF must fail with a message naming the variable and the path — never a silent fallback to the embedded template. → test in Task 2.
3. **Template override that is a directory** (e.g. `--template apps/rux/resources/`): refused before typst with the path in the message. → test in Task 2.
4. **Unknown `--resource-template` with `--dump-data`**: nothing is written (the directory is not even created) and the CLI exits with `INVALID_ARGUMENT`, not `IO`. → tests in Task 3 and Task 6.
5. **`-o` into a directory that does not exist**: fails before typst runs, leaves no file, exit `INVALID_ARGUMENT`. → test in Task 6.

(Also covered by a manual step in Task 5: `report-build` given a relative or missing DATA_DIR resolves it / exits 2 with a hint.)

---

## File Structure

| File | Responsibility |
|---|---|
| `libs/reusex/include/core/report_template_default.hpp.in` (new) | Template for the generated header holding `report.typ` verbatim |
| `libs/reusex/cmake/reusexLibrary.cmake` | Reads `report.typ`, guards the raw-string delimiter, configures the header, re-configures on edit |
| `libs/reusex/include/core/report_generator.hpp` | `ReportOptions`, new overload, `dump_report_data` |
| `libs/reusex/src/core/report_generator.cpp` | Template resolution, data writing shared by PDF + dump, typst call |
| `apps/rux/resources/report.typ` | Reads data dir from `sys.inputs`; header comment points to the guide |
| `apps/rux/resources/report-sample/` (new) | Committed sample `data.json` + 8 synthetic JPEGs |
| `scripts/report-sample-images.sh` (new) | Regenerates the synthetic JPEGs (provenance) |
| `REUSE.toml` | SPDX annotation for the sample dir |
| `devshells/report.nix` (new) | `nix develop .#report` |
| `apps/rux/include/export/pdf.hpp`, `apps/rux/src/export/pdf.cpp` (new) | `rux export pdf` |
| `apps/rux/src/export.cpp` | Registers `pdf`, footer |
| `tests/support/typst.hpp` (new) | `have_typst()`, `typst_compile_report()` shared by core + rux_app tests |
| `tests/unit/core/test_report_template.cpp` (new) | Embedding, overrides, dump, sample tests |
| `tests/unit/core/test_report_survey.cpp` | Drops `ReportTemplate_CopiesInSync` |
| `tests/unit/rux_app/test_export_pdf.cpp` (new) | CLI tests |
| `docs/guides/report-template.md` (new) | Data contract, preview loop, real data |
| `.claude/skills/design-studio/SKILL.md`, `references/pdf-report.md` (new) | PDF report design loop |
| `CLAUDE.md` | `export pdf` in the CLI table, `.#report` pointer |

---

### Task 1: Embed `report.typ` at build time; delete the pasted copy

**Files:**
- Create: `libs/reusex/include/core/report_template_default.hpp.in`
- Modify: `libs/reusex/cmake/reusexLibrary.cmake` (insert after the SAM3 block ending at line 196)
- Modify: `libs/reusex/src/core/report_generator.cpp:89-269` (delete `kTypstTemplate`), `:369-380` (`run_typst`), includes
- Modify: `apps/rux/resources/report.typ:32-33` (comment only)
- Create: `tests/unit/core/test_report_template.cpp`
- Modify: `tests/unit/core/test_report_survey.cpp:5-8, 20-28, 72-82, 185-199`

**Interfaces:**
- Consumes: nothing new.
- Produces: generated header `<reusex/core/report_template_default.hpp>` declaring `namespace reusex::core::detail { constexpr const char *kDefaultReportTemplate; }` — the exact bytes of `apps/rux/resources/report.typ`.

- [ ] **Step 1: Write the failing test**

Create `tests/unit/core/test_report_template.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The PDF report template workflow (report-template-workflow spec): the
// embedded template is the file on disk, the run-time overrides, the data
// dump, and the committed sample data.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/report_template_default.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace fs = std::filesystem;

namespace {
std::string slurp(const fs::path &path) {
  std::ifstream f(path, std::ios::binary);
  REQUIRE(f.good());
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>{}};
}
} // namespace

TEST_CASE("ReportTemplate_EmbeddedCopy_MatchesFileOnDisk", "[report][typst]") {
  // One read guards the CMake embedding: the generated header must hold
  // apps/rux/resources/report.typ byte for byte, comments included.
  const auto on_disk =
      slurp(fs::path(REUSEX_SOURCE_DIR) / "apps/rux/resources/report.typ");
  CHECK(std::string(reusex::core::detail::kDefaultReportTemplate) == on_disk);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --target reusex_unit_tests 2>&1 | tail -5`
Expected: FAIL — `fatal error: reusex/core/report_template_default.hpp: No such file or directory`

- [ ] **Step 3: Add the header template and the CMake embedding**

Create `libs/reusex/include/core/report_template_default.hpp.in`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// GENERATED by libs/reusex/cmake/reusexLibrary.cmake from
// apps/rux/resources/report.typ — do not edit the generated copy.

#pragma once

namespace reusex::core::detail {
/// apps/rux/resources/report.typ, verbatim: the Typst template
/// generate_ressourcekortlaegning_pdf() compiles unless a template file is
/// given (ReportOptions::template_path or $REUSEX_REPORT_TEMPLATE).
constexpr const char *kDefaultReportTemplate = R"reusextypst(@REUSEX_REPORT_TEMPLATE_TEXT@)reusextypst";
} // namespace reusex::core::detail
```

In `libs/reusex/cmake/reusexLibrary.cmake`, directly after the `configure_file(... engine_build_default.hpp ... @ONLY)` call (line 196), insert:

```cmake

# -----------------------------------------------
# Embed the PDF report's Typst template
# -----------------------------------------------
# apps/rux/resources/report.typ is the only copy of the Ressourcekortlægning
# template (report-template-workflow spec). Listing it in
# CMAKE_CONFIGURE_DEPENDS re-runs configure when it is edited, so the
# generated header — and report_generator.cpp — rebuild with it.
set(REUSEX_REPORT_TEMPLATE_TYP
    ${CMAKE_SOURCE_DIR}/apps/rux/resources/report.typ)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${REUSEX_REPORT_TEMPLATE_TYP})
file(READ ${REUSEX_REPORT_TEMPLATE_TYP} REUSEX_REPORT_TEMPLATE_TEXT)
string(FIND "${REUSEX_REPORT_TEMPLATE_TEXT}" ")reusextypst\"" _report_delim)
if(NOT _report_delim EQUAL -1)
    message(FATAL_ERROR
        "${REUSEX_REPORT_TEMPLATE_TYP} contains the raw-string delimiter "
        "')reusextypst\"' used to embed it; rename the delimiter in "
        "report_template_default.hpp.in")
endif()
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core/report_template_default.hpp.in
    ${CMAKE_BINARY_DIR}/generated/reusex/core/report_template_default.hpp
    @ONLY
)
```

(`cmake/Installation.cmake:76` installs everything under `generated/` with `*.hpp`; the header is installed alongside `version.hpp`, same as the SAM3 precedent. That is harmless.)

- [ ] **Step 4: Use the embedded template in the generator**

In `libs/reusex/src/core/report_generator.cpp`:

1. Add the include after `#include <reusex/core/logging.hpp>`:
   ```cpp
   #include <reusex/core/report_template_default.hpp>
   ```
2. Delete lines 89–269 (the `// ── Typst template (embedded) ──` heading through the closing `)typst";`).
3. In `run_typst`, replace `f << kTypstTemplate;` with:
   ```cpp
       f << core::detail::kDefaultReportTemplate;
   ```

- [ ] **Step 5: Update the template's own comment and drop the sync test**

In `apps/rux/resources/report.typ`, replace lines 32–33:

```
// The same text is embedded in libs/reusex/src/core/report_generator.cpp
// (kTypstTemplate); the ReportTemplate_CopiesInSync test fails when they differ.
```

with:

```
// This file is the only copy: CMake embeds it into the generated header
// reusex/core/report_template_default.hpp at build time.
```

In `tests/unit/core/test_report_survey.cpp`:
- lines 5–8, header comment: replace `the blocking count stored with a\n// version, that the two copies of the Typst template agree and — where typst\n// is installed — that the template compiles.` with `the blocking count stored with a\n// version and — where typst is installed — that the template compiles.`
- delete the `slurp` and `trim` helpers (lines 72–82) and the `#include <fstream>` / `#include <iterator>` lines.
- delete `TEST_CASE("ReportTemplate_CopiesInSync", ...)` (lines 185–199).

- [ ] **Step 6: Reconfigure, build, run**

Run:
```bash
cmake -B build && cmake --build build --target reusex_unit_tests && \
ctest --test-dir build -R 'ReportTemplate_|ReportPdf_|ReportSurvey' --output-on-failure --parallel $(nproc)
```
Expected: `ReportTemplate_EmbeddedCopy_MatchesFileOnDisk` PASS; `ReportPdf_*_Compiles` PASS (or SKIPPED without typst); no `ReportTemplate_CopiesInSync` listed.

Then prove the rebuild dependency: `echo '// probe' >> apps/rux/resources/report.typ && cmake --build build --target reusex_unit_tests 2>&1 | grep -c 'Re-running CMake\|report_generator'` → non-zero; then `git checkout apps/rux/resources/report.typ && cmake --build build --target reusex_unit_tests`.

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/core/report_template_default.hpp.in libs/reusex/cmake/reusexLibrary.cmake \
        libs/reusex/src/core/report_generator.cpp apps/rux/resources/report.typ \
        tests/unit/core/test_report_template.cpp tests/unit/core/test_report_survey.cpp
git commit -F - <<'EOF'
report: embed report.typ at build time, drop the pasted copy

apps/rux/resources/report.typ is now the only copy of the template; CMake
reads it into reusex/core/report_template_default.hpp (same pattern as
engine-build.json) and re-configures when it changes. ReportTemplate_
CopiesInSync is replaced by a byte-for-byte embedding check.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 2: `ReportOptions` — template override and `$REUSEX_REPORT_TEMPLATE`

**Files:**
- Modify: `libs/reusex/include/core/report_generator.hpp`
- Modify: `libs/reusex/src/core/report_generator.cpp` (includes; new `resolve_template`; `run_typst`; public API)
- Test: `tests/unit/core/test_report_template.cpp`

**Interfaces:**
- Consumes: `reusex::core::detail::kDefaultReportTemplate` (Task 1).
- Produces:
  ```cpp
  namespace reusex {
  struct ReportOptions {
    std::optional<std::int64_t> resource_template_id;
    std::filesystem::path template_path;
  };
  std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(ProjectDB &db, const ReportOptions &options);
  std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(ProjectDB &db, std::optional<std::int64_t> resource_template_id = std::nullopt);
  }
  ```
  Error contract: unknown template id → `std::out_of_range` (first); missing/non-file template → `std::runtime_error` whose `what()` contains the path and its origin (`ReportOptions::template_path` or `$REUSEX_REPORT_TEMPLATE`); both before any typst work. Typst runs with `--ignore-system-fonts`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/core/test_report_template.cpp`. Add these includes at the top (after the existing ones):

```cpp
#include <catch2/matchers/catch_matchers_string.hpp>

#include <core/ProjectDB.hpp>
#include <core/report_generator.hpp>

#include "../../support/temp_path.hpp"

#include <cstdint>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <vector>
```

Add to the anonymous namespace:

```cpp
using Catch::Matchers::ContainsSubstring;
using reusex::ProjectDB;
using reusex::test_support::TempDir;
using reusex::test_support::TempPath;

bool have_typst_on_path() {
  return std::system("command -v typst > /dev/null 2>&1") == 0;
}

/// Sets an environment variable for one scope. Each TEST_CASE runs in its
/// own ctest process, so this cannot leak into another test.
struct ScopedEnv {
  std::string name;
  ScopedEnv(std::string n, const std::string &value) : name(std::move(n)) {
    ::setenv(name.c_str(), value.c_str(), 1);
  }
  ~ScopedEnv() { ::unsetenv(name.c_str()); }
};

/// A template that fails compilation with @p marker in typst's error text:
/// proves which template file the generator actually compiled.
fs::path write_marker_template(const fs::path &dir, const std::string &marker) {
  const auto path = dir / (marker + ".typ");
  std::ofstream(path) << "#panic(\"" << marker << "\")\n";
  return path;
}
```

Append the test cases:

```cpp
TEST_CASE("ReportPdf_OldTwoArgumentOverload_StillExists", "[report]") {
  // ruxd calls generate_ressourcekortlaegning_pdf(db, optional<int64_t>);
  // taking the address with that exact type fails to compile if the
  // overload disappears or changes shape.
  using OldSig = std::vector<std::uint8_t> (*)(ProjectDB &,
                                               std::optional<std::int64_t>);
  const OldSig fn = &reusex::generate_ressourcekortlaegning_pdf;
  CHECK(fn != nullptr);
}

TEST_CASE("ReportPdf_MissingTemplateOverride_ThrowsNamingIt", "[report]") {
  TempPath tmp("test_report_template");
  ProjectDB db(tmp.path);
  reusex::ReportOptions opt;
  opt.template_path = "/nonexistent/reusex/report-override.typ";
  CHECK_THROWS_AS(reusex::generate_ressourcekortlaegning_pdf(db, opt),
                  std::runtime_error);
  CHECK_THROWS_WITH(reusex::generate_ressourcekortlaegning_pdf(db, opt),
                    ContainsSubstring("/nonexistent/reusex/report-override.typ"));
}

TEST_CASE("ReportPdf_TemplateOverrideIsDirectory_Throws", "[report]") {
  TempPath tmp("test_report_template");
  TempDir dir("test_report_template_dir");
  ProjectDB db(tmp.path);
  reusex::ReportOptions opt;
  opt.template_path = dir.path;
  CHECK_THROWS_WITH(reusex::generate_ressourcekortlaegning_pdf(db, opt),
                    ContainsSubstring(dir.path.string()));
}

TEST_CASE("ReportPdf_StaleTemplateEnvVar_ThrowsNamingVariable", "[report]") {
  // A stale $REUSEX_REPORT_TEMPLATE (e.g. in ruxd's environment) must fail
  // loudly, never fall back to the embedded template.
  TempPath tmp("test_report_template");
  ProjectDB db(tmp.path);
  ScopedEnv env("REUSEX_REPORT_TEMPLATE", "/nonexistent/reusex/env.typ");
  CHECK_THROWS_WITH(reusex::generate_ressourcekortlaegning_pdf(db),
                    ContainsSubstring("REUSEX_REPORT_TEMPLATE") &&
                        ContainsSubstring("/nonexistent/reusex/env.typ"));
}

TEST_CASE("ReportPdf_UnknownTemplateIdAndMissingFile_ReportsTheIdFirst",
          "[report]") {
  TempPath tmp("test_report_template");
  ProjectDB db(tmp.path);
  reusex::ReportOptions opt;
  opt.resource_template_id = 999;
  opt.template_path = "/nonexistent/reusex/report-override.typ";
  CHECK_THROWS_AS(reusex::generate_ressourcekortlaegning_pdf(db, opt),
                  std::out_of_range);
}

TEST_CASE("ReportPdf_TemplateOverride_IsTheFileCompiled", "[report][typst]") {
  if (!have_typst_on_path())
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempPath tmp("test_report_template");
  TempDir dir("test_report_template_dir");
  ProjectDB db(tmp.path);
  reusex::ReportOptions opt;
  opt.template_path = write_marker_template(dir.path, "option-marker");
  CHECK_THROWS_WITH(reusex::generate_ressourcekortlaegning_pdf(db, opt),
                    ContainsSubstring("option-marker"));
}

TEST_CASE("ReportPdf_TemplateEnvVar_IsTheFileCompiled", "[report][typst]") {
  if (!have_typst_on_path())
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempPath tmp("test_report_template");
  TempDir dir("test_report_template_dir");
  ProjectDB db(tmp.path);
  ScopedEnv env("REUSEX_REPORT_TEMPLATE",
                write_marker_template(dir.path, "env-marker").string());
  CHECK_THROWS_WITH(reusex::generate_ressourcekortlaegning_pdf(db),
                    ContainsSubstring("env-marker"));
}

TEST_CASE("ReportPdf_OptionAndEnvVar_OptionWins", "[report][typst]") {
  if (!have_typst_on_path())
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempPath tmp("test_report_template");
  TempDir dir("test_report_template_dir");
  ProjectDB db(tmp.path);
  ScopedEnv env("REUSEX_REPORT_TEMPLATE",
                write_marker_template(dir.path, "env-marker").string());
  reusex::ReportOptions opt;
  opt.template_path = write_marker_template(dir.path, "option-marker");
  CHECK_THROWS_WITH(reusex::generate_ressourcekortlaegning_pdf(db, opt),
                    ContainsSubstring("option-marker"));
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target reusex_unit_tests 2>&1 | grep -m3 error`
Expected: FAIL — `'ReportOptions' is not a member of 'reusex'`.

- [ ] **Step 3: Extend the header**

In `libs/reusex/include/core/report_generator.hpp`, add `#include <filesystem>` to the includes, update the file comment's last paragraph to:

```cpp
// Requires `typst` to be present in PATH at runtime. The devshells gain it via
// `pkgs.typst` (shell.nix, devshells/report.nix); in production supply it in
// the process's PATH. Typst runs with --ignore-system-fonts, so the PDF does
// not depend on the host's fonts.
```

and replace the `generate_ressourcekortlaegning_pdf` declaration block with:

```cpp
/// Options for generate_ressourcekortlaegning_pdf() and dump_report_data().
struct ReportOptions {
  /// Add a Ressourcetabel: every non-rejected resource through this
  /// template's keys (core::resource_report_section).
  std::optional<std::int64_t> resource_template_id;
  /// Typst template to use instead of the embedded one. Empty: the
  /// $REUSEX_REPORT_TEMPLATE environment variable, else embedded.
  std::filesystem::path template_path;
};

/// Generate a Ressourcekortlægning PDF for the given project.
///
/// Writes data.json + images/ (see dump_report_data) and the Typst template
/// into a temporary directory and executes:
///   typst compile report.typ out.pdf --root <tmpdir> --ignore-system-fonts
///
/// The template is ReportOptions::template_path, else the file named by
/// $REUSEX_REPORT_TEMPLATE, else apps/rux/resources/report.typ as embedded
/// at build time. A named template is copied into the temporary directory,
/// so it must be self-contained (no #import of sibling files).
///
/// @returns Raw PDF bytes on success.
/// @throws std::out_of_range when resource_template_id names no template.
/// @throws std::runtime_error when the named template file does not exist
///         or is not a regular file (the message names the path and whether
///         it came from template_path or $REUSEX_REPORT_TEMPLATE). Both are
///         checked before any typst work.
/// @throws std::runtime_error if typst is not in PATH, data assembly fails,
///         or the compilation exits non-zero (typst's error text included).
std::vector<std::uint8_t>
generate_ressourcekortlaegning_pdf(ProjectDB &db, const ReportOptions &options);

/// Backward-compatible form (ruxd): forwards to the ReportOptions overload
/// with only @p resource_template_id set — so $REUSEX_REPORT_TEMPLATE still
/// applies.
std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(
    ProjectDB &db,
    std::optional<std::int64_t> resource_template_id = std::nullopt);
```

- [ ] **Step 4: Implement template resolution and the new overload**

In `libs/reusex/src/core/report_generator.cpp`, add `#include <iterator>`, `#include <string_view>` and `#include <system_error>` to the std includes. In the anonymous namespace, after `ext_for_mime`, add:

```cpp
constexpr const char *kTemplateEnvVar = "REUSEX_REPORT_TEMPLATE";

/// The Typst source this run compiles: options.template_path, else the file
/// named by $REUSEX_REPORT_TEMPLATE, else the copy of
/// apps/rux/resources/report.typ embedded at build time. A named file that
/// is missing is an error — never a silent fallback to the embedded copy.
std::string resolve_template(const ReportOptions &options) {
  std::filesystem::path path = options.template_path;
  std::string origin = "ReportOptions::template_path";
  if (path.empty()) {
    const char *env = std::getenv(kTemplateEnvVar); // NOLINT(concurrency-mt-unsafe)
    if (env == nullptr || *env == '\0')
      return core::detail::kDefaultReportTemplate;
    path = env;
    origin = std::string("$") + kTemplateEnvVar;
  }
  std::error_code ec;
  if (!std::filesystem::is_regular_file(path, ec))
    throw std::runtime_error(fmt::format(
        "report template {} (from {}) does not exist or is not a regular file",
        path.string(), origin));
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error(fmt::format(
        "report template {} (from {}) cannot be read", path.string(), origin));
  reusex::info("report template: {} (from {})", path.string(), origin);
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>{}};
}
```

Change `run_typst` to take the template text and ignore system fonts:

```cpp
std::vector<std::uint8_t> run_typst(const std::filesystem::path &tmpdir,
                                    const std::string &template_text) {
  const auto typ_path = tmpdir / "report.typ";
  const auto out_path = tmpdir / "out.pdf";
  const auto err_path = tmpdir / "typst.stderr";

  {
    std::ofstream f(typ_path, std::ios::binary);
    f << template_text;
    if (!f.good())
      throw std::runtime_error(
          "report_generator: failed to write template to " + typ_path.string());
  }

  // --ignore-system-fonts: the PDF must come out the same in the devshell,
  // CI, ruxd and the nix package, so only fonts embedded in typst (or named
  // via $TYPST_FONT_PATHS) are used.
  const std::string cmd = "typst compile --ignore-system-fonts '" +
                          typ_path.string() + "' '" + out_path.string() +
                          "' --root '" + tmpdir.string() + "' 2>'" +
                          err_path.string() + "'";
```

(the rest of `run_typst` is unchanged).

Replace the public `generate_ressourcekortlaegning_pdf` definition with:

```cpp
std::vector<std::uint8_t>
generate_ressourcekortlaegning_pdf(ProjectDB &db, const ReportOptions &options) {
  // Everything bad input can break is checked before any typst work: an
  // unknown resource template first, then a missing template file.
  std::optional<core::ResourceReportSection> resources;
  if (options.resource_template_id)
    resources =
        core::resource_report_section(db, *options.resource_template_id);
  const std::string template_text = resolve_template(options);

  TempDir tmpdir;

  auto data = assemble_report_data(db, tmpdir.path);
  data["resources"] =
      resources ? resources_json(*resources) : nlohmann::json(nullptr);
  {
    std::ofstream f(tmpdir.path / "data.json");
    f << data.dump();
    if (!f.good())
      throw std::runtime_error("report_generator: failed to write data.json");
  }

  reusex::info(
      "generate_ressourcekortlaegning_pdf: invoking typst ({} materials, "
      "{} columns, {} resource table(s))",
      data.at("materials").size(), data.at("columns").size(),
      resources ? resources->tables.size() : 0);

  const auto pdf = run_typst(tmpdir.path, template_text);

  reusex::info("generate_ressourcekortlaegning_pdf: {} bytes", pdf.size());
  return pdf;
}

std::vector<std::uint8_t>
generate_ressourcekortlaegning_pdf(ProjectDB &db,
                                   std::optional<std::int64_t> template_id) {
  ReportOptions options;
  options.resource_template_id = template_id;
  return generate_ressourcekortlaegning_pdf(db, options);
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run:
```bash
cmake --build build --target reusex_unit_tests && \
ctest --test-dir build -R 'ReportPdf_|ReportTemplate_' --output-on-failure --parallel $(nproc)
```
Expected: all PASS (the three `[typst]` marker tests PASS inside `nix develop`, SKIP without typst). The pre-existing `ReportPdf_UnknownResourceTemplate_ThrowsBeforeTypst` (calls `generate(db, 999)`) still compiles and passes — proves the `int` argument still picks the old overload.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/core/report_generator.hpp libs/reusex/src/core/report_generator.cpp \
        tests/unit/core/test_report_template.cpp
git commit -F - <<'EOF'
report: ReportOptions with a template override and $REUSEX_REPORT_TEMPLATE

generate_ressourcekortlaegning_pdf(db, ReportOptions) compiles a template
file instead of the embedded one when asked; a missing file throws naming it
and where it came from. The (db, optional<int64_t>) form ruxd calls forwards
to it. Typst now runs with --ignore-system-fonts so output does not depend on
host fonts.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 3: `dump_report_data`, the `images/` layout, data-dir input, and the data contract doc

**Files:**
- Modify: `libs/reusex/include/core/report_generator.hpp` (declare `dump_report_data`)
- Modify: `libs/reusex/src/core/report_generator.cpp` (`assemble_report_data` → `write_report_data`; `dump_report_data`)
- Modify: `apps/rux/resources/report.typ` (header comment lines 5–33, `#let data` line 35, `#image(...)` line 147)
- Create: `docs/guides/report-template.md`
- Test: `tests/unit/core/test_report_template.cpp`

**Interfaces:**
- Consumes: `ReportOptions`, `resolve_template`, `run_typst(tmpdir, text)` (Task 2).
- Produces:
  ```cpp
  /// Write data.json + images/ into dir exactly as the PDF run sees them.
  void dump_report_data(ProjectDB &db, const ReportOptions &options, const std::filesystem::path &dir);
  ```
  Layout: `<dir>/data.json`, `<dir>/images/<material guid><ext>`; each material's `thumbnail_path` is `"images/<guid>.<ext>"` (was `"thumbnails/..."`). `dir` may be missing (created), empty, or a previous dump (has `data.json`; its `images/` is replaced); anything else → `std::runtime_error`. `options.template_path` is ignored. The template reads its data directory from `sys.inputs.at("data-dir", default: ".")`.

- [ ] **Step 1: Write the failing tests**

Add to the includes of `tests/unit/core/test_report_template.cpp`:

```cpp
#include "../../support/survey_fixture.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
```

Add to the anonymous namespace:

```cpp
/// Two passports (one with a JPEG thumbnail), one survey type, two parts.
void seed_report_project(ProjectDB &db) {
  using namespace reusex::test_support;
  make_passport(db, "guid-with-thumb");
  make_passport(db, "guid-without-thumb");
  db.set_material_thumbnail("guid-with-thumb", {0xFF, 0xD8, 0xFF, 0xD9},
                            "image/jpeg");
  const auto type = make_type(db, "Døre, massiv eg");
  make_part(db, "RX-001", type);
  make_part(db, "RX-002", type);
}

std::int64_t screening_template(const ProjectDB &db) {
  for (const auto &rec : db.resource_templates())
    if (rec.seed == std::optional<std::string>("screening"))
      return rec.id;
  FAIL("no seeded screening template");
  return 0;
}

std::vector<std::string> entries(const fs::path &dir) {
  std::vector<std::string> names;
  for (const auto &e : fs::directory_iterator(dir))
    names.push_back(e.path().filename().string());
  std::sort(names.begin(), names.end());
  return names;
}
```

Append the tests:

```cpp
TEST_CASE("ReportDump_ProjectWithThumbnail_WritesDataReferencingExistingFiles",
          "[report][dump]") {
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  seed_report_project(db);
  TempDir out("test_report_dump_dir");
  const auto dir = out.path / "dump"; // missing: the dump creates it

  reusex::dump_report_data(db, {}, dir);

  CHECK(entries(dir) == std::vector<std::string>{"data.json", "images"});
  const auto data = nlohmann::json::parse(slurp(dir / "data.json"));
  REQUIRE(data.at("materials").size() == 2);
  int with_image = 0;
  for (const auto &m : data.at("materials")) {
    if (!m.at("has_thumbnail").get<bool>())
      continue;
    ++with_image;
    const auto rel = m.at("thumbnail_path").get<std::string>();
    CHECK(rel.rfind("images/", 0) == 0);
    CHECK(fs::is_regular_file(dir / rel));
  }
  CHECK(with_image == 1);
  CHECK(data.at("resources").is_null());
  CHECK(data.at("survey").contains("rows"));
  CHECK(data.at("survey").contains("blocking"));
}

TEST_CASE("ReportDump_WithResourceTemplate_WritesItsTables", "[report][dump]") {
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  seed_report_project(db);
  TempDir out("test_report_dump_dir");
  reusex::ReportOptions opt;
  opt.resource_template_id = screening_template(db);

  reusex::dump_report_data(db, opt, out.path);

  const auto data = nlohmann::json::parse(slurp(out.path / "data.json"));
  const auto &tables = data.at("resources").at("tables");
  // 11 screening keys: Betegnelse + 10 others -> two stacked tables.
  REQUIRE(tables.size() == 2);
  CHECK(tables[0].at("headers")[0] == "Betegnelse");
}

TEST_CASE("ReportDump_UnknownResourceTemplate_ThrowsAndWritesNothing",
          "[report][dump]") {
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  TempDir out("test_report_dump_dir");
  const auto dir = out.path / "dump";
  reusex::ReportOptions opt;
  opt.resource_template_id = 999;
  CHECK_THROWS_AS(reusex::dump_report_data(db, opt, dir), std::out_of_range);
  CHECK_FALSE(fs::exists(dir));
}

TEST_CASE("ReportDump_ForeignNonEmptyDirectory_RefusedAndUntouched",
          "[report][dump]") {
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  TempDir out("test_report_dump_dir");
  std::ofstream(out.path / "notes.txt") << "mine";
  CHECK_THROWS_WITH(reusex::dump_report_data(db, {}, out.path),
                    ContainsSubstring(out.path.string()));
  CHECK(entries(out.path) == std::vector<std::string>{"notes.txt"});
}

TEST_CASE("ReportDump_IntoPreviousDump_ReplacesItsImages", "[report][dump]") {
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  seed_report_project(db);
  TempDir out("test_report_dump_dir");
  reusex::dump_report_data(db, {}, out.path);
  std::ofstream(out.path / "images" / "stale.jpg") << "old";

  reusex::dump_report_data(db, {}, out.path);

  CHECK_FALSE(fs::exists(out.path / "images" / "stale.jpg"));
  CHECK(fs::is_regular_file(out.path / "images" / "guid-with-thumb.jpg"));
}

TEST_CASE("ReportPdf_ProjectWithThumbnail_Compiles", "[report][typst]") {
  // The PDF run uses the same images/ layout and the template's default
  // data-dir "." — an image reference that resolves wrongly fails here.
  if (!have_typst_on_path())
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  // A real 1x1 baseline greyscale JPEG (159 bytes, made with ImageMagick):
  // typst decodes images, so marker bytes alone would fail the compile.
  reusex::test_support::make_passport(db, "guid-real-jpeg");
  const std::vector<std::uint8_t> jpeg = {
      0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01,
      0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43,
      0x00, 0x10, 0x0B, 0x0C, 0x0E, 0x0C, 0x0A, 0x10, 0x0E, 0x0D, 0x0E, 0x12,
      0x11, 0x10, 0x13, 0x18, 0x28, 0x1A, 0x18, 0x16, 0x16, 0x18, 0x31, 0x23,
      0x25, 0x1D, 0x28, 0x3A, 0x33, 0x3D, 0x3C, 0x39, 0x33, 0x38, 0x37, 0x40,
      0x48, 0x5C, 0x4E, 0x40, 0x44, 0x57, 0x45, 0x37, 0x38, 0x50, 0x6D, 0x51,
      0x57, 0x5F, 0x62, 0x67, 0x68, 0x67, 0x3E, 0x4D, 0x71, 0x79, 0x70, 0x64,
      0x78, 0x5C, 0x65, 0x67, 0x63, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x01,
      0x00, 0x01, 0x01, 0x01, 0x11, 0x00, 0xFF, 0xC4, 0x00, 0x14, 0x00, 0x01,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x03, 0xFF, 0xC4, 0x00, 0x14, 0x10, 0x01, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x3F, 0x00,
      0x77, 0xFF, 0xD9};
  db.set_material_thumbnail("guid-real-jpeg", jpeg, "image/jpeg");
  const auto pdf = reusex::generate_ressourcekortlaegning_pdf(db);
  REQUIRE(pdf.size() > 4);
  CHECK(std::string(pdf.begin(), pdf.begin() + 4) == "%PDF");
}
```

(The byte array was verified to decode with typst 0.15.1 while writing this plan; an arithmetic-coded JPEG does not — typst rejects `DAC` headers.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target reusex_unit_tests 2>&1 | grep -m3 error`
Expected: FAIL — `'dump_report_data' is not a member of 'reusex'`.

- [ ] **Step 3: Declare `dump_report_data`**

In `libs/reusex/include/core/report_generator.hpp`, after the two `generate_ressourcekortlaegning_pdf` declarations:

```cpp
/// Write data.json + images/ into dir exactly as the PDF run sees them.
///
/// The result is what `report-watch DIR` (nix develop .#report) and
/// `rux export pdf --dump-data DIR` work with; the format is documented in
/// docs/guides/report-template.md. ReportOptions::template_path is ignored.
///
/// @p dir may be missing (it is created), empty, or a previous dump (it
/// holds a data.json; its images/ is replaced, never merged).
/// @throws std::out_of_range when resource_template_id names no template
///         (nothing is written, @p dir is not created).
/// @throws std::runtime_error when @p dir is a file, or a non-empty
///         directory without a data.json (left untouched), or on I/O errors.
void dump_report_data(ProjectDB &db, const ReportOptions &options,
                      const std::filesystem::path &dir);
```

- [ ] **Step 4: Implement — one writer shared by the PDF run and the dump**

In `libs/reusex/src/core/report_generator.cpp`, rename `assemble_report_data(ProjectDB &db, const std::filesystem::path &tmpdir)` to the function below. Its body is today's body with three changes: the directory is `dir / "images"`, the relative name is `"images/" + guid + ext`, and it adds `resources` and writes `data.json` itself:

```cpp
/// Write the report's data directory: data.json plus images/<guid><ext> for
/// every material with a thumbnail. Shared by the PDF run (into a temp dir)
/// and dump_report_data, so both see byte-identical input.
nlohmann::json
write_report_data(ProjectDB &db,
                  const std::optional<core::ResourceReportSection> &resources,
                  const std::filesystem::path &dir) {
  const auto summary = db.project_summary();
  const auto defs = db.list_property_definitions();

  std::string project_name;
  if (!summary.projects.empty() && !summary.projects.front().name.empty())
    project_name = summary.projects.front().name;
  else
    project_name = summary.path.stem().string();

  nlohmann::json data;
  data["project_name"] = project_name;
  data["generated_at"] = now_iso8601_rg();

  nlohmann::json cols = nlohmann::json::array();
  for (const auto &d : defs)
    cols.push_back({{"id", d.id}, {"name", d.name}, {"type", d.type}});
  data["columns"] = std::move(cols);

  const auto image_dir = dir / "images";
  std::filesystem::create_directories(image_dir);

  nlohmann::json mats = nlohmann::json::array();
  for (const auto &info : summary.materials) {
    const auto &guid = info.guid;

    auto thumb_opt = db.material_thumbnail(guid);
    bool has_thumb = thumb_opt.has_value();
    std::string thumb_path;

    if (has_thumb) {
      const auto &[blob, mime] = *thumb_opt;
      const std::string fname = "images/" + guid + ext_for_mime(mime);
      std::ofstream f(dir / fname, std::ios::binary);
      f.write(reinterpret_cast<const char *>(blob.data()),
              static_cast<std::streamsize>(blob.size()));
      if (f.good())
        thumb_path = fname;
      else
        has_thumb = false;
    }

    const auto props_map = db.passport_stored_properties(guid);
    nlohmann::json props = nlohmann::json::object();
    for (const auto &[k, v] : props_map)
      props[k] = v;

    mats.push_back({{"guid", guid},
                    {"has_thumbnail", has_thumb},
                    {"thumbnail_path", thumb_path},
                    {"properties", std::move(props)}});
  }
  data["materials"] = std::move(mats);

  // (keep today's survey block here verbatim — the comment and loop from
  //  "// Survey (Kortlægning): reportable types only" through
  //  data["survey"] = {...};)

  data["resources"] =
      resources ? resources_json(*resources) : nlohmann::json(nullptr);
  {
    std::ofstream f(dir / "data.json");
    f << data.dump(2);
    if (!f.good())
      throw std::runtime_error("report_generator: failed to write " +
                               (dir / "data.json").string());
  }
  return data;
}
```

(The survey block is the existing code at today's lines 335–363; move it unchanged. `dump(2)` — indented, so a dumped `data.json` is readable when hand-editing for design rounds.)

In `generate_ressourcekortlaegning_pdf(ProjectDB &, const ReportOptions &)`, replace the `auto data = assemble_report_data(...)` line, the `data["resources"] = ...` line and the `data.json` writing block with:

```cpp
  const auto data = write_report_data(db, resources, tmpdir.path);
```

Add the public function after `generate_ressourcekortlaegning_pdf(ProjectDB &, std::optional<std::int64_t>)`:

```cpp
void dump_report_data(ProjectDB &db, const ReportOptions &options,
                      const std::filesystem::path &dir) {
  namespace fs = std::filesystem;
  // An unknown template fails before anything touches the disk.
  std::optional<core::ResourceReportSection> resources;
  if (options.resource_template_id)
    resources =
        core::resource_report_section(db, *options.resource_template_id);

  std::error_code ec;
  if (fs::exists(dir, ec)) {
    if (!fs::is_directory(dir, ec))
      throw std::runtime_error(fmt::format(
          "dump_report_data: {} exists and is not a directory", dir.string()));
    const bool previous_dump = fs::is_regular_file(dir / "data.json", ec);
    if (!previous_dump && !fs::is_empty(dir, ec))
      throw std::runtime_error(fmt::format(
          "dump_report_data: {} is not empty and holds no data.json; refusing "
          "to write a report dump into it",
          dir.string()));
    // A previous dump's images: replace, never mix in stale files.
    fs::remove_all(dir / "images", ec);
    if (ec)
      throw std::runtime_error(fmt::format(
          "dump_report_data: cannot clear {}: {}",
          (dir / "images").string(), ec.message()));
  }
  fs::create_directories(dir);

  const auto data = write_report_data(db, resources, dir);
  reusex::info("dump_report_data: wrote {} ({} materials, {} resource "
               "table(s))",
               dir.string(), data.at("materials").size(),
               resources ? resources->tables.size() : 0);
}
```

- [ ] **Step 5: Make the template read its data directory from `sys.inputs`**

In `apps/rux/resources/report.typ`, replace the header comment block (lines 5–33, from `// Ressourcekortlægning (Material Resource Mapping) report template.` through the two comment lines Task 1 wrote) with:

```
// Ressourcekortlægning (Material Resource Mapping) report template.
//
// Data contract (data.json + images/): docs/guides/report-template.md.
//
// The data directory comes from `--input data-dir=DIR`; it defaults to the
// directory holding this file, which is how the generator runs it:
//   typst compile report.typ out.pdf --root <tmpdir>                    (generator)
//   typst compile report.typ out.pdf --root / --input data-dir=<dir>    (report-build)
// Typst refuses a source file outside --root (symlinks are resolved first),
// so the preview scripts use --root / rather than linking this file into
// the data directory.
//
// This file is the only copy: CMake embeds it into the generated header
// reusex/core/report_template_default.hpp at build time.
```

Replace `#let data = json("data.json")` with:

```
#let data_dir = sys.inputs.at("data-dir", default: ".")
#let data = json(data_dir + "/data.json")
```

and in the material table replace `#image(mat.thumbnail_path, width: 2.8cm, height: 2.2cm, fit: "contain")` with:

```
        #image(data_dir + "/" + mat.thumbnail_path, width: 2.8cm, height: 2.2cm, fit: "contain")
```

(Both invocations were verified with typst 0.15.1 while writing this plan.)

- [ ] **Step 6: Write the data contract doc**

Create `docs/guides/report-template.md`:

````markdown
# PDF report template (Ressourcekortlægning)

`apps/rux/resources/report.typ` is the only copy of the template. CMake
embeds it into `reusex/core/report_template_default.hpp` at build time;
`generate_ressourcekortlaegning_pdf()` compiles that embedded copy unless a
template file is named by `ReportOptions::template_path`
(`rux export pdf --template FILE.typ`) or `$REUSEX_REPORT_TEMPLATE`. A named
file that does not exist is an error, never a silent fallback.

## Data contract

A report data directory holds:

```
<dir>/data.json
<dir>/images/<name>.jpg      (or .png / .webp / .gif, from the stored MIME type)
```

The template reads it from `sys.inputs.at("data-dir", default: ".")`. The
generator writes it into a temp dir next to the template (default `"."`);
the preview scripts pass `--root / --input data-dir=<abs dir>`.
`rux export pdf --dump-data DIR` writes exactly what the PDF run sees.

`data.json`:

| Field | Type | Meaning |
|---|---|---|
| `project_name` | string | First project's name, else the `.rux` file stem |
| `generated_at` | string | UTC ISO 8601 (`2026-10-13T09:30:00Z`) |
| `columns[]` | `{id, name, type}` | User-defined passport property columns |
| `materials[]` | `{guid, has_thumbnail, thumbnail_path, properties}` | One per material passport; `thumbnail_path` is relative to the data dir (`images/<guid>.jpg`) or `""`; `properties` is keyed by **column name** |
| `survey.rows[]` | `{name, bim7aa, eak, quantity, mass, treatment, environment}` | Reportable (approved, not withheld) survey types; all values preformatted Danish strings (`"620 m²"`, `"190 t"`, `"—"`) |
| `survey.circularity[]` | `{label, tonnes}` | Tonnes per treatment over exactly those rows |
| `survey.blocking` | int | Types keeping the report a draft (> 0 = draft) |
| `resources` | `null` or `{name, tables[]}` | Ressourcetabel through a resource template; each table `{headers[], rows[][]}`, at most 8 columns, each led by `Betegnelse` |

Every value the template prints comes from data, so names containing Typst
markup (`#panic(...)`, `*x*`) print literally.

## Fonts

Typst runs with `--ignore-system-fonts` everywhere (generator and preview
scripts), so the PDF does not depend on the host. Today the template uses
only fonts embedded in the typst binary (Libertinus Serif, New Computer
Modern, DejaVu Sans Mono). The `.#report` shell also puts the Qt client's
bundled TTFs (`apps/rux/qt/resources/fonts`: Archivo, Oswald, JetBrains Mono,
OFL-1.1) on `TYPST_FONT_PATHS`, but the generator inside ruxd / the nix
package does not see them yet — a template that uses them must first make
the generator pass a font path, or ruxd's PDF silently falls back.
````

- [ ] **Step 7: Run tests to verify they pass**

Run:
```bash
cmake -B build && cmake --build build --target reusex_unit_tests && \
ctest --test-dir build -R 'ReportDump_|ReportPdf_|ReportTemplate_' --output-on-failure --parallel $(nproc)
```
Expected: all PASS (typst ones SKIP outside the devshell). `ReportTemplate_EmbeddedCopy_MatchesFileOnDisk` still passes — the edited template was re-embedded.

- [ ] **Step 8: Commit**

```bash
git add libs/reusex/include/core/report_generator.hpp libs/reusex/src/core/report_generator.cpp \
        apps/rux/resources/report.typ docs/guides/report-template.md tests/unit/core/test_report_template.cpp
git commit -F - <<'EOF'
report: dump_report_data and the data-dir/images layout

dump_report_data() writes data.json + images/ exactly as the PDF run sees
them (one shared writer). Thumbnails move from thumbnails/ to images/; field
names are unchanged. The template reads its data dir from
--input data-dir=..., defaulting to "." for the generator. The data contract
is documented once, in docs/guides/report-template.md.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 4: Committed sample data that compiles

**Files:**
- Create: `scripts/report-sample-images.sh`
- Create: `apps/rux/resources/report-sample/data.json`
- Create: `apps/rux/resources/report-sample/images/sample-0{1..8}.jpg` (generated)
- Modify: `REUSE.toml` (append annotation)
- Create: `tests/support/typst.hpp`
- Test: `tests/unit/core/test_report_template.cpp`

**Interfaces:**
- Consumes: template `data-dir` input (Task 3).
- Produces: `tests/support/typst.hpp`:
  ```cpp
  namespace reusex::test_support {
  bool have_typst();
  /// typst compile <REUSEX_SOURCE_DIR>/apps/rux/resources/report.typ OUT
  ///   --ignore-system-fonts --root / --input data-dir=<abs DATA_DIR>
  /// stderr -> LOG. Returns std::system's status (0 = success).
  int typst_compile_report(const std::filesystem::path &data_dir,
                           const std::filesystem::path &out_pdf,
                           const std::filesystem::path &log);
  }
  ```
  and the sample at `apps/rux/resources/report-sample/` (default `DATA_DIR` of Task 5).

- [ ] **Step 1: Write the shared typst helper and the failing tests**

Create `tests/support/typst.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Compile the repo's report template against a report data directory the
// way `report-build` (nix develop .#report) does. Needs REUSEX_SOURCE_DIR,
// which both unit test binaries define.
// Include as "../../support/typst.hpp" from tests/unit/<module>/.

#include <cstdlib>
#include <filesystem>
#include <string>

namespace reusex::test_support {

inline bool have_typst() {
  return std::system("command -v typst > /dev/null 2>&1") == 0;
}

inline int typst_compile_report(const std::filesystem::path &data_dir,
                                const std::filesystem::path &out_pdf,
                                const std::filesystem::path &log) {
  const std::string tmpl =
      std::string(REUSEX_SOURCE_DIR) + "/apps/rux/resources/report.typ";
  const std::string cmd =
      "typst compile --ignore-system-fonts --root / --input 'data-dir=" +
      std::filesystem::absolute(data_dir).string() + "' '" + tmpl + "' '" +
      out_pdf.string() + "' 2>'" + log.string() + "'";
  return std::system(cmd.c_str());
}

} // namespace reusex::test_support
```

Add `#include "../../support/typst.hpp"` and `#include <set>` to `tests/unit/core/test_report_template.cpp`, then append:

```cpp
namespace {
const fs::path kSampleDir =
    fs::path(REUSEX_SOURCE_DIR) / "apps/rux/resources/report-sample";
}

TEST_CASE("ReportSample_DataJson_ExercisesTheTemplate", "[report][sample]") {
  // The spec's checklist for the committed sample, so it cannot quietly
  // shrink into something that no longer stresses the template.
  const auto text = slurp(kSampleDir / "data.json");
  const auto data = nlohmann::json::parse(text);

  const auto &mats = data.at("materials");
  CHECK(mats.size() >= 15);
  int without_image = 0;
  std::size_t longest_name = 0;
  std::set<std::string> referenced;
  for (const auto &m : mats) {
    longest_name = std::max(
        longest_name,
        m.at("properties").at("Betegnelse").get<std::string>().size());
    if (!m.at("has_thumbnail").get<bool>()) {
      ++without_image;
      continue;
    }
    const auto rel = m.at("thumbnail_path").get<std::string>();
    referenced.insert(rel);
    REQUIRE(fs::is_regular_file(kSampleDir / rel));
    CHECK(fs::file_size(kSampleDir / rel) <= 60 * 1024);
  }
  CHECK(without_image >= 2);
  CHECK(longest_name >= 100); // one very long name (bytes, UTF-8)

  // No unreferenced images: every file in images/ is used.
  for (const auto &e : fs::directory_iterator(kSampleDir / "images"))
    CHECK(referenced.count("images/" + e.path().filename().string()) == 1);

  CHECK(data.at("survey").at("rows").size() >= 5);
  CHECK(data.at("survey").at("blocking").get<int>() > 0); // draft state

  // One template with many fields: Betegnelse + >= 15 others.
  std::set<std::string> keys;
  for (const auto &t : data.at("resources").at("tables"))
    for (const auto &h : t.at("headers"))
      keys.insert(h.get<std::string>());
  CHECK(keys.size() >= 16);

  for (const char *ch : {"æ", "ø", "å", "Æ", "Ø"})
    CHECK(text.find(ch) != std::string::npos);
}

TEST_CASE("ReportSample_CommittedData_Compiles", "[report][typst][sample]") {
  if (!reusex::test_support::have_typst())
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempDir out("test_report_sample");
  const auto pdf = out.path / "sample.pdf";
  const auto log = out.path / "typst.stderr";
  const int rc = reusex::test_support::typst_compile_report(kSampleDir, pdf, log);
  INFO(slurp(log));
  REQUIRE(rc == 0);
  CHECK(slurp(pdf).rfind("%PDF", 0) == 0);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
cmake --build build --target reusex_unit_tests && \
ctest --test-dir build -R 'ReportSample_' --output-on-failure --parallel $(nproc)
```
Expected: FAIL — `ReportSample_DataJson_ExercisesTheTemplate` fails at `REQUIRE( f.good() )` (no `data.json`); `ReportSample_CommittedData_Compiles` fails (typst: file not found).

- [ ] **Step 3: Generate the synthetic images**

Create `scripts/report-sample-images.sh` (and `chmod +x` it):

```bash
#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Regenerates the synthetic placeholder photos of the PDF report sample
# (apps/rux/resources/report-sample/images/): labelled gradients, no
# third-party content. Needs ImageMagick 7 (`magick`), e.g. from
# `nix develop .#report` or `nix shell --inputs-from . nixpkgs#imagemagick`.
set -euo pipefail
root=$(git rev-parse --show-toplevel)
out="${1:-$root/apps/rux/resources/report-sample/images}"
font="$root/apps/rux/qt/resources/fonts/Archivo-SemiBold.ttf"
mkdir -p "$out"
# name  size  top-colour  bottom-colour  label
shot() {
  magick -size "$2" "gradient:$3-$4" -font "$font" -fill '#1d1d1b' \
    -pointsize 30 -gravity center -annotate +0+0 "$5" \
    -quality 72 -strip "$out/$1.jpg"
}
shot sample-01 640x480 '#b9893f' '#e7cf96' 'Mursten, gule\nRX-001'
shot sample-02 640x480 '#7a5a3a' '#c9a77c' 'Tagspær, fyr\nRX-002'
shot sample-03 480x640 '#5b4027' '#a8845c' 'Indvendig dør\nmassiv eg\nRX-003'
shot sample-04 640x400 '#c08a52' '#ead2b0' 'Gulvbrædder, bøg\nRX-004'
shot sample-05 640x480 '#6d6f73' '#c4c6c9' 'Radiator,\nstøbejern\nRX-005'
shot sample-06 480x640 '#e4e9ec' '#8fa3ad' 'Vindue,\nkoblede rammer\nRX-006'
shot sample-07 800x450 '#9a9a96' '#d8d8d2' 'Facadeelement,\nsandwich\nRX-007'
shot sample-08 640x480 '#f4f4f0' '#bfc4c0' 'Køkkenlåger\nRX-008'
echo "wrote 8 images to $out"
```

Run: `nix shell --inputs-from . nixpkgs#imagemagick -c scripts/report-sample-images.sh && ls -l apps/rux/resources/report-sample/images`
Expected: 8 files, each ~6–9 kB (portrait, landscape and wide sizes on purpose: they exercise `fit: "contain"`).

- [ ] **Step 4: Write `data.json` (one-off generator; the JSON is hand-maintained afterwards)**

Run from the repo root (the script is not committed — from now on edit `data.json` by hand):

```bash
python3 - apps/rux/resources/report-sample/data.json <<'EOF'
import json, sys, pathlib
out = pathlib.Path(sys.argv[1])
cols = [("col-betegnelse", "Betegnelse", "text"), ("col-materiale", "Materiale", "text"),
        ("col-maengde", "Mængde", "number"), ("col-tilstand", "Tilstand", "text"),
        ("col-placering", "Placering", "text"), ("col-bemaerkning", "Bemærkning", "text")]
# (betegnelse, materiale, mængde, tilstand, placering, bemærkning, type index)
res = [
  ("Mursten, gule blødstrøgne", "Tegl", "1240 m²", "God", "Facade mod gård", "Kalkmørtel, let at nedbryde", 0),
  ("Tagspær, fyr", "Træ", "86 stk", "Middel", "Tagetage", "Enkelte spær med råd i fodremmen", 1),
  ("Indvendige døre, massiv eg", "Træ", "42 stk", "God", "1.–3. sal", "Originale greb bevaret", 2),
  ("Gulvbrædder, bøg", "Træ", "310 m²", "Middel", "Stuer, 2. sal", "Slidte ved dørtrin", 1),
  ("Radiatorer, støbejern", "Jern og stål", "38 stk", "God", "Alle etager", "Trykprøves før genbrug", 3),
  ("Vinduer, trærammer med koblede rammer", "Træ og glas", "22 stk", "Dårlig", "Gadefacade", "Kitfalse skal prøvetages", 2),
  ("Facadeelementer, sandwich", "Beton", "64 stk", "God", "Gavl øst", "Løftebeslag intakte", 4),
  ("Køkkenlåger, hvidlakerede", "Træ", "57 stk", "God", "Køkkener, 1.–4. sal", "Hængsler genbruges ikke", 2),
  ("Betonhuldæk", "Beton", "412 m²", "God", "Etagedæk", "Skæres i fulde elementer", 4),
  ("Stålbjælker, HE200B", "Jern og stål", "18 stk", "God", "Kælder", "Brandmaling skal fjernes", 3),
  ("Gipsvægge på stålskelet", "Gips", "260 m²", "Middel", "Kontorer, stueetage", "Kun skelet genbruges", 1),
  ("Ældre loftsplader af træuld med maling, formodet indhold af PCB og bly i overfladebehandlingen — skal prøvetages før genbrug",
   "Træuld", "145 m²", "Dårlig", "Trappeopgang", "Afventer prøvesvar fra laboratoriet", 1),
  ("Fliser, glaserede (badeværelser på 1.–3. sal)", "Tegl og keramik", "96 m²", "Middel", "Badeværelser", "Fuger med mulig asbest", 0),
  ("Trappeløb, terrazzo", "Beton", "6 stk", "God", "Hovedtrappe", "Løftes som hele løb", 4),
  ("Brandmandsdøre, stål", "Jern og stål", "9 stk", "God", "Bagtrappe", "Godkendelse skal fornyes", 3),
]
types = [
  ("Mursten og fliser", "211 Ydervægge", "17.01.02", "1336 m²", "148,8 t", "Genbrug", "Ren (prøvesvar)"),
  ("Træ, konstruktivt og gulve", "251 Etagedæk", "17.02.01", "86 m³", "43 t", "Genanvendelse", "Ren"),
  ("Døre, vinduer og låger", "231 Vinduer, døre", "17.02.02", "121 stk", "5,1 t", "Genbrug", "Ren"),
  ("Stål og radiatorer", "223 Bjælker", "17.04.05", "65 stk", "12,6 t", "Genanvendelse", "Ren"),
  ("Beton, huldæk og trapper", "251 Etagedæk", "17.01.01", "412 m²", "196,8 t", "Nyttiggørelse", "Ren (prøvesvar)"),
]
producers = ["Ukendt", "Hasle Klinker", "Junckers", "Dansk Stål A/S", "Spæncom"]
headers = ["Betegnelse", "Kode", "Type", "BIM7AA", "EAK", "Mængde", "Enhed", "Tons",
           "Tilstand", "Placering", "Etage", "Behandling", "Miljø", "Producent",
           "Produktionsår", "Dimensioner"]
rows = []
for i, r in enumerate(res):
    qty, unit = r[2].split(" ")
    t = types[r[6]]
    rows.append([r[0], f"RX-{i+1:03d}", t[0], t[1], t[2], qty, unit,
                 f"{(i * 37 % 90) / 10 + 0.4:.1f}".replace(".", ","), r[3], r[4],
                 ["Kælder", "Stue", "1. sal", "2. sal", "3. sal"][i % 5], t[5], t[6],
                 producers[i % 5], str(1931 + i * 3), f"{600 + i * 20} × {300 + i * 10} mm"])
# Same chunking as core::resource_report_section: <= 8 columns, Betegnelse first.
tables, others = [], headers[1:]
for k in range(0, len(others), 7):
    idx = [0] + [1 + k + j for j in range(min(7, len(others) - k))]
    tables.append({"headers": [headers[j] for j in idx],
                   "rows": [[row[j] for j in idx] for row in rows]})
mats = []
for i, r in enumerate(res):
    guid = f"sample-{i+1:02d}"
    has = i < 8
    mats.append({"guid": guid, "has_thumbnail": has,
                 "thumbnail_path": f"images/{guid}.jpg" if has else "",
                 "properties": {c[1]: v for c, v in zip(cols, r[:6])}})
data = {
  "project_name": "Prøvegade 12, København Ø — nedrivning af baghus",
  "generated_at": "2026-10-13T09:30:00Z",
  "columns": [{"id": c[0], "name": c[1], "type": c[2]} for c in cols],
  "materials": mats,
  "survey": {
    "rows": [{"name": t[0], "bim7aa": t[1], "eak": t[2], "quantity": t[3], "mass": t[4],
              "treatment": t[5], "environment": t[6]} for t in types],
    "circularity": [{"label": "Genbrug", "tonnes": "153,9 t"},
                    {"label": "Genanvendelse", "tonnes": "55,6 t"},
                    {"label": "Nyttiggørelse", "tonnes": "196,8 t"}],
    "blocking": 2,
  },
  "resources": {"name": "Fuld registrering (alle felter)", "tables": tables},
}
out.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(f"wrote {out}: {len(mats)} materials, tables {[len(t['headers']) for t in tables]}")
EOF
```

Expected output: `... 15 materials, tables [8, 8, 2]`.

- [ ] **Step 5: SPDX annotation**

Append to `REUSE.toml`:

```toml

# Synthetic sample data for the PDF report template
# (apps/rux/resources/report.typ): data.json is invented, the JPEGs are
# labelled gradients made by scripts/report-sample-images.sh. JSON cannot
# carry an SPDX header.
[[annotations]]
path = "apps/rux/resources/report-sample/**"
SPDX-FileCopyrightText = "2026 Povl Filip Sonne-Frederiksen"
SPDX-License-Identifier = "GPL-3.0-or-later"
```

Run: `reuse lint 2>&1 | tail -3`
Expected: `Congratulations! Your project is compliant ...`

- [ ] **Step 6: Run tests to verify they pass**

Run:
```bash
ctest --test-dir build -R 'ReportSample_' --output-on-failure --parallel $(nproc)
```
Expected: both PASS (inside `nix develop`, which has typst).

- [ ] **Step 7: Commit**

```bash
git add scripts/report-sample-images.sh apps/rux/resources/report-sample REUSE.toml \
        tests/support/typst.hpp tests/unit/core/test_report_template.cpp
git commit -F - <<'EOF'
report: committed sample data for the template, compiled by a test

apps/rux/resources/report-sample/: 15 materials over 5 survey types, 8
synthetic JPEGs (scripts/report-sample-images.sh), a long name, a 16-field
resource template, a draft state and Danish text throughout. One test checks
the sample still exercises those cases, another compiles it with typst.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 5: `nix develop .#report`

**Files:**
- Create: `devshells/report.nix`
- Modify: `docs/guides/report-template.md` (append "Preview loop")

**Interfaces:**
- Consumes: sample dir (Task 4), template `data-dir` input (Task 3), `$REUSEX_REPORT_TEMPLATE` convention (Task 2).
- Produces: commands `report-watch [DATA_DIR]`, `report-build [DATA_DIR]`, `report-shots [DATA_DIR]`, `menu`; outputs `shots/report/report.pdf`, `shots/report/page-NN.png`. Task 7's skill refers to these names.

- [ ] **Step 1: Confirm the shell does not exist yet**

Run: `nix develop .#report -c true 2>&1 | tail -1`
Expected: FAIL — `does not provide attribute 'devShells.x86_64-linux.report'`.

- [ ] **Step 2: Write the devshell**

Create `devshells/report.nix`:

```nix
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# `nix develop .#report` — the PDF report template loop (Typst). Light on
# purpose: no inputsFrom the C++ package, so it enters in seconds.
# docs/guides/report-template.md explains the loop and the data contract.
{pkgs, ...}: let
  motd = ''
    echo ""
    echo "  ┌─────────────────────────────────────────────┐"
    echo "  │       ReUseX  •  Report template shell      │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  report-watch [DATA_DIR]   recompile shots/report/report.pdf on every save"
    echo "  report-build [DATA_DIR]   compile once (non-zero exit on a Typst error)"
    echo "  report-shots [DATA_DIR]   compile, then shots/report/page-NN.png per page"
    echo "  menu                      show this message"
    echo ""
    echo "  DATA_DIR defaults to apps/rux/resources/report-sample."
    echo "  Real data: rux -p x.rux export pdf --dump-data DIR"
    echo ""
  '';
  # Shared by all three commands. Typst refuses a source file outside --root
  # and resolves symlinks before checking, so the template cannot be linked
  # into the data dir. Instead --root is / and report.typ reads the data dir
  # from `--input data-dir=...`.
  preamble = name: ''
    set -euo pipefail
    repo=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
    template="''${REUSEX_REPORT_TEMPLATE:-$repo/apps/rux/resources/report.typ}"
    data="''${1:-$repo/apps/rux/resources/report-sample}"
    if [ ! -f "$data/data.json" ]; then
      echo "${name}: no data.json in $data" >&2
      echo "  (dump a project's data with: rux -p x.rux export pdf --dump-data DIR)" >&2
      exit 2
    fi
    data=$(realpath "$data")
    template=$(realpath "$template")
    out="$repo/shots/report"
    mkdir -p "$out"
    export TYPST_FONT_PATHS="$repo/apps/rux/qt/resources/fonts"
    export TYPST_IGNORE_SYSTEM_FONTS=1
  '';
  report-build = pkgs.writeShellScriptBin "report-build" ''
    ${preamble "report-build"}
    typst compile --root / --input "data-dir=$data" "$template" "$out/report.pdf"
    echo "report-build: $out/report.pdf (data: $data)"
  '';
  report-watch = pkgs.writeShellScriptBin "report-watch" ''
    ${preamble "report-watch"}
    echo "report-watch: $out/report.pdf (data: $data), Ctrl-C to stop"
    exec typst watch --root / --input "data-dir=$data" "$template" "$out/report.pdf"
  '';
  report-shots = pkgs.writeShellScriptBin "report-shots" ''
    ${preamble "report-shots"}
    typst compile --root / --input "data-dir=$data" "$template" "$out/report.pdf"
    rm -f "$out"/page-*.png
    pdftoppm -r 110 -png "$out/report.pdf" "$out/page"
    # pdftoppm pads page numbers to the page count's width (page-1.png for
    # fewer than 10 pages); normalise to page-NN.png so a listing sorts.
    for f in "$out"/page-*.png; do
      n=''${f##*/page-}
      n=''${n%.png}
      new=$(printf '%s/page-%02d.png' "$out" "$((10#$n))")
      [ "$f" = "$new" ] || mv "$f" "$new"
    done
    ls "$out"/page-*.png
  '';
in
  pkgs.mkShell {
    packages = [
      pkgs.typst
      pkgs.poppler-utils
      pkgs.imagemagick # scripts/report-sample-images.sh
      pkgs.git
      report-build
      report-watch
      report-shots
      (pkgs.writeShellScriptBin "menu" motd)
    ];

    shellHook = ''
      export TYPST_FONT_PATHS="$PWD/apps/rux/qt/resources/fonts"
      export TYPST_IGNORE_SYSTEM_FONTS=1
      ${motd}
    '';
  }
```

- [ ] **Step 3: Track it, then exercise every command**

```bash
git add devshells/report.nix
nix develop .#report -c report-build
nix develop .#report -c report-shots
ls shots/report/
nix develop .#report -c report-build ./apps/rux/resources/report-sample   # relative DATA_DIR
nix develop .#report -c report-build /nonexistent; echo "exit=$?"
nix develop .#report -c bash -c 'printf "#panic(\"x\")\n" > /tmp/bad.typ; REUSEX_REPORT_TEMPLATE=/tmp/bad.typ report-build; echo "exit=$?"'
```
Expected: `report.pdf` and `page-01.png`, `page-02.png`, … in `shots/report/`; the relative path compiles; `/nonexistent` prints the `--dump-data` hint and `exit=2`; the panic template prints typst's error and a non-zero exit.

Then `timeout 5 nix develop .#report -c report-watch; echo "exit=$?"` → prints `watching ...`, exit 124 from timeout (proves it starts and keeps running).

Read every `shots/report/page-*.png` with the Read tool and confirm the sample renders (title, Kortlægning table with 5 rows and the draft note, three Ressourcetabel tables, Materialepas with 8 images and 7 `—` cells). Layout problems are expected and belong to sub-project C — only check that nothing is missing.

Also run `alejandra -c devshells/report.nix && deadnix devshells/report.nix && statix check devshells/report.nix` (the default shell's linters).
Expected: no findings.

- [ ] **Step 4: Document the loop**

Append to `docs/guides/report-template.md`:

````markdown
## Preview loop: `nix develop .#report`

A light shell (typst, pdftoppm, ImageMagick; no C++ toolchain):

```bash
nix develop .#report
report-watch                 # live: shots/report/report.pdf on every save of report.typ
report-build [DATA_DIR]      # one-shot; non-zero exit on a Typst error
report-shots [DATA_DIR]      # build + shots/report/page-NN.png at 110 dpi
```

`DATA_DIR` defaults to `apps/rux/resources/report-sample/` (committed,
synthetic). The scripts compile the repo's `report.typ` in place with
`--root / --input data-dir=<abs DATA_DIR>`, so edits are picked up without a
build; `$REUSEX_REPORT_TEMPLATE` points them (and the generator) at another
template file. Outputs under `shots/` are gitignored.

The sample is hand-maintained: edit `data.json` directly; regenerate the
JPEGs with `scripts/report-sample-images.sh`. `ReportSample_*` tests keep it
compiling and keep it exercising long names, missing images, a 16-field
template, a draft state and Danish characters.
````

- [ ] **Step 5: Commit**

```bash
git add devshells/report.nix docs/guides/report-template.md
git commit -F - <<'EOF'
devshell: nix develop .#report for the PDF template loop

report-watch / report-build / report-shots compile apps/rux/resources/
report.typ in place against the committed sample or any dumped data dir
(--root / --input data-dir=...), with no system fonts, and render pages to
shots/report/page-NN.png.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 6: `rux export pdf`

**Files:**
- Create: `apps/rux/include/export/pdf.hpp`
- Create: `apps/rux/src/export/pdf.cpp`
- Modify: `apps/rux/src/export.cpp` (include, footer, registration)
- Modify: `CLAUDE.md` (CLI table `export` row; report shell pointer)
- Modify: `docs/guides/report-template.md` (append "Real project data")
- Test: `tests/unit/rux_app/test_export_pdf.cpp`

**Interfaces:**
- Consumes: `reusex::ReportOptions`, `generate_ressourcekortlaegning_pdf(db, const ReportOptions &)`, `dump_report_data` (Tasks 2–3); `tests/support/typst.hpp` (Task 4).
- Produces:
  ```cpp
  struct SubcommandExportPdfOptions {
    fs::path output_path = fs::current_path() / "report.pdf";
    fs::path dump_dir;                                  // --dump-data
    std::optional<std::int64_t> resource_template_id;   // --resource-template
    fs::path template_path;                             // --template
  };
  void setup_subcommand_export_pdf(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
  int run_subcommand_export_pdf(SubcommandExportPdfOptions const &opt, const RuxOptions &global_opt);
  ```
  Exit statuses: `SUCCESS`; `INVALID_ARGUMENT` for an unknown resource template or a missing output directory (checked before typst); `IO` for any other failure (typst error, missing template file, unwritable output). `--dump-data` excludes `-o` and `--template`.

- [ ] **Step 1: Write the failing tests**

Create `tests/unit/rux_app/test_export_pdf.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux export pdf`: the PDF and --dump-data paths, and that a dump is
// directly usable by the report preview scripts (typst compile against the
// repo's report.typ with --input data-dir=...).

#include <catch2/catch_test_macros.hpp>

#include <exit_status.hpp>
#include <export/pdf.hpp>
#include <global-params.hpp>
#include <reusex/core/ProjectDB.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"
#include "../../support/typst.hpp"

#include <CLI/CLI.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

namespace fs = std::filesystem;
using reusex::test_support::TempDir;
using reusex::test_support::TempPath;

namespace {
/// A closed project with one passport and two manual survey parts. Closed so
/// the subcommand opens it itself, read-only, as it does for a user.
void seed(const fs::path &path) {
  reusex::ProjectDB db(path);
  reusex::test_support::make_passport(db, "guid-a");
  const auto t = reusex::test_support::make_type(db, "Døre, massiv eg");
  reusex::test_support::make_part(db, "RX-001", t);
  reusex::test_support::make_part(db, "RX-002", t);
}

std::string slurp(const fs::path &p) {
  std::ifstream f(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>{}};
}
} // namespace

TEST_CASE("ExportPdf_DumpData_IsAcceptedByTypst", "[rux][report]") {
  TempPath db("export_pdf");
  seed(db.path);
  TempDir out("export_pdf_dump");
  RuxOptions global;
  global.project_db = db.path;
  SubcommandExportPdfOptions opt;
  opt.dump_dir = out.path / "dump";

  REQUIRE(run_subcommand_export_pdf(opt, global) == RuxError::SUCCESS);
  REQUIRE(fs::is_regular_file(opt.dump_dir / "data.json"));
  CHECK_FALSE(fs::exists(opt.output_path)); // --dump-data writes no PDF

  if (!reusex::test_support::have_typst())
    SKIP("typst is not on PATH (the nix devshell provides it)");
  const auto pdf = out.path / "report.pdf";
  const auto log = out.path / "typst.stderr";
  const int rc =
      reusex::test_support::typst_compile_report(opt.dump_dir, pdf, log);
  INFO(slurp(log));
  CHECK(rc == 0);
}

TEST_CASE("ExportPdf_DumpWithUnknownResourceTemplate_InvalidArgumentNoDir",
          "[rux][report]") {
  TempPath db("export_pdf");
  seed(db.path);
  TempDir out("export_pdf_dump");
  RuxOptions global;
  global.project_db = db.path;
  SubcommandExportPdfOptions opt;
  opt.dump_dir = out.path / "dump";
  opt.resource_template_id = 999;

  CHECK(run_subcommand_export_pdf(opt, global) == RuxError::INVALID_ARGUMENT);
  CHECK_FALSE(fs::exists(opt.dump_dir));
}

TEST_CASE("ExportPdf_OutputDirectoryMissing_FailsBeforeTypst",
          "[rux][report]") {
  TempPath db("export_pdf");
  seed(db.path);
  TempDir out("export_pdf_out");
  RuxOptions global;
  global.project_db = db.path;
  SubcommandExportPdfOptions opt;
  opt.output_path = out.path / "no-such-dir" / "report.pdf";

  CHECK(run_subcommand_export_pdf(opt, global) == RuxError::INVALID_ARGUMENT);
  CHECK_FALSE(fs::exists(opt.output_path));
}

TEST_CASE("ExportPdf_Output_WritesPdf", "[rux][report][typst]") {
  if (!reusex::test_support::have_typst())
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempPath db("export_pdf");
  seed(db.path);
  TempDir out("export_pdf_out");
  RuxOptions global;
  global.project_db = db.path;
  SubcommandExportPdfOptions opt;
  opt.output_path = out.path / "report.pdf";

  REQUIRE(run_subcommand_export_pdf(opt, global) == RuxError::SUCCESS);
  CHECK(slurp(opt.output_path).rfind("%PDF", 0) == 0);
}

TEST_CASE("ExportPdf_MissingTemplateFile_IoErrorNoPdf", "[rux][report]") {
  TempPath db("export_pdf");
  seed(db.path);
  TempDir out("export_pdf_out");
  RuxOptions global;
  global.project_db = db.path;
  SubcommandExportPdfOptions opt;
  opt.output_path = out.path / "report.pdf";
  opt.template_path = out.path / "missing.typ";

  CHECK(run_subcommand_export_pdf(opt, global) == RuxError::IO);
  CHECK_FALSE(fs::exists(opt.output_path));
}

TEST_CASE("ExportPdf_DumpDataWithOutputOrTemplate_RejectedByParser",
          "[rux][report]") {
  auto global = std::make_shared<RuxOptions>();
  {
    CLI::App app;
    setup_subcommand_export_pdf(app, global);
    CHECK_THROWS_AS(app.parse("pdf --dump-data d --template t.typ", false),
                    CLI::ExcludesError);
  }
  {
    CLI::App app;
    setup_subcommand_export_pdf(app, global);
    CHECK_THROWS_AS(app.parse("pdf --dump-data d -o r.pdf", false),
                    CLI::ExcludesError);
  }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build --target reusex_unit_tests_vision 2>&1 | grep -m3 error`
Expected: FAIL — `export/pdf.hpp: No such file or directory`.

- [ ] **Step 3: Write the subcommand**

Create `apps/rux/include/export/pdf.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>

namespace fs = std::filesystem;

struct SubcommandExportPdfOptions {
  fs::path output_path = fs::current_path() / "report.pdf";
  /// --dump-data: write data.json + images/ here instead of a PDF.
  fs::path dump_dir;
  std::optional<std::int64_t> resource_template_id;
  /// --template: Typst file to use instead of the embedded report.typ.
  fs::path template_path;
};

void setup_subcommand_export_pdf(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_export_pdf(SubcommandExportPdfOptions const &opt,
                              const RuxOptions &global_opt);
```

Create `apps/rux/src/export/pdf.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/pdf.hpp"
#include "exit_status.hpp"
#include "global-params.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/report_generator.hpp>

#include <spdlog/spdlog.h>

#include <fstream>
#include <stdexcept>

void setup_subcommand_export_pdf(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportPdfOptions>();
  auto *sub = parent.add_subcommand(
      "pdf", "Export the Ressourcekortlægning PDF report (or its data)");

  sub->footer(R"(
DESCRIPTION:
  Renders the Ressourcekortlægning report with Typst (needs `typst` on PATH)
  from the project's survey, material passports and, optionally, a resource
  template's Ressourcetabel. --dump-data writes the report's input instead
  (data.json + images/), exactly as the PDF run sees it, for designing the
  template: `nix develop .#report` then `report-watch DIR`.

EXAMPLES:
  rux -p x.rux export pdf -o report.pdf
  rux -p x.rux export pdf -o report.pdf --resource-template 2
  rux -p x.rux export pdf -o report.pdf --template my-report.typ
  rux -p x.rux export pdf --dump-data shots/report-data

NOTES:
  - Template: --template, else $REUSEX_REPORT_TEMPLATE, else the built-in one
  - Data format: docs/guides/report-template.md
)");

  auto *out = sub->add_option("-o,--output", opt->output_path,
                              "Output PDF path (default: report.pdf)")
                  ->default_val(opt->output_path);
  sub->add_option("--resource-template", opt->resource_template_id,
                  "Add a Ressourcetabel through this resource template id");
  auto *tmpl = sub->add_option("--template", opt->template_path,
                               "Typst template file to use instead of the "
                               "built-in report.typ");
  auto *dump = sub->add_option("--dump-data", opt->dump_dir,
                               "Write data.json + images/ to this directory "
                               "instead of a PDF");
  dump->excludes(out);
  dump->excludes(tmpl);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling export pdf subcommand");
    rux::finish(run_subcommand_export_pdf(*opt, *global_opt));
  });
}

int run_subcommand_export_pdf(SubcommandExportPdfOptions const &opt,
                              const RuxOptions &global_opt) {
  try {
    reusex::ReportOptions report;
    report.resource_template_id = opt.resource_template_id;
    report.template_path = opt.template_path;

    if (opt.dump_dir.empty()) {
      // Fail before the (slow) typst run, not after it.
      const auto parent = fs::absolute(opt.output_path).parent_path();
      if (!fs::is_directory(parent)) {
        spdlog::error("Output directory does not exist: {}", parent.string());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    spdlog::info("Opening project database: {}", global_opt.project_db.string());
    reusex::ProjectDB db(global_opt.project_db, /*readOnly=*/true);

    if (!opt.dump_dir.empty()) {
      reusex::dump_report_data(db, report, opt.dump_dir);
      spdlog::info("Wrote report data to {} (preview: nix develop .#report, "
                   "then report-watch {})",
                   opt.dump_dir.string(), opt.dump_dir.string());
      return RuxError::SUCCESS;
    }

    const auto pdf = reusex::generate_ressourcekortlaegning_pdf(db, report);
    std::ofstream ofs(opt.output_path, std::ios::binary);
    if (!ofs) {
      spdlog::error("Failed to open output file: {}", opt.output_path.string());
      return RuxError::IO;
    }
    ofs.write(reinterpret_cast<const char *>(pdf.data()),
              static_cast<std::streamsize>(pdf.size()));
    if (!ofs) {
      spdlog::error("Failed to write output file: {}",
                    opt.output_path.string());
      return RuxError::IO;
    }
    spdlog::info("Wrote {} ({} bytes)", opt.output_path.string(), pdf.size());
    return RuxError::SUCCESS;
  } catch (const std::out_of_range &e) {
    spdlog::error("Unknown resource template: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("export pdf failed: {}", e.what());
    return RuxError::IO;
  }
}
```

(`std::out_of_range` from `resource_report_section` is the only `out_of_range` source here — it is thrown before any typst work by both library calls.)

- [ ] **Step 4: Register it**

In `apps/rux/src/export.cpp`: add `#include "export/pdf.hpp"` after `#include "export/materialepas.hpp"`; in the footer's SUBCOMMANDS list add, after the `materialepas` line,

```
  pdf              Export the Ressourcekortlægning PDF report (or its data)
```

and in EXAMPLES, after the `materialepas` example,

```
  rux export pdf -o report.pdf              # Ressourcekortlægning PDF
```

and add `setup_subcommand_export_pdf(*sub, global_opt);` after `setup_subcommand_export_materialepas(*sub, global_opt);`.

- [ ] **Step 5: Run tests to verify they pass**

Run:
```bash
cmake -B build && cmake --build build --target reusex_unit_tests_vision rux && \
ctest --test-dir build -R 'ExportPdf_' --output-on-failure --parallel $(nproc)
```
Expected: all six PASS (inside `nix develop`). Then an end-to-end smoke from the shell:

```bash
./build/apps/rux/rux export pdf --help | head -5
./build/apps/rux/rux -p /tmp/does-not-exist.rux export pdf -o /tmp/r.pdf; echo "exit=$?"; ls /tmp/does-not-exist.rux 2>&1
```
Expected: help prints; the missing project exits non-zero and does **not** create `/tmp/does-not-exist.rux` (read-only open).

- [ ] **Step 6: Docs**

In `CLAUDE.md`, CLI table, change the `export` row's sub-commands from
`` `ply`, `e57`, `materialepas`, `csv`, `rhino`, `semantic-images`, `speckle`, `colmap` `` to
`` `ply`, `e57`, `materialepas`, `csv`, `rhino`, `semantic-images`, `speckle`, `colmap`, `pdf` ``.
Under "### Documentation" (end of that section) add:

```markdown
The Ressourcekortlægning PDF template (`apps/rux/resources/report.typ`, the
only copy — CMake embeds it) has its own light shell, `nix develop .#report`
(`report-watch` / `report-build` / `report-shots`); data contract and loop:
[`docs/guides/report-template.md`](docs/guides/report-template.md).
```

Append to `docs/guides/report-template.md`:

````markdown
## Real project data

```bash
rux -p x.rux export pdf --dump-data shots/report-data [--resource-template ID]
nix develop .#report -c report-watch shots/report-data
```

`--dump-data` writes into a missing or empty directory, or replaces a
previous dump (its `images/` is cleared first); any other non-empty
directory is refused. A real project's photos stay out of git: dump under
`shots/` (gitignored), never into `apps/rux/resources/`.

`rux -p x.rux export pdf -o report.pdf [--resource-template ID] [--template FILE.typ]`
renders the PDF the same way ruxd does.
````

- [ ] **Step 7: Commit**

```bash
git add apps/rux/include/export/pdf.hpp apps/rux/src/export/pdf.cpp apps/rux/src/export.cpp \
        tests/unit/rux_app/test_export_pdf.cpp CLAUDE.md docs/guides/report-template.md
git commit -F - <<'EOF'
rux: export pdf [-o FILE | --dump-data DIR]

Thin wrapper over generate_ressourcekortlaegning_pdf / dump_report_data.
A dump is directly usable by report-watch; an unknown resource template or a
missing output directory fails with INVALID_ARGUMENT before typst runs.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 7: design-studio skill — the PDF report loop

**Files:**
- Modify: `.claude/skills/design-studio/SKILL.md` (frontmatter `description`, new section at the end)
- Create: `.claude/skills/design-studio/references/pdf-report.md`

**Interfaces:**
- Consumes: command names from Task 5, `rux export pdf --dump-data` from Task 6, `docs/guides/report-template.md`.
- Produces: documentation only.

- [ ] **Step 1: Check the current trigger text (the "failing test" for a doc change)**

Run: `grep -c -i 'pdf\|typst\|report' .claude/skills/design-studio/SKILL.md`
Expected: `0` — the skill does not trigger on report work today.

- [ ] **Step 2: Broaden the description**

Replace the `description:` line in the frontmatter with:

```yaml
description: The design workflow for the native Qt client of the rux GUI (apps/rux/qt, in progress: QSS generated from tokens.css, screenshot loop via qt_shot.sh) and for the Ressourcekortlægning PDF report (Typst template apps/rux/resources/report.typ, page-screenshot loop via `nix develop .#report` / report-shots). Use this whenever you design, build, restyle, redesign, or "make it look better/more polished" for any Qt client page or workspace (Database, 3D/Viewer, Posegraf, Pipeline, Log, the app shell); whenever you touch styles/app.qss or a Qt widget/page; whenever you work on the PDF report, its Typst template, its layout, its sample data or print quality; and whenever you work from docs/Design Review.md for a Qt page. Also use when matching the existing rux GUI look, or when a token in apps/rux/qt/theme/tokens.css needs refreshing from rux-frontend. The web half of the rux GUI (React/Vite/TS, apps/rux/frontend) moved to the standalone rux-frontend repo — its design-studio skill lives there now, not here.
```

- [ ] **Step 3: Add the section**

Append to `.claude/skills/design-studio/SKILL.md`:

```markdown

## PDF report (Typst)

The Ressourcekortlægning report is an official document (it is attached to
a demolition-permit application), so print quality is correctness, not taste.
`apps/rux/resources/report.typ` is the only copy of the template; CMake
embeds it, so no C++ build is needed to see a change.

The loop, in `nix develop .#report`:

1. `report-shots [DATA_DIR]` — compile and render every page to
   `shots/report/page-NN.png` (default data: the committed
   `apps/rux/resources/report-sample/`).
2. **Read every PNG** — all pages, not the first one.
3. Critique against the checklist in `references/pdf-report.md`.
4. Fix `report.typ`, repeat. Keep `report-watch` running in another pane
   when the user wants to watch.

Real data: `rux -p x.rux export pdf --dump-data shots/report-data`, then
`report-shots shots/report-data`. Never commit a real project's dump.
Data contract: `docs/guides/report-template.md`. Read
`references/pdf-report.md` before the first change.
```

- [ ] **Step 4: Write the reference**

Create `.claude/skills/design-studio/references/pdf-report.md`:

````markdown
# PDF report — reference

## Where things are

| What | Where |
|---|---|
| Template (only copy) | `apps/rux/resources/report.typ` |
| Data contract | `docs/guides/report-template.md` |
| Sample data (synthetic, hand-maintained) | `apps/rux/resources/report-sample/` (`data.json`, `images/`) |
| Sample images | `scripts/report-sample-images.sh` |
| Generator (C++) | `libs/reusex/src/core/report_generator.cpp` |
| Tests that keep it compiling | `ReportSample_*`, `ReportTemplate_*`, `ReportPdf_*` (`tests/unit/core/`), `ExportPdf_*` (`tests/unit/rux_app/`) |

## The loop

```bash
nix develop .#report
report-shots                          # sample data
report-shots shots/report-data        # a dump: rux -p x.rux export pdf --dump-data shots/report-data
```

Read every `shots/report/page-NN.png`, critique, fix, repeat. `report-build`
exits non-zero on a Typst error — read its message before guessing.

Gotchas:
- The template reads its data via `sys.inputs.at("data-dir", default: ".")`;
  every path to data or images must go through `data_dir`. A bare
  `json("data.json")` or `image(mat.thumbnail_path)` works in the generator
  and breaks the preview (or the other way round).
- System fonts are ignored everywhere. A font not embedded in typst must be
  shipped to the generator too (see the guide's "Fonts" section), or ruxd's
  PDF silently differs from the preview.
- When the sample stops exercising a case, extend `data.json` by hand; the
  `ReportSample_DataJson_ExercisesTheTemplate` test lists the minimums.

## Print critique checklist

- Nothing wider than the text block (tables, images, long names, URLs).
- No table whose column count comes from data — a data-driven column set is
  split or turned into cards (the resource tables already cap at 8).
- No card or table row split across pages; no orphaned headings at a page
  bottom.
- Readable in greyscale: no meaning carried by colour alone.
- Images ≥ 150 dpi at their printed size; `fit: "contain"`, never stretched.
- Danish formatting: decimal comma (`1.234,5`), dates `13. oktober 2026` or
  `13.10.2026`, units with a space (`620 m²`, `190 t`).
- Every page has the header and footer (project, title, page X / Y).
- Draft state (`survey.blocking > 0`) is unmistakable on the first page.
- File size sane: < 10 MB for 150 resources (check `ls -l shots/report/report.pdf`).
````

- [ ] **Step 5: Verify and commit**

Run: `grep -c -i 'typst' .claude/skills/design-studio/SKILL.md && reuse lint | tail -1`
Expected: a count ≥ 1; REUSE compliant (`.claude/skills/**` is annotated in `REUSE.toml`).

```bash
git add .claude/skills/design-studio/SKILL.md .claude/skills/design-studio/references/pdf-report.md
git commit -F - <<'EOF'
design-studio: PDF report section and print checklist

The skill now triggers on report/PDF/Typst work and describes the
report-shots -> read every page -> critique -> fix loop, how to get real
data, and a print critique checklist.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

## Final verification (after Task 7)

```bash
cmake --build build && ctest --test-dir build -LE package --output-on-failure --parallel $(nproc)
nix develop .#report -c report-shots
git status --short   # clean
```

Expected: full suite green; `shots/report/page-*.png` present and untracked-ignored.

ruxd compatibility: ruxd builds against the installed package and calls `generate_ressourcekortlaegning_pdf(db, template_id)` with `std::optional<std::int64_t>` — unchanged signature (pinned by `ReportPdf_OldTwoArgumentOverload_StillExists`). Its behaviour changes in exactly two ways, both intended by the spec: `$REUSEX_REPORT_TEMPLATE` in its environment now selects the template, and thumbnails live under `images/` in the temp dir (internal).
