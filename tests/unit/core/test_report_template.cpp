// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The PDF report template workflow (report-template-workflow spec): the
// embedded template is the file on disk, the run-time overrides, the data
// dump, and the committed sample data.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/core/report_template_default.hpp>

#include <core/ProjectDB.hpp>
#include <core/report_generator.hpp>

#include "../../support/temp_path.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {
std::string slurp(const fs::path &path) {
  std::ifstream f(path, std::ios::binary);
  REQUIRE(f.good());
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>{}};
}

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
} // namespace

TEST_CASE("ReportTemplate_EmbeddedCopy_MatchesFileOnDisk", "[report][typst]") {
  // One read guards the CMake embedding: the generated header must hold
  // apps/rux/resources/report.typ byte for byte, comments included.
  const auto on_disk =
      slurp(fs::path(REUSEX_SOURCE_DIR) / "apps/rux/resources/report.typ");
  CHECK(std::string(reusex::core::detail::kDefaultReportTemplate) == on_disk);
}

TEST_CASE("ReportPdf_OldTwoArgumentOverload_StillExists", "[report]") {
  // ruxd calls generate_ressourcekortlaegning_pdf(db, optional<int64_t>);
  // taking the address with that exact type fails to compile if the
  // overload disappears or changes shape.
  using OldSig =
      std::vector<std::uint8_t> (*)(ProjectDB &, std::optional<std::int64_t>);
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
  CHECK_THROWS_WITH(
      reusex::generate_ressourcekortlaegning_pdf(db, opt),
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
