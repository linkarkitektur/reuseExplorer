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

#include <cstdlib>
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
  // Uses the embedded template: a developer's exported override must not
  // change what this test compiles (own ctest process, so no leak).
  ::unsetenv("REUSEX_REPORT_TEMPLATE");
  TempPath db("export_pdf");
  seed(db.path);
  TempDir out("export_pdf_dump");
  RuxOptions global;
  global.project_db = db.path;
  SubcommandExportPdfOptions opt;
  opt.dump_dir = out.path / "dump";
  // Inside the TempDir, so the "no PDF written" check below does not look at
  // ctest's working directory.
  opt.output_path = out.path / "report.pdf";

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

TEST_CASE("ExportPdf_OutputIsDirectory_FailsBeforeTypst", "[rux][report]") {
  TempPath db("export_pdf");
  seed(db.path);
  TempDir out("export_pdf_out");
  RuxOptions global;
  global.project_db = db.path;
  SubcommandExportPdfOptions opt;
  opt.output_path = out.path / "a-directory";
  fs::create_directories(opt.output_path);

  CHECK(run_subcommand_export_pdf(opt, global) == RuxError::INVALID_ARGUMENT);
  CHECK(fs::is_directory(opt.output_path));
  CHECK(fs::is_empty(opt.output_path));
}

TEST_CASE("ExportPdf_Output_WritesPdf", "[rux][report][typst]") {
  // Uses the embedded template: a developer's exported override must not
  // change what this test compiles (own ctest process, so no leak).
  ::unsetenv("REUSEX_REPORT_TEMPLATE");
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

TEST_CASE("ExportPdf_MissingTemplateFile_InvalidArgumentNoPdf",
          "[rux][report]") {
  TempPath db("export_pdf");
  seed(db.path);
  TempDir out("export_pdf_out");
  RuxOptions global;
  global.project_db = db.path;
  SubcommandExportPdfOptions opt;
  opt.output_path = out.path / "report.pdf";
  opt.template_path = out.path / "missing.typ";

  CHECK(run_subcommand_export_pdf(opt, global) == RuxError::INVALID_ARGUMENT);
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
