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

#include "../../support/source_dir.hpp"
#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"
#include "../../support/typst.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <set>
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
} // namespace

TEST_CASE("ReportTemplate_EmbeddedCopy_MatchesFileOnDisk", "[report][typst]") {
  // One read guards the CMake embedding: the generated header must hold
  // apps/rux/resources/report.typ byte for byte, comments included.
  const auto on_disk = slurp(reusex::test_support::source_dir() /
                             "apps/rux/resources/report.typ");
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
  if (!reusex::test_support::have_typst())
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
  if (!reusex::test_support::have_typst())
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
  if (!reusex::test_support::have_typst())
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

TEST_CASE("ReportDump_HostileGuid_ImageStaysInsideDir", "[report][dump]") {
  // The guid comes from the .rux and may be untrusted: one that is not a safe
  // file name gets images/material-<index><ext>, never a path built from it.
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  const std::vector<std::string> hostile = {"../../escape", "a/b"};
  for (const auto &g : hostile) {
    reusex::test_support::make_passport(db, g);
    db.set_material_thumbnail(g, {0xFF, 0xD8, 0xFF, 0xD9}, "image/jpeg");
  }
  TempDir out("test_report_dump_dir");
  const auto dir = out.path / "x" / "dump";

  reusex::dump_report_data(db, {}, dir);

  const auto data = nlohmann::json::parse(slurp(dir / "data.json"));
  REQUIRE(data.at("materials").size() == hostile.size());
  std::set<std::string> guids;
  for (std::size_t i = 0; i < data.at("materials").size(); ++i) {
    const auto &m = data.at("materials")[i];
    guids.insert(m.at("guid").get<std::string>());
    REQUIRE(m.at("has_thumbnail").get<bool>());
    const auto rel = m.at("thumbnail_path").get<std::string>();
    char want[32];
    std::snprintf(want, sizeof(want), "images/material-%04zu.jpg", i);
    CHECK(rel == want);
    const auto file = fs::weakly_canonical(dir / rel);
    CHECK(fs::is_regular_file(file));
    CHECK(file.parent_path() == fs::weakly_canonical(dir / "images"));
  }
  CHECK(guids == std::set<std::string>(hostile.begin(), hostile.end()));
  // Where the raw guids would have landed: dir/images/../../escape.jpg is
  // out.path/x/escape.jpg; dir/images/a/b.jpg is dir/images/a/.
  CHECK_FALSE(fs::exists(out.path / "x" / "escape.jpg"));
  CHECK_FALSE(fs::exists(dir / "images" / "a"));
  CHECK(entries(out.path) == std::vector<std::string>{"x"});
  CHECK(entries(out.path / "x") == std::vector<std::string>{"dump"});
  CHECK(entries(dir / "images") ==
        std::vector<std::string>{"material-0000.jpg", "material-0001.jpg"});
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

TEST_CASE("ReportDump_ForeignDirWithDataJson_RefusedAndUntouched",
          "[report][dump]") {
  // A data.json that merely exists is not enough to call a directory a
  // previous dump: it must carry the keys a dump always writes.
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  TempDir out("test_report_dump_dir");
  fs::create_directories(out.path / "images");
  std::ofstream(out.path / "data.json") << R"({"name":"x"})";
  std::ofstream(out.path / "images" / "x.jpg") << "jpeg-bytes";

  CHECK_THROWS_WITH(reusex::dump_report_data(db, {}, out.path),
                    ContainsSubstring(out.path.string()));

  CHECK(entries(out.path) == std::vector<std::string>{"data.json", "images"});
  CHECK(slurp(out.path / "data.json") == R"({"name":"x"})");
  CHECK(fs::is_regular_file(out.path / "images" / "x.jpg"));
}

TEST_CASE("ReportDump_PreviousDumpWithExtraFile_Refused", "[report][dump]") {
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  seed_report_project(db);
  TempDir out("test_report_dump_dir");
  reusex::dump_report_data(db, {}, out.path);
  std::ofstream(out.path / "notes.txt") << "do not touch";

  CHECK_THROWS_WITH(reusex::dump_report_data(db, {}, out.path),
                    ContainsSubstring(out.path.string()));

  CHECK(fs::is_regular_file(out.path / "notes.txt"));
  CHECK(fs::is_regular_file(out.path / "images" / "guid-with-thumb.jpg"));
}

TEST_CASE("ReportDump_TargetIsRegularFile_Throws", "[report][dump]") {
  TempPath tmp("test_report_dump");
  ProjectDB db(tmp.path);
  TempDir out("test_report_dump_dir");
  const auto file_path = out.path / "not-a-dir";
  std::ofstream(file_path) << "x";

  CHECK_THROWS_AS(reusex::dump_report_data(db, {}, file_path),
                  std::runtime_error);
  CHECK(fs::is_regular_file(file_path));
}

TEST_CASE("ReportPdf_ProjectWithThumbnail_Compiles", "[report][typst]") {
  // The PDF run uses the same images/ layout and the template's default
  // data-dir "." — an image reference that resolves wrongly fails here.
  // Uses the embedded template: a developer's exported override must not
  // change what this test compiles (own ctest process, so no leak).
  ::unsetenv("REUSEX_REPORT_TEMPLATE");
  if (!reusex::test_support::have_typst())
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

namespace {
const fs::path kSampleDir =
    reusex::test_support::source_dir() / "apps/rux/resources/report-sample";
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
    longest_name =
        std::max(longest_name,
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
  const int rc =
      reusex::test_support::typst_compile_report(kSampleDir, pdf, log);
  INFO(slurp(log));
  REQUIRE(rc == 0);
  CHECK(slurp(pdf).rfind("%PDF", 0) == 0);
}
