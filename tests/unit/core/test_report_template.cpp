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
