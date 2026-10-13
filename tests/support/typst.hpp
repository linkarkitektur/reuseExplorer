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
