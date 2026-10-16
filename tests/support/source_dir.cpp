// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "source_dir.hpp"

// REUSEX_SOURCE_DIR is set on this one file only (tests/CMakeLists.txt).
std::filesystem::path reusex::test_support::source_dir() {
  return REUSEX_SOURCE_DIR;
}
