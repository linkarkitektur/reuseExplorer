// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "source_dir.hpp"

#include <cstdlib>

// REUSEX_SOURCE_DIR is set on this one file only (tests/CMakeLists.txt).
// REUSEX_TEST_SOURCE_DIR overrides it at run time: the nix checks.tests
// derivation runs ctest in a different derivation from the one that compiled
// the tests, so the compile-time path no longer exists there (reusex-uk8).
std::filesystem::path reusex::test_support::source_dir() {
  if (const char *dir = std::getenv("REUSEX_TEST_SOURCE_DIR");
      dir != nullptr && *dir != '\0')
    return dir;
  return REUSEX_SOURCE_DIR;
}
