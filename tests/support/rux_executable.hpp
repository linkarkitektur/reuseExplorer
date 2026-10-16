// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The `rux` binary the integration tests run as a subprocess.
//
// REUSEX_RUX_EXECUTABLE (tests/CMakeLists.txt) is the build tree's rux. The
// environment variable REUSEX_TEST_RUX_EXECUTABLE, when set, wins: the nix
// checks.tests derivation runs ctest against the test binaries of another
// derivation (reusex-uk8), whose build tree is gone, and points this at the
// installed bin/rux instead.

#include <cstdlib>
#include <string>

#ifndef REUSEX_RUX_EXECUTABLE
#error                                                                         \
    "REUSEX_RUX_EXECUTABLE must be defined by the build (see tests/CMakeLists.txt)"
#endif

namespace reusex::test_support {

inline std::string rux_executable() {
  if (const char *path = std::getenv("REUSEX_TEST_RUX_EXECUTABLE");
      path != nullptr && *path != '\0')
    return path;
  return REUSEX_RUX_EXECUTABLE;
}

} // namespace reusex::test_support
