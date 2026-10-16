// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The repository's source directory, for tests that read checked-in files
// (docs/CONTRACTS.md, the report template, tokens.css, engine-build.json).
// Include as "../../support/source_dir.hpp" from tests/unit/<module>/.
//
// This is a function defined in source_dir.cpp, not a REUSEX_SOURCE_DIR
// compile definition on the test executables, on purpose (reusex-37v): an
// absolute path in a target-wide -D lands in the command line of every test
// TU, which ccache hashes verbatim, so no test object could ever be shared
// between two worktrees. Only source_dir.cpp carries the path now.
//
// The environment variable REUSEX_TEST_SOURCE_DIR, when set, wins over the
// compiled-in path. The nix checks.tests derivation sets it, because it runs
// the test binaries of another derivation, whose source tree is gone
// (reusex-uk8).

#include <filesystem>

namespace reusex::test_support {

std::filesystem::path source_dir();

} // namespace reusex::test_support
