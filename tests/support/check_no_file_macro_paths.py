#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Guard against reusex-4hy: a test deriving a fixture path from __FILE__.

With the dev shell's shared ccache (CCACHE_BASEDIR + CCACHE_NOHASHDIR,
#55/docs/guides/ci-cache.md), ccache rewrites each source's absolute path to
one relative to the repo root before it reaches the compiler, so __FILE__
bakes in a build-tree-relative path that does not resolve from ctest's
WORKING_DIRECTORY. That is invisible in the nix sandbox build, which never
does this rewrite, so the bug only ever shows up in a local dev build.

The fix is always the same: locate the repo root through
reusex::test_support::source_dir() (tests/support/source_dir.hpp), which
keeps the absolute path on one TU (tests/support/source_dir.cpp) built with
its own, non-shared compile definition. Run from anywhere with plain python3:

    python3 tests/support/check_no_file_macro_paths.py
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# tests/support itself is exempt: source_dir.cpp is the one sanctioned place
# an absolute path is allowed to live, and it does not use __FILE__ (it reads
# the REUSEX_SOURCE_DIR compile definition instead).
SCAN_DIRS = [REPO_ROOT / "tests" / "unit", REPO_ROOT / "tests" / "integration"]
SUFFIXES = {".cpp", ".hpp"}


def main() -> int:
    offenders: list[str] = []
    for scan_dir in SCAN_DIRS:
        if not scan_dir.is_dir():
            continue
        for path in sorted(scan_dir.rglob("*")):
            if path.suffix not in SUFFIXES or not path.is_file():
                continue
            text = path.read_text(encoding="utf-8", errors="replace")
            for lineno, line in enumerate(text.splitlines(), start=1):
                if "__FILE__" in line:
                    offenders.append(
                        f"{path.relative_to(REPO_ROOT)}:{lineno}: {line.strip()}"
                    )

    if offenders:
        print("__FILE__ found in tests/unit or tests/integration (reusex-4hy):")
        for offender in offenders:
            print(f"  {offender}")
        print(
            "\n__FILE__-derived paths break under the dev shell's shared ccache "
            "(CCACHE_BASEDIR rewrites absolute source paths to relative before "
            "compiling). Use reusex::test_support::source_dir() from "
            "tests/support/source_dir.hpp instead."
        )
        return 1

    print(f"ok: no __FILE__ path construction in {', '.join(str(d) for d in SCAN_DIRS)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
