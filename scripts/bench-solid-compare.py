#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Compare two solid-model benchmark reports from scripts/bench-solid.sh
(baseline vs candidate) and gate on regressions, the solid-model counterpart
of scripts/bench-compare.py (docs/design/solid-model-json.md).

Usage:
    scripts/bench-solid-compare.py baseline.json candidate.json
    scripts/bench-solid-compare.py --threshold 10 baseline.json candidate.json

Rows are matched by name, metrics by key. Each known metric has a direction
(lower or higher is better; room_count_error compares its magnitude). A
metric regresses when it got worse by more than --threshold percent of the
baseline value (any worsening beyond --abs-tolerance when the baseline is 0),
or when it went from a number to n/a. A row that ran in the baseline but
failed in the candidate also regresses. Rows present in one report only, and
skipped rows, are reported but never fail the comparison.

Exit status:
    0  nothing regressed beyond --threshold
    1  at least one regression, or an input could not be read

Python 3 stdlib only.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass

LOWER = "lower"
HIGHER = "higher"
LOWER_ABS = "lower |x|"

#: Direction of every metric in SolidModelReport::to_json().
DIRECTIONS: "dict[str, str]" = {
    "room_count_error": LOWER_ABS,
    "room_iou_mean": HIGHER,
    "wall_position_rms_m": LOWER,
    "wall_thickness_err_mean_m": LOWER,
    "wall_thickness_err_p95_m": LOWER,
    "wall_recall": HIGHER,
    "wall_precision": HIGHER,
    "opening_recall": HIGHER,
    "opening_precision": HIGHER,
    "fit_mean_m": LOWER,
    "fit_within_share": HIGHER,
    "seen_share_mean": HIGHER,
    "assumed_share": LOWER,
}


@dataclass
class Line:
    """One printed comparison line."""

    row: str
    metric: str
    baseline: str
    candidate: str
    change: str
    verdict: str
    regression: bool = False


def load_report(path: str) -> "dict[str, dict]":
    """Read a bench-solid.sh report and return its rows keyed by name."""
    try:
        with open(path, encoding="utf-8") as f:
            report = json.load(f)
    except FileNotFoundError:
        raise SystemExit(f"error: file not found: {path}")
    except json.JSONDecodeError as exc:
        raise SystemExit(f"error: malformed JSON in {path}: {exc}")
    if not isinstance(report, dict) or "rows" not in report:
        raise SystemExit(
            f"error: {path} is not a bench-solid report (no 'rows' key)"
        )
    rows: "dict[str, dict]" = {}
    for row in report["rows"]:
        name = row.get("name")
        if not name:
            raise SystemExit(f"error: {path}: a row has no name")
        if name in rows:
            raise SystemExit(f"error: {path}: duplicate row '{name}'")
        rows[name] = row
    return rows


def fmt(value) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, int) and not isinstance(value, bool):
        return f"{value:+d}" if value else "0"
    return f"{value:.4g}"


def worsening(direction: str, base: float, cand: float) -> float:
    """How much worse cand is than base (<= 0 when not worse)."""
    if direction == HIGHER:
        return base - cand
    if direction == LOWER_ABS:
        return abs(cand) - abs(base)
    return cand - base


def compare_metric(
    row: str, key: str, base, cand, threshold: float, abs_tol: float
) -> Line:
    direction = DIRECTIONS.get(key)
    b, c = fmt(base), fmt(cand)
    if base is None and cand is None:
        return Line(row, key, b, c, "", "n/a")
    if base is None:
        return Line(row, key, b, c, "", "NEW")
    if cand is None:
        return Line(row, key, b, c, "", "LOST", regression=True)
    if direction is None:
        return Line(row, key, b, c, f"{cand - base:+.4g}", "info")
    worse = worsening(direction, float(base), float(cand))
    ref = abs(float(base))
    if ref > 0.0:
        pct = 100.0 * worse / ref
        change = f"{100.0 * (float(cand) - float(base)) / ref:+.1f}%"
        regressed = pct > threshold
    else:
        change = f"{cand - base:+.4g}"
        regressed = worse > abs_tol
    if regressed:
        verdict = "REGRESSED"
    elif worse < -abs_tol:
        verdict = "improved"
    else:
        verdict = "ok"
    return Line(row, key, b, c, change, verdict, regression=regressed)


def compare(
    baseline: "dict[str, dict]",
    candidate: "dict[str, dict]",
    threshold: float,
    abs_tol: float = 1e-9,
) -> "list[Line]":
    lines: "list[Line]" = []
    for name in sorted(set(baseline) | set(candidate)):
        if name not in candidate:
            lines.append(Line(name, "*", "", "", "", "REMOVED"))
            continue
        if name not in baseline:
            lines.append(Line(name, "*", "", "", "", "ADDED"))
            continue
        brow, crow = baseline[name], candidate[name]
        bstat, cstat = brow.get("status"), crow.get("status")
        if bstat == "ok" and cstat == "failed":
            lines.append(
                Line(name, "*", "ok", "failed", crow.get("reason", ""),
                     "FAILED", regression=True)
            )
            continue
        if bstat != "ok" or cstat != "ok":
            lines.append(Line(name, "*", str(bstat), str(cstat), "", "skipped"))
            continue
        bm, cm = brow.get("metrics", {}), crow.get("metrics", {})
        keys = [k for k in DIRECTIONS if k in bm or k in cm]
        keys += sorted(
            k for k in set(bm) | set(cm)
            if k not in DIRECTIONS and k != "counts"
        )
        for key in keys:
            lines.append(
                compare_metric(name, key, bm.get(key), cm.get(key),
                               threshold, abs_tol)
            )
    return lines


def print_table(lines: "list[Line]", out=sys.stdout) -> None:
    header = ("row", "metric", "baseline", "candidate", "change", "verdict")
    table = [header] + [
        (ln.row, ln.metric, ln.baseline, ln.candidate, ln.change, ln.verdict)
        for ln in lines
    ]
    widths = [max(len(r[i]) for r in table) for i in range(len(header))]
    for r in table:
        print("  ".join(cell.ljust(w) for cell, w in zip(r, widths)).rstrip(),
              file=out)


def main(argv: "list[str] | None" = None) -> int:
    parser = argparse.ArgumentParser(
        description="Compare two bench-solid.sh reports."
    )
    parser.add_argument("baseline")
    parser.add_argument("candidate")
    parser.add_argument(
        "--threshold", type=float, default=5.0,
        help="allowed worsening in percent of the baseline (default 5)",
    )
    parser.add_argument(
        "--abs-tolerance", type=float, default=1e-9,
        help="worsening ignored when the baseline value is 0 (default 1e-9)",
    )
    args = parser.parse_args(argv)
    if args.threshold < 0:
        raise SystemExit("error: --threshold must be >= 0")

    lines = compare(load_report(args.baseline), load_report(args.candidate),
                    args.threshold, args.abs_tolerance)
    print_table(lines)
    regressions = [ln for ln in lines if ln.regression]
    if regressions:
        print(
            f"\n{len(regressions)} regression(s) beyond "
            f"{args.threshold:g}%: "
            + ", ".join(f"{ln.row}/{ln.metric}" for ln in regressions),
            file=sys.stderr,
        )
        return 1
    print(f"\nno regression beyond {args.threshold:g}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
