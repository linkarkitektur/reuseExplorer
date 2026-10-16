#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Compare two pose benchmark reports from scripts/bench-poses.sh (baseline vs
candidate) and gate on regressions, the pose-reference counterpart of
scripts/bench-solid-compare.py (reusex-rcv.1.6).

Usage:
    scripts/bench-poses-compare.py baseline.json candidate.json
    scripts/bench-poses-compare.py --threshold 10 baseline.json candidate.json

Rows are matched by name ("<manifest-row>:<config>", see bench-poses.sh),
metrics by key. Every scored metric is "lower is better" (an error measure):
ATE RMSE/median/max, rotation RMSE, both RPE pairs. `matched` and `scale` are
reported but never gate (no direction: a different matched count or a Sim3
scale is informational, not a regression by itself). `segment_ate_m` (a list)
and `options` (the run's own settings) are never compared. A metric regresses
when it got worse by more than --threshold percent of the baseline value (any
worsening beyond --abs-tolerance when the baseline is 0), or when it went
from a number to n/a. A row that ran in the baseline but failed in the
candidate also regresses. Rows present in one report only, and skipped rows,
are reported but never fail the comparison.

Exit status:
    0  nothing regressed beyond --threshold
    1  at least one regression, no row both reports ran (all skipped, failed
       or unmatched: nothing was compared), or an input could not be read

Python 3 stdlib only.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass

LOWER = "lower"

#: Direction of every scored metric in the JSON `rux analyze poses --json`
#: prints (apps/rux/src/analyze/poses.cpp). `segment_ate_m` (a list) and
#: `options` (an object) are excluded entirely, not merely undirected.
DIRECTIONS: "dict[str, str]" = {
    "ate_rmse_m": LOWER,
    "ate_median_m": LOWER,
    "ate_max_m": LOWER,
    "rot_rmse_deg": LOWER,
    "rpe_trans_m": LOWER,
    "rpe_rot_deg": LOWER,
    "rpe_trans_frames_m": LOWER,
    "rpe_rot_frames_deg": LOWER,
}

#: Present in the JSON but never scored: not a scalar error measure
#: (`segment_ate_m`, `options`) or informational only (`matched`, `scale`).
EXCLUDED_KEYS = {"segment_ate_m", "options"}
INFO_KEYS = {"matched", "scale"}


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
    """Read a bench-poses.sh report and return its rows keyed by name."""
    try:
        with open(path, encoding="utf-8") as f:
            report = json.load(f)
    except FileNotFoundError:
        raise SystemExit(f"error: file not found: {path}")
    except json.JSONDecodeError as exc:
        raise SystemExit(f"error: malformed JSON in {path}: {exc}")
    if not isinstance(report, dict) or "rows" not in report:
        raise SystemExit(
            f"error: {path} is not a bench-poses report (no 'rows' key)"
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
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


def worsening(direction: str, base: float, cand: float) -> float:
    """How much worse cand is than base (<= 0 when not worse)."""
    if direction == LOWER:
        return cand - base
    raise ValueError(f"unknown direction {direction!r}")


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
        change = (f"{cand - base:+.4g}"
                 if isinstance(base, (int, float)) and isinstance(cand, (int, float))
                 else "")
        return Line(row, key, b, c, change, "info")
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
        keys += [k for k in INFO_KEYS if k in bm or k in cm]
        keys += sorted(
            k for k in set(bm) | set(cm)
            if k not in DIRECTIONS and k not in INFO_KEYS and k not in EXCLUDED_KEYS
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
        description="Compare two bench-poses.sh reports."
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
    compared = {ln.row for ln in lines if ln.metric != "*"}
    if not compared:
        print("\nerror: no row could be compared (every row is skipped, "
              "failed, or present in one report only)", file=sys.stderr)
        return 1
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
