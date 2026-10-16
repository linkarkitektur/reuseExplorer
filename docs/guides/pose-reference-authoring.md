<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Pose references: scoring a trajectory against a frozen ground truth

A **pose reference** is the pose analogue of a solid-model reference plan
([`docs/guides/solid-model-reference-authoring.md`](solid-model-reference-authoring.md)):
a trajectory that a person (or the visual drift-correction loop,
`reusex-rcv.1`) has decided is "right enough to optimize against", frozen so
the automatic pipeline (`rux optimize`, `rux register`, the Manhattan prior,
...) can be scored against it. This guide covers the file layout, how it is
scored, and the DRAFT marking convention. Authoring one with the
drift-correction loop itself is `reusex-rcv.1.8`'s guide, once that loop
(`rux optimize --edits --watch`) exists; this page is about the artefact the
loop produces and how the benchmark consumes it, not about how to find and
fix drift.

## File layout

```
tests/fixtures/pose_reference/
├── house2.trajectory.tum   # the frozen trajectory, TUM format
└── house2.edits.yaml       # the pose-edits file that produced it (reusex-rcv.1.2)
```

A pose reference is two files, named `<scan>.trajectory.tum` and
`<scan>.edits.yaml`, committed together. The trajectory is what gets scored
against; the edits file is the provenance -- what a person decided and why,
replayable against `rux optimize --edits` to reproduce the trajectory from
the project's stored (drifted) poses. Keep both: a trajectory with no edits
file is unreviewable (nobody can tell what judgement produced it), and an
edits file with no frozen trajectory is not yet a benchmark target.

### TUM format

One line per pose: `timestamp tx ty tz qx qy qz qw`, written by
`rux export trajectory -o x.tum` from a project's stored sensor-frame poses
(`sensor_frames.transform`, or `initial_transform` with `--initial`), ordered
by node id. `#`-prefixed and blank lines are ignored on read.
`timestamp` holds the frame's real capture time when the import recorded one,
or its node id cast to a double otherwise (house1 has no timestamps); either
way two trajectories of the same project line up, which is all the scoring
in `rux analyze poses` needs (it matches by `timestamp`, not node id --
`core/trajectory.hpp` explains why).

```bash
cp scan.rux /tmp/ref/                 # never touch a reference dataset in place
rux -p /tmp/ref/scan.rux export trajectory -o /tmp/ref/scan.trajectory.tum
```

## Scoring

```bash
rux -p scan.rux analyze poses --reference house2.trajectory.tum
rux -p scan.rux analyze poses --reference house2.trajectory.tum --json -o report.json
rux -p scan.rux analyze poses --reference house2.trajectory.tum --align se3
```

Reports, over poses matched by timestamp:

- **ATE**: RMSE / median / max translation error and RMS rotation error.
  `--align none` (the default) compares poses as stored -- right when both
  trajectories already share a gauge, e.g. the project's own current poses
  against its own frame-0-anchored reference. `--align se3` / `--align sim3`
  run a Umeyama alignment first (rigid, or rigid + scale) when they do not,
  e.g. scoring an independently-built trajectory.
- **RPE**: relative pose error at a fixed matched-index gap (`--rpe-frames`,
  default 10) and at a fixed path length travelled (`--rpe-distance`, default
  1 m). Needs no alignment -- a shared global offset cancels out of a
  relative comparison -- so it is informative even under `--align none`.
- **per-segment ATE** (`--segments`, default 12 contiguous chunks of the
  matched sequence): where along the trajectory the error concentrates,
  under the one whole-trajectory alignment.

With no `--estimate`, the project's own current poses are scored (mirroring
`rux export trajectory` without writing a file); `--estimate other.tum`
scores an independently exported trajectory instead.

### Benchmarking several pipeline configurations

`scripts/bench-poses.sh` runs several pose-stage configurations of a project
(`none`, `optimize`, `manhattan`, `register`, `opt-register`, ...) against a
reference, one JSON report (`scripts/bench-poses.manifest` documents the
columns and the per-config commands); `scripts/bench-poses-compare.py` gates
two such reports per metric, lower is always better for every scored metric
(ATE RMSE/median/max, rotation RMSE, both RPE pairs). `matched` and the
Sim3 `scale` are reported but never gate.

```bash
scripts/bench-poses.sh                                   # scripts/bench-poses.manifest
scripts/bench-poses-compare.py baseline.json candidate.json
```

A row with no committed reference yet (`kind=synthetic`, e.g. the
`office_corridor` fixture) derives a placeholder one: it exports the
project's own trajectory as the reference, injects fixed-seed synthetic
drift (`rux edit perturb-poses`), and scores every config against the
undrifted export. This proves the mechanics end to end with no external
dataset (reference export, perturbation, per-config scoring, report shape);
whether a config actually corrects the injected drift still depends on the
scan having enough structure for it to work with -- `office_corridor` is only
10 frames, too sparse for `rux optimize` to find plane landmarks once
perturbed, so it correctly leaves the poses unchanged rather than gating
anything. A real "worse vs. better" demonstration needs a scan with enough
structure (verified manually against an ARKitScenes copy for
`reusex-rcv.1.6`: `optimize` roughly halved ATE RMSE after a 0.2-ratio
perturbation). None of this is a substitute for a reviewed pose reference
once one exists for a scan.

## Marking a draft

A pose reference no maintainer has reviewed is a DRAFT, the same convention
as a solid-model reference plan: say so in the edits file's top-level
`note`/`description` field ("DRAFT - needs maintainer review") and keep the
renders the trajectory was checked against (the drift-correction loop's QA
output, `reusex-rcv.1.4`/`.5`) next to the review request, so the reviewer
sees what the author saw. The reviewer removes the mark when the reference is
accepted. Until then, a benchmark regression against a DRAFT reference is a
signal to look at, not a release blocker.

## Calibrating trust

Before trusting a loop-made reference on a scan with no independent ground
truth, calibrate the loop where the truth IS known: an ARKitScenes scan with
`rux edit perturb-poses` injected drift and its undrifted seed kept as truth
(`reusex-rcv.1.7`). `rux analyze poses --reference <undrifted seed>.tum` on
the perturbed project reports the injected drift magnitude; it should agree
with what `rux edit perturb-poses` itself reports (`median_position_error`,
`max_position_error`) -- the same consistency check this bead's acceptance
criteria run, and a sanity check worth re-running whenever either tool
changes.
