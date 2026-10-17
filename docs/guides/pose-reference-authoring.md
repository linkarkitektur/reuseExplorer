<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Pose references: the correction loop and scoring against a frozen ground truth

A **pose reference** is the pose analogue of a solid-model reference plan
([`docs/guides/solid-model-reference-authoring.md`](solid-model-reference-authoring.md)):
a trajectory that a person (or the visual drift-correction loop,
`reusex-rcv.1`) has decided is "right enough to optimize against", frozen so
the automatic pipeline (`rux optimize`, `rux register`, the Manhattan prior,
...) can be scored against it. This guide covers both halves: authoring a
trajectory with `rux optimize --edits --watch` (below), and the file layout,
scoring and DRAFT marking convention for the frozen artefact the loop
produces (`## Pose references`).

## Correcting drift with the pose loop

`rux optimize --edits e.yaml --watch` is the pose analogue of the floor-plan
loop in [solid-model-reference-authoring.md](solid-model-reference-authoring.md):
a person or an agent writes pose edits
([`docs/design/pose-edits.md`](../design/pose-edits.md)), saves, and gets a
re-solved trajectory, fresh renders and a metrics block within one cycle.
Background and measurements: `docs/research/visual-drift-loop.md`
(epic `reusex-rcv.1`).

### The loop

```
rux analyze drift / the block  ->  write / edit e.yaml  ->  save
        ^                                                    |
        |                                                    v
        +---- look at the PNGs + metrics  <--  re-solve, preview, render
```

1. **Work on a copy** (opening a `.rux` can migrate it). The loop never
   writes the project unless `--apply` is given, and then only
   `sensor_frames.transform`, never `initial_transform`.
2. **Start with an empty edits file** and the loop:

   ```bash
   printf 'schema: reusex.pose_edits.v1\nframe: world\nedits: []\n' > e.yaml
   rux -p copy.rux optimize --edits e.yaml --watch --fast \
       --render qa/h2.png --json qa/metrics.json --plan plan.yaml
   ```

   At load the loop solves the graph once WITHOUT edits and with the plane
   factors on: the **reference**, exactly what plain `rux optimize` gives
   (the `reference` line; `qa/h2.ref.passes.png` / `.ref.front.png`).
   `--fast` cycles then solve on top of it -- odometry linearised at the
   reference, no plane factors -- so an edit corrects plain optimize's
   trajectory, not the drifted odometry (`reusex-rcv.1.21`). Every cycle
   re-solves from there (edits never compound),
   back-projects a preview cloud with the candidate poses (the stored cloud
   is not touched, and is usually stale against the poses anyway), writes
   per storey `qa/h2.s<k>.passes.png` (plan cut, points coloured by capture
   segment), `.graph.png` (trajectory, edits as pins), `.drift.png`
   (pass-to-pass residual heatmap), `.plan.png` (with `--plan`) and one
   `qa/h2.front.png` elevation, and prints the block.
3. **Read the block.** The numbers that matter, in order:
   - `cross-segment` median (segment pairs matched only against each other)
     and the `worst segment pairs`: where the passes disagree;
   - `vs plain optimize`: the same cross-segment median and pass-to-pass for
     the reference, and the verdict (`better` / `the same` / `WORSE than
     no edits`). An edit set that does not beat it is worse than doing
     nothing: keep no edit (`reusex-rcv.1.24`). Differences under about
     1 mm are sampling noise;
   - `shear (untouched)`: how the nodes that NO edit names moved against
     the reference, split by side of the last edited node: `upstream`
     (towards the gauge, frame 0) and `downstream` (the carried tail).
     Movement is not shear: drift accumulates from the gauge, so a correct
     move of segment k carries k+1..n with it and spreads its correction
     over the links before it. **Shear** is an untouched segment moved
     (> `--shear-flag`) to where it fits the other passes WORSE than in the
     reference (its cross-segment residual up by more than 1 mm and 10%);
     those are the nodes counted `in segments that now fit worse <- SHEAR`
     and the only ones a gain is discounted for (reusex-sb2,
     regression-221, `reusex-rcv.1.22`);
   - each edit's residual and `honoured` / `NOT HONOURED`. GNC alone
     judges each of a move's n priors (sigma * sqrt(n)), so it called a
     block 12 cm / 6.6 deg off its target honoured (`reusex-rcv.1.28`); an
     edit must also pass its own bound. An anchor or relative edit: its RMS
     residual within 2 sigma of its sigma in translation and rotation. A
     move (all DoF): the line under it says `realised T% / R% of <move>,
     <how far> from target, block bent <rms>` -- the block's rigid fit
     (initial -> solved) against the delta, and what is left over. It is
     honoured when it realised at least 80% of its delta, or ended within
     2 sigma of its target, in translation and in rotation. The bend is NOT
     judged: a `--fast` graph measures plain optimize's shape, so a moved
     block bends 0.5-3 cm away from its initial shape however well it is
     moved, which made almost every suggestion read NOT HONOURED by its RMS
     (`reusex-9ge`). `NOT HONOURED: the graph fought the move off` is
     therefore a real conflict -- another edit or guard holds the block --
     and the fix is to drop one of the two, not to tighten the sigma. A move
     restricted with `dofs:` keeps the RMS rule;
   - `sampling`: the residuals depend on the frame step and depth stride
     (reusex-74q), so compare cycles only at the same `--drift-frame-step` /
     `--drift-depth-stride` / `--segments`.
4. **Paste a suggestion.** The `suggestions` are segment-ICP moves already
   refitted onto the initial poses the solve starts from, in the file's
   frame, as YAML items that paste under `edits:`. Paste ONE move per cycle
   (the first is the best; each is measured against all other segments as
   they are). A segment that changes storey is split, and the note says
   which nodes were left out. A suggestion whose range overlaps an existing
   move says `REPLACES <id>`: delete that one. A suggestion that would
   replace a pasted suggestion is held back until that move's predicted
   gain is realised (its segment's residual fell by at least half the
   before -> after its note predicted; the note travels in the file, so
   this holds across `--watch-cycles 0` runs). The block prints
   `held back -- ...` with the numbers: drop the unrealised move rather than
   replace it. A REPLACES of a realised move says so (`reusex-rcv.1.28`:
   two moves of one segment used to alternate, each predicting a gain the
   other undid). The segment that holds the
   gauge (frame 0's prior) is never suggested: its move outvotes the gauge
   and drags the whole trajectory along (`reusex-rcv.1.23`); the block says
   so in a comment. When an untouched segment sheared, a shear guard
   follows: one all-DoF anchor at its middle node, at the pose plain
   optimize (the reference) gives it -- never at a drifted pose.
5. **Look** at the passes / graph / drift PNGs: a doubled wall closing is the
   evidence, not the number. Keep an edit only if the residuals improve
   without shear and its own residual is small; otherwise revert it. A file
   that does not parse, or an edit that cannot be lowered, keeps the last
   good solve on screen and prints the error naming the edit.
6. **Finish without `--fast`**: the plane factors move the trajectory too,
   so check the final file with the full solve, and keep it only if its
   `vs plain optimize` line says `better`; then `--apply` on the copy and
   `rux create clouds`.

### A worked example: house2 (2026-10-16)

On a copy of house2 (384 frames, two storeys, no stored pose graph:
odometry derived in memory), `--fast`, 12 segments, depth stride 8:

| cycle | edits | cross-segment median | pass-to-pass | shear (untouched > 5 cm) |
|---|---|---|---|---|
| 0 | none | 53.1 mm | 15.9 mm | 0 |
| 1 | top suggestion: seg 11 (358-389) split at the storey change -> move 358-378, + anchor 200 [x, y, yaw] | 40.3 mm | 14.8 mm | 166 nodes, segs 7-10 ~0.11 m |
| 2 | + the shear guard on node 385 | 41.3 mm | 14.9 mm | 165 |
| 3 | + anchors at 245, 277, 309, 341 [x, y, yaw] | 43.6 mm | 15.3 mm | 182 (vertical: z was left free) |
| 4 | the anchors on all six DoF | 43.3 mm | 16.0 mm | 10 (next to the move) |

Cycle 1's 13 mm gain was mostly shear; the honest gain at cycle 4 is about
10 mm. The upper storey's doubled long wall is still visible in
`s1.passes.png`: the move alone does not close it, which is the next thing
to look at. Cycle times (8 threads): `--fast` 4.9 s (the solve takes
0.1 s, the drift diagnostics 3.5 s, the preview cloud and nine images
1.3 s); the full solve 3.6 s more. The depth is read once, at load, into a
frame cache (12 MB here, reported on the `frame cache` line); each cycle
only re-transforms it (reusex-rcv.1.17). NewOffice (3876 frames) runs a
`--fast` cycle in 12.4 s.

## Pose references: scoring a trajectory against a frozen ground truth

This half covers the artefact the loop above produces once a cycle's result
is accepted: the file layout, how it is scored, and the DRAFT marking
convention -- not how to find and fix drift (that is the loop section
above).

### File layout

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

#### TUM format

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

### Scoring

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

#### Benchmarking several pipeline configurations

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

### Marking a draft

A pose reference no maintainer has reviewed is a DRAFT, the same convention
as a solid-model reference plan: say so in the edits file's top-level
`note`/`description` field ("DRAFT - needs maintainer review") and keep the
renders the trajectory was checked against (the drift-correction loop's QA
output, `reusex-rcv.1.4`/`.5`) next to the review request, so the reviewer
sees what the author saw. The reviewer removes the mark when the reference is
accepted. Until then, a benchmark regression against a DRAFT reference is a
signal to look at, not a release blocker.

### Calibrating trust

Before trusting a loop-made reference on a scan with no independent ground
truth, calibrate the loop where the truth IS known: an ARKitScenes scan with
`rux edit perturb-poses` injected drift and its undrifted seed kept as truth
(`reusex-rcv.1.7`). `rux analyze poses --reference <undrifted seed>.tum` on
the perturbed project reports the injected drift magnitude; it should agree
with what `rux edit perturb-poses` itself reports (`median_position_error`,
`max_position_error`) -- the same consistency check this bead's acceptance
criteria run, and a sanity check worth re-running whenever either tool
changes.

The first calibration (2026-10-16, nine cells, `scripts/bench-drift-loop.sh`,
[visual-drift-loop.md §9](../research/visual-drift-loop.md#9-calibration-where-the-truth-is-known-2026-10-16-reusex-rcv17))
found that the loop above, driven by its own rules, does **not** yet beat
plain `rux optimize`: the shear guards pin untouched nodes at their drifted
`--fast` poses, the shear indicator flags a correct downstream correction,
and suggestions on the gauge segment move the whole trajectory
(`reusex-rcv.1.21`-`.24`). With those fixed (the reference, fit-based shear,
no gauge suggestions) the re-run ends <= plain `optimize` on 8 of 9 cells
and better on 4 ([§9.7](../research/visual-drift-loop.md#97-re-run-after-the-fixes-2026-10-16-reusex-rcv121-24));
the one loss was flagged `WORSE than no edits` by its own block. So: keep
the final edits only when the full solve's `vs plain optimize` verdict is
`better`, compare the passes cut with `.ref.passes.png`, and treat a
loop-made reference as DRAFT until scored.
