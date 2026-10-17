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
   other undid). A suggestion whose block holds a node an anchor pins (a
   shear guard pasted earlier) says `OVERRIDES <id>`: the two conflict by
   construction -- the guard holds its node where plain optimize had it,
   the move carries the block away -- so drop the guard with the move
   (`reusex-r5z`; keeping the guard and moving only the free part of the
   block ended 15 mm worse in the calibration). The segment that holds the
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

### The house2 reference, start to DRAFT (2026-10-17, `reusex-rcv.1.8`)

The worked example above stopped at "the doubled wall is the next thing to
look at". This one carries house2 through to the committed DRAFT,
`tests/fixtures/pose_reference/house2.{edits.yaml,trajectory.tum}`. The
edits file's header has the full provenance, the replay command and the
per-crossing notes; this section is how it was found.

**0. Record what you start from.** On the plan's copy of house2 (sha256
`118645e0...`, schema v32) all 384 `initial_transform`s are NULL and the
pipeline log has no optimize/register entry: the solve seeds from
`sensor_frames.transform` as the RTABMap import wrote it (`reusex-m53`).
Plain `rux optimize` on a second copy gives the reference trajectory, and
`rux analyze drift --plan house2.plan.yaml` on both gives the baseline:
pass-to-pass 15.9 mm stored, 13.8 mm plain optimize (floor 7.2 mm),
segment-pair median 53.1 / 28.4 mm, crossings 8/13 / 7/12.

**1. Suggestions alone made it worse.** The top suggestions (seg 11 split
at the stair, then seg 3) both ended `WORSE than no edits`: a move of one
segment carries the rest of the trajectory with it in `--fast`, which has
no plane factors to hold it, and the segment ICP had measured the move
against the others *as they were*. The fix that made every later step
legible: guards on every segment no move names (all-DoF anchors at the
plain-optimize pose, three per segment, sigma 1 cm / 0.2 deg). With them a
move is measured and solved against plain optimize's trajectory, and
`shear` stays at 0.

**2. Look, do not trust a colour.** The reference's storey-0 passes cut
showed rotated yellow walls through the hall and the bathroom. With 12
segments and 8 pass colours, seg 3 and seg 11 are both yellow; a cut that
colours single nodes (here a matplotlib back-projection of the frame depth
in the level frame) showed it was 384-389 alone, the last six frames. Their
colour images (rotate the iPad frames 90 deg to read them) show the walk
down the stair into living room S and down the tiled hall to the back
door, with motion blur. Plain optimize had put cameras 388-389 north of the
hall wall (the stored `388 -> 389` through-wall crossing).

**3. Fit from a start the images allow.** A 2D wall-band ICP of 386-389
against passes 1-245 from yaw grid starts alone found a wrong minimum (yaw
+12 deg, the hall walls landed on the kitchen walls one room north, cameras
inside a wall). Starting from a shift that puts the cameras in the hall
(the images say they are) gave within-3-cm 0.27 -> 0.89. A section along
the hall then showed the tail 15 cm low and tilted: a 6-DoF point-to-plane
refinement brought floor, ceiling and the back-door wall onto passes 1-69.
384-385 the same way (within-3-cm 0.16 -> 0.99). Each fit became a `move`
on the stored poses, with the images, the cut and the numbers in its note.

**4. Two solver flags the reference needs.** The tail's odometry links are
wrong by 10-13 deg. With odometry as known inliers the tail moves either
dragged the whole graph (up to 1.7 m) or were fought off (41-62%
realised); `--odometry-robust` lets GNC drop the broken links and leaves
plain optimize unchanged (28.4 / 13.8 mm). The best remaining suggestion
was seg 0 (35.9 -> 13.5 mm), which holds the gauge: `--prior-sigma-trans 1
--prior-sigma-rot 1` frees node 1 and the guards fix the gauge instead. Its
section showed why it was worth it: seg 0's floor 4-5 cm above seg 1's and
tilted, the end of the hall's north wall doubled.

**5. Check the full solve, then refit once.** Without `--fast`, the plane
factors pulled 384 back onto the ghost (`NOT HONOURED`, realised 62%):
its move needed sigma 5 mm / 0.1 deg. After that, refitting the two tail
moves on the edited full solve (they had been fitted on the fast one) took
the segment-pair median from 24.5 to 23.7 mm.

**6. Show the plateau.** Kept: the two tail moves, seg 0, seg 3 and seg 4
(each `better` and visibly closing walls). Dropped, each tried in its own
cycle: seg 1 (twice, with sigma 5 mm and 2 cm: pass-to-pass 11.0 -> 11.5 /
11.7 mm), seg 11 without the tail (11.2 mm, storey-1 doubled share 0.22 ->
0.24), a hand move of seg 7's living-room frames (pass-to-pass and both doubled
shares worse, segment-pair median unchanged), seg 8
(a wash). The remaining suggestions predicted <= 5 mm on one segment.
A suggestion whose `move` refits onto the stored poses with a large
residual (45 mm for seg 1 and seg 11) cannot be expressed as one: the stored
shape of that block differs from plain optimize's, and the move pulls it
back.

| | stored | plain optimize | DRAFT reference |
|---|---|---|---|
| pass-to-pass median (p90) | 15.9 (56.1) mm | 13.8 (43.8) mm | **11.0 (36.2) mm**, 1.5x the floor |
| frames above the flag | 63 of 248 | 26 of 250 | 14 of 258 |
| segment-pair median | 53.1 mm | 28.4 mm | 23.7 mm |
| worst segment pair | 130.3 mm | 105.2 mm | 116.0 mm |
| through-wall crossings | 8 of 13 | 7 of 12 | 6 of 11, all at doorways |
| storey-0 doubled-wall patches | 197/724 (0.27) | 193/737 (0.26) | 190/703 (0.27) |

What the numbers miss, read from the renders:

- The worst pair (seg 0 into seg 2) is the two faces of the 0.18 m bathroom
  wall, seen from either side, falling inside the 0.3 m pair gate once
  seg 0 is corrected; in plain optimize they were 0.5 m apart, outside it.
  A worst-pair bound of 2x the floor (`reusex-rcv.1.8`) cannot be met on a
  scan with thin walls seen from both sides.
- The doubled-wall share barely moves: mapping the bimodal patches of both
  solves shows most are common to every solve (the stair treads, kitchen
  units, furniture against walls); the ones the tail fix removed are offset
  by a few new ones. Draw doubling, do not score it (§3.4 of the research
  note says the same).
- No pass-to-pass number sees the storeys' headings: storey 1 sits ~4 deg
  off storey 0 over the gable room and ~8 deg in the north, and only the
  stair links them. DRAFT v1 left this; the review asked for it (below).

### house2 v2: the Manhattan rule (2026-10-17, PR #84 review)

The maintainer's review of v1 added an acceptance rule: once straightened,
every wall is at 90 deg to the others, open doors excepted. Pass-to-pass
residuals cannot see a heading error (each pass agrees with itself), so
this needs its own number.

**Measure the grid first.** Two measures, both against one grid per storey
estimated the way the Manhattan prior does it (`PlaneGraphOptimizer.cpp`:
weighted circular mean of 4*phi of the wall normals, re-centred three
times over the walls within 15 deg):

- **wall planes**: `rux create clouds / planes / storeys / classify` on the
  candidate copy, every vertical plane with role `wall`, its normal refitted
  by PCA in its storey's level frame, weighted by area;
- **wall patches**: the depth back-projected with the trajectory, wall-band
  points (|n_z| < 0.15) in 0.5 m cells, a PCA normal per planar patch at
  least 0.3 m along the wall; and per **segment**, the grid of its own wall
  points against the storey-0 grid: the segment's heading error.

Door leaves are excluded: a plane shorter than 1.2 m along the wall whose
top stays more than 0.25 m below the ceiling (patches: a run of patches
shorter than 1.2 m whose top, read from the unbanded cloud, stays below
floor + 2.3 m). A height-only rule is wrong on storey 1: its knee walls
under the roof slope never reach the band's top.

On plain optimize the segment headings ran +5.5 deg (seg 0) down to -8.4
deg (the stair descent), and storey 1's grid sat -4.4 deg off storey 0's.

**`--manhattan` first, then the guards from its solve.** The prior alone
straightens most of it (storey grids 0.1 deg apart, segment headings median
1.1 / max 3.9 deg, pass-to-pass 12.6 mm). Run with the v1 edits as they were
it changed nothing: the v1 guards pinned every segment at plain optimize's
rotated poses. So the guards now come from the Manhattan solve, and the v1
moves were refitted on it (the tail fits carried over relative to passes
1-245, then a 6-DoF point-to-plane refine).

**Heading moves for what the prior cannot reach.** Frames tie to landmarks
through plane factors with a 14 deg normal sigma, so a block that sees a
room only from inside keeps its heading. Four blocks did: 246-293 (the
gable room, the review's "seg 5", +2.1 deg), 326-357 (-3.3), 358-379 (-4.3)
and 230-245 (-2.7). Each was turned about its camera centroid onto the
grid, then its translation refitted against all other passes, all four
together, twice; each became a `move`. The remaining v1 suggestions were
re-tried on the new base: seg 0 and seg 4 kept, seg 3 dropped (pass-to-pass
11.2 mm, segment-pair median 29.3 mm, both worse).

**The noisy right side of the ground floor** (review point 1) was the stair
bottom: a cut that colours single nodes showed frames 380-383 drawing the
stair-hall east jamb 0.35 m west of where 378-379 and passes 102-245 draw
it. A 2D fit (within 3 cm 0.52 -> 0.99) and a 6-DoF refine made the fifth
new move.

| | plain optimize | DRAFT v1 | DRAFT v2 |
|---|---|---|---|
| pass-to-pass median (p90) | 13.8 (43.8) mm | 11.0 (36.2) mm | **10.4 (34.5) mm** |
| segment-pair median | 28.4 mm | 23.7 mm | 25.6 mm |
| storey-1 grid vs storey 0 | -4.39 deg | -5.54 deg | **-0.03 deg** |
| segment heading, median / max | 3.87 / 8.38 deg | 2.80 / 9.35 deg | **0.25 / 1.19 deg** |
| wall planes s0: median, area within 1 / 2 deg | 2.10, 41 / 51% | 2.15, 28 / 58% | 0.65, 50 / 92% |
| wall planes s1 | 2.28, 23 / 49% | 2.26, 23 / 49% | 1.47, 43 / 77% |
| through-wall steps, scan's own walls | 4 | 3 | 1 (an open door leaf) |

The per-wall target (every wall within +-1 deg) is not met: the spread left
on storey 1 lies within rooms seen in one pass, which a whole-block move
cannot reach (`reusex-wg8`: a "this block is Manhattan" edit). Two lessons:

- Straightening costs a little segment-pair agreement (23.7 -> 25.6 mm):
  the v1 trajectory had found self-consistent but rotated passes. Report
  both numbers; neither alone decides.
- Through-wall crossings against `house2.plan.yaml` rose from 6 / 11 to
  11 / 20, because that plan was drawn on the stored, rotated poses. Count
  crossings against the scan's own wall points under the same poses instead
  (a step whose line passes within 3 cm of them), until the plan is re-fit
  (`reusex-ghy`).

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
as a solid-model reference plan: say so in a comment header at the top of
both files (the reader accepts no top-level `note` key; `#` lines are
comments in YAML and in TUM), with the source scan, the poses the solve
starts from, the replay command with every solver flag the reference needs,
the before/after numbers and the unresolved areas
(`tests/fixtures/pose_reference/house2.edits.yaml` is the example), and
keep the renders the trajectory was checked against (the drift-correction
loop's QA output, `reusex-rcv.1.4`/`.5`) next to the review request, so the
reviewer sees what the author saw. The reviewer removes the mark when the reference is
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
