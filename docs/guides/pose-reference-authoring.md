<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Correcting drift with the pose loop

`rux optimize --edits e.yaml --watch` is the pose analogue of the floor-plan
loop in [solid-model-reference-authoring.md](solid-model-reference-authoring.md):
a person or an agent writes pose edits
([`docs/design/pose-edits.md`](../design/pose-edits.md)), saves, and gets a
re-solved trajectory, fresh renders and a metrics block within one cycle.
Background and measurements: `docs/research/visual-drift-loop.md`
(epic `reusex-rcv.1`).

## The loop

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

   Every cycle re-solves from the initial poses (edits never compound),
   back-projects a preview cloud with the candidate poses (the stored cloud
   is not touched, and is usually stale against the poses anyway), writes
   per storey `qa/h2.s<k>.passes.png` (plan cut, points coloured by capture
   segment), `.graph.png` (trajectory, edits as pins), `.drift.png`
   (pass-to-pass residual heatmap), `.plan.png` (with `--plan`) and one
   `qa/h2.front.png` elevation, and prints the block.
3. **Read the block.** The numbers that matter, in order:
   - `cross-segment` median (segment pairs matched only against each other)
     and the `worst segment pairs`: where the passes disagree;
   - `shear (untouched)`, printed next to them: how far the nodes that NO
     edit names moved against the solve without edits. A GT-free residual
     improves when an edit shears the whole trajectory (reusex-sb2,
     regression-221), so a gain with `<- SHEAR` is not a gain yet;
   - each edit's residual and `honoured` / `NOT HONOURED`;
   - `sampling`: the residuals depend on the frame step and depth stride
     (reusex-74q), so compare cycles only at the same `--drift-frame-step` /
     `--drift-depth-stride` / `--segments`.
4. **Paste a suggestion.** The `suggestions` are segment-ICP moves already
   refitted onto the initial poses the solve starts from, in the file's
   frame, as YAML items that paste under `edits:`. Paste ONE move per cycle
   (the first is the best; each is measured against all other segments as
   they are). A segment that changes storey is split, and the note says
   which nodes were left out. A suggestion whose range overlaps an existing
   move says `REPLACES <id>`: delete that one. When untouched segments
   moved, shear guards follow: one all-DoF anchor per moved segment at the
   pose the no-edit solve gives it.
5. **Look** at the passes / graph / drift PNGs: a doubled wall closing is the
   evidence, not the number. Keep an edit only if the residuals improve
   without shear and its own residual is small; otherwise revert it. A file
   that does not parse, or an edit that cannot be lowered, keeps the last
   good solve on screen and prints the error naming the edit.
6. **Finish without `--fast`**: the plane factors move the trajectory too,
   so check the final file with the full solve, then `--apply` on the copy
   and `rux create clouds`.

## A worked example: house2 (2026-10-16)

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
