<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# A visual drift-correction loop, and a pose ground truth to optimize against

**Status:** research write-up, epic `reusex-rcv.1` (child of `reusex-rcv`),
2026-10-16. Quick measured experiments with throwaway Python over read-only
copies of house1, house2 and NewOffice; no production code. The plan itself
lives in the beads (`bd show reusex-rcv.1`), not here.

**Question (maintainer, 2026-10-16):** "use the similar workflow to improve
drift and optimize the point cloud ... visualize the cloud and graph, find
places where there is drift, correct the nodes by updating their position
interactively and that should then recompute the pose graph, until it looks
alright. This way we then have a ground truth that we can optimize against.
I'm happy for any other whacky and out there ideas."

"The similar workflow" is the solid-model reference loop: a hand-authored
`plan.yaml`, `rux render --plan … --watch` / `rux view --plan … --watch`
hot reload, QA renders (`rux render --qa`, sections, elevations, deviation
layer) and a scored benchmark (`rux analyze solid`, `scripts/bench-solid.sh`)
([guide](../guides/solid-model-reference-authoring.md),
[format](../design/solid-model-json.md), [ideas lab](solid-model-ideas-lab.md)).

---

## 0. TL;DR

- **The loop is feasible and most of its parts already exist.** The stored
  pose graph (schema v28), repeatable re-solves from `initial_transform`,
  `frame_pair_icp`, the Manhattan prior, `rux render` with `--watch`, the QA
  sheet and the drift benchmark are all there. Missing: a place to write
  **node-level** corrections (anchors, block moves, "these two walls are one"),
  the solver hooks to obey them, render layers that make drift *visible*
  (colour by capture pass, the graph, a drift heatmap), and a pose-scoring
  tool.
- **Where to look can be found automatically.** A per-frame *pass-to-pass
  residual* (a frame's depth against frames captured > 30 s earlier or later)
  sits far above the sensor noise floor on the drifted houses (house1
  15.6 mm vs 3.6 mm, house2 16.0 mm vs 7.2 mm) and responds to `rux optimize`
  on NewOffice (frames above 30 mm: 38 → 19). Per-segment rigid ICP turns
  a hotspot into a concrete suggested edit (house2, frames 358-389:
  65 → 31 mm residual for a 0.19 m / 1.2° move).
- **Pooled numbers are blind; pairwise ones are not.** A synthetic 1-2°
  break in the middle of house1 barely moves any global median (14.1 →
  14.5 mm) or the doubled-wall share (0.345 → 0.374). The detector has to
  compare *passes* (segment x segment), never the whole cloud, which is the
  regression-221 lesson in a new form.
- **The ground truth is a trajectory plus the edits that made it,** scored
  with ATE/RPE *and* GT-free consistency (pass-to-pass residual,
  through-wall crossings, plan support), never a start-end gap or planarity
  alone. Before trusting it, calibrate the loop where the truth is known
  (ARKitScenes + `rux edit perturb-poses`).
- **Most promising whacky ideas** (§5): wall doubling as an explicit factor,
  the floor plan as fixed landmarks, rigid sub-maps glued by overlap.
  Measured: per-frame plan fitting is too noisy to anchor anything by itself;
  rigid sub-maps cut the cross-segment residual by 30 % with plain ICP.

## 1. What exists

| Piece | Where | Relevance |
|---|---|---|
| Stored pose graph (v28): `pose_graph_edges` with measurement, GTSAM-order information, `source`, `status` (auto/confirmed/rejected), metrics (`residual`, `deviation_*`, `seed_disagreement_*`); `sensor_frames.initial_transform` | `core/pose_graph.hpp`, `ProjectDB` | The source of truth; every solve starts from the initial poses, so a re-solve after an edit is repeatable, not compounding (pose-graph editor design D2) |
| `rux graph init/discover/prune` | `slam/pose_graph.hpp`, `graph_discover.hpp`, `graph_prune.hpp` | Loop-edge front-ends (ORB, panorama, file), PCM with confirmed edges as fixed inliers |
| `rux optimize`: plane-landmark GNC solve of the stored graph; opt-in Manhattan prior (PR #44) | `slam/PlaneGraphOptimizer.hpp` | The only **absolute** factor is the frame-0 gauge prior; everything else is relative. Confirmed loop edges are GNC known inliers |
| Pair tools: `refine_frame_pair_icp` (point-to-plane), `frame_pair_matches` (ORB + RANSAC fit) | `slam/frame_pair_icp.hpp`, `frame_pair_matches.hpp` | Turn "these two frames see the same thing" into a measured edge |
| Qt Rediger (`EditWorkspace`, `GraphPanel`, `PairImages`/`MatchCanvas`, pair strip) | `apps/rux/qt` | Edits **edges** between a pair A/B (move B with a box widget, ICP, confirm/reject); no node anchors, no range moves |
| `rux render`: frustums, panoramas, `--qa` sheet, sections, deviation layer, `--plan`, `--watch` (+ `--watch-project`) | `visualize/`, `apps/rux/src/render.cpp` | The render half of the loop; no graph, pass-coloured or drift layer |
| Drift benchmark: `rux edit perturb-poses` (seeded OU-bias + random-walk drift, frame 0 pinned), `scripts/bench-arkitscenes-drift.sh` | `slam/perturb_poses.hpp` | Ground truth known: the place to calibrate a loop (§4.7) |
| `tools/gt_curator` (#221, Gradio) | `tools/gt_curator/` | Prior art of exactly this idea: manual frame-pair anchors → GTSAM solve → before/after render |
| GT floor plans (DRAFT) | `tests/fixtures/solid_model/*.plan.yaml` | NewOffice's walls are within 2° of the grid (median 0.12°); house1/house2 follow the scan **as registered**, drift included (their headers say so) |

**The gt_curator lesson (docs/research/regression-221-investigation.md).** Its
anchors were relative edges between the start and the end of NewOffice, with
loosened odometry. The solve spread 9.8 m of correction linearly along 3875
odometry links: a global shear that *doubled* the entrance walls, while every
GT-free metric (start↔end gap, flatness, thickness) improved. Two design rules
follow: (1) a person must be able to pin the **middle** of a trajectory
(absolute or block constraints, not only endcap edges), and (2) the loop must
show and score **pass-to-pass** consistency, because local planarity cannot see
a doubled wall: region growing splits it into two flat planes.

**State of the datasets** (copies only):

| | house1 | house2 | NewOffice |
|---|---|---|---|
| Frames | 238 (no timestamps) | 384 (1 Hz) | 3876 |
| Stored graph | none | none | 3875 odometry_init + 102 ORB loop edges |
| `initial_transform` | no | no | yes; `transform` differs by 0.145 m median (optimized) |
| Stored cloud built from | current poses | current poses | **initial** poses (20 mm back-projection residual vs 51 mm for the current poses) |
| Storeys | 1 | 2 (own level frames, ~0.5-1° apart) | none (reference floor at z = -1.27) |

The NewOffice row is a finding in itself: `rux optimize` ran after
`create clouds`, nothing warned, and the NewOffice reference plan was drawn on
that cloud, i.e. in the initial-pose frame (bead `reusex-rcv.1.15`).

## 2. The design in one picture

```
 drift detectors ──► where to look (segment pairs, heatmap cells, crossings)
        │                         │
        ▼                         ▼
 QA renders: plan cut / sections coloured by capture PASS, graph, drift heatmap
        │                                   ▲
        ▼                                   │
 person / agent writes pose-edits.yaml ──► rux optimize --edits --watch
   anchor · move (node range) · same_surface ·  (re-solve from initial poses,
   relative · on_plan_wall · reject_edge         preview cloud, metrics block)
        │
        ▼  "looks alright" + metrics within bounds
 freeze: trajectory.tum + edits.yaml  (DRAFT pose reference)
        │
        ▼
 rux analyze poses / bench-poses.sh  ──► optimize the automatic pipeline
```

## 3. Measurements

All on copies; depth back-projected with `sensor_frames.transform` (or
`initial_transform`) ∘ `local_transform` and the stored intrinsics, depth
stride 4-8, 0.2-4 m. The back-projection convention was checked first: frames
land within 2 cm (median) of the stored house2 cloud.

### 3.1 Pass-to-pass residual: drift vs the noise floor

For every back-projected point, the point-to-plane distance (local PCA normal)
to the nearest point within 0.3 m that came from a frame > 30 s away
(house1: > 30 frames). The noise floor is the same measure against frames 1-3
away, where drift cannot have accumulated.

| | noise floor (median / p90) | pass-to-pass (median / p90) | ratio | frames with median > 30 mm |
|---|---|---|---|---|
| house1 | 3.6 / 12.2 mm | 15.6 / 60.3 mm | 4.3x | 29 of 130 |
| house2 | 7.2 / 21.6 mm | 16.0 / 56.1 mm | 2.2x | 64 of 248 |
| NewOffice, `initial_transform` | 7.9 / 27.3 mm | 10.9 / 38.4 mm | 1.4x | 38 of 958 |
| NewOffice, `transform` (optimized) | | 10.6 / 36.4 mm | 1.3x | **19** of 957 |

The measure separates the drifted houses from the noise floor and responds to
`rux optimize` on NewOffice. Its segment x segment form localises: on house2
(16 time segments) most pairs sit at 10-25 mm, a handful at 40-60 mm
(segments 8-9 vs 2, 4 vs 9, 7 vs 14, 15 vs 2 and 11).

### 3.2 Sensitivity: pooled metrics do not see a pass break

A rigid break injected at the middle of house1 (second half rotated about the
mid-trajectory camera, or shifted in x):

| break | pooled pass-to-pass median | doubled-wall patch share | patch spread median |
|---|---|---|---|
| none | 14.1 mm | 0.345 | 69.0 mm |
| 0.5° yaw | 15.0 | 0.343 | 71.3 |
| 1° | 14.5 | 0.360 | 72.4 |
| 2° | 14.1 | 0.374 | 74.2 |
| 0.10 m | 14.4 | 0.362 | 75.3 |

Neither pooled number is a usable drift metric. The reason is dilution: the
nearest far-in-time point is usually from another frame of the *same* half, and
furniture, reveals and sensor noise produce same-facing parallel layers
everywhere. Restricting the match to the other half (first-half points against
second-half points only) is what works, and it found more than the injected
break: the two halves of the **unmodified** house1 already disagree at 69 mm
against 32 mm within halves, and shifting the second half 0.10 m in x
*lowers* the across-half residual to 55 mm. So the cost is minimisable, which
is what §3.3 exploits.

### 3.3 Segment ICP turns a hotspot into an edit

Each of 12 time segments of house2, aligned rigidly (robust point-to-plane ICP,
Huber 2 cm, 0.2 m gate) to the union of all the others:

| segment (node ids) | residual before → after | suggested move |
|---|---|---|
| 1-33 | 55.3 → 26.4 mm | 0.12 m, 1.0° |
| 34-69 | 32.0 → 19.2 mm | 0.16 m, 2.6° (yaw -2.4°) |
| 102-133 | 23.5 → 14.3 mm | 0.23 m, 2.0° |
| 198-229 | 43.3 → 40.2 mm | 0.20 m, 1.4° (non-rigid: needs splitting) |
| 230-261 | 36.4 → 35.4 mm | 0.17 m, 1.8° (same) |
| 358-389 | 65.0 → 31.3 mm | 0.19 m, 1.2° |
| the other 6 | 17-27 → 15-27 mm | ≤ 0.12 m |

Where a rigid move helps, it halves the residual; where it does not
(198-261), the segment is internally inconsistent and must be split, which
the loop can show as well. These rows are what `optimize --watch` should print
as ready-to-paste `move` edits.

### 3.4 Doubled walls: visible, but a weak metric without provenance

Two detectors on the wall band (1.2-2.3 m above the floor, |n_z| < 0.3):

- *Same-facing bimodal patches*: per 0.5 m cell and 20° normal bin, the
  offsets of points along the mean normal; a patch is doubled when the smoothed
  1 cm histogram has two peaks ≥ 4 cm apart with a valley ≤ 60 % of the smaller.
  Stored clouds: house1 0.13, house2 0.26 / 0.24 (storeys 0 / 1), NewOffice
  0.45 of patches, median layer separation 0.09-0.13 m. On clouds fused here
  from the current poses (2 cm voxel): house1 0.35, house2 0.39, NewOffice 0.39.
- *Rotated copies*: per 1 m cell, two wall-normal azimuth modes 3-25° apart:
  house1 0.06, house2 0.18 (storey 0) / 0.08, NewOffice 0.40 (furniture).

On house2 the bimodal patches sit on the ghosted interior walls named in the
plan's header, and the rotated-copy cells pick out the diagonal ghost lines
(around (-1.4, -6.3) and (0.4, -2.5) in the storey-0 frame). But §3.2 shows the
share barely moves with a real break: furniture and sensor noise dominate it.
Doubling is worth *drawing* (a passes-coloured render makes two copies two
colours) and worth turning into constraints between the passes that produce it
(§5.1), not worth scoring by itself.

### 3.5 The camera cannot walk through a wall

Trajectory steps (≤ 1.5 m) that cross a reference-plan wall axis, and whether
the crossing lies within 0.25 m of an opening on that wall:

| | crossings | at an opening | through a wall |
|---|---|---|---|
| house1 | 16 | 16 | **0** |
| house2 | 13 | 5 | 8 (7 different walls, both storeys) |
| NewOffice (initial or current poses) | 42 | 33 | 9 (5 on `p-storage-glass`, 4 on `p-wc-south`) |

house1 is the clean control. On house2 the count is confounded by the plan
(it draws only the openings it could read), on NewOffice it agrees with the
known misplaced storage door (`reusex-g75`). Either way, a through-wall step
is a place to look, and it shares code with the door-evidence bead
`reusex-rqn.3`.

### 3.6 Plan as landmark: per-frame fits are too noisy

Each frame's wall-band points fitted to the reference plan's wall faces with
a robust 2D point-to-line ICP (x, y, yaw about the camera), keeping frames that
see two wall directions:

| | frames fitted / well-conditioned | plan support (< 5 cm) before → after | agree within 5 cm and 1° | off by > 15 cm or 3° |
|---|---|---|---|---|
| NewOffice, `initial_transform` (the poses the plan was drawn on) | 315 / 110 | 0.33 → 0.49 | 22 % | 47 % |
| house2, `transform` | 193 / 62 | 0.37 → 0.59 | 11 % | 55 % |

On NewOffice the right answer is "no correction", since the plan was measured
from this very cloud, and still half the frames want more than 15 cm or 3°.
A single iPad frame (256 x 192 depth, a partial view with furniture) does not
pin itself to a plan. The plan can only enter as many weak, associated, GNC-
guarded plane factors (§5.2), never as per-frame alignment.

### 3.7 Rigid sub-maps

The 12 house2 segments as rigid bodies, block-Jacobi aligned to each other
(half-step damped point-to-plane ICP, segment 0 fixed, no odometry, no graph):
cross-segment residual median 53.0 → 43.4 → 41.5 → 39.4 → **37.2 mm** in four
rounds, p90 103 → 72 mm, with more segment pairs coming into overlap (71 →
83). Rigid blocks break continuity at their seams, which the graph would
absorb. As a coarse first stage it is cheap and robust (§5.3).

## 4. Design of the loop

### 4.1 Pose-edits document (`reusex.pose_edits.v1`)

A YAML (hand) or JSON (tools) file, the pose analogue of `plan.yaml`. Comments
carry provenance: which render showed what, which colour images were checked.

```yaml
schema: reusex.pose_edits.v1
project_guid: "…"          # guards against editing the wrong project
frame: world               # or storey:<guid> (level frame)
edits:
  - id: m-late-pass
    kind: move             # rigid delta for a node range (a pass)
    nodes: [358, 389]
    delta: {xyz: [0.15, -0.11, 0.0], yaw_deg: 1.2}
    sigma: {trans_m: 0.03, rot_deg: 0.5}
    note: "seg 358-389 vs rest 65 -> 31 mm (analyze drift); plan cut passes.png: kitchen wall doubled 12 cm"
  - id: a-1203
    kind: anchor           # absolute prior on one node, DoF mask
    node: 1203
    pose: current
    dofs: [x, y, yaw]
    sigma: {trans_m: 0.02, rot_deg: 0.3}
  - id: s-hall-west
    kind: same_surface     # two ghost copies are one wall: rank-1 constraint
    a: {nodes: [1, 33],   region: [-5.2, -3.4, -4.8, -2.0]}
    b: {nodes: [358, 389], region: [-5.2, -3.4, -4.8, -2.0]}
    sigma_m: 0.01
  - id: w-x-north
    kind: on_plan_wall     # points of a node range lie on plan wall W
    plan: tests/fixtures/solid_model/newoffice.plan.yaml
    wall: x-north
    nodes: [2300, 2400]
  - {id: r-412-980, kind: relative, a: 412, b: 980, pose: current}
  - {id: x-orb-17, kind: reject_edge, a: 17, b: 3611, edge_type: loop_closure}
```

Records live in `core` and the reader/writer in `io`, the solid-model split, so
`slam` can consume edits without linking `io` (peers, STANDARDS §1). The Qt
client edits the same file, so a person in Rediger and an agent on the file are
interchangeable (`reusex-rcv.1.16`).

### 4.2 Lowering into the solve

| edit | factor(s) | why this form |
|---|---|---|
| anchor | `PriorFactor<Pose3>`, information ~0 on masked DoFs | pins the middle of a trajectory, the gap gt_curator had |
| move | a prior per node of the range at `delta · initial`, sigma x √n | the block carries one anchor's authority; its odometry keeps it rigid inside |
| relative | `BetweenFactor`, `known_inlier` | identical to a confirmed manual edge |
| same_surface | ICP between the two regions' points → `BetweenFactor` with information `JᵀWJ` (rank ~1-3, along the wall normal) | constrains only what the doubled wall actually measures |
| on_plan_wall | the same against the plan face | a person's "this is wall W" |
| reject_edge | tombstone in memory | as `status = rejected` |

Edit factors are known inliers for GNC, but with a large *finite* TLS threshold
(like `loop_trust_inlier_cost`) so a contradictory edit is reported with its
residual rather than obeyed silently. No edits must give bit-identical poses.

### 4.3 `rux optimize --edits e.yaml --watch`

Same shape as `rux render --plan --watch`: render once, then on each save of
the edits file re-solve from the initial poses (in memory unless `--write`),
rebuild a **preview cloud** straight from the stored depth with the new poses
(seconds; never trust a stored cloud, §1), re-render the QA panels and print a
metrics block (and `--json` for agents):

- each edit's residual and inlier flag; pose change vs the last cycle and vs
  initial;
- pass-to-pass residual median/p90 vs the floor, the 5 worst segment pairs,
  the top 3 segment-ICP suggestions **as ready-to-paste YAML**;
- through-wall crossings; plan support per wall with `--plan` (reusing
  `render --watch`'s wall-support block);
- a broken file keeps the last good solve and prints the reader's error.

`--fast` drops the plane factors (odometry + loops + edits only) so a cycle on
NewOffice takes seconds; the final reference is solved with the full graph.

### 4.4 Render layers

- `passes`: every point coloured by the time segment of its frame (label token
  scale). The one layer that makes ghosting obvious: two copies, two colours.
- `graph`: trajectory polyline + stored edges coloured by deviation/residual
  (Rediger's χ² scale) + edit pins.
- `drift`: the per-cell pass-to-pass residual as a heatmap with legend.

All three in `populate_scene`, so `rux render`, `rux view` and the Qt 3D
workspace agree; the QA sheet gains a passes plan panel and a drift panel.

### 4.5 Detectors, ranked by the evidence above

| detector | evidence | role |
|---|---|---|
| segment x segment pass-to-pass residual | §3.1-3.2: separates drift from the floor; sees a break that pooled numbers miss | primary "where" and the acceptance metric |
| segment ICP suggestion | §3.3: concrete moves that halve the residual | primary "what to do" |
| through-wall crossings (needs a plan) | §3.5: 0/16 on the clean house, 8/13 on house2 | cheap, also catches plan errors |
| Manhattan residual vs time | `reusex-rqn.1`: wall azimuth trends 7-15° over the capture on the houses (corr. -0.75 / -0.94) | rotational drift between unrevisited rooms, where pass-to-pass has no overlap |
| stored-edge inconsistency (`deviation_*`, `seed_disagreement_*`) | written by every `rux optimize` | which loop edges fight the solve |
| doubled-wall patches | §3.4: visible, but a weak score | drawing + seeds for same_surface |
| plane flatness by region | regression-221: blind to doubling | context only |

### 4.6 The agent loop

An agent runs the same loop from files: read the `--json` block, take the worst
segment pair or heatmap cell, render a zoomed plan cut and two sections there
with `--layers passes,graph`, **look** at them and at 2-4 colour images of the
frames involved, then accept a suggestion or write its own edit with a note
that cites the PNG. Keep an edit only if the pass-to-pass residual and plan
support improve and the edit's own residual is small, otherwise revert.
Guardrails: a maximum delta per edit unless the images justify more, one edit
per node range per cycle, stop after K cycles without gain, and a held-out 20 %
of segment pairs that the acceptance test never sees, to catch an agent (or a
person) optimising the metric instead of the building.

### 4.7 The pose ground truth, and how to trust it

**Format:** `tests/fixtures/pose_reference/<scan>.trajectory.tum` (node id or
timestamp, tx ty tz qx qy qz qw) + `<scan>.edits.yaml` + the solver settings,
DRAFT until the maintainer reviews it (as decision D5 for plans).

**Scores** (`rux analyze poses --reference`, `scripts/bench-poses.sh`): ATE
(RMSE/median/max; no alignment by default because the gauge is shared, SE(3)
or Sim(3) Umeyama on request), RPE at 1 m and at 10 frames, per-segment ATE,
and the consistency metrics of the fused cloud: pass-to-pass residual,
through-wall crossings, plan support. Never a start↔end gap or a planarity
number alone.

**Calibration before trust.** A loop-made reference is only as good as the
visual check. On ARKitScenes `perturb-poses` variants, where the undrifted seed
is the truth, run the loop (an agent without GT access, and once the
maintainer) and measure ATE(loop) against ATE(drifted) and ATE(`rux
optimize`), and whether the GT-free metrics rank the cycles like the true ATE
(Spearman). Caveat from `reusex-xka`: that drift is tilt-dominated inside
single rooms, unlike the houses' yaw drift across unrevisited rooms, so a
yaw-drift mode or a multi-room capture is needed for the result to transfer.

**Circularity with the floor plans.** The house plans follow the drifted scan,
so a corrected house2 needs its plan re-fitted in the corrected frame; until
then the plan cannot score poses there. NewOffice's plan is consistent with
its *initial* poses (§1).

## 5. Whacky ideas

Ranked by measured promise and by how directly they reuse what exists.

1. **Wall doubling as an explicit factor.** The symptom people see is the
   signal: a same-facing double layer 5-30 cm apart whose two layers come from
   different passes *is* a relative measurement between those passes, along
   the wall normal. Auto-propose `same_surface` constraints per inconsistent
   segment pair (§3.1) where the passes form separate layers, screen them with
   PCM like loop edges, add them as rank-deficient factors. It needs no image
   matching across lighting, devices or viewpoints: the failure that sank the
   ORB panorama edges (PCM rejected 186 of 189) and MapAnything (145 of 148). Bead
   `reusex-rcv.1.11`.
2. **The floor plan as the map (plan-constrained registration).** A
   drift-free plan (NewOffice's walls are within 2° of the grid) is a set of
   *known* plane landmarks: associate per-frame plane detections to plan faces
   and add `OrientedPlane3Factor`s against fixed landmarks under GNC, which
   also fixes the gauge globally. §3.6 says how: many weak associated factors,
   never per-frame fits. Further out: solve plan and poses jointly, with the
   YAML walls as priors on wall variables. Bead `reusex-rcv.1.12`.
3. **Rigid sub-maps glued by overlap and the plan.** Solve segment (or
   room-visit) poses first as rigid bodies, then the frames: −30 % cross-segment
   residual with plain ICP (§3.7). Emitting the result as `move` edits makes it
   inspectable and overridable in the same loop. Bead `reusex-rcv.1.13`.
4. **The camera cannot pass through walls.** Turn §3.5 into a factor: a
   trajectory step that crosses a plan wall away from its openings gets a
   hinge penalty pushing it to the nearest opening. Cheap and very
   architectural; plan errors must be fixed first (the NewOffice crossings are
   a door error).
5. **Gravity.** Tilt drift is real (house2 storeys ~1° apart, `reusex-32o`).
   If the captures carry gravity (RTABMap links, ARKit/StrayScanner IMU,
   Insta360 gyro via `reusex-bx2.2`), a `Pose3AttitudeFactor` per frame
   removes roll/pitch drift for free. First find out which datasets have it
   (`reusex-rcv.1.14`).
6. **Targeted learned matching.** Instead of matching all candidate pairs
   (XFeat on the office endcaps: 163 edges before PCM, 3 after), run the matcher only across the
   segment pairs the detector flags, at the frames that see the doubled
   wall. Fewer, better-placed candidates, so a smaller false-positive load.
7. **Panoramas as global anchors.** One aligned 360° image sees a whole room
   from one pose and is shared by every pass through that room. Blocked on
   panorama alignment itself (`reusex-bx2`: 6 of 18 aligned).
8. **Render-and-compare.** The Gaussian-splat trainer (`reusex_gsplat`) is
   differentiable with respect to camera poses: photometric pose refinement
   against a splat of the corrected reference, as a polishing stage. Costly;
   only after the geometric loop works.
9. **A VLM as the drift spotter.** The agent of §4.6 already looks at renders;
   a passes-coloured plan cut is exactly the kind of image where "this wall is
   drawn twice in two colours" is easy to see and hard to compute.

## 6. Risks and open questions

- **Metric gaming.** The loop optimises the same consistency metrics that later
  score the pipeline. The held-out segment pairs (§4.6), the image check
  per edit and the synthetic calibration (§4.7) are the guards.
- **Non-rigid segments.** Some house2 segments do not improve with a rigid move
  (§3.3); the loop needs splitting (shorter ranges) and the solver must spread
  a block edit over odometry rather than tear the block off.
- **Cycle time on NewOffice.** 3876 frames with plane landmarks may take
  minutes per solve; `--fast` and an in-memory preview cloud are the
  mitigation, to be measured in `reusex-rcv.1.5`.
- **Where edits live.** Version 1 keeps them in a file passed to `optimize`
  (no schema bump), like `plan.yaml` before `rux edit plan --import`.
  Persisting anchors in the project is a later decision.

## 7. The plan (beads)

Epic `reusex-rcv.1`; order: `rcv.1.1` measure ∥ `rcv.1.2` format →
`rcv.1.3` lowering → `rcv.1.4` layers → `rcv.1.5` watch; `rcv.1.6` scoring
(independent) → `rcv.1.7` calibration → `rcv.1.8` GT authoring → `rcv.1.10`
bench; then `rcv.1.9` agent protocol, `rcv.1.16` Rediger node edits, the factor
ideas `rcv.1.11`-`rcv.1.14`, and `rcv.1.15` (stale-cloud warning).
`bd show reusex-rcv.1` holds the design and acceptance of each.

## 8. Reproducing

The prototypes were throwaway Python (numpy/scipy) reading `.rux` copies
directly with sqlite3: cloud/normals blobs, `sensor_frames` (row-major double
`transform` / `initial_transform`, PNG depth in mm, `camera_model` JSON with
intrinsics and `local_transform`). Back-projection: optical point
`((u-cx) z/fx, (v-cy) z/fy, z)`, world = `transform · local_transform · p`.
Parameters are given in each subsection; the measure bead `reusex-rcv.1.1` ports
them to C++ with acceptance tests pinned to the numbers above.
