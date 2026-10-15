<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Solid-model ideas lab: what actually moves the floor plan

**Status:** research write-up, epic `reusex-rqn`, 2026-10-16. Quick measured
experiments against the reference plans in `tests/fixtures/solid_model/`
(NewOffice, house1, house2; all three are DRAFT references, decision D5).
The prototypes were throwaway Python over read-only copies of the scans;
only the mesh-support metric (§3) landed in the code. Every number below is
from `rux analyze solid --model <detected.json>` against the reference
(room IoU, room count, wall recall/precision) or from the per-space and mesh
measures defined in §1, and every result was also checked on plan
renders of the cloud with the reference overlaid.

**Question (maintainer, 2026-10-16):** use the labels to filter out the
furniture, keep points with the relevant labels or planes of particular
sizes and orientations, run SAM with more classes or better prompts, and
try unusual ideas. What do we have, and what gains the most?

---

## 0. TL;DR

| Rank | Idea | Measured gain | Cost | Verdict |
|---|---|---|---|---|
| 1 | **(e) Rooms from a clutter-free height band**: walls = vertical points at 1.2-2.3 m, footprint = ceiling ∪ floor, rooms = watershed of the free-space distance transform, cut at necks narrower than 2 × 0.5 m | NewOffice room IoU 0.579 → **0.663**, room count error +22 → **+1**; house1 0.602 → 0.632; house2 0.469 → 0.523. Used as the MIP's room labels, the NewOffice mesh has **74 % less fake interior wall** (238 → 63 m²), unsupported mesh 0.198 → 0.122, and the solve is 4.5× faster (235 → 52 s). Both glass-fronted meeting rooms come out as separate rooms | medium (this is W4.1, `reusex-z1v.6.1`) | **pursue**, child bead |
| 2 | **Mesh-support metric** (`mesh_unsupported_share`) | the point-to-mesh fit cannot see fake walls: on NewOffice it moved 0.279 → 0.265 m while fake walls dropped 74 %. The new metric moves 0.198 → 0.122 | cheap | **landed** on this branch |
| 3 | **(d) Capture trajectory**: wall crossings of the camera path are doors | house1: 16/16 crossings lie at a reference opening. NewOffice: the crossings that miss one cluster at 3 places where the reference door is misplaced (camera frames show the door) | cheap | **pursue** (door evidence + reference QA), child bead |
| 4 | **(b) Richer SAM prompts** (16 classes incl. glass partition, desk, monitor, sofa) | labelled share 58 → 85 %, furniture share 7 → 14 %, ceiling recovered. `glass partition` lands on exactly the 3 glass walls (14-25 % of their points vs ≤ 3 % on every other wall). **No gain for rooms by itself** | 26 min GPU on NewOffice | **pursue for openings/glass**, child bead |
| 5 | (f) Manhattan prior | NewOffice: all 26 reference walls within 2° of the dominant axes (median 0.12°). Houses: drift, only 50-79 % within 2°; snapping halves the position accuracy on house2 | cheap | only together with drift correction: evidence added to `reusex-rqn.1` |
| 6 | (h) Auto-tune against the references | train (NewOffice + house1) mean room IoU 0.585 → 0.647; holdout house2 0.523 → 0.526; the train optimum overfits (holdout 0.468) | cheap | use the bench to pick robust defaults; no separate bead |
| 7 | (a) Structure-only cloud (drop furniture labels / furniture-height planes) before planes/rooms/mesh | rooms ±0.01-0.03 IoU (Leiden and rooms-5l7 alike); mesh mixed (NewOffice fake walls 238 → 165-169 m², but house1 worse and the semantic cut breaks the NewOffice mesh: fit 0.28 → 0.58 m) | cheap | **not worth it as a cloud filter**; the height band (1) does the same job better |
| 8 | (c) Free-space carving from depth rays | footprint IoU vs reference spaces: same as ceiling ∪ floor (0.877 vs 0.878 NewOffice, 0.925 vs 0.940 house1, 0.723 vs 0.682 house2) | cheap | no gain as footprint; not pursued |
| 9 | (g) VLM room type per frame (gemma4:26b, 2.3 s/frame) | furnished NewOffice: majority vote right for 6/8 spaces (per frame ~50 %); empty house1: mostly "other" | 2-3 s/frame | naming only, covered by `reusex-z1v.10.3` |

The single biggest lever is **where the walls are looked for**: in a 1.2-2.3 m
band, above the furniture and below the ceiling clutter, with rooms found in
2D free space. Semantic furniture removal is beaten by that height band and
adds nothing on top of it. Labels earn their keep elsewhere: glass and doors.

---

Follow-up beads: `reusex-rqn.2` (band rooms), `reusex-rqn.3` (trajectory
doors), `reusex-rqn.4` (SAM prompt defaults), `reusex-a51` (NewOffice
reference doors), `reusex-8r0` (`--solver auto` fallback); evidence added to
`reusex-rqn.1`, `reusex-k89` and `reusex-5l7`.

## 1. Set-up and measures

- Data: copies of NewOffice (`~/repos/NewOffice/project.rux`, 1.21 M points,
  3876 frames, no storeys table, reference level frame = z + 1.27), house1
  (238 k points, 238 frames, one storey with a split level, empty house) and
  house2 (466 k points, 384 frames, two storeys in their own drifted frames,
  no semantic labels at all). Planes, rooms and mesh were rerun with the
  current `main` defaults; `create mesh` needed `--max-cells 20000` on
  NewOffice (7343 cells) and house2 (10717 cells), the default 5000 fails
  (tracked: `reusex-sba`).
- Detected models were written in the solid-model JSON format and scored
  with `rux analyze solid --model` (room IoU and count, wall recall,
  precision and RMS).
- Per-space measures (own): rasterise reference spaces and detected labels
  at 5 cm; per reference space, the share covered by the dominant detected
  label (`dom_cov`), the number of detected labels covering > 10 % of it
  (`frags`) and how much of that dominant label lies in the space
  (`dom_prec`, low = merged with a neighbour).
- Mesh measures: `unsupported share` = mesh area with no scan point within
  0.25 m (landed, §3); `fake interior wall` = vertical mesh area at
  0.3 m to ceiling - 0.3 m that lies inside reference spaces and more than 0.4 m
  from every reference wall axis.

## 2. Inventory: what is at our disposal

| | NewOffice | house1 | house2 |
|---|---|---|---|
| Semantic labels | 13-class map (ONNX defaults + plant/lamp); stored pre-v29, so after migration API class 0 ("ceiling") is lost and 505 k points carry a wrapped -1 | 18-class TensorRT default map; **unreliable**: 49 % of "ceiling" points are vertical, "wall" covers 6.7 % of the cloud | none (no `segmentation_images`) |
| Furniture classes in the map | table, chair, shelf, bench, plant (7.3 % of points) | cabinet, shelf only (house is empty) | n/a |
| Camera trajectory | 3876 poses, every reference space holds ≥ 122 of them | 238, every space ≥ 4 | 384, every space ≥ 7, drifted |
| Depth + confidence | yes (PNG mm + conf) | yes | yes |
| Panoramas | 18 | 0 | 0 |
| Storeys / plane_classes | none / none | 1 storey / none | 2 storeys / none |

SAM 3.1 TensorRT engines are in `~/.cache/reusex/models/sam3.1/engines/<gpu>`;
the RTX 6000 Ada was free. **Gap found:** the TensorRT default prompt list
(`vision/tensor_rt/Data.hpp`) deliberately has no movable furniture
(no desk, table, chair, sofa, monitor), while the ONNX default list
(`vision/onnx/Sam3Data.hpp`) is a different 13-class set with table and chair.
A project annotated with the TensorRT defaults cannot filter furniture at all.

## 3. Landed: solid-to-scan support metric

`fit_report` measured only scan → mesh. A fake wall between two
over-split rooms sits next to desks and floor points, so the scan → mesh
distance hardly changes when it disappears. `geometry::mesh_support()` samples
the mesh (about one sample per 5 cm × 5 cm, R2 low-discrepancy pattern,
deterministic) and reports the area share with no scan point within
`support_dist_m`; `rux analyze solid` prints and emits it as
`mesh_unsupported_share` (+ `mesh_area_m2`, `counts.support_samples`),
`--support-dist` binds the option, `bench-solid-compare.py` gates it
(lower is better).

Default 0.25 m chosen on the NewOffice pair below (base = the Leiden rooms
from `main`, 57 rooms; 2D = rooms from §4.1, 9 rooms):

| support distance | 0.05 | 0.10 | 0.15 | **0.25** | 0.50 |
|---|---|---|---|---|---|
| base | 0.538 | 0.411 | 0.320 | 0.198 | 0.082 |
| 2D rooms | 0.473 | 0.340 | 0.244 | 0.122 | 0.030 |

Below 0.15 m the share is dominated by few-cm plane offsets of correct
walls (house1/2: 0.66-0.74 at 0.05 m for both variants); at 0.25 m it
separates the two NewOffice meshes by 38 % relative and leaves the houses,
whose rooms did not change, equal (0.277 vs 0.272, 0.282 vs 0.273). The C++
result matches the prototype to three decimals.

## 4. Experiments

### 4.1 (e) Rooms from a clutter-free band: the main result

Prototype: per storey in the level frame, 5 cm grid. Wall cells = ≥ 3
points with |n_z| < 0.3 between floor + 1.2 m and min(floor + 2.3 m,
ceiling - 0.15 m). Footprint = cells with up-facing points below 0.9 m
(floor, both split levels) or down-facing points above 1.9 m
(ceiling), closed by 0.4 m, holes filled. Free = footprint minus walls
(dilated 5 cm). Rooms = cores where the free-space distance > 0.5 m
(a door is < 1 m wide), grown back by a watershed of -distance, regions
< 1 m² dropped.

| Rooms | NewOffice IoU / count err | house1 | house2 |
|---|---|---|---|
| `main` Leiden (`create rooms`) | 0.579 / **+22** | 0.602 / -1 | 0.469 / +3 |
| rooms-5l7 visibility merge (PR #42) | 0.601 / +16 | 0.602 / -1 | 0.469 / +3 |
| 2D band, first try (band 1.6-2.3) | 0.613 / +1 | 0.555 / -3 | 0.524 / -4 |
| **2D band, tuned (1.2-2.3, r 0.5)** | **0.663 / +1** | **0.632 / -2** | **0.523 / -4** |
| 2D, band 0.3-2.3 (furniture in) | 0.322 / +5 | 0.599 / -2 | 0.485 / -5 |

Renders: NewOffice, the open office is one region, meeting rooms 1 and 2
are separate and cut along their glass partitions, closed office, kitchen and
meeting/storage room each one region; the WC stalls (< 1 m wide) and
part of the printer room are lost to the 0.5 m core radius, and the vestibule
outside the entrance shows up as an extra room (the reference leaves it
out). house1: twelve rooms reduced to ten; kitchen and hall are one region
(the reference's boundary between them has no wall), and the entrance hall
merges with WC 2 through the door in their thin shared wall. With the
first-try band (1.6 m) the living room also joined the hall over the
parapet; the 1.2 m band separates it. house2 storey 0: living room N, kitchen and
stair hall merge (band walls sparse where the scan is drifted), and the
footprint leaks out through windows and the conservatory.

Fed to the MIP as the `rooms` cloud (each point takes its 2D cell's room;
`create mesh --max-cells 20000`, everything else default):

| Mesh | NewOffice base → 2D | house1 | house2 |
|---|---|---|---|
| fake interior wall [m²] | 238 → **63** | 42 → 42 | 80 → 84 |
| unsupported share (0.25 m) | 0.198 → **0.122** | 0.277 → 0.272 | 0.282 → 0.273 |
| fit mean [m] | 0.279 → 0.265 | 0.214 → 0.208 | 0.315 → 0.288 |
| solve time | 235 s → 52 s | 10 → 11 s | 132 → 93 s |

QA plan cuts: base NewOffice has full-height section lines across the open
office at the desk-screen rows and along both corridor edges (the
`reusex-5l7` symptom); with the 2D rooms only one spurious box around a
desk-screen cluster on the east side remains.

### 4.2 (a) Structure-only cloud

- 2D band at 0.3-2.3 m, NewOffice: furniture in 0.322; old furniture labels
  removed 0.480; richer furniture labels (§4.3) removed 0.474; band 1.6-2.3
  instead 0.613, and removing furniture there changes nothing (0.613). The
  height band beats the labels and the two do not add up.
- C++ rooms on filtered clouds: Leiden 0.579 → 0.584 (semantic) / 0.614
  (geometric: planes lying entirely 0.1-1.9 m above the floor dropped);
  rooms-5l7 0.601 → 0.609 (semantic, via `--filter`) / 0.608 (geometric);
  houses ±0.003 / -0.044 (house2). Structure-only occluders change the
  visibility merge by less than one room.
- Mesh: geometric filter NewOffice fake walls 238 → 169 m², unsupported
  0.198 → 0.177, but house1 unsupported 0.277 → 0.368. Semantic filter on
  NewOffice: fake walls 165 m² but the mesh loses surfaces (scan → mesh fit
  0.28 → 0.58 m).

### 4.3 (b) Richer SAM prompts

`rux create annotate --prompts-file` with wall, floor, ceiling, door, window,
glass partition, desk, table, chair, monitor, cabinet, shelf, plant, sofa,
radiator, ceiling lamp on a NewOffice copy: 953 s for 3876 frames, then
`create project` 620 s. Labelled share 58 → 85 %; furniture classes 7.3 →
14.1 %; ceiling 16.6 % (lost in the stored labels, §2).

`glass partition` (0.8 % of points) is the useful new class: within 0.25 m
of `p-mr1-glass` it is 25 % of the points, `p-storage-glass` 18 %,
`p-mr2-lobby` 14 %, and ≤ 3 % on each of the other 23 walls. `door` labels
mark the north end of `p-mr1-glass`, where §4.4 places the meeting-room
door. Adding glass-labelled cells as walls in §4.1 lowers the NewOffice IoU
(0.663 → 0.604): the labels also bleed onto windows and furniture. They
are evidence for openings and glass walls, not for room cuts.

### 4.4 (d) Trajectory

- Every reference space in all three scans contains camera positions
  (NewOffice ≥ 122, house1 ≥ 4, house2 ≥ 7).
- Crossings of the camera path with reference walls: house1 16/16 at a
  reference opening (10 of the 16 doors/openings crossed). NewOffice 24/42
  at an opening; the other 18 cluster at three places: `p-mr1-glass`
  s = 3.25-4.11 (6 crossings; the reference door is at s = 0.81-1.71;
  frames 170 and 290 show the operator walking through the open glass
  door), `p-mr2-lobby` s = 1.56-1.67 (reference door 0.33-1.23) and
  `p-wc-south` s ≈ 3.4-3.6 (stall doors the reference does not model).
  house2: 5/13 (drift).
- Rooms from walked space (trajectory dilated 0.6 m, necks cut) separate
  meeting room 1 along its glass wall but over-split rooms the operator
  walked loops in (NewOffice IoU 0.624, count +8).
- "Gate test" on adjacent candidate rooms (all path crossings of their
  shared boundary within one ≤ 1.3 m gate, boundary ≥ 2 m): house1 5/5
  correct; NewOffice 4 true and 5 false separations, every false one with
  exactly 2 crossings; requiring ≥ 3 crossings removes them and keeps
  meeting room 1 (6 crossings within 0.83 m).

### 4.5 (c) Free-space carving

Depth rays (every 8th pixel, confidence ≥ 2, ≤ 4 m) carved into 10 cm voxels
(30 s for 3876 frames). As a footprint it adds nothing over ceiling ∪
floor (table in §0); both leak where the sensor looked through windows, and
house2's reference does not tile the scanned area, which caps precision.

### 4.6 (f) Manhattan prior

Dominant direction from the band normals: NewOffice 50.75° (reference
50.86°); deviations of reference walls from the two axes: NewOffice median
0.12°, max 1.9°; house1 median 0.9°, max 4.5°; house2 median 2.1°, max 5.5°
(storey 0) and 9.9° (storey 1), the two storeys' axes 7° apart. With the
band wall extractor, snapping to the axes vs free angles: NewOffice recall
0.654 vs 0.615 (precision 0.152 vs 0.205); house2 recall 0.561 vs 0.439 but
RMS 0.085 vs 0.044 m. A global prior only helps once drift is corrected,
which is `reusex-rqn.1`.

Walls in general: no wall stage exists yet, so these are crude detectors
(band peaks, or one wall per vertical plane); recall 0.6-0.96 at precision
0.09-0.6, limited by fragmentation (one-to-one matching). Not ranked
further: W3 (`reusex-z1v.5.*`) owns walls.

### 4.7 (g) VLM room type, (h) tuning

- gemma4:26b through Ollama (qwen2.5vl not installed), 12 room types,
  ~2.3 s per frame. NewOffice (5 frames per reference space): open office,
  both meeting rooms, closed office, bathroom and printer/server room by
  majority, kitchen and meeting/storage room not. house1 (empty, 4 per
  space): mostly "other"; 8/48 frames right.
- 72 parameter combinations of §4.1 scored on NewOffice + house1, house2
  held out: table in §0. The defaults in §4.1 are the robust choice (r 0.5
  instead of the train optimum 0.45).

## 5. Glass-fronted rooms (`reusex-k89`)

Visibility cannot separate meeting rooms 1 and 2 from the open office
because sight lines pass through the glass. Measured evidence that can:

1. **2D free-space necks** (§4.1, tuned): both rooms come out separate
   (meeting room 1: 89 % of it in one region that is 89 % inside it;
   meeting room 2: 83 % / 94 %), cut along the glass. With a band starting
   at 1.6 m instead of 1.2 m, meeting room 1 merges, so the low part of the
   band carries the glass frame and mullion points.
2. **SAM `glass partition`**: 14-25 % of the points on the three glass walls,
   ≤ 3 % elsewhere.
3. **Trajectory gate**: 6 crossings of meeting room 1's boundary within one
   0.83 m gate.
4. Not useful: full-height columns along the glass walls (mullions) are
   too thin for the 5 cm cloud: 5 % of `p-mr1-glass` vs 40-100 % on solid walls.

## 6. Reproduction

Prototype scripts (not committed) lived in the session scratch directory:
`lab.py` (loaders, model JSON, scoring), `rooms2d.py`, `walls2d.py`,
`carve.py`, `traj_rooms.py`, `gate_merge.py`, `mesh_metrics.py`,
`exp_*.py`, `tune.py`. Every detected model was scored with

```bash
rux analyze solid --reference tests/fixtures/solid_model/<ds>.plan.yaml \
    --model detected.json --json
rux -p copy.rux analyze solid --reference ... --model detected.json \
    --cloud cloud --mesh mesh          # + fit and mesh support
```
