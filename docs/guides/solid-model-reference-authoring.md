<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Authoring a solid-model reference plan

The solid-model benchmark (`rux analyze solid`, `scripts/bench-solid.sh`)
scores a detected building model against a **reference plan**: a solid-model
JSON document (format: [`docs/design/solid-model-json.md`](../design/solid-model-json.md))
that a person or an agent authored on one of our own scans. Nothing we have is
trusted ground truth, so a reference is only as good as the visual check it
went through. This guide is that check: author walls, spaces and openings
by measuring the scan, draw them over a plan cut of the cloud, and correct
until the drawing and the scan agree.

The result is a benchmark target, never ground truth. Mark a reference that
has not been reviewed by the maintainer as a draft (see
[Marking a draft](#marking-a-draft)).

## The loop

```
measure the cloud  ->  write / edit plan.json  ->  rux edit plan --dry-run
        ^                                                   |
        |                                                   v
        +--------  compare  <--  rux render --view plan --plan plan.json
```

1. **Work on a copy.** Opening a `.rux` can migrate it. Copy the project
   (and its `-wal` file) to a scratch directory and point every command at
   the copy.

   ```bash
   cp scan.rux scan.rux-wal /tmp/ref/   # -wal only if it exists
   ```

2. **Look at plan cuts.** A plan view clips everything above the cut
   height over the detected floor, leaving the walls in section:

   ```bash
   rux -p /tmp/ref/scan.rux render -o cut.png --view plan:1.2 --layers cloud
   rux -p /tmp/ref/scan.rux render -o cut-low.png --view plan:0.4 --layers cloud
   ```

   A cut at waist height (1.2 m) passes through windows and doors; a low cut
   (0.3 to 0.5 m) shows door openings as gaps and below-sill wall. Without
   `rux create planes` the cut is measured from the bounding box instead of
   the floor plane (the render logs which).

3. **Measure.** The render is for judging, not for reading coordinates to
   the centimetre. Read positions from the cloud itself: export it
   (`rux export ply`), or read the `cloud` / `normals` blobs directly, and
   fit what you need. What worked on NewOffice:

   - Rotate the plan into the building's two wall directions (the peak of a
     histogram of vertical-point normal angles modulo 90 degrees), so walls
     become `a = const` or `b = const` lines.
   - Plot a horizontal slice (e.g. 0.9 to 1.6 m above the floor) at a known
     scale with a 0.5 m grid, per region of the building; that is where you
     decide which point lines are walls and which are furniture.
   - Take each face position as the mode of a 1 cm histogram of points near
     the guessed line whose normal is roughly perpendicular to it. Fit the
     two ends separately when the line drifts over a long wall: a 0.1 m drift
     over 25 m is common in a SLAM scan, and one global position is then
     wrong at both ends.
   - A wall seen from both sides gives two faces: the axis is their mean, the
     thickness their distance. A wall seen from one side only (all external
     walls, when the exterior was not scanned) has `thickness: -1` and its
     axis on the scanned face; say so in its `metadata`.
   - Doors are gaps in the wall points **below head height** (slice 0.3 to
     1.9 m: a slice up to the ceiling picks up the lintel and closes the gap).
     An open door leaf shows as a short line at an angle to the wall.
   - Windows sit in niches behind the inner face. Plot an elevation of the
     points just behind a facade (colour by depth): frames, sills and heads
     are easy to read there, and the niche extents give `s_start` / `s_end`.

4. **Write the JSON.** Coordinates are in the storey's level frame:
   z up, the floor at z = 0. The simplest frame for a reference is the
   project's world x / y plus a `level_transform` that only shifts z (world
   floor at `z_f` -> `[1,0,0,0, 0,1,0,0, 0,0,1,-z_f, 0,0,0,1]`), so the plan
   does not depend on how any detector chooses its level origin. Give every
   element a readable `guid` (`x-north`, `p-wc-south`, `o-n-win-03`): the
   overlay labels walls by guid, and that is how you map a render back to the
   file. Put measurement notes in `metadata`.

5. **Validate.** The reader is strict; it names the offending element:

   ```bash
   rux edit plan --import plan.json --dry-run
   # and write it back in canonical form (source "imported", locked):
   rux edit plan --import plan.json --dry-run --export plan.canonical.json
   ```

   `--dry-run` never opens the project. Without it, `--import` stores the
   plan in the project's solid-model tables; until those tables exist
   (storeys, walls, spaces, openings), it stores nothing and fails.

6. **Overlay and compare.**

   ```bash
   rux -p /tmp/ref/scan.rux render -o check.png --view plan:1.2 \
       --layers cloud --plan plan.json --size 3200x2400
   ```

   In a top or plan view the plan is drawn flattened on top of the cut cloud
   and is not clipped by the cut:

   | Element | Drawn as | Colour (`--label-*` token) |
   |---|---|---|
   | wall | translucent footprint (axis +/- thickness / 2) with an outline, plus the axis; a wall of unknown thickness is its axis only | `--label-2` (green) |
   | space | translucent fill and outline, labelled with its name | `--label-0` (orange) |
   | door / window / other opening | a bar along the host axis with a tick across the wall at each end | `--label-5` / `--label-1` / `--label-3` |

   Wall guids are drawn at the wall midpoints; `--no-plan-labels` turns the
   labels off. Any other view (`orbit:N`, `front`, `frame:<id>`) draws the
   plan in 3D on its storey's floor.

   Look for: footprints that sit beside the cut wall points instead of on
   them, openings that do not line up with gaps, spaces that overlap a wall
   or leave a strip between their outline and the wall face. Zoom in by
   rendering larger (`--size`), or crop the measurement plots per region.

7. **Iterate** until the overlay matches everywhere you claim anything.
   Leave out what the scan cannot tell you rather than guessing; if a space
   is open-plan with no wall between it and the next, draw the boundary
   where a person would split it and say so in its `metadata`.

8. **Commit** the plan as `tests/fixtures/solid_model/<scan>.plan.json`
   (JSON only, never the cloud) and add its row to
   `scripts/bench-solid.manifest`. A row whose project is not in the
   repository uses a `$VARIABLE` path, so it is skipped where the variable is
   unset.

## Marking a draft

A reference no maintainer has reviewed says so in two places: its
`description` starts with "DRAFT - needs maintainer review", and its
storey's `metadata` carries `"status": "draft - needs maintainer review"`.
Keep the overlay renders the plan was checked with next to the review
request, so the reviewer sees what the author saw. The reviewer removes both
marks when the plan is accepted.

## Where the table-backed path plugs in

`rux edit plan --import` calls `io::store_solid_model()` and `--export`
calls `io::load_solid_model()` (`libs/reusex/include/io/solid_model_json.hpp`).
Both are where each solid-model table, as it lands, adds its rows, so that
`JSON -> tables -> JSON` stays lossless. Until then the authoring loop above
works on the JSON alone: validate with `--dry-run`, check with
`rux render --plan`.
