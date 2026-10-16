<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX Pipeline Stage Contracts

This document describes what each pipeline stage **consumes** and **produces**
in a ReUseX project database (`*.rux`). It complements the module-boundary rules
in [`STANDARDS.md` §1](STANDARDS.md#1-module-boundaries): STANDARDS §1 governs
*code* dependencies between library modules; this document governs the *data*
dependencies between pipeline stages.

> **This file is a prose mirror, not the source of truth.** The machine-readable
> contract lives in
> [`libs/reusex/include/core/stage_contract.hpp`](../libs/reusex/include/core/stage_contract.hpp)
> (table in `src/core/stage_contract.cpp`), and everything that enforces the
> contract reads *that*: `core::check_stage_inputs()`, every `rux create`
> subcommand, and the refusal path of `pipeline::run_stage()`. The **Consumes**
> and **Produces** rows below are asserted against the table by
> `tests/unit/core/test_stage_contract.cpp`, so this page cannot silently drift
> — but when the two disagree, the table wins and this page is the bug (#246).

Each stage is a `rux` subcommand that reads named point clouds / tables from the
project, does its work, and writes named point clouds / tables back. The names
are stable strings (`"cloud"`, `"planes"`, …) used as keys in `ProjectDB`.

`rux validate --stage <name>` asserts that a stage's **inputs** are present and
mutually consistent before it runs. That is the *same call* every `rux create`
subcommand makes, so a stage that validates cannot then be refused when run.

## Conventions

- **Named clouds** are `ProjectDB` point clouds addressed by name. The canonical
  per-scan clouds — `cloud`, `normals`, `planes`, `rooms`, `instances`,
  `labels` — are **index-aligned**: point *i* is the same physical sample in all
  of them (STANDARDS §3.2). Any stage that filters or reorders one MUST apply
  the same operation to its siblings.
- **Per-plane clouds** `plane_centroids` (`PointXYZ`) and `plane_normals`
  (`Normal`) are indexed by plane label, not by point, so they are shorter than
  the per-point clouds and aligned only with each other.
- Alignment is recorded in the table as a property of the **artifact**
  (`Alignment::per_point` / `per_plane` / `none`), not of the stage, so no stage
  can declare a per-plane cloud index-aligned with a per-point one.
- **Tables** are relational rows keyed by cloud name and id — `sensor_frames`,
  `segmentation_images`, `instances`, `instance_materials`,
  `building_components`, `meshes` — or, for the solid-model tables
  (`storeys`, `plane_classes`, ...), by a stable GUID with `source` / `locked` columns: a
  re-run replaces the unlocked rows and keeps the locked ones, and a
  re-detected element keeps the GUID of the stored element it matches. New
  GUIDs are deterministic (`core::deterministic_guid` over the project GUID,
  `ProjectDB::project_guid()`). A file copy of a `.rux` is the **same**
  project: same project GUID, same row GUIDs.
- **Options struct** is the single library-side parameter object for the stage;
  CLI flags mirror its defaults (STANDARDS §4).
- Several commands can be pointed at a differently-named cloud (`rux create
  instances --semantic-cloud foo`). Those flags are passed to the checker as
  *overrides* of the contract's declared name; the contract itself does not
  change.
- **External artifacts** are files a stage writes outside the project, at a
  path the user chose. They are recorded in the table's `external_outputs`
  field — prose, not a name — and are *not* pipeline artifacts: nothing in the
  project can depend on a file on disk, so they are invisible to
  `producing_stage()`, to the DAG assertion and to `check_stage_inputs()`. A
  stage may legitimately have an empty **Produces** row as long as it has one
  of these. **No stage does today**: `gsplat` was the last one, and #322 moved
  its trained model into the project. The field stays because the reason for it
  has not — a future stage whose product cannot be a `ProjectDB` row still
  needs somewhere to say so.

## Stages

### `import` (`rux import rtabmap`)

| | |
|---|---|
| Consumes | An external RTABMap scan file (not project data) |
| Produces | `sensor_frames`, `pose_graph_edges` (RTABMap links; `rux import rtabmap` only), one row in `scans` |
| Options  | import-specific CLI options (depth filters) |
| Checks   | nothing in-project (reads an external scan) |

**Multi-session identity.** Each invocation of `rux import rtabmap` creates
one row in the `scans` table (`id`, `source_path`, `imported_at`, `id_offset`,
`provenance`).  Every imported `sensor_frame` carries a `scan_id` foreign key
back to that row.

**Node-ID remapping.** RTABMap sessions restart their node IDs at 1.  To avoid
collisions across sessions, the importer computes `id_offset = MAX(node_id)`
from the existing `sensor_frames` at call time (0 when the project is empty)
and stores each frame as `node_id = rtabmap_id + id_offset`.  The offset is
recorded in `scans.id_offset` so downstream tools can recover the within-session
ID from any stored frame.

**FK safety invariant.** Tables that reference `sensor_frame.node_id`
(`segmentation_images`, `panoramic_images`, `glass_confidence_images`) enforce
referential integrity via `ON DELETE CASCADE / SET NULL` and SQLite FK
enforcement (always on for write-mode opens).  They cannot hold references
above the offset watermark.  The one table without a FK constraint
(`pose_graph_edges`) only receives edges for the importing session's own
(offset) id range.

**Re-import guard.** Importing the same `source_path` a second time creates a
second scan by design (e.g. re-processing after editing the RTABMap DB).
A `WARN` is logged; the import is not blocked.

**Pose graph.** `rux import rtabmap` stores **every** link in the RTABMap
database's link table (not just its optimized graph's) into
`pose_graph_edges` (Neighbor/NeighborMerged → `odometry`, Global/LocalSpace/
LocalTime/User closures → `loop_closure`, source `rtabmap`, ids offset like the
frames) and each imported pose in `sensor_frames.initial_transform`. Gravity,
prior, landmark and virtual links are ignored and counted in the import log.
Links that are self-links, point to a node that was not imported, or carry an
invalid (null/non-rigid) transform are skipped with a warning.

### `graph_init` (`rux graph init`)

Snapshot each frame's current pose into `sensor_frames.initial_transform` and
store one odometry edge per consecutive pair of frames of each scan (source
`odometry_init`). `rux optimize` runs it automatically when the graph has no
odometry edge with a measurement. On a graph that already has odometry, a
plain run only fills gaps: frames without a snapshot and consecutive in-scan
pairs without odometry (e.g. a newly imported scan). `--force` re-snapshots
every frame and replaces the `auto` `odometry_init` edges; confirmed and
rejected edges are kept.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `pose_graph_edges` |
| Options  | `InitPoseGraphOptions` (`libs/reusex/include/slam/pose_graph.hpp`) |
| Checks   | ≥2 stored frames; without `--force`, refuses when odometry exists and there is nothing to add |

### `graph_discover` (`rux graph discover`)

Run the loop-edge front-ends (ORB frame pairs, 360 panoramas, an external
`reusex.loop_edges.v1` file) seeded from the initial poses, skip every pair the
stored graph already decides (including rejected tombstones), PCM over stored
active edges ∪ new candidates (confirmed edges are fixed inliers), and store the
survivors with status `auto`. Initialises odometry first when there is none.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `pose_graph_edges` |
| Options  | `GraphDiscoverOptions` (`libs/reusex/include/slam/graph_discover.hpp`) |
| Checks   | ≥2 stored frames; at least one source enabled |

### `optimize` / `register` (`rux optimize`, `rux register`)

Solve the stored pose graph: seeds from `sensor_frames.initial_transform`, one
factor per active `pose_graph_edges` row, writes poses to
`sensor_frames.transform` and residual/deviation/seed-disagreement back to
every edge. Repeatable; never adds, removes or re-statuses edges (`rux
register`: joint pairwise registration, writes poses only).

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `sensor_frames` (updated poses), `pose_graph_edges` (per-edge metrics; `optimize` only) |
| Options  | `PlaneGraphOptions` / `JprParams` (`libs/reusex/include/slam/`) |
| Checks   | ≥2 stored frames, ≥2 of them carrying depth (refinement is depth-driven) |

### `clouds` (`rux create clouds`)

Back-project depth frames into a fused point cloud.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `cloud` (`PointXYZRGB`), `normals` (`Normal`) |
| Options  | `ReconstructionParams` (`libs/reusex/include/segmentation/reconstruct.hpp`) |
| Checks   | ≥1 stored sensor frame |

Each `normals` entry is a unit normal oriented toward the sensor(s) that
observed its point: the sign agrees with the per-frame normals inside its
voxel, each of which faces its own camera's optical centre. The two faces of a
wall therefore carry opposite normals. `planes` inherits this orientation
for `plane_normals`, and `mesh`'s face pairing relies on it (reusex-0pi;
before that fix every stored normal faced the world origin).

The stage records this in the parameters of `cloud` and `normals` as
`"normal_orientation": "sensor"` (`core/normal_orientation.hpp`), and
`planes` copies the key onto `plane_normals` when `normals` carries it
(reusex-0se). A cloud without the key — made before that, or imported (E57) —
has normals whose sign means nothing a stage may rely on; `classify` then
ignores which way a plane faces.

### `annotate` (`rux create annotate`)

Run a semantic segmentation model over the stored frames.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `segmentation_images` |
| Options  | `rux create annotate` CLI options (`-n/--net`, batching, CUDA) |
| Checks   | ≥1 stored sensor frame |

### `project` (`rux create project`)

Project the 2D semantic labels onto the 3D cloud by multi-view majority vote:
each frame that sees a point votes for the class at its pixel, background
never votes, and an exact tie goes to the lowest class (`core::majority_label`).

| | |
|---|---|
| Consumes | `cloud`, `segmentation_images`, the latest `annotate_class_map` pipeline-log entry (class names) |
| Produces | `labels` (`Label`, stamped `label_encoding`, with `label_definitions` keyed by point label) |
| Options  | `ProjectLabelsOptions` (`libs/reusex/include/vision/project.hpp`): `--max-depth`, `--depth-tolerance` |
| Checks   | `cloud` present; at least one frame carries a segmentation image |

### `planes` (`rux create planes`)

Detect planar surfaces (noise-adaptive region growing).

| | |
|---|---|
| Consumes | `cloud`, `normals` |
| Produces | `planes` (`Label`), `plane_centroids` (`PointXYZ`), `plane_normals` (`Normal`) |
| Options  | `SegmentPlanesOptions` (`libs/reusex/include/segmentation/segment_planes.hpp`) |
| Checks   | `cloud`, `normals` present and index-aligned |

### `rooms` (`rux create rooms`)

Partition the scan into rooms. `method = band` (default since the maintainer
decision of 2026-10-16, reusex-icg) finds rooms per storey in 2D: wall cells
from vertical points in a clutter-free height band above the storey floor,
footprint from floor and ceiling points, exterior flooded from the grid
border, rooms = watershed of the free-space distance transform, cut where
the free space narrows below 2 x `band_core_radius`. Every point within
`assign_margin` of a storey's interior takes the room of the nearest room
cell; others stay 0. `method = leiden` (`--method leiden`) instead clusters
the plane visibility graph and does not need storeys.

The band method also reads `normals` and the `storeys` table (level frames,
floor and ceiling heights); they are not declared inputs because Leiden does
not need them, so the stage checks them itself (`storeys` precedes `rooms`
in the stage table all the same). When the method *resolves to band only
because it is the default* (no explicit `--method`) and the project has no
storeys rows, the stage logs a warning naming the reason and both fixes
(`rux create storeys`, or `--method leiden` to silence it) and falls back to
Leiden for that run, recording the fallback in the result message. An
*explicit* `--method band` without storeys still refuses as invalid input —
asking for band by name is an instruction, not a preference.

| | |
|---|---|
| Consumes | `cloud`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `rooms` (`Label`) |
| Options  | `SegmentRoomsOptions` (`libs/reusex/include/segmentation/segment_rooms.hpp`); `band`: `BandRoomsOptions` (`segmentation/band_rooms.hpp`), parameter keys `band_*` |
| Checks   | `cloud`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned; `band`: `storeys` has rows (refused if explicit, else falls back to leiden with a warning) and `normals` is aligned with `cloud` (checked by the stage) |

### `instances` (`rux create instances`)

Separate semantic labels into spatial instances (connected-component clustering).

| | |
|---|---|
| Consumes | `cloud`, and a semantic label cloud — `labels`, or `planes` as a fallback |
| Produces | `instances` (label cloud + `instances` table) |
| Options  | `SegmentInstancesRequest` (`libs/reusex/include/segmentation/segment_instances.hpp`) |
| Checks   | `cloud` present; a semantic label cloud present; the two index-aligned |

`--semantic-cloud <name>` overrides `labels`. An explicit override is an
instruction, not a preference: the fallback is dropped and the named cloud must
exist.

### `storeys` (`rux create storeys`)

Group the horizontal planes into surfaces by height, fit each storey's floor
and ceiling to its surface's own points (iterated least squares) and measure
the vertical as the bisector of their normals (`segmentation/level.hpp`,
`detect_storeys`). When `plane_normals` face their sensors (see `clouds`
above) an up-facing surface is a floor and a down-facing one a ceiling, so
the floor over a slab is the strongest up-facing surface above the ceiling
below it however small it is; a cloud whose normals all face the world origin
(before reusex-0pi) is read by height alone, and the success message says
which. The scan is never rewritten: each row stores the world → level
transform (storey-0 floor at z = 0) next to its floor and ceiling height in
that frame.

| | |
|---|---|
| Consumes | `cloud`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `storeys` (`storeys` table, schema v30: one row per storey, a stable GUID — see below) |
| Options  | `LevelOptions` (`libs/reusex/include/segmentation/level.hpp`), parameter keys are its field names; plus `guid_match_m` (`core::StoreyIdentityOptions`, `core/storey_identity.hpp`) |
| Checks   | `cloud`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned |

Never zero rows: a scan with one level, no ceiling or no horizontal plane is
one storey whose `metadata` records the reason (logged at `warn`).

Identity: `idx` is the rank from the bottom and moves when a re-run finds a
storey below the old lowest one; the GUID does not. A re-detected storey takes
the GUID of the stored storey whose floor lies within `guid_match_m` (default
0.5 m) of its own, compared in the new run's level frame; only an unmatched
storey gets a fresh GUID, `uuid5(project_guid, "storeys/z<floor cm>")`,
suffixed until unique.

Frames: the rows of one run share one `level_transform` unless pose drift
left a storey rotated against the others. With `per_storey_frame` (default
on) a storey whose own floor/ceiling bisector is more than
`storey_frame_tol_deg` (0.25°) off the shared vertical gets its own frame,
rotated to its own vertical and placed so that its floor centroid keeps its
shared-frame level coordinates. `z_floor` is that centroid's height above the
storey-0 floor centroid along the shared vertical, in either kind of frame;
`z_ceiling` is measured on the row's vertical through it. Level x/y agree with
the shared frame's at the floor centroid, and level `(0, 0, z_floor)` stays a
world point at the storey's floor height, which is what GUID matching and the
solid-model benchmark take to world through each row's own frame
(reusex-z1v.3.5). Each row's `metadata` records its `frame`
(`shared`/`storey`), its own `vertical` and `tilt_vs_shared_deg`.

Rows with `locked = 1` survive a re-run unchanged, frame included; a new
storey whose GUID or `idx` a locked row holds is not written. The stage
warns, with the angle and shift, when a kept locked row's frame differs from
that of the new row it stands for (same GUID, else nearest floor), and when
its height range overlaps a new row's (measured in that row's frame). `rux get storeys` prints the table; `rux del storeys` removes it, and
needs `--force` when any row is locked.

### `classify` (`rux create classify`)

Classify every plane of the plane segmentation (`segmentation/classify_planes.hpp`,
`classify_planes`): its orientation against the vertical of its storey's level
frame, the majority semantic class of its points with the vote share (when a
`labels` cloud exists), its role — `wall`, `floor`, `ceiling`, `slab`,
`furniture`, `clutter` or `unknown` — its storey, area and height range. The
role comes from the class through the class → role map
(`core/class_role_map.json`, or `--role-map file.json`) when the class is
trusted and the geometry does not contradict it, otherwise from geometric
rules.

| | |
|---|---|
| Consumes | `cloud`, `planes`, `plane_normals`, `storeys` (plus the semantic labels cloud when present, see below) |
| Produces | `plane_classes` (table, schema v31: one row per plane label, a stable GUID) |
| Options  | `ClassifyPlanesOptions` (`libs/reusex/include/segmentation/classify_planes.hpp`), parameter keys are its field names; plus `role_map` (path of a role-map JSON file, `""` = built in) |
| Checks   | `cloud`/`planes` present + aligned; `plane_normals` present; ≥1 storey |

`oriented_normals` is not a parameter: a floor is told from a ceiling by which
way its normal faces only when `plane_normals` (or `normals`) carries
`"normal_orientation": "sensor"` (see `clouds`). Without it the stage warns
and classifies without the sign; `metadata.oriented_normals` records which.
The semantic vote uses the `labels` cloud and its label definitions when
they exist (an optional enrichment, so not a declared input, as for
`survey`); a `labels` cloud whose size disagrees with `cloud` is ignored with
a warning.

Each row carries `role_source` — `semantic` (the class map decided),
`rules` (no usable class), `overruled` (the class's role was contradicted by
the geometry), `none` (a plane with no points) or `manual` — and
`storey` = the storey's `idx` (−1 for a plane outside every storey, e.g. a
roof above the top storey's ceiling). It is the idx, not a foreign key, so a
storeys re-run does not orphan or block the rows; re-run `classify` after
`storeys`.

Identity: the rows are keyed by the `planes` cloud row and the plane label,
and go with the cloud (`rux del clouds.planes`). The GUID is
`uuid5(project_guid, "plane_classes/planes/<label>")`. A planes re-run keeps
the cloud row, so the old rows stay until `classify` runs again; rows with
`locked = 1` survive that re-run unchanged (the stage warns when a locked
plane's point count no longer matches the new segmentation). `rux get
plane_classes` (or `rux get clouds.planes.classes`) prints the table; `rux del
plane_classes` removes it, and needs `--force` when any row is locked.

### `mesh` (`rux create mesh`)

Solidify the cell complex into **one watertight envelope mesh** covering every
room whose solver-selected cells have `room_label > 0` — `rooms` only weights
which cells are "inside" during the MIP solve; the output is not split back
into per-room meshes, and the solver's per-cell room/wall labels
(`Solidifier::solve()`'s `room_labels`/`wall_labels`) are discarded rather than
carried onto the mesh (`libs/reusex/src/reconstruction/mesh.cpp`). Keeping
those labels for a room-partitioned output is tracked under WP9
(`reusex-z1v.11.2`).

| | |
|---|---|
| Consumes | `cloud`, `normals`, `rooms`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `mesh` (`meshes` table) — one envelope mesh, not room-partitioned |
| Options  | `MeshOptions` → `mesh()` (`libs/reusex/include/reconstruction/mesh.hpp`); builds a `SolidifierOptions` subset for `Solidifier` internally |
| Checks   | `cloud`/`normals`/`rooms`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned |

### `texture` (`rux create texture`)

Texture-map the reconstructed mesh from the cloud's colors.

| | |
|---|---|
| Consumes | `mesh`, `cloud`, `sensor_frames` |
| Produces | `textured_mesh` |
| Options  | `TextureQualityParams` (`libs/reusex/include/reconstruction/texture_mesh.hpp`) |
| Checks   | named mesh + named cloud present; ≥1 stored sensor frame |

`--mesh-name` / `--cloud-name` override `mesh` / `cloud`.

### `windows` (`rux create windows`)

Derive window building components from the instances and the mesh.

| | |
|---|---|
| Consumes | `cloud`, `labels`, `instances`, `mesh` |
| Produces | `building_components` |
| Options  | `rux create windows` CLI options (`--mode`, `--offset`, `--alpha`) |
| Checks   | all four present; `cloud`/`labels`/`instances` index-aligned |

`--semantic`, `--instances` and `--mesh` override `labels`, `instances` and
`mesh` respectively.

### `materials` (`rux create materials`)

Create one material passport per instance and link it back to the instance.

| | |
|---|---|
| Consumes | `instances`, `survey_dismissed_instances` (skipped) |
| Produces | `instance_materials` (the `instance_materials` link table + `material_passports`) |
| Options  | `rux create materials` CLI options (`--instances`, `--clear`) |
| Checks   | `instances` present |

Generates a `material_passports` row per instance in the instance-label cloud
and records the instance→passport link in `instance_materials`. This is the
prerequisite that lets `attributes` iterate materials.

### `survey` (`rux create survey`)

Fill the Ressourcekortlægning (Kortlægning in the web GUI) from the instances
table: one survey type per semantic class, one bygningsdel (survey part) per
instance.

| | |
|---|---|
| Consumes | `instances`, `survey_dismissed_instances` (schema v26 tombstones), cloud positions and posed sensor frames (for views, optional) |
| Produces | `survey_types`, `survey_parts`, `survey_parts.view_*` (schema v27) |
| Options  | `rux create survey` CLI options (`--instances`, `--semantic`, `--rooms`) |
| Checks   | `instances` cloud present |

Processes instances in ascending `instance_id`. Each unmapped `semantic_class`
gets one new survey type (named from the semantic label cloud's definitions
when that optional cloud, named "labels" by default, is present —
`"Klasse <n>"` if unnamed, or `"Uklassificeret"` for class `-1`); each
instance without an existing part (joined on instance guid, so a `rux create
instances` rerun that keeps GUIDs does not duplicate) gets one new survey
part. When an optional rooms label cloud (named "rooms" by default) exists
and is aligned with `instances`, each part's room is the room most of its
points fall in (`majority_room`), named from that cloud's label definitions
or `"Rum <id>"`; a missing or size-mismatched rooms cloud is logged and parts
get no room. Idempotent — existing types/parts are never modified, so edits
made in the GUI survive a rerun.

Deleting a scan-backed part (or its whole type) in Kortlægning records the
instance guid in `survey_dismissed_instances`. `sync_survey` skips every
tombstoned instance, so a deleted part is not re-created, and reports how
many it skipped as `parts_dismissed`. `rux create materials` skips them too,
so a deleted part's instance does not get a new passport.

An instance cloud written before schema v10 can have labels but no
`instances` rows: the v10 migration backfilled rows only for clouds that had
material links. `sync_survey` then writes the rows first (one per distinct
label, with its point count, a fresh guid, and the class parsed from the
`"SM{class}-{id} (Np)"` definition, else `-1`), logs a warning with the count,
and reports it as `instances_backfilled`. `instances_seen` counts the rows
considered, so a caller can tell "no instances" (0) from "all already
surveyed".

A part's `instance_guid` can stop resolving — the linked instance was
deleted, or `rux create instances` ran again without carrying the guid over
(e.g. `--clear`). `sync_survey` does not delete or hide such a part: it keeps
its code, quantity and room, reads back with a null `cloud`/`instance_id`,
and still counts toward its type's `quantity` total. `sync_survey` reports
the count and codes (`SurveySyncReport::parts_orphaned` /
`orphaned_codes`) and logs a warning; each part is also flagged individually
(`SurveyPart.orphaned`) so the GUI can surface it. Deciding what to do with an
orphaned part — re-file it under a new instance, or drop it — is a Phase 3
product decision.

Then every instance-backed part without a view (new or pre-existing — the
backfill for old projects) gets the best-ranked frame (`instance_evidence`)
as its view, region = the instance's bbox in that frame padded 15 % per side.
A view is never overwritten. No sensor frames: skipped with one `info` line;
no positions cloud or misaligned clouds: `warn`, no views; parts no frame
sees keep none (`views_unavailable`).

### Editor write: segment resource (web GUI, POST /frames/{id}/segment/resource)

<!-- No backticks in this heading: test_stage_contract parses backticked
     `### ` headings as pipeline stages, and this is not one. -->

Turn one label of a frame's saved segmentation into a labelled selection, a new
instance and a survey part. An editor write, not a pipeline stage: it is logged
in `pipeline_log` as `segment_resource` but is not a `Stage` and has no
`rux validate --stage` entry.

| | |
|---|---|
| Consumes | `cloud`, the frame's `segmentation_images` row, its stored pose, intrinsics and depth image |
| Produces | `labels` and `instances` (created all-zero when absent, cloud-sized), one `instances` row, label definitions for both, one `survey_parts` row (and a `survey_types` row when needed), that part's view (`survey_parts.view_*`, schema v27: the mask's bbox in this frame, padded 15 %) |
| Options  | `MaskProjectionOptions`, `MaskSelectionOptions` (`libs/reusex/include/core/mask_selection.hpp`) |
| Checks   | frame posed with depth; `labels`/`instances`, when present, index-aligned with `cloud`; an explicit `type_id` is not a rejected survey type; an echoed `mask_revision` still matches the saved segmentation |

Existing `instances` rows are updated in place (point counts), never deleted
and re-inserted, so `instance_materials` links survive. A rewritten `labels`
or `instances` keeps its original `stage`/`parameters` (e.g. reconstruct's
`storage_order`); the edit is recorded in `pipeline_log`. A later
`rux create instances` re-clusters from `labels`; the hand-made instance keeps
its GUID (and its part's link) only when reconciliation matches it by overlap.
The part never lands in a rejected (Afvist) type: the automatic type choice
skips rejected types and creates a new one when only a rejected type matches.

### `attributes` (`rux create attributes`)

Describe each material by cropping its linked instance(s) out of their best
sensor view and asking a vision-language model.

| | |
|---|---|
| Consumes | `cloud`, `instances`, `instance_materials`, `sensor_frames` |
| Produces | `material_annotations` (table) |
| Options  | `DescribeConfig` (`libs/reusex/include/vision/describe.hpp`) |
| Checks   | `cloud` present; `instances` present; ≥1 material passport; ≥1 stored sensor frame |

For each material passport the stage resolves its linked instance(s) via
`instance_materials`, gathers their 3D points (from `cloud`, index-aligned with
the `instances` label cloud), pinhole-projects them into every posed sensor
frame, picks the frame with the most in-bounds points as the best view, and
crops the 2D bounding box (plus padding) from that frame's color image. The crop
is sent to an OpenAI-compatible chat/completions endpoint (default a local
Ollama at `http://localhost:11434/v1`, swappable to OpenAI or another provider)
and the answer — a free-text description plus arbitrary key/value attributes, as
driven by the prompt — is stored in the `material_annotations` table (with its
`material_annotation_kv` child rows) keyed on the material GUID. A material with
no linked instance, no usable view, or an unusable answer is logged and skipped
— descriptions/attributes are never fabricated (STANDARDS §5). `--instances` and
`--cloud` override the declared names.

### `gsplat` (`rux create gsplat`)

Train a 3D Gaussian splat from the fused cloud and the posed sensor frames.

| | |
|---|---|
| Consumes | `cloud`, `sensor_frames` |
| Produces | `splat` |
| Options  | `GsplatStageOptions` (`libs/reusex/include/gsplat/train.hpp`) |
| Checks   | seed cloud present; ≥2 stored sensor frames |

`splat` is a row in the `gaussian_splats` table (schema v12) holding the
INRIA-format `.ply` the trainer produced, alongside its Gaussian count and
spherical-harmonic degree. `--name <name>` overrides the declared name, the
same way `--seed-cloud` overrides `cloud`. An INRIA `.ply` trained elsewhere —
or by a run predating v12 — is brought in with `rux import gsplat`.

`-o/--out` additionally writes the identical bytes to a file, and
`--render-dir` writes checkpoint PNGs. Neither is a contract artifact: they are
an export and a diagnostic, and nothing in the project depends on them. See
`rux create gsplat --help`.

`--seed-cloud <name>` overrides `cloud`. Content-aligned 360 panoramas are an
**optional extra source of training views** (`--use-panoramas`), not a
prerequisite: a project with no panoramas trains from the sensor frames alone,
so they are deliberately absent from the contract's inputs.

The contract's ≥2-frame floor is structural — a radiance field fitted to one
view is that view. It is not a quality bar: 3DGS needs dense multi-view overlap
over a contiguous capture segment, which no row count can assert. See `rux
create gsplat --help`.

`rux validate --stage gsplat` works in a CPU-only build, where `rux create
gsplat` is not compiled in at all: the stage table is pure data, so the entry
exists everywhere even though the trainer needs `WITH_CUDA` and the vendored
rasterizer.

## Typical pipeline order

```
import → [graph_init] → [graph_discover] → (optimize|register) → clouds → planes → [storeys] → rooms → mesh → texture
                                                                                   ↘ storeys → classify
                                                    ↘ annotate → project → instances → windows
                                                    ↘ gsplat (leaf: produces `splat`)
```

`[storeys]` before `rooms` matters because `rooms` defaults to `method =
band`, which needs it; without storeys the default falls back to `leiden`
(with a warning) rather than failing, but `rooms --method band` explicitly
still refuses. The stage table lists `storeys` before `rooms` for that
reason.

Stage order in the table is load-bearing and asserted by a test: an input may
only be produced by a **strictly earlier** stage. That invariant is what lets a
missing prerequisite be turned into a "run these commands in order" hint
automatically, instead of the hand-written hint strings the CLI used to carry.
