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
  `building_components`, `meshes`.
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

### `annotate` (`rux create annotate`)

Run a semantic segmentation model over the stored frames.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `segmentation_images` |
| Options  | `rux create annotate` CLI options (`-n/--net`, batching, CUDA) |
| Checks   | ≥1 stored sensor frame |

### `project` (`rux create project`)

Project the 2D semantic labels onto the 3D cloud.

| | |
|---|---|
| Consumes | `cloud`, `segmentation_images` |
| Produces | `labels` (`Label`) |
| Options  | `rux create project` CLI options |
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

Partition the plane graph into rooms (Leiden clustering).

| | |
|---|---|
| Consumes | `cloud`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `rooms` (`Label`) |
| Options  | `SegmentRoomsOptions` (`libs/reusex/include/segmentation/segment_rooms.hpp`) |
| Checks   | `cloud`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned |

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

### `mesh` (`rux create mesh`)

Solidify the cell complex into a room-partitioned mesh.

| | |
|---|---|
| Consumes | `cloud`, `normals`, `rooms`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `mesh` (`meshes` table) |
| Options  | `SolidifierOptions` → `Solidifier` / `mesh()` (`libs/reusex/include/reconstruction/`) |
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
| Consumes | `instances`, `survey_dismissed_instances` (schema v26 tombstones) |
| Produces | `survey_types`, `survey_parts` |
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
| Produces | `labels` and `instances` (created all-zero when absent, cloud-sized), one `instances` row, label definitions for both, one `survey_parts` row (and a `survey_types` row when needed) |
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
import → [graph_init] → [graph_discover] → (optimize|register) → clouds → planes → rooms → mesh → texture
                                                    ↘ annotate → project → instances → windows
                                                    ↘ gsplat (leaf: produces `splat`)
```

Stage order in the table is load-bearing and asserted by a test: an input may
only be produced by a **strictly earlier** stage. That invariant is what lets a
missing prerequisite be turned into a "run these commands in order" hint
automatically, instead of the hand-written hint strings the CLI used to carry.
