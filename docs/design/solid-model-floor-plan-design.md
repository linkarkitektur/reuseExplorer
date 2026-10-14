<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Solid model v2: floor-plan-first reconstruction, typed building elements, IFC

**Status:** design, 2026-10-13; decisions D1–D8 closed 2026-10-13 (§8). Reviewed and extended from
[`docs/research/solid-model-scan-to-bim.md`](../research/solid-model-scan-to-bim.md)
(the research). This document is the **architecture of record** for the
workstream; the **plan of record is the beads graph** (`bd show reusex-z1v`, the
workstream epic, or `bd list -l workstream:solid-model`). Executor
agents read this file first, then their bead, then the research doc for the
algorithm details it cites.

The research is accurate where it reads code — every "verified defect" in its
§3 was re-checked on 2026-10-13 against this tree and holds — and its central
thesis is right: our MIP is not the problem, the **evidence we feed it** is.
This spec keeps the research's phases, changes four of its recommendations
(§2.2), and widens the scope to what the research could not see: how the
result threads through `ProjectDB`, the stage contract, the Qt and web front
ends, the survey/material-passport workflow and the exports.

---

## 1. Goal and success criteria

**Goal.** `rux` turns a scan into a *typed* building model — storeys, walls
with measured thickness, spaces with polygons, openings with host walls — that
carries evidence, survives re-runs and manual edits, and exports as IFC4, DXF,
Rhino and Speckle. The current one-envelope-mesh output is replaced as the
primary product; the MIP stays as an optional global-consistency refinement.

**Success is a number** (STANDARDS §8, DIRECTION "measurable before
improvable"). On the solid-model benchmark (§7) a run must report:

| Metric | What it measures |
|---|---|
| room count error, room IoU (2D) | spaces vs reference polygons |
| wall position RMS, wall thickness error (mean, P95) | walls vs reference axes |
| opening recall / precision (door, window) | openings vs reference |
| point-to-solid fit: mean, share within 2 cm | solids vs the scan itself (GT-free) |
| seen share per wall, share of walls `assumed` | evidence coverage |

No stage in this workstream is "done" without its line in that report.

---

## 2. Review of the research

### 2.1 Confirmed (re-read on 2026-10-13)

- **Label-0 collision** — `vision/tensor_rt/Sam3.cpp:188` numbers prompt
  classes from 0; `core::api_to_point_label(0)` returns 0 = `kUnlabeled`;
  `tests/unit/core/test_label_semantics.cpp:47` pins that. STANDARDS §3.1 says
  points are `1..N`, so the code violates the contract and the first prompt
  class ("wall" on TensorRT) is unlabeled in every `labels` cloud. Confirmed
  as a bug blocking everything semantic.
- **Dead CLI flags** `create mesh -a/-l` (`apps/rux/include/create/mesh.hpp`
  says so in its own comment); `docs/guides/Process walk through.md:325`
  still recommends them.
- **Docs overstate** — CONTRACTS.md `mesh` row: "room-partitioned mesh";
  `mesh.cpp:201` discards `wall_labels` and emits one envelope.
- **Infinite wall lines** — `CellComplexConstructor.cpp:143` builds a
  `Line_2` per vertical plane; every horizontal plane becomes a section.
- The `merge_planes` suspicion (two wall faces merged before pairing) is
  untested and stays a WP0 test task.

### 2.2 Where this spec differs from the research

1. **IFC writer: native first, IfcOpenShell only in tests.** The research
   recommends an IfcOpenShell `IfcParse` nix overlay. For ~20 entity types
   with extruded-area solids, a hand-written Part-21 writer in `io/ifc.cpp`
   is one to two weeks and zero closure growth; IfcOpenShell C++ is a new
   heavyweight dependency (STANDARDS §1 requires an issue), a nix overlay to
   maintain, and weight on the 5 GB cachix budget. Validation runs through
   `python3Packages.ifcopenshell` (already in nixpkgs) as a **test-only**
   dependency. Upgrade path: swap the writer for `IfcParse` if the entity set
   grows past what a flat writer can keep correct. Decision D3.
2. **Entity tables, not a `plane_classes` cloud.** Everything the research
   proposes storing "as a table/cloud" becomes a relational table with a GUID
   per row (§3.2). Label clouds stay the per-point layer; tables are the
   entity layer. This also answers #138's open question "where do room GUIDs
   live": in the `spaces` table.
3. **Keep the planes.** The research's wall extractor works on raw wall-class
   points (offset-profile peaks). Our region-growing planes already exist and
   carry inliers; their normals are the better source of axis families
   (non-Manhattan support for free). The extractor therefore takes **both**:
   plane normals seed the axis families, points supply the profile. One
   `WallFace` struct, two evidence sources, no plane is thrown away.
4. **Ground truth: there is none yet, so WP0 builds the tool to author it.**
   Every reference we have (NewOffice's `project.rux` components, `export.3dm`)
   was generated by this library and cannot be trusted as truth (D5). The
   benchmark therefore ships with a **reference-authoring loop** (W0.6): a
   reference plan in the W0.5 JSON format, authored either by hand in a 3D
   tool through the Blender MCP path (#267) or in our own library with
   `rux render` as the visual feedback, and checked visually against the cut
   cloud. S3DIS is used **locally only** for benchmarking and is never
   committed (W0.7 loader; data path outside the repo). MuSHRoom and
   ARKitScenes are used where their licences allow.

### 2.3 Added scope (what the research did not cover)

The research stops at "typed outputs + IFC". The project needs, in addition:

- a **levelled frame** stored, not applied (§3.3), so raw data is untouched;
- **deterministic GUIDs** so a re-run and an IFC re-import update instead of
  duplicate (§3.2);
- **manual edits that survive re-runs** (`source`, `locked`), same pattern
  `sync_survey` already uses for survey parts (§3.2);
- **quantities into the survey**: walls, slabs and spaces become survey parts
  with measured m², m³ and thickness — the material-passport angle that is the
  project's actual mission (§5.4);
- **front ends**: new `visualize::Layer`s so `rux render`, the Qt 3D workspace
  and the web Kortlægning "Plan"/"Rum" evidence tabs draw the typed model
  without new code paths; a `GET …/model/*` JSON contract for ruxd (§5);
- **DXF/SVG floor plans** and JSON for the agent-modelling workstream (§6);
- the **scene graph (#138)** folds in: its `scene_relations` table is the
  relation store, its node types are the tables below (§3.2, D4).

---

## 3. Target architecture

### 3.1 Principle

```
   points + labels + planes            (what we have: per-point evidence)
            │
            ▼  level, storeys, per-plane class       ── WP1, WP2
   typed evidence in a levelled frame
            │
            ▼  walls (faces→pairs→thickness→graph)  ── WP3
            ▼  spaces (occupancy → polygons)        ── WP4
            ▼  openings (face rasters + instances)  ── WP5
   relational building model (tables with GUIDs, evidence, source/locked)
            │
            ├─▶ solids (extrusions, CSG openings)   ── WP6   ─▶ mesh per element
            ├─▶ exports: IFC, DXF, Rhino, Speckle, JSON       ── WP6, WP7
            ├─▶ survey parts with quantities        ── WP8
            └─▶ (optional) MIP over the bounded arrangement  ── WP9
```

Every arrow is a pipeline stage that reads and writes only `ProjectDB`
(DIRECTION: "every stage independently re-runnable"). Every entity row
carries evidence columns (STANDARDS §5: no silent degeneracy) and a `source`.

### 3.2 Data model (ProjectDB, schema v27+)

One schema bump per work package, additive only. All tables below are
scoped to one `.rux`; multi-scan assembly is out of scope (see §6).

**Conventions shared by every new table**

| Column | Meaning |
|---|---|
| `guid TEXT UNIQUE` | deterministic uuid5 (§3.2 GUIDs) |
| `source TEXT` | `detected` \| `manual` \| `imported` |
| `locked INTEGER` | 1 = a re-run must not overwrite this row (manual edit) |
| `confidence REAL` | 0..1, -1 unknown |
| `metadata TEXT` | JSON for evidence that has no column yet |
| `created_at`, `run_id` | the `pipeline_log` row that wrote it |

Coordinates in `walls`, `spaces`, `openings` are in the **storey's level
frame** (§3.3), metres, z up, floor at z = 0.

```sql
-- WP1
CREATE TABLE storeys (
  id INTEGER PRIMARY KEY, guid TEXT NOT NULL UNIQUE,
  idx INTEGER NOT NULL,            -- 0 = lowest
  name TEXT,
  z_floor REAL NOT NULL, z_ceiling REAL NOT NULL,   -- in the level frame of storey 0
  floor_plane BLOB, ceiling_plane BLOB,             -- 4 doubles each, world frame
  level_transform BLOB NOT NULL,   -- 16 doubles row-major, world -> level frame
  source TEXT, locked INTEGER DEFAULT 0, confidence REAL DEFAULT -1,
  metadata TEXT, created_at TEXT, run_id INTEGER);

-- WP2: one row per label of the "planes" cloud
CREATE TABLE plane_classes (
  cloud_id INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
  plane_label INTEGER NOT NULL,
  orientation TEXT NOT NULL,       -- horizontal | vertical | slanted
  semantic_class INTEGER,          -- majority SAM3 class over inliers, NULL if none
  class_confidence REAL,           -- vote share
  role TEXT,                       -- wall | floor | ceiling | slab | furniture | clutter | unknown
  storey_id INTEGER REFERENCES storeys(id),
  point_count INTEGER, area REAL, z_min REAL, z_max REAL,
  PRIMARY KEY (cloud_id, plane_label));

-- WP3
CREATE TABLE walls (
  id INTEGER PRIMARY KEY, guid TEXT NOT NULL UNIQUE,
  storey_id INTEGER NOT NULL REFERENCES storeys(id),
  x0 REAL, y0 REAL, x1 REAL, y1 REAL,   -- wall axis (centre line), level frame
  z_base REAL, height REAL,
  thickness REAL, thickness_ci_lo REAL, thickness_ci_hi REAL,
  lean_deg REAL,
  is_external INTEGER,                   -- room-on-both-sides test
  assumed INTEGER DEFAULT 0,             -- one-sided: thickness defaulted
  seen_share REAL,                       -- fraction of the face area with evidence
  face_a_points INTEGER, face_b_points INTEGER, co_visibility REAL,
  axis_family INTEGER,                   -- index into the storey's axis families
  source TEXT, locked INTEGER DEFAULT 0, confidence REAL DEFAULT -1,
  metadata TEXT, created_at TEXT, run_id INTEGER);

CREATE TABLE wall_joins (                -- IfcRelConnectsPathElements
  wall_a INTEGER NOT NULL REFERENCES walls(id) ON DELETE CASCADE,
  wall_b INTEGER NOT NULL REFERENCES walls(id) ON DELETE CASCADE,
  kind TEXT NOT NULL,                    -- corner | tee | collinear
  at_a TEXT NOT NULL, at_b TEXT NOT NULL, -- start | end | path
  PRIMARY KEY (wall_a, wall_b));

-- WP4
CREATE TABLE spaces (
  id INTEGER PRIMARY KEY, guid TEXT NOT NULL UNIQUE,
  storey_id INTEGER NOT NULL REFERENCES storeys(id),
  name TEXT, kind TEXT,                  -- room | corridor | open_plan | unknown
  polygon BLOB NOT NULL,                 -- outer ring + holes, see geometry/space_polygon.hpp
  z_floor REAL, z_ceiling REAL,
  area REAL, perimeter REAL, volume REAL, compactness REAL,
  room_label INTEGER,                    -- link to the "rooms" label cloud, NULL if none
  source TEXT, locked INTEGER DEFAULT 0, confidence REAL DEFAULT -1,
  metadata TEXT, created_at TEXT, run_id INTEGER);

CREATE TABLE space_boundaries (          -- IfcRelSpaceBoundary
  space_id INTEGER NOT NULL REFERENCES spaces(id) ON DELETE CASCADE,
  wall_id INTEGER NOT NULL REFERENCES walls(id) ON DELETE CASCADE,
  side TEXT NOT NULL,                    -- a | b
  s_start REAL, s_end REAL,              -- along the wall axis
  PRIMARY KEY (space_id, wall_id, side));

-- WP5
CREATE TABLE openings (
  id INTEGER PRIMARY KEY, guid TEXT NOT NULL UNIQUE,
  wall_id INTEGER NOT NULL REFERENCES walls(id) ON DELETE CASCADE,
  kind TEXT NOT NULL,                    -- door | window | opening
  s_start REAL, s_end REAL, z_sill REAL, z_head REAL,
  state TEXT,                            -- open | closed | unknown  (doors)
  is_external INTEGER,
  source_instance_guid TEXT,             -- SAM3 instance that supported it
  source TEXT, locked INTEGER DEFAULT 0, confidence REAL DEFAULT -1,
  metadata TEXT, created_at TEXT, run_id INTEGER);

-- WP6 (from #138, unchanged shape)
CREATE TABLE scene_relations (
  id INTEGER PRIMARY KEY,
  from_guid TEXT NOT NULL, to_guid TEXT NOT NULL,
  relation_type TEXT NOT NULL,           -- contains | connects_to | supports | adjacent_to | paired_with | group_member
  weight REAL, metadata TEXT);
```

**Records (core PODs).** Following the #227 pattern, `core/solid_model.hpp`
declares plain structs mirroring these rows (`StoreyRecord`, `WallRecord`,
`WallJoinRecord`, `SpaceRecord`, `SpaceBoundaryRecord`, `OpeningRecord`,
`PlaneClassRecord`, `SceneRelationRecord`) with no Eigen/CGAL/PCL types
(polygon = `std::vector<std::array<double,2>>` rings). `ProjectDB` grows
`save_*`/`load_*`/`list_*`/`delete_*` per table plus `entity_of_guid()`.
Geometry-typed views (`Eigen::Vector2d` axis, `CoplanarPolygon` faces) live in
header-only adapters under `geometry/` (`geometry/solid_model_adapters.hpp`),
compiled into their consumers, never into `reusex_geometry_common`.

**GUIDs.** `core/guid.hpp` gains `deterministic_guid(ns, name)` (uuid5,
RFC 4122 §4.3, SHA-1). Namespace = the project's `projects.guid`; name =
`"<table>/<storey idx>/<stable key>"` where the stable key is a quantised
geometric signature (wall: axis endpoints rounded to 5 cm; space: polygon
centroid rounded to 5 cm + storey; opening: host wall guid + s_start rounded
to 5 cm). A re-run that finds the same wall gets the same GUID, so IFC
re-import updates, survey parts keep their link, and `locked` rows match.

**`building_components` bridge.** The legacy table stays and is *written*
by WP6 (one row per wall face polygon and per opening polygon, `WallData`
finally populated with `{thickness, height, guid}`) so `Layer::components`,
`ComponentsPane`, `export rhino/speckle` keep working untouched. It is a
derived artefact; the tables above are canonical. `rux create windows` is
deprecated at WP5 and removed when WP6 lands.

### 3.3 Frames: level and storeys (WP1)

The scan is never rewritten. WP1 computes `level_transform` (world → levelled,
floor of storey 0 at z = 0, true vertical = normalised bisector of the floor
and ceiling normals as in the research §1.3 b) and stores it on each storey
row. Every later stage reads points through it (`geometry/level_frame.hpp`:
`to_level(p)`, `to_world(p)`), every export applies the inverse. Multi-storey:
Z-histogram peaks over `role = floor|ceiling` planes; the `idx` order is
by `z_floor`. A single-storey scan is one row; the stage never produces zero
rows silently (STANDARDS §5: it says why).

### 3.4 Module and function map

| Module (layer) | New header | Entry point | Notes |
|---|---|---|---|
| `core` (2) | `solid_model.hpp` | records | PODs only; ProjectDB persistence |
| `core` (2) | `guid.hpp` | `deterministic_guid()` | uuid5 |
| `core` (2) | `stage_contract.hpp/.cpp` | rows for new stages | the one source of truth |
| `geometry_common` (1½) | `level_frame.hpp`, `space_polygon.hpp`, `solid_model_adapters.hpp` | transform, polygon ring blob codec, Eigen views | header-only adapters follow #227 |
| `segmentation` (3) | `level.hpp` | `detect_storeys(cloud, planes, plane_classes…) -> StoreysResult` | WP1 |
| `segmentation` (3) | `classify_planes.hpp` | `classify_planes(cloud, planes, labels, …) -> vector<PlaneClassRecord>` | WP2; rules fallback when no labels |
| `reconstruction` (3) | `walls.hpp` | `extract_walls(…) -> WallsResult{walls, joins, faces}` | WP3; sub-steps are free functions, each unit-testable: `axis_families`, `wall_faces`, `pair_faces`, `measure_thickness`, `join_walls` |
| `reconstruction` (3) | `spaces.hpp` | `extract_spaces(…) -> SpacesResult{spaces, boundaries}` | WP4; `occupancy_grid`, `flood_rooms`, `polygonize_walls` |
| `reconstruction` (3) | `openings.hpp` | `extract_openings(…) -> OpeningsResult` | WP5; `face_raster`, `classify_opening` |
| `reconstruction` (3) | `solids.hpp` | `build_solids(…) -> vector<ElementSolid>` | WP6; per-element `pcl::PolygonMesh` + IFC profile |
| `reconstruction` (3) | `solid_metrics.hpp` | `fit_report(…)`, `compare_to_reference(…)` | WP0 benchmark |
| `reconstruction` (3) | `CellComplex*.cpp` | takes `walls` + `storeys` instead of planes | WP9 |
| `io` (3) | `ifc.hpp`, `dxf.hpp`, `solid_model_json.hpp` | writers | WP6/WP7 |
| `io` (3) | `export_scene.hpp` | `ExportScene` grows `storeys/walls/spaces/openings` | WP6 |
| `visualize` (4) | `scene.hpp` | `Layer::walls/spaces/openings/storeys` | WP4/5; plan view uses them |
| `pipeline` (4) | `stages.hpp`, `job_types.hpp` | `JobStage::storeys/classify/walls/spaces/openings/solids` | each WP |
| `apps/rux` | `create/{storeys,classify,walls,spaces,openings,solids}.cpp`, `export/{ifc,dxf}.cpp`, `analyze/solid.cpp`, `get` routes | thin wrappers | each WP |
| `bindings/python` | read-only accessors for the tables | | WP7 |

Peer rule (STANDARDS §1): `reconstruction` must not link `segmentation`. WP3
needs the storeys and plane classes → it reads the **records** from
`ProjectDB` (core), never `segmentation` headers. `pipeline` is where a stage
composes peers.

### 3.5 Stage DAG (the contract table)

New `core::PipelineStage` values, in order, inserted after `instances` and
before `mesh`, in this order (`openings` precedes `spaces` because the
contract's strictly-earlier rule applies to optional inputs too):

| Stage | Consumes | Produces | Replaces |
|---|---|---|---|
| `storeys` | `cloud`, `planes`, `plane_centroids`, `plane_normals` (+ `labels` optional) | `storeys` | — |
| `classify` | `cloud`, `planes`, `storeys` (+ `labels` optional) | `plane_classes` | — |
| `walls` | `cloud`, `planes`, `plane_classes`, `storeys` | `walls`, `wall_joins` | — |
| `openings` | `cloud`, `walls`, `storeys` (+ `instances`, `labels`, `glass_confidence_images` optional) | `openings` | `windows` (deprecated) |
| `spaces` | `cloud`, `walls`, `storeys`, `plane_classes` (+ `openings` for doorway cuts, `rooms` optional) | `spaces`, `space_boundaries` | the *use* of `rooms` downstream; `rooms` stage stays |
| `solids` | `storeys`, `walls`, `spaces`, `openings` | `solid_mesh` (meshes table, one named mesh per element kind), `building_components` (bridge) | the primary role of `mesh` |
| `mesh` (existing) | + `walls`, `storeys` when present (WP9) | `mesh` | — |

Each stage: a `SegmentXOptions`/`XOptions` struct in the library header is
the single parameter home (STANDARDS §4), mirrored by CLI flags and by
`pipeline::stage_parameters()` so the Qt Pipeline page and the GUI get the
form for free. Each stage logs its numbers (counts, rejected candidates and
why) at `info`, and `warn`s with the reason on empty output.

### 3.6 Data flow end to end

```
sensor_frames ─ clouds ─▶ cloud/normals ─ planes ─▶ planes, plane_centroids/normals
      │                                            │
      └─ annotate ─▶ segmentation_images ─ project ─▶ labels ─ instances ─▶ instances
                                                   │                          │
                           storeys ◀───────────────┤                          │
                              │                    │                          │
                           classify ◀──────────────┘  (labels optional)       │
                              │ plane_classes                                 │
                           walls ──▶ walls, wall_joins                        │
                              │                                               │
                           openings ◀─────────────────────────────────────────┘ (instances optional)
                              │ openings
                           spaces ──▶ spaces, space_boundaries   (openings cut doorways; rooms optional)
                              │
                           solids ──▶ solid_mesh*, building_components
                              │
          ┌───────────────────┼─────────────────────┬──────────────────┐
       export ifc        export dxf/rhino/speckle   survey (parts+qty)  analyze solid
```

### 3.7 Determinism and parameters

Everything is histogram and robust-statistics work: deterministic given a
fixed grid origin. Grid origins are anchored to the storey level frame's
origin (the floor-plane centroid), never to a bounding-box minimum that moves
with one outlier point. Bootstrap CIs use a seeded RNG (`seed` in the options
struct, default 0). No iteration is unbounded.

---

## 4. Work packages

Each WP is a beads epic; each bullet is a bead. Effort estimates are the
research's, adjusted. Model guidance per bead: **opus** for algorithm and
design beads, **sonnet** for migrations, CLI wrappers, docs and scripts.

### WP0 — Fix and measure (unblocks everything)

- **W0.1 Label-0 fix (opus).** `api_to_point_label` becomes `api + 1`,
  `point_label_to_api` the inverse; STANDARDS §3.1 wording confirmed as
  `points 1..N+1`. Schema v27 migration rewrites every stored `labels`-like
  cloud whose `parameters` JSON lacks `"label_encoding":"api_plus_one"` by
  +1 (chunked blobs, in a transaction) and rewrites its `label_definitions`
  keys; new writes carry the key. Audit every reader of semantic labels
  (`segment_instances`, `create_windows`, `sync_survey`, `reconstruct.cpp`
  vote, `project.cpp`, `mask_selection`, the renderer's legend, the Qt
  `Klasse %1` names, Python bindings) for inline `± 1`. Test pins the new
  rule; an integration test proves the first prompt class becomes an
  instance. Note for the agent: `instances` and `planes`/`rooms` clouds are
  **not** semantic and are not shifted.
- **W0.2 `create project` fusion (opus).** Multi-view majority vote,
  background never overwrites a class, depth cutoff and tolerance as
  options, deterministic tie-break (lowest class id), unlabeled votes
  ignored in `reconstruct.cpp`'s voxel vote.
- **W0.3 Dead flags and docs (sonnet).** Remove `-a/-l` from `create mesh`
  or wire them; correct CONTRACTS `mesh` row and the walk-through.
- **W0.4 `merge_planes` face-merge test (opus).** Synthetic two-face wall
  (0.15 m apart, opposite normals); assert `merge_planes` keeps both. Fix if
  it fails (the fix is likely "do not treat ±n as equal for walls").
- **W0.5 Benchmark harness (opus for metrics, sonnet for scripts).**
  Note (D5): nothing we have today is ground truth; W0.5 builds the metrics
  and the reference *format*, W0.6 the way to author references, W0.7 the
  S3DIS local loader.
  `reconstruction/solid_metrics.hpp`; reference format = a small JSON floor
  plan (`docs/design/solid-model-reference.schema.json`: storeys, walls as
  axis+thickness, spaces as polygons, openings); `rux analyze solid
  --reference plan.json [--json]`; `scripts/bench-solid.sh` runs the DAG on
  the fixture set and `bench-compare.py` diffs. Fixtures: NewOffice with a
  hand-modelled reference (from `export.3dm`), one MuSHRoom room, one
  ARKitScenes scan. The colleague's script output is converted once into
  the reference format as the baseline row.
- **W0.6 Reference-plan authoring loop (opus).** `rux edit plan`: round-trip
  the W0.5 JSON reference through the `storeys/walls/spaces/openings`
  tables with `source = imported, locked = 1`, so the same tables, layers and
  exports serve authored references and detected models alike; `rux render
  --layers cloud,walls,spaces --view plan` is the visual feedback; a
  Blender MCP path (#267) imports the plan, lets a person or agent correct
  it against the cloud, and exports the JSON back. Authoring NewOffice's
  reference this way is the acceptance test. This is also the seed of the
  human-in-the-loop plan editor (§5, §6 item 2).
- **W0.7 S3DIS local benchmark loader (sonnet).** `rux import s3dis <area>`
  reads the per-room `.txt` clouds and per-point annotations into a `.rux`
  (room membership → reference spaces, wall points → reference wall faces),
  from a path outside the repo (`$REUSEX_S3DIS_DIR`); `bench-solid.sh`
  skips it when unset. Nothing from S3DIS is ever committed.

### WP1 — Level and storeys

- **W1.1 `segmentation/level.hpp` (opus):** floor/ceiling fit (iterated
  LSQ, 2 cm inliers, over horizontal planes), bisector vertical, transform;
  Z-histogram storeys (`find_peaks`-equivalent: prominence + min distance
  in the options struct). Unit tests on synthetic two-storey clouds, tilted
  by 1°.
- **W1.2 `storeys` table + records + ProjectDB API + v28 migration (sonnet).**
- **W1.3 Stage, CLI `rux create storeys`, JobStage, contract row, CONTRACTS
  entry, `rux get storeys` (sonnet).**
- **W1.4 Quick win for the MIP (opus):** `CellComplexConstructor` slices only
  at storey floor/ceiling planes when `storeys` exists. Benchmark before/after
  (STANDARDS §8.1) on NewOffice: cell count, solve time, fit.

### WP2 — Plane classification (semantics × geometry)

- **W2.1 `classify_planes` (opus):** orientation from the normal; majority
  SAM3 class over inliers with vote share; rule fallback (Poux 2019/2020:
  floor = lowest large horizontal, ceiling = highest, wall = vertical,
  touching floor/ceiling band, height > 2 m; table = horizontal 0.6–1.2 m);
  `role` from a class → role map that lives next to `label_semantics` and is
  user-editable (`rux create classify --role-map file.json`).
- **W2.2 Table, records, stage, CLI, contract (sonnet).**
- **W2.3 Feed the MIP (opus):** `mesh` drops `role in (furniture, clutter)`
  planes when `plane_classes` exists. Benchmark.

### WP3 — Walls with measured thickness

- **W3.1 Axis families (opus):** 4θ circular mean for the dominant pair
  (the building is never grid-aligned, D6) + clustering of vertical plane
  normals for extra families only when their inlier mass clears
  `min_family_share`; options: `max_families` (default 2),
  `family_angle_deg`, `min_family_share`.
- **W3.2 Faces and runs (opus):** per family, offset profile of wall-role
  points (1 cm bins, [1,2,3,2,1] smoothing, peak prominence/height/separation
  as options); split into runs along s at gaps > 0.6 m; drop < 0.5 m.
- **W3.3 Pairing and thickness (opus):** four tests (gap 4–45 cm, overlap
  ≥ 50 cm, empty valley ≤ 15 %, co-visibility ≥ 30 % on 25 cm cells);
  Theil–Sen per face over cell medians, OLS, thickness at mid-height, lean,
  bootstrap CI over cells (seeded); one-sided → `assumed` with default
  thickness; room-on-both-sides → `is_external`.
- **W3.4 Wall graph (opus):** squaring (≤ 8°), snapping to point medians
  (10 cm then 4 cm), corner/tee joins (0.9 m / 2.2 m), collinear bridging,
  seen share per wall. Output `walls` + `wall_joins`.
- **W3.5 Table, records, stage `walls`, CLI, contract, `rux get walls --json`
  (sonnet).**
- **W3.6 `Layer::walls` in `visualize::populate_scene` (sonnet):** boxes in
  the level frame, `assumed` walls hatched/dimmer; `--view plan` draws them
  over the cut cloud. Qt 3D workspace layer panel entry.
- **W3.7 Bench line** (sonnet): wall position / thickness metrics live.

### WP4 — Spaces from floor occupancy

- **W4.1 Occupancy grid + flood (opus):** per storey, 10 cm cells, level
  frame; blocked by rasterised wall graph and by the layer-fill wall test
  (≥ 50 % of 10 cm layers between door height and ceiling margin occupied);
  floor evidence = `role = floor` points, dilated 2 cells; exterior flood
  from the border; interior components; doorway cuts from `openings` when
  present, else narrow-constriction cut (configurable width).
- **W4.2 Polygonise (opus):** arrangement of the wall graph per storey
  (CGAL `Arrangement_2` is already a dependency), faces labelled by the
  occupancy component they contain; merge faces of one component; holes;
  metrics (area, perimeter, volume, compactness); `kind` heuristics.
  Link `room_label` to the Leiden `rooms` cloud by point majority when the
  cloud exists.
- **W4.3 Table, records, stage, CLI, contract, `Layer::spaces` (sonnet).**
- **W4.4 Compare to Leiden rooms on the bench (opus):** room IoU per
  fixture; decide (D2 input) whether `rooms` stays the room evidence for the
  MIP or `spaces` replaces it.

### WP5 — Openings

- **W5.1 Face raster with occluder check (opus):** per wall face (s, z) at
  10 cm; a cell is open when no wall points in the slab zone **and** no
  occluder within 1.2 m in front; morphological opening; components;
  rectangle fit; min 0.5 × 0.4 m, fill ≥ 0.55.
- **W5.2 Classification (opus):** door if sill < 0.3 m and 1.7–2.4 m tall;
  floor-to-ceiling = opening; else window; mullion merging; SAM3
  door/window instances and `glass_confidence_images` raise confidence and
  set `source_instance_guid`; internal/external from the room on the far
  side; door state from leaf points in the swept volume.
- **W5.3 Table, records, stage, CLI, contract, `Layer::openings`, deprecate
  `create windows` (sonnet).**
- **W5.4 Spaces split at doors (sonnet):** re-run W4.1 cut using `openings`.
- **W5.5 Bench line:** opening recall/precision.

### WP6 — Solids and exports

- **W6.1 `build_solids` (opus):** wall boxes in the wall frame (lean
  optional) with openings subtracted (CGAL Nef or `corefinement`), slabs
  (storey floor/ceiling polygon × thickness between storeys), spaces
  (polygon × height). Each solid = `pcl::PolygonMesh` + an IFC-ready
  profile (polyline + extrusion depth + placement). Written to `meshes` as
  `solid_walls`, `solid_slabs`, `solid_spaces` and to the
  `building_components` bridge.
- **W6.2 `ExportScene` grows typed elements and relations (sonnet);** Rhino
  gets layers `Walls/Slabs/Spaces/Openings` with user-text attributes
  (thickness, guid, storey); Speckle maps to `Objects.BuiltElements.Wall`
  / `Floor` / `Room` / `Opening` (see the `speckle` skill).
- **W6.3 `io/ifc.hpp` native IFC4 writer (opus):** Project/Site/Building/
  Storey/Space spatial structure; IfcWall + IfcWallType + material layer set
  (thickness written three times: body, layer set, Qto); IfcSlab; IfcDoor /
  IfcWindow in IfcOpeningElement via IfcRelVoidsElement/IfcRelFillsElement;
  IfcRelConnectsPathElements from `wall_joins`; IfcRelSpaceBoundary from
  `space_boundaries`; Pset_WallCommon.IsExternal; a `ReUseX_ScanEvidence`
  pset (thickness CI, seen share, co-visibility, points per face, assumed);
  material-passport KV as a pset per element; GlobalIds = our deterministic
  GUIDs (IFC's 22-char base64 encoding). SI metres explicit.
- **W6.4 IFC validation test (sonnet):** `python3Packages.ifcopenshell` as a
  test-only dependency in `shell.nix`/`default.nix` (`checkInputs`);
  `ifcopenshell.validate` + read-back: solid thickness == Qto, element count,
  storey containment. Also smoke-open in Bonsai/BlenderBIM if available
  (manual, documented).
- **W6.5 `rux export ifc` CLI (sonnet).**
- **W6.6 `scene_relations` (sonnet):** `contains` (space ⊃ instance by
  point majority — #138 Stage 1), `connects_to` (space–door–space), written
  by `solids`; `rux get relations`.

### WP7 — Floor plans, JSON, bindings (cheap, high leverage)

- **W7.1 `rux export dxf` (sonnet):** walls as closed polylines on
  `WALL`/`WALL_ASSUMED`, spaces on `SPACE` with area text, openings on
  `DOOR`/`WINDOW`; one layer set per storey. Hand-written DXF R12 ASCII
  (no dependency). SVG via the same geometry for the web.
- **W7.2 `io/solid_model_json.hpp` (sonnet):** one JSON document for the
  whole model (what the agent-modelling workstream #267 and ruxd both
  consume); `rux get model --json`.
- **W7.3 Python bindings (sonnet):** read-only `storeys()`, `walls()`,
  `spaces()`, `openings()`.
- **W7.4 ruxd + rux-frontend (cross-repo, filed here as a contract bead):**
  `GET /api/v1/cases/{cid}/model` (W7.2 JSON) and `…/model/plan.svg?storey=k`
  (W7.1); the Kortlægning "Plan" and "Rum" evidence tabs use the SVG;
  the Viewport draws walls/spaces as a mesh layer from `solid_*` meshes
  (its `meshLayer.ts` already streams meshes). Design in rux-frontend's
  design-studio, not here.

### WP8 — Survey integration (the mission)

- **W8.1 `sync_survey` extends to elements (opus):** one survey type per
  element kind (Vægge, Dæk, Rum, Døre, Vinduer) and one part per wall / slab
  / space / opening, with **measured quantities** (wall: net area, volume,
  thickness; space: floor area, volume; opening: w × h) in the part's
  quantity fields, linked by element GUID, respecting `survey_dismissed_*`
  and `locked`. A re-run updates quantities of unlocked parts only.
- **W8.2 Wall types (opus, "whacky but useful"):** cluster walls by
  (thickness ± CI, height, material KV from `attributes`) into wall types →
  `IfcWallType` per cluster, survey quantity per type ("142 m² of 115 mm
  plasterboard partition"). This is how architects count reuse.
- **W8.3 Space naming (sonnet):** `create attributes` style VLM call on each
  space's best frame → `spaces.name`/`kind` (office, corridor, WC); optional.

### WP9 — The MIP as an alternative path (experiment, per D2)

- **W9.1 CellComplex from the wall graph (opus):** arrangement from bounded
  wall segments + joins; sections only at storeys; room evidence from
  `spaces` occupancy instead of ray votes.
- **W9.2 Keep solver labels (sonnet):** `Solidifier_to_mesh` emits per-room
  meshes and wall labels; `mesh` becomes room-partitioned for real.
- **W9.3 Bench: MIP-refined per-room meshes vs W6.1 solids.** Report fit,
  watertightness and run time; the maintainer then decides where (if
  anywhere) the MIP sits in the default pipeline. Until then `rux create
  mesh` stays as the alternative path and is never removed by this
  workstream.

---

## 5. Front ends and the user's loop

- **`rux render` / Qt 3D workspace:** `Layer::walls|spaces|openings|storeys`
  in `visualize::populate_scene` (one code path for both, as the 2026-10-08
  entry established). `--view plan[:h]` draws the typed plan over the cut
  cloud; `assumed` walls and low-`seen_share` walls draw dimmer so the user
  sees where the scan did not look.
- **Qt Pipeline page:** the new stages appear automatically through
  `stage_parameters()` with their options forms and "Kopiér som rux-kommando".
- **Qt Q4 (future, not in this plan): a 2D "Plan" workspace** —
  QGraphicsView of the wall graph with drag-to-move endpoints, split/merge
  spaces, set opening kind; writes rows with `source = manual, locked = 1`.
  That is the human-in-the-loop BIM editor; the data model above is designed
  so it needs no schema change.
- **Web (rux-frontend / ruxd):** W7.4. Kortlægning's evidence tabs stop
  showing a render of point labels and show the real plan with the part's
  wall/space highlighted; the ConfidenceBar maps to `confidence`/`seen_share`.
- **Agent modelling (#267):** the agent gets `rux get model --json` instead of
  points. The Blender add-on can build the simplified model from it directly.

---

## 6. Opportunities this opens (ranked, not all planned)

1. **Reuse quantities by wall type** (W8.2) — the first output that answers
   the mission's question with numbers an architect can put in a tender.
2. **Human-in-the-loop plan editing** (Qt Q4 / web) — because `source`,
   `locked` and deterministic GUIDs are in the schema from day one, a
   manual correction survives every re-run. Nobody else in the open-source
   space has "scan → editable BIM → re-run keeps my edits".
3. **Fit check as a trust signal** — point-to-solid distance per element
   plus seen share gives a per-wall "how sure are we" the GUI can show, and a
   per-project QA report (`analyze solid`) for the PDF (`report_pdfs`).
4. **Slab thickness between storeys** — ceiling of storey n and floor of
   n+1 measured like a wall pair: the same code path, a free `IfcSlab`
   quantity.
5. **Room connectivity graph** (`connects_to` via doors) → circulation,
   escape-route lengths, "which rooms share a wall" for acoustic/material
   questions; cheap once `space_boundaries` exist.
6. **Columns and beams** — SAM3 prompts exist (`column`, `beam`); a vertical
   cylinder/box fit per instance → `IfcColumn`/`IfcBeam`. Add after WP6 as
   the same `solids` pattern.
7. **Furniture as parametric library instances** (research Phase 8) —
   ratio/symmetry matching against a small OFL-licensed library; output
   `IfcFurnishingElement` + Rhino blocks; ties to the passport inventory.
8. **Drift diagnostics from walls** — two faces of one wall leaning together
   is the signature of a wrong vertical or pose drift; `lean_deg` per wall
   becomes a scan-quality metric for the SLAM workstream (#221) for free.
9. **Multi-scan assembly by storey** — `rux assemble` could align scans on
   their wall graphs (2D line matching is far better posed than ICP on
   clouds). Out of scope here; the tables make it possible.
10. **Gaussian-splat-informed openings** — splat density on a wall face is a
    denser occupancy signal than the sparse cloud; a later evidence source for
    W5.1 with no API change.
11. **Panorama-anchored plans** — the aligned 360 panoramas already have
    poses; placing them on the DXF/SVG plan gives the on-site "where was this
    taken" view the GUI's 360° tab wants.

---

## 7. Benchmark (WP0.5) — what "better" means

Fixtures: NewOffice with a reference **authored through W0.6** (the
colleague's script output, converted, is the baseline row, not truth), one
MuSHRoom capture, one ARKitScenes scan where its licence allows, and S3DIS
**locally only** (W0.7; never committed). Authored references live under
`tests/fixtures/solid_model/` as JSON; S3DIS-derived ones do not.
Every WP adds its metric line; `scripts/bench-solid.sh` + `bench-compare.py`
gate regressions at `--threshold`. Before/after is mandatory for every change
to `mesh` (STANDARDS §8.1).

---

## 8. Decisions for the maintainer

Filed as `decision` beads under `reusex-z1v.1`, all closed on 2026-10-13;
the close reasons there are authoritative, this table summarises them.

| # | Question | Recommendation |
|---|---|---|
| D1 | DIRECTION: add workstream "Solid model / scan-to-BIM"? Priority? | **Decided: yes, priority 2** after SLAM quality; GUI, 3DGS, infra move down one place |
| D2 | Fate of the MIP | **Decided: keep the code as an alternative path**; its place (possibly a per-room refinement of the W6.1 solids) is undecided and needs measurement — WP9 is that experiment, not a commitment |
| D3 | IFC dependency | **Decided: as recommended.** Native Part-21 writer; `python3Packages.ifcopenshell` test-only; revisit IfcOpenShell C++ if the writer outgrows ~25 entity types |
| D4 | Scene graph (#138) | **Decided: merge.** `scene_relations` is the relation store, the new tables are its nodes; #138's Stages 1–2 become W6.6 |
| D5 | Ground truth | **Decided: no trusted GT exists.** Build the reference-authoring loop (W0.6) and author references on our own scans (NewOffice, then every new capture); S3DIS locally for benchmarking, never committed (W0.7); MuSHRoom/ARKitScenes where licences allow |
| D6 | Manhattan | **Decided:** buildings are mostly orthogonal but never grid-aligned, so the dominant-pair rotation (4θ mean) is mandatory and 2 families is the default; extra families only on strong evidence; tune on our own captures as they accumulate |
| D7 | Data contract | **Decided: as recommended.** New tables (§3.2) + `building_components` bridge until WP6 |
| D8 | Label-0 migration | **Decided: as recommended.** Rewrite stored clouds by +1 in a v27 migration (one-off O(points)), flagged by `label_encoding` in parameters |

---

## 9. Handoff protocol for executor agents

1. `bd prime`, `bd ready -l workstream:solid-model`, `bd show <id>`; claim
   with `bd update <id> --claim`.
2. Work in a worktree off `main` (`EnterWorktree`); one agent per bead; never
   the main `build/`.
3. Read, in this order: the bead; this spec's section it names; the research
   doc's algorithm section it cites; STANDARDS §1/§3/§4/§5/§6/§7;
   CONTRACTS.md for the stage row you add.
4. TDD (STANDARDS §7): synthetic fixtures under `tests/fixtures/solid_model/`
   (tiny clouds built in code, not binary blobs), `tests/support/temp_path.hpp`
   for temp files. Tests go in the **light** binary unless they need
   torch/TensorRT.
5. A schema bump: `LATEST_SCHEMA_VERSION` in `ProjectDB.cpp`, a migration
   with `CREATE TABLE IF NOT EXISTS`, a test that opens a pre-bump fixture.
6. A new stage: contract row in `stage_contract.cpp`, CONTRACTS.md prose,
   `JobStage`, `stage_parameters()`, CLI, `rux validate --stage`, the
   `test_stage_contract` DAG assertion, and the Qt CLI round-trip test.
7. Before "done": `scripts/check.sh`, the bench line for your WP, then
   `bd close <id>` with `--reason` naming the metric before/after. Report in
   the bead's notes (`bd update <id> --notes`), not only in the transcript.
8. Do not edit DIRECTION's Mission or Priorities. Append to its changelog when
   a WP lands.
