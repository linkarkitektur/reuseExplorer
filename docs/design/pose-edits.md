<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Pose edits (`reusex.pose_edits.v1`)

A pose-edits document is a list of corrections to a scan's sensor poses,
written by a person or an agent while looking at renders of the registered
cloud. `rux optimize --edits <file>` lowers each edit into extra factors of
the stored pose-graph solve and reports how well the solution honours it. It
is the pose analogue of the solid-model plan
([solid-model-json.md](solid-model-json.md)): YAML for hand editing, with
comments carrying provenance (which render showed what), and JSON for tools.
Background, measurements and the loop it serves:
[visual-drift-loop.md](../research/visual-drift-loop.md) (epic `reusex-rcv.1`).

- Schema: [pose-edits.schema.json](pose-edits.schema.json)
- Records: `libs/reusex/include/core/pose_edits.hpp` (`core::PoseEdits`)
- Reader / writer: `libs/reusex/include/io/pose_edits.hpp`
- Lowering into factors: `libs/reusex/include/slam/pose_edit_lowering.hpp`

## Document

```yaml
schema: reusex.pose_edits.v1   # required
project_guid: "5f1c..."         # optional: refused on any other project
frame: world                    # or storey:<guid>
edits: []                       # the edits, in any order
```

| key | meaning |
|---|---|
| `schema` | always `reusex.pose_edits.v1` |
| `project_guid` | `ProjectDB::project_guid()` of the project the edits were written for (the `project_guid` project setting, written once per project). `rux optimize --edits` refuses the file on another project, so edits written on a copy apply to every copy of the same project and to nothing else. Omit it to skip the check |
| `frame` | the frame of every pose, delta, pivot and region in the file: `world`, or `storey:<guid>`, the level frame of that storey (`rux create storeys`; z is the measured vertical, the floor of storey 0 at z = 0). Plan cuts are drawn in the level frame, so regions read off a plan render go in as they are |
| `edits` | a list; each edit has a unique `id`, a `kind`, its own fields, and an optional free-text `note` |

The reader is strict: an unknown key, a missing field, a wrong type, a
non-finite number, an unknown DoF name, a non-rigid matrix or a duplicate id
throws, and the message names the edit and the field
(`pose edits: edit 'm-late-pass': delta.xyz: must be [x, y, z]`).

## Conventions

- **Nodes** are `sensor_frames.node_id`s. A node set is `[first, last]`
  (inclusive) or `{list: [12, 15, 19]}`. Nodes that are not in the solve (no
  depth, no surfels) are skipped with a warning; an edit none of whose nodes
  is in the solve is an error.
- **Poses** use the stored `sensor_frames.transform` convention (sensor frame
  to world), expressed in `frame`. A pose is `current` (the node's
  `transform` when the edits are lowered), `initial` (its
  `initial_transform`), or a transform.
- **Transforms** are `{xyz: [x, y, z], yaw_deg, pitch_deg, roll_deg}` with
  R = Rz(yaw) Ry(pitch) Rx(roll) about the frame's axes, or
  `{matrix: [16 numbers, row-major]}` (must be rigid). Omitted values are 0.
  A delta (`move`) rotates about its `pivot`, by default the centroid of the
  moved nodes' positions: `p' = pivot + xyz + R (p - pivot)`. So
  `{yaw_deg: 1.2}` turns a block in place, and `{xyz: [0.15, -0.11, 0]}`
  shifts it, which is how a person reads a plan cut. A `matrix` delta is
  applied as it is (about the frame origin), which is what a segment ICP
  produces.
- **DoF masks** (`dofs: [x, y, yaw]`) select which components an anchor,
  move or relative constrains. For anchor and move they are taken in the
  document frame: translation of the camera centre along x / y / z, and the
  yaw / pitch / roll (Z-Y-X Euler angles) of the camera's levelled frame,
  the camera rotation with its most vertical body axis put on z, so a
  portrait or a downward-looking camera has a well-defined heading. Because
  the components are Euler angles, a mask is exact: with `[x, y, yaw]` the
  target's height and tilt exert no force at all. For a relative edit they
  are the components of `b`'s pose error in `b`'s sensor frame. Default: all
  six.
- **Sigmas**: `sigma: {trans_m, rot_deg}` (default 0.02 m, 0.5 deg) for
  anchor / move / relative; `sigma_m` (default 0.01 m) for same_surface /
  on_plan_wall.

Every solve starts from `initial_transform` (the pose-graph editor's
repeatable re-solve), so applying the same file twice gives the same poses:
edits never compound.

## Kinds

### `anchor`: pin one node, also in the middle of a trajectory

An absolute prior on one node's pose. The regression-221 lesson is the reason
it exists: relative constraints between the two ends of a trajectory let the
solver spread the correction as a global shear; an anchor in the middle does
not.

```yaml
schema: reusex.pose_edits.v1
frame: world
edits:
  # plan.png: the hall corner at node 1203 sits on the plan; keep it there
  # but leave height and tilt to the planes.
  - id: a-1203
    kind: anchor
    node: 1203
    pose: current
    dofs: [x, y, yaw]
    sigma: {trans_m: 0.02, rot_deg: 0.3}
```

Lowering: one prior factor on the node whose residual is the position error
and the yaw / pitch / roll error (see DoF masks), masked by `dofs`.

### `move`: shift a pass or segment rigidly

A rigid delta for a node range: the range's initial poses moved by `delta`.

```yaml
schema: reusex.pose_edits.v1
frame: world
edits:
  - id: m-late-pass
    kind: move
    nodes: [358, 389]
    delta: {xyz: [0.15, -0.11, 0.0], yaw_deg: 1.2}
    sigma: {trans_m: 0.03, rot_deg: 0.5}
    note: >-
      seg 358-389 vs rest 65 -> 31 mm (segment ICP); plan cut passes.png:
      kitchen wall doubled 12 cm
```

Lowering (`reusex-9fx`): a move corrects where the block sits relative to
the trajectory BEFORE it, not a place in the world. It becomes one relative
factor from the node solved just before the block (`p`) to the block's first
node (`a`), at the corrected offset `initial_p^-1 * delta * initial_a`, with
the move's sigma, and it **cuts** the odometry chain there: every odometry
edge from before `a` to `a` or later is left out of the solve (loop edges
stay). The block keeps the shape its own odometry gives it, carries the tail
after it, and follows `p` wherever the rest of the solve (plane factors,
loops, other edits) puts it; nothing before the block is pulled toward it.
On the initial poses this is the same placement as before: the block's
first node lands at `delta * initial_a`.

The earlier lowering, one absolute prior per node at `delta * initial` with
information / n, pinned the block's *initial* (drifted) shape at one world
place. A delta fitted on a `--fast` candidate then fought the plane factors
of the full solve, and the correction spread over the softer odometry chain
before the block (regression 221). It is still the fallback when there is no
node before the block (it holds the gauge), when the block is not one run of
the solve (a node list with holes), or when the move has a DoF mask (a cut
would leave the masked DoFs unconstrained); the solve logs which.

### `relative`: a manual edge

The pose of node `b` in node `a`'s frame (stored convention), the same as a
confirmed loop edge.

```yaml
schema: reusex.pose_edits.v1
frame: world
edits:
  - id: r-412-980
    kind: relative
    a: 412
    b: 980
    pose: current          # b relative to a as the current poses have it
    sigma: {trans_m: 0.02, rot_deg: 0.5}
```

`pose` is `current` or `initial` (the relative pose of the two nodes'
current / initial poses) or a transform. `dofs` select components in `b`'s
sensor frame.

### `same_surface`: two copies of one wall are one wall

Two node ranges see the same surface: the points of range `a` inside
`a.region` and of range `b` inside `b.region` lie on one plane. This is what
a doubled wall in a plan cut shows, and it constrains only what the doubled
wall measures: the offset along the wall normal and the tilt of the plane,
not the slide along the wall.

```yaml
schema: reusex.pose_edits.v1
frame: world
edits:
  - id: s-hall-west
    kind: same_surface
    a: {nodes: [1, 33], region: [-5.2, -3.4, -4.8, -2.0]}
    b: {nodes: [358, 389], region: [-5.2, -3.4, -4.8, -2.0]}
    sigma_m: 0.01
    note: "plan.png: west hall wall drawn twice, 9 cm apart"
```

Lowering: both ranges' depth is back-projected with the initial poses and
cut to the region (xy box in the document frame, all heights); a plane is
fitted to each side with RANSAC. A point-to-plane fit of `b`'s points onto
`a`'s plane gives the correction (the offset along the normal and two
tilts). It becomes relative-pose factors between the frames of `a` and the
frames of `b` (up to 12 each, evenly spread), so both blocks act as wholes
and the correction goes into the links between them rather than bending
either block. The information of a `b` frame comes from its own plane
points, `J = [q x n, n]` per point in that frame, and all pairs together
carry `J^T J / (K sigma_m^2)` over the K points: rank 3 (the normal offset
and two tilts), with no information along the wall or about its normal.

### `on_plan_wall`: these points lie on plan wall W

```yaml
schema: reusex.pose_edits.v1
frame: storey:2b1e6d8a-0000-5000-8000-000000000001
edits:
  - id: w-x-north
    kind: on_plan_wall
    plan: tests/fixtures/solid_model/newoffice.plan.yaml
    wall: x-north
    nodes: [2300, 2400]
    region: [0.0, 4.0, 6.0, 4.6]
    sigma_m: 0.01
```

Read and written by v1; not yet lowered (`rux optimize` warns and skips it;
bead `reusex-rcv.1.12`).

### `reject_edge`: leave a stored edge out

```yaml
schema: reusex.pose_edits.v1
frame: world
edits:
  - id: x-orb-17
    kind: reject_edge
    a: 17
    b: 3611
    edge_type: loop_closure
    note: "pair strip: the two frames show different doors"
```

The stored edge (either direction) is left out of this solve, as if its
status were `rejected`; the project's `pose_graph_edges` row is not changed
(use `rux graph prune --pair` for that).

## Residuals and contradictions

Every edit is reported after the solve: its translation and rotation error
over the constrained DoFs (a move: its junction, or the worst node of the
absolute fallback), its chi-square
(whitened squared residual), and whether GNC kept it as an inlier. Edit
factors are not GNC known inliers: they get the large but finite TLS
threshold `edit_inlier_cost` (`rux optimize --edit-inlier-cost`), so an edit
that contradicts the rest of the graph, or another edit, is down-weighted and
reported at `warn` with its residual rather than obeyed silently.
