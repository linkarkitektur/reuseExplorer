<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Solid model & scan-to-BIM: what Florent Poux does, what we have, what we're missing

**Status:** research / proposal. Nothing here is decided. Written 2026-10-13 for review
(e.g. by Fable) before any implementation or `docs/DIRECTION.md` change.

**Question asked:** our solid-model reconstruction (planes → CellComplex → MIP) and to
some degree our segmentation give worse results than (a) Florent Poux's Neurones 3D
pipeline and (b) a colleague's Claude-generated Rhino script. What do they do that we
don't, and how do we get there?

---

## 0. TL;DR

1. **Both better-performing approaches are *floor-plan first, then extrude*.** They
   find storeys from a Z histogram, level the scan, find walls as **density evidence in
   a 2D plan** (raster or 1D offset profile), build a **bounded wall graph** (corners
   and tees joined within a tolerance), find rooms by **flood-filling free floor
   space**, detect openings from **vertical occupancy on wall faces**, and only then
   extrude to solids. Everything is a robust local statistic with explicit,
   interpretable parameters, and every output carries evidence.
2. **Our pipeline is a global volumetric MIP (Ochmann-style) that never sees the
   semantics.** Its failure modes are structural, not tuning:
   - every vertical plane becomes an *infinite line* across the building;
   - every horizontal plane (including table tops) becomes a *building-wide slicing
     level*;
   - wall thickness is a greedy pairing with a fixed 0.25 m fallback;
   - the output drops room IDs and the solver's wall labels, leaving one unlabelled
     envelope mesh.
3. **Several cheap, high-leverage fixes exist before any new pipeline**, chiefly the
   label-0 bug that silently deletes the first SAM3 class ("wall" with the TensorRT
   defaults), storey-only slicing, and feeding semantic labels into plane
   classification.
4. **Recommendation:** add a new **floor-plan reconstruction path** modelled on Poux's
   published algorithms (which are concrete enough to port to C++), with typed outputs
   (walls with measured thickness, slabs, spaces, openings), and an **IFC4 export**.
   Keep the MIP as an optional global-consistency step over the *bounded* wall graph
   instead of infinite lines. Phased plan in §5.

---

## 1. Florent Poux / Neurones 3D

### 1.1 What is and isn't public

- **Neurones 3D (N3D)** — <https://neurones3d.com/> — is a proprietary,
  closed-source Rust/wgpu Windows app (v0.34.2 at the time of writing) with a paid SDK.
  The **full point-cloud → IFC pipeline is not open source.**
  - The scan-to-BIM page (updated 2026-09-24) still says N3D *does not write IFC*; it
    hands DXF/GeoJSON plus a room table to the modelling tool.
  - The 2026-10-12 article and 2026-10-13 video show an unreleased build that writes
    IFC4 (3,802 entities on an S3DIS office).
  - The roadmap lists "IFC plans as a building" for v0.36.
- **Public and portable:**
  - The papers (§1.4).
  - Two MIT-licensed Python kits in
    [`florentPoux/3d-data-materials`](https://github.com/florentPoux/3d-data-materials).
  - The book code in
    [`florentPoux/3D-Data-Science`](https://github.com/florentPoux/3D-Data-Science).
  - The full inline code of the 2026-10-12 Medium article
    [*3D Scan-to-BIM 101*](https://medium.com/data-science-collective/3d-scan-to-bim-101-hands-on-automation-course-92f9e45acd7d)
    (walls → IFC).
- **Known only by video title or marketing:**
  - floor occupancy
  - IfcSpace generation
  - columns and beams
  - "41 furniture objects replaced by parametric library models"
  - "merging with recovery"
  - "corners joined into one graph"

  Video: [3D SCAN-TO-BIM Fully Automated: Point Cloud to IFC](https://www.youtube.com/watch?v=AQHyML8-h0s).
  We could not get its transcript (YouTube rate-limited), so §1.3's descriptions of
  those stages are **inferred** from the docs plus the older papers.

### 1.2 The 17-stage pipeline (from the video description)

> semantic classes → storeys from peak detection → wall axis → faces voted across heights
> → thickness → merging with recovery → corners joined into one graph → openings
> (open/closed) → floor occupancy → IfcSpace → columns & beams → furniture replaced by
> parametric library models → IFC4 model → fit check against the cloud → drawings &
> exports (IFC, GLB, DXF, CSV, JSON, PDF)

Cited references in the video: IFC4, `scipy.signal.find_peaks`, NetworkX, Smart Point
Cloud, semantic modelling.

### 1.3 The algorithms, stage by stage

**(a) Semantic segmentation: segments first, then classes.**
- N3D runs three stages:
  1. Parameter-free region growing
     ([Poux, Mattes, Kobbelt 2020](https://doi.org/10.5194/isprs-archives-xliv-4-w1-2020-111-2020);
     [Automation in Construction 2022](https://doi.org/10.1016/j.autcon.2022.104250),
     [OA copy](https://orbi.uliege.be/handle/2268/291886)).
  2. A **decomposition into primitives**: H-plane, V-plane, slanted, cylinder, linear,
     cluster, residue. On S3DIS Area 4 this cuts 9,626 regions to 1,501.
  3. A **random forest on per-segment features**, giving 10 indoor classes at about
     82% per-point accuracy on CPU
     ([docs](https://neurones3d.com/docs/unsupervised-segmentation/),
     [classification](https://neurones3d.com/point-cloud-classification/)).
- **Region-growing parameters are derived from the data:**
  - ε = the median, over points, of the smallest radius where PCA planarity > 50%.
  - τ = the median point count in an ε-sphere.
  - α = the cone angle that holds τ points on a sphere of radius 2ε.
  - Growth accepts points within α of the region normal and within 3ε of the plane.
  - The plane is refit at exponentially increasing intervals, which is very close to
    our vendored `PlanarRegionGrowing`.
  - Edge refinement: unassigned points join the nearest region within an ε band, with
    no normal test.
- **Segment features for the random forest:**
  - relative centroid, normal, and variances along the tangent, bitangent and normal
  - length, width, height, area, volume
  - graph centralities (degree, betweenness, …) over the segment adjacency graph
- **Older work** classified segments by ontology rules instead
  ([IJGI 2019](https://doi.org/10.3390/ijgi8050213),
  [self-learning ontology 2020](https://doi.org/10.5194/isprs-archives-xliii-b2-2020-309-2020)):
  - floor = largest horizontal element at extreme low Z
  - wall = vertical element connected to floor/ceiling, height > 2 m
  - table = horizontal element at 0.7–1.1 m
- **Newest work** ([ISPRS 2026](https://doi.org/10.5194/isprs-archives-XLIX-B2-2026-1311-2026))
  labels each segment zero-shot: SAM2 + CLIP over six orthographic renders per segment,
  plus an OpenUSD scene graph (near / adjacent / above / below / inside).
- **Key point for us:** classification happens **per geometric segment**, not per
  point. Labels are voted over a segment that is already geometrically coherent.

**(b) Levelling, storeys, floor and ceiling.**
- Code seen:
  [`open_vocab_3d_semantics.py`](https://github.com/florentPoux/3d-data-materials/tree/main/open-vocabulary-3d-semantics)
  and Step 2 of the Medium article.
- Floor and ceiling planes are fit by iterated least squares, keeping inliers within
  2 cm.
- **True vertical = the normalised bisector of the floor and ceiling normals.** On
  multi-station scans the two tilt in opposite directions by about 1°, and two faces of
  one wall leaning together is the signature of a wrong vertical.
- Translate so the floor is at z = 0.
- A 1D histogram of Z picks the bands: floor = the lowest bin over 25% of the maximum
  count, ceiling = the highest such bin.
- Multi-storey: `find_peaks` on the Z histogram; storeys are consecutive floor and
  ceiling peaks.

**(c) Walls: two-face measurement.** Medium article, full code inline.
1. **Manhattan axis.** Wall-point PCA normals (k = 16), trusted when |n_z| < 0.25 and
   planarity > 0.45. **4θ circular mean:** φ = ¼·atan2(Σ sin 4θ, Σ cos 4θ), with axes φ
   and φ + 90°.
2. **Directional families.** A point belongs to axis *d* if |n·d| > cos 15°. Compute
   its offset o = p·d and its position along the wall, s.
3. **1D offset profile.** A 1 cm histogram of o, smoothed with [1,2,3,2,1]/9. Each
   `find_peaks` peak (height 60, separation 4 cm, prominence 30) is a **wall face**.
   Split each face into runs along *s* at gaps > 0.6 m (doorways); drop runs < 0.5 m
   (jambs).
4. **Pair faces into walls.** A pair must pass all four tests:
   - gap between 4 and 45 cm
   - shared length ≥ 50 cm
   - **empty valley** between the peaks (≤ 15% of the weaker peak)
   - **co-visibility**: on 25×25 cm (s, z) cells, ≥ 30% of the cells seen on one face
     are also seen on the other

   These tests reject linings and panels in front of walls, and they survive cropped
   scans. Pair greedily by longest shared stretch.
5. **Thickness.**
   - Per (s, z) cell, take the median offset per face.
   - Fit a Theil–Sen line offset(z) per face; drop outlier cells.
   - Fit an OLS line per face; thickness is the distance between the lines at
     mid-height, and the slope gives the lean.
   - **Bootstrap over cells, not points**, for a 95% CI.
   - Measure with *all* wall points; filtering by normals biases thin walls thicker.
6. **Room-on-both-sides test.** Density of non-wall points below 1.2 m in a strip
   10–60 cm beyond each face decides internal vs external (IsExternal).

**(d) Walls: raster and vector route, extension to corners.**
[`scan-to-floor-plan-dxf/walls_to_vectors.py`](https://github.com/florentPoux/3d-data-materials/tree/main/scan-to-floor-plan-dxf),
MIT, with 17 tests.
1. Take a wall band (floor + 0.3 m to ceiling − 0.3 m) and rasterise it on a 2 cm grid.
2. Closing, then remove small blobs, then skeletonise, then prune spurs.
3. Recursively split each chain where it deviates more than 4 cm from its chord, fit
   each piece by total least squares, and merge collinear runs.
4. **Square** walls within 8° onto the dominant axis.
5. **Snap** each wall to the median of the raw wall points; two passes, 10 cm then 4 cm.
6. **Join (extension to corners).** Intersect every non-parallel pair of walls and
   process the pairs sorted by distance:
   - **corner** if both endpoints are within 0.75 m of the intersection: move both;
   - **tee** if one endpoint is within 0.75 m and the intersection lies on the other
     segment: move only that endpoint.

   N3D production uses **0.9 m for corners and 2.2 m for tees**
   ([docs](https://neurones3d.com/docs/floor-plan/)).
7. Build room polygons with `shapely.polygonize(unary_union(walls))`.
8. Compute a "seen share" per wall. Walls seen less than 50% go on a `WALL_ASSUMED`
   layer.

**(e) Rooms: free-space flooding.**
- N3D finds rooms by "flooding the free space between floor and ceiling, split at the
  doorways … a table does not wall off the room"
  ([docs](https://neurones3d.com/docs/scene-graph-and-spaces/)).
- It reports area, volume, height, perimeter, compactness, connects-to and contains per
  room. Long thin spaces full of furniture holes are labelled "Open plan".
- The algorithm itself is not public. Our inference: a floor occupancy grid in which
  only wall evidence blocks a cell, flooded with doorway cuts, then each component
  polygonised and extruded to an IfcSpace. This is essentially what the colleague's
  script does (§2).

**(f) Openings.**
- "Classified from vertical occupancy: a door where the wall is open to the floor, a
  window where it is open above a sill."
- The video lists "openings (open/closed)", probably meaning door leaf present or not.

**(g) Objects and furniture: library matching and Z constraints.**
- Only the 2018 method is public:
  [Remote Sensing 10(9):1412](https://doi.org/10.3390/rs10091412), reproduced as
  Ch. 6 of the [thesis](https://orbi.uliege.be/bitstream/2268/235520/1/THE_SMART_POINT_CLOUD_digital.pdf).
- **Pose.** Robust PCA (outlier-reweighted).
- **Gravity constraint.** Rotate each part about Z only, so it stays upright and aligned
  with its parent (the floor, or the chair the part belongs to).
- **Parts.** Voxel histograms along the principal axes find cuts, which split an object
  into legs, seat and backrest.
- **Multi-LoD models:**
  - L0: bounding box
  - L1: parametric shape
  - L2: cuboid assembly
  - L3: voxel-refined

  RMSE is about 5 / 2 / 1 cm for L1 / L2 / L3.
- **Library match (ModelNet10).**
  - No ICP and no descriptor retrieval; library models are pre-oriented.
  - Rank by height/length and height/width **ratio differences**, plus a symmetry
    score.
  - Fit by matching centroids, then **independent per-axis scale**.
- **Scene aggregation.**
  - Order: floor → ceiling → walls → beams/columns → doors/windows → furniture.
  - Doors and windows are boolean-subtracted from walls (CSG).
  - Context rules such as "the feet have the same height" act as a Z snap.
- The 2026 video's "furniture replaced by parametric library models" is almost
  certainly the productised descendant of this method (inferred).

**(h) IFC export.** The Medium article uses ifcopenshell 0.8.4.
- Explicit SI metres. `assign_unit()` with no arguments defaults to millimetres.
- Spatial structure: IfcProject → IfcSite → IfcBuilding → IfcBuildingStorey.
- Each wall gets an IfcWallType with an IfcMaterialLayerSet at the measured thickness,
  and `add_wall_representation(length, height, thickness)` (an extruded box in the wall
  frame).
- Thickness is written **three times** (body solid, layer set,
  `Qto_WallBaseQuantities.Width`) because tools disagree on which one to read.
- `Pset_WallCommon.IsExternal`.
- A custom **`Scan_Evidence`** pset: thickness CI, P5/P95, lean, co-visibility, points
  per face, noise.
- **Deterministic GlobalIds** via uuid5 from the scan, class, name and id, so a
  re-import updates elements instead of duplicating them.
- **Read-back validation:** `ifcopenshell.validate`, the solid's thickness must equal
  the Qto, plus a point-to-solid fit.
- Class mapping: wall → IfcWall, floor/ceiling → IfcSlab, door → IfcDoor,
  window → IfcWindow, room → IfcSpace, furniture → library instances.

**(i) Watertightness.**
- Poux does **no global watertight solve**. Solids are per-element extrusions, and
  rooms are extruded polygons.
- N3D's "clean mesher" (dual contouring plus a flood fill) is closed but explicitly not
  manifold.
- He cites Ochmann, Vock & Klein 2019, our MIP's lineage, only for the two-face wall
  idea.

### 1.4 Publications

| Paper | Link | Relevance |
|---|---|---|
| Poux et al., *3D Point Cloud Semantic Modelling: Integrated Framework for Indoor Spaces and Furniture*, Remote Sensing 2018 | <https://doi.org/10.3390/rs10091412> | Furniture library matching, gravity constraints, CSG, IFC mapping |
| Poux & Billen, *Voxel-based 3D Point Cloud Semantic Segmentation: Unsupervised Geometric and Relationship Featuring vs Deep Learning*, IJGI 2019 | <https://doi.org/10.3390/ijgi8050213> | Unsupervised voxel + rule classification (S3DIS IoU: floor 92.4, ceiling 85.4, wall 65.2) |
| Poux, *The Smart Point Cloud* (PhD, ULiège 2019) | <https://orbi.uliege.be/bitstream/2268/235520/1/THE_SMART_POINT_CLOUD_digital.pdf> | Readable full text of the 2018 and 2019 papers |
| Poux & Ponciano, *Self-learning ontology for instance segmentation of 3D indoor point cloud*, ISPRS 2020 | <https://doi.org/10.5194/isprs-archives-xliii-b2-2020-309-2020> | Rule learning on top of the ontology |
| Poux, Mattes, Kobbelt, *Unsupervised segmentation of indoor 3D point cloud: application to object-based classification*, ISPRS 2020 | <https://doi.org/10.5194/isprs-archives-xliv-4-w1-2020-111-2020> | Parameter-free region growing plus a random forest on segments |
| Poux, Mattes, Selman, Kobbelt, *Automatic region-growing system for the segmentation of large point clouds*, AutCon 2022 | <https://doi.org/10.1016/j.autcon.2022.104250> | Scaled version of the above; N3D's segmenter |
| Bassier, Vergauwen, Poux, *Point Cloud vs. Mesh Features for Building Interior Classification*, RS 2020 | <https://doi.org/10.3390/rs12142224> | Segment feature design |
| Poux & Key, *LLM-Supervised Point Cloud Processing*, ISPRS 2026 | <https://doi.org/10.5194/isprs-archives-XLIX-B2-2026-1311-2026> | SAM2/CLIP segment labelling, OpenUSD scene graph |
| Poux et al., *Model for Semantically Rich Point Cloud Data*, ISPRS Annals 2017 | <https://doi.org/10.5194/isprs-annals-IV-4-W5-107-2017> | Smart Point Cloud data model |

Other pointers:
- N3D docs: [floor plan](https://neurones3d.com/docs/floor-plan/),
  [scene graph & spaces](https://neurones3d.com/docs/scene-graph-and-spaces/),
  [primitives](https://neurones3d.com/docs/decompose-into-primitives/),
  [USD & IFC](https://neurones3d.com/docs/usd-and-ifc/),
  [meshing](https://neurones3d.com/docs/meshing/),
  [scan-to-BIM](https://neurones3d.com/scan-to-bim/),
  [roadmap](https://neurones3d.com/roadmap.html).
- Medium:
  [scan → CAD floor plan](https://medium.com/data-science-collective/how-to-turn-a-3d-scan-into-a-cad-floor-plan-f38562744833),
  [shape detection for indoor modelling](https://medium.com/data-science/3d-point-cloud-shape-detection-for-indoor-modelling-70e36e5f2511).
- Book code: [`3D-Data-Science`](https://github.com/florentPoux/3D-Data-Science),
  chapters 07 (features), 09 (RANSAC / region growing), 10 (CadQuery CSG toy) and
  12 (RANSAC + DBSCAN).
- Not Poux, but an open-source point-cloud → IFC tool worth evaluating:
  *Cloud2BIM* (AutCon 2025),
  <https://www.sciencedirect.com/science/article/pii/S0926580525003437>.

---

## 2. The colleague's Rhino script ("PointCloudRooms")

- Source: Claude artifact <https://claude.ai/artifact/WxMr2SAxqYruJGPD2DviGu>.
- A C# RhinoCommon script for the Rhino 8 Script Editor, about 34 kB, single storey.
- Its pipeline is a minimal version of Poux's §1.3 (b), (d), (e) and (f), entirely on a
  2D raster.

| Step | What it does | Key defaults |
|---|---|---|
| Floor / ceiling | Mode of the Z histogram in the bottom / top 35% of the 0.5–99.5 percentile range | 20 mm bins |
| Grid angle | Rotate the wall-band points (35–85% of storey height) 0–90° and maximise Σ(bin count²) of the axis histograms; refine to 0.05° | — |
| Wall raster | In the band from door height to ceiling − margin, split into 10 cm layers; a cell is wall if ≥ 50% of its layers have points | 100 mm cells, door height 2150 mm |
| Floor raster | Points within ±150 mm of the floor | — |
| Morphology | Dilate the wall mask by 1 cell and the floor mask by 2 cells (bridges furniture holes) | — |
| Exterior | Flood from the border through cells that are neither wall nor floor | — |
| Rooms | 4-connected components of free cells; drop components under 1.5 m² or narrower than 1.4 m | — |
| Room solid | **An axis-aligned box per room.** Trim niches (rows/columns under 30% of max occupancy); local floor and ceiling modes; snap each face to the wall-point depth peak within −400/+350 mm (25 mm bins) | — |
| Openings | Raster each box face at 10 cm. A cell is open if it has no wall points in the slab zone and no occluders within 1.2 m in front; morphological opening, connected components, rectangle fit | Min 500×400 mm, fill ≥ 0.55 |
| Opening class | Door if it starts below 300 mm and is 1.7–2.4 m tall; floor-to-ceiling "opening"; otherwise window. Merge mullions; internal if a room exists within 700 mm behind | — |
| Output | Rhino Breps per room plus attributes (area, height, angle); opening polylines on Doors / Windows / Openings layers | — |

**Why it beats our MIP on typical offices.**
- Nothing about it is global or combinatorial, so it never "explodes".
- Furniture can't create spurious levels or walls. The wall test requires vertical
  continuity across many layers, and floor dilation bridges furniture holes.
- Rooms are space (an occupancy component), not clusters of wall points.
- Openings use an occluder check, which is the key trick: missing wall points behind
  furniture are not treated as openings.

**Limits.**
- Rooms are **rectangles only**; L-shaped and non-Manhattan rooms are lost.
- Single storey; no sloped surfaces.
- No wall thickness or wall objects: walls are the gaps between boxes.
- Open plans become one room.

**Takeaway.** Its *ideas* (wall band × layer fill, floor-dilated exterior flood, the
occluder-aware opening raster, face snapping to depth peaks) are cheap and robust, and
belong in our pipeline. Its *output representation* (one box per room) does not.

---

## 3. What ReUseX does today

Read from code on 2026-10-13. Full stage contracts are in `docs/CONTRACTS.md`.

```
create clouds → annotate → project → planes → rooms → instances → mesh → texture → windows
```

| Stage | Algorithm | Where |
|---|---|---|
| Planes | Noise-adaptive planar region growing (radius 0.5 m, 25°, d = clamp(3σ, 1–15 cm)) | `segmentation/segment_planes.cpp`, `extern/include/pcl/planar_region_growing.hpp` |
| Rooms | Embree mutual-visibility graph between plane-inlier samples (O(N²), unweighted) → Leiden → k-NN propagation. A room is a **cluster of surface points**, with no polygon or height | `segmentation/segment_rooms.cpp` |
| Semantics | SAM3 open-vocabulary 2D → 3D, either a voxel vote during `create clouds` or a z-buffer projection in `create project` (last frame wins, no multi-view vote, hard-coded 7 m / 0.15 m) | `vision/project.cpp:236-298`, `segmentation/reconstruct.cpp:197-385` |
| Instances | Euclidean clusters per class (0.5 m). No OBB, pose or model | `segmentation/segment_instances.hpp` |
| Mesh: preprocessing | CGAL soft regularisation (10°); `merge_planes` (treats ±n as equal, 0.5 m offset tolerance); hard vertical/horizontal snap; **greedy opposite-plane pairing** (0.6 m), else a **synthetic back plane at 0.25 m** | `reconstruction/mesh.cpp:101-115`, `geometry/utils.cpp` |
| CellComplex | Each vertical plane becomes an **infinite line** clipped to the bounding box + 1 m in a CGAL arrangement. **Every horizontal plane** becomes a global section level. Cells are prisms; capped at 5000 cells (#213) | `reconstruction/CellComplexConstructor.cpp` |
| Evidence | Room probabilities from 100 rays per cell centre against room-labelled discs; face coverage = occupied fraction of a 0.2 m grid | `CellComplexRoomProbabilities.hpp`, `CellComplexFaceCoverage.hpp` |
| Solidifier MIP | Ochmann-style: binary room and wall variables per cell; data term vs α·(1 − coverage)·area; consistency constraints; HiGHS / cuOpt, 120 s; per-section path when there are ≥ 2000 cells | `reconstruction/Solidifier.cpp` |
| Output | Faces between selected and non-selected cells, fan-triangulated into **one unlabelled PolygonMesh**. Faces between rooms are dropped; **solver wall labels are discarded** (`mesh.cpp:201`) | `Solidifier_to_mesh.cpp` |
| Windows | Coplanar mesh regions are wall candidates; window-instance points are projected and offset 0.5 m. Windows only: `ComponentType::door` and `wall` exist, but `WallData{}` is a stub | `reconstruction/create_windows.cpp`, `geometry/BuildingComponent.hpp` |
| Export | Rhino (.3dm), Speckle, PLY, E57, CSV, COLMAP, MaterialEPAS. **No IFC** | `io/` |

**Verified defects.** These are bugs regardless of direction.

1. **Label-0 collision.** SAM3 class ids start at 0 (`vision/tensor_rt/Sam3.cpp:188`).
   `core::api_to_point_label` passes 0 through unchanged
   (`core/label_semantics.hpp:81-89`, pinned by `tests/unit/core/test_label_semantics.cpp:47`).
   But point label 0 is `kUnlabeled`.
   - Result: the **first prompt class is indistinguishable from unlabelled**. That is
     "wall" in the TensorRT defaults (`vision/tensor_rt/Data.hpp:38`) and "ceiling"
     in the ONNX defaults.
   - `segment_instances` skips label 0, so the walls never become instances.
   - STANDARDS §3 describes API 0..N vs points 1..N, which implies a +1 shift that the
     code does not do.
   - Already noted as "class-0 off-by-one" in earlier GUI work.
2. **Dead CLI flags.** `create mesh -a/--angle` and `-l/--distance` are parsed but never
   reach `MeshOptions` (`apps/rux/include/create/mesh.hpp:18-28`). The walk-through
   guide still recommends them.
3. **Docs overstate the output.** `CONTRACTS.md:184` and `docs/guides/Process walk through.md`
   call the mesh "room-partitioned / watertight per room". It is one envelope mesh.

**Suspected and untested.** `merge_planes` uses |dot|, a 0.5 m offset and about 26° of
angle slack (`geometry/utils.cpp:227-231`). That likely **merges the two faces of a wall
into one plane** before `make_pairs`, which then invents a 0.25 m back face. Worth a unit
test first.

---

## 4. Gap analysis

| Capability | Poux / N3D | Colleague script | ReUseX today | Gap |
|---|---|---|---|---|
| Gravity / levelling | Floor/ceiling-normal bisector | Assumes z-up | Assumes z-up everywhere | **Missing**: small, foundational |
| Storeys | Z-histogram `find_peaks` | Z mode, one storey | None; every horizontal plane is a global section | **Missing**: also the main cause of MIP sliver sections |
| Semantic segmentation | Per *segment* (region growing → primitives → random forest; or SAM2/CLIP per segment) | None (geometry only) | Per *point* SAM3 projection, last frame wins; label-0 bug | Fusion step missing: **vote labels over planes/segments** |
| Semantics → geometry | Classes select wall/floor/ceiling points | Z bands act as proxy classes | **Not used at all** by planes, rooms or mesh | **Missing**: biggest conceptual gap |
| Dominant axes | 4θ circular mean | Histogram-variance sweep | Soft CGAL regularisation only | Missing (cheap) |
| Wall faces | 1D offset-profile peaks per axis family, split into runs at gaps | Raster layer fill | Region-growing planes | Partially present (planes); no runs or extents |
| Wall pairing / thickness | Four tests (gap, overlap, empty valley, co-visibility); Theil–Sen + cell bootstrap CI; lean | None | Greedy opposite-normal within 0.6 m, else a fixed 0.25 m | **Weak**: no measurement, no CI, likely face merging |
| Wall extents / corners | Bounded segments; corner/tee join graph (0.9 m / 2.2 m); squaring and snapping | Box faces snapped to depth peaks | Infinite lines; the MIP decides activity | **Missing**: root cause of cell explosion |
| Rooms | Free-space flood split at doors; polygon + metrics | Floor-occupancy components (boxes) | Leiden over a visibility graph of wall points; no polygon | **Different paradigm**: need occupancy-based rooms |
| Openings | Vertical occupancy on wall faces; door / window / open-closed | Occluder-aware face raster; classes; internal/external | Windows from SAM3 instances, offset 0.5 m; no doors | Mostly missing |
| Solids | Extruded walls, slabs, spaces; CSG openings | Boxes per room | One envelope mesh | **Missing typed solids** |
| Object modelling | OBB, gravity-aligned; library ratio/symmetry match; per-axis scale; Z snap | — | Point clusters only | Missing |
| Evidence / QA | Seen share, `WALL_ASSUMED`, Scan_Evidence pset, fit check | — | `analyze quality/accuracy` on clouds only | **No solid-model metric** |
| IFC | IFC4 via ifcopenshell; deterministic GUIDs; read-back validation | — | None (Rhino/Speckle only) | Missing |
| Global consistency | None (local heuristics + joins) | None | MIP (watertight by construction) | **Our one real advantage**, if fed clean input |

**Root-cause reading.** Our MIP is not inherently worse than heuristics. Ochmann-style
labelling is state of the art when its *input arrangement* is clean. We feed it infinite
wall lines plus furniture-level slicing plus semantically blind planes, which gives cell
explosion, slivers, timeouts (#213, #226) and spurious walls. Both better approaches win
mainly by **cleaning and typing the evidence before any geometry is assembled**.

---

## 5. Proposed plan

Phases are ordered by leverage per effort. Each phase is independently useful and
measurable (STANDARDS §8). Estimates are rough.

### Phase 0: Fix and measure (≈ 1 week)
- **Fix the label-0 collision.** Shift API classes by +1 into point labels, migrate or
  flag existing `labels`, and update the test and STANDARDS §3. Blocks everything
  semantic.
- **Fix `create project` fusion.** Multi-view majority vote with a background-never-
  overwrites rule; make the depth cutoff and tolerance parameters. Make the
  `reconstruct` vote deterministic on ties and ignore unlabelled votes.
- **Fix flags and docs.** Remove or wire the dead `create mesh -a/-l`; correct
  CONTRACTS and the walk-through.
- **Test the suspected wall-face merge** in `merge_planes`.
- **Build a solid-model benchmark.** Without it nothing below is measurable. Metrics:
  - room count and room IoU (2D polygons)
  - wall-position error and wall-thickness error
  - opening recall
  - point-to-solid fit (mean, % within 2 cm, as Poux does)

  Candidate ground truth:
  - S3DIS (room annotations; Poux reports on it, so results are comparable)
  - ARKitScenes 3DOD for furniture
  - one or two of our own scans with a hand-modelled Rhino reference, where the
    colleague's output can serve as a baseline

### Phase 1: Levelling + storeys (≈ 3–5 days)
- `segmentation/level.hpp`: floor/ceiling fit and bisector vertical; store the transform.
- Storey detection by Z-histogram peaks over horizontal-plane points.
- **Quick win for the existing MIP:** CellComplex slices only at storey-level horizontal
  planes (floor, ceiling, slab). Furniture-level planes are excluded. This should cut
  cell count and slivers without changing the solver.

### Phase 2: Semantics × geometry fusion (≈ 1 week)
- Per-plane class = the majority of SAM3 point labels over its inliers, with a
  confidence. Fall back to rules (orientation + height + extent, Poux 2019/2020 style)
  where labels are missing.
- Persist it as a `plane_classes` table/cloud (needs a CONTRACTS entry).
- Use it to:
  - drop furniture and clutter planes from reconstruction;
  - choose floor and ceiling per storey;
  - restrict wall candidates.
- Optional later: a random forest on segment features (Poux 2020), or SAM2/CLIP on
  segment renders, if the SAM3 projection proves too noisy.

### Phase 3: Wall extraction with measured thickness (≈ 2 weeks)
- A new `reconstruction/walls.{hpp,cpp}` porting the Medium article's algorithm (§1.3 c):
  1. Dominant axes by 4θ mean; extend to *N* families from clustered plane normals so
     non-Manhattan walls survive.
  2. Offset-profile peaks give faces; split them into runs along s.
  3. Four-test pairing.
  4. Theil–Sen thickness + cell bootstrap CI + lean.
  5. Room-both-sides test for internal/external.
  6. One-sided walls get a default thickness, flagged `assumed`.
- **Wall graph:** squaring, snapping, corner/tee joins (start with N3D's 0.9 m / 2.2 m),
  collinear bridging, and a per-wall seen share.
- Output: fill `BuildingComponent{type=wall}` (finally populating `WallData`): axis
  segment, thickness ± CI, height, lean, evidence. Exportable to Rhino and Speckle
  immediately.

### Phase 4: Rooms from floor occupancy (≈ 1–1.5 weeks)
- Per storey, a 2D grid in the levelled frame. A cell is blocked only by the wall graph
  (rasterised) and full-height wall-band evidence (the colleague's layer-fill test). A
  cell is floor if it has floor-class points; dilate floor to bridge furniture holes.
- Exterior flood from the border; connected components of the interior; split at
  doorways (Phase 5 openings, or narrow-constriction detection as a fallback).
- **Room polygons** from `polygonize(wall graph)` faces labelled by occupancy component,
  not rectangles. Per-room floor/ceiling height and metrics.
- Compare against the Leiden rooms on the benchmark; keep the better one, or use
  occupancy rooms as the MIP's room evidence instead of ray votes.

### Phase 5: Openings (≈ 1 week)
- Per wall face, an (s, z) occupancy raster with the **occluder check** from the
  colleague's script.
- Combine with the SAM3 door/window instances for classification.
- Door / window / opening classes by sill and head height; mullion merging;
  host-wall link (`parent_id`).
- Replaces the 0.5 m-offset window polygons in `create windows`.

### Phase 6: Solids + IFC export (≈ 1.5–2 weeks)
- Solids:
  - walls = extruded boxes in the wall frame (optionally leaning), with openings
    subtracted;
  - slabs = storey floor/ceiling polygons × thickness;
  - spaces = room polygons extruded floor → ceiling.

  All of this is simple, exact geometry (CGAL or OpenNURBS breps), already exportable
  to Rhino.
- **`rux export ifc`:** IFC4 with
  - the spatial structure Project → Site → Building → Storey → Space;
  - IfcWall + IfcWallType + material layer set (thickness written three times, as Poux
    does);
  - IfcSlab, IfcDoor / IfcWindow + IfcOpeningElement, IfcSpace;
  - a `Scan_Evidence` pset;
  - deterministic GlobalIds from our existing `core/guid.hpp`;
  - read-back validation in a test.
- **Dependency decision needed:**
  - IfcOpenShell C++ (heavy; OCCT/CGAL; would need a nix overlay), vs
  - a small hand-written IFC4 STEP writer for this handful of entities (extruded area
    solids only; very feasible), vs
  - writing IFC from Python via the bindings (would need write APIs, which the bindings
    don't expose).

  Recommendation: a minimal native writer first, IfcOpenShell for validation in CI or
  tests only.
- `docs/research/scene-graph-object-relations.md` already sketches the IFC spatial
  hierarchy; reuse it.

### Phase 7: Global consistency, the MIP as a refinement (optional, ≈ 1–2 weeks)
- Rebuild the CellComplex arrangement from the **bounded wall graph** (segments plus
  joins) and storey slabs instead of infinite plane lines.
- Run the existing Solidifier over it with occupancy-based room evidence.
- Keep the solver's wall labels and room IDs in the output (fix `Solidifier_to_mesh`).
- Expected outcome: a small, well-posed MIP that turns the local heuristics into a
  guaranteed watertight, consistent model. This combines our strength with theirs.

### Phase 8: Objects and furniture (≈ 2–3 weeks, lower priority)
- Per instance: robust OBB, gravity-constrained (rotate about Z only), **Z snap to the
  storey floor**, and support relations (on-floor, on-table) from the scene graph.
- Library matching à la Poux 2018: ratio + symmetry ranking against a small parametric
  library per class (chair, table, desk, cabinet, shelf), then per-axis scaling.
- Output as IfcFurnishingElement instances, and as Rhino blocks.
- Ties into the material-passport and reuse inventory (DIRECTION's mission).

### Not recommended
- **Reimplementing N3D wholesale.** It's proprietary, and its published sub-algorithms
  are what matter.
- **Deep 3D semantic segmentation networks.** SAM3 2D → 3D is already our path; the
  missing piece is *fusion over segments*, not a new model.
- **Porting the colleague's box-per-room output.** Borrow its raster tests instead.

---

## 6. Scene graph and IFC types

### 6.1 Poux's scene graphs

Poux has used a hierarchical, relational structure at every stage of his work. The
relations themselves are always simple geometric rules; none of them is learned.

| Work | Nodes | Relations |
|---|---|---|
| Smart Point Cloud data model ([2017](https://doi.org/10.5194/isprs-annals-IV-4-W5-107-2017), thesis) | Point → patch/segment → object, three levels | Ontology-typed (ifcOWL-inspired) |
| Semantic modelling ([RS 2018](https://doi.org/10.3390/rs10091412)) | Floor, ceiling, wall, beam, door, window, furniture, and furniture parts | Voxel 26-adjacency with DE-9IM: **guest / host / twin**. Part-to-whole aggregation order; a CSG tree |
| N3D ([docs](https://neurones3d.com/docs/scene-graph-and-spaces/)) | Primitives, segments or classes, plus **rooms as volumes of air** | Typed edges `touches, supports, covers, coplanar, parallel, near`; per room `connects-to` and `contains`. Exported as JSON, GraphML and CSV |
| LLM-supervised ([ISPRS 2026](https://doi.org/10.5194/isprs-archives-XLIX-B2-2026-1311-2026)) | Labelled segments: centroid, XY oriented bounding box, z range | `near` (< 2 m), `adjacent` (box faces < 0.3 m), `above`/`below` (ΔZ > 0.5 m), `inside`/`contains`. Serialised to **OpenUSD**; an LLM agent queries and edits it |
| Video, 2026-10-13 | Walls | "corners joined into one graph" (NetworkX) |

### 6.2 What we already have

- **#138 research.** [`scene-graph-object-relations.md`](scene-graph-object-relations.md)
  (2026-09-22) proposes:
  - an `instance_spatial` table (centroid, AABB, room);
  - a typed `scene_relations` edge table keyed by GUID (`contains`, `paired_with`,
    `group_member`, `adjacent_to`);
  - a thin query facade instead of the Boost.Graph `SceneGraph` stub;
  - a five-stage plan whose Stage 4 is IFC export of the spatial relations.
- **What it lacked.** Building-element nodes. At the time, the only nodes available
  were SAM3 instances and room *labels*. The floor-plan pipeline in §5 provides the
  missing ones: storeys, spaces (polygons), walls, slabs and openings.

### 6.3 Recommendation: model the graph on IFC's relationships

Make the scene graph the backbone of the solid model, not a separate feature. Use
the #138 storage design (GUID nodes + a typed edge table in ProjectDB), but take the
**node types and the core edge types from IFC**. IFC export then becomes a direct
mapping instead of a translation layer.

| Our relation | Produced by (phase) | IFC relationship |
|---|---|---|
| storey ⊃ space, space ⊃ element | P1 storeys, P4 rooms, Stage 1 of #138 containment | `IfcRelAggregates` (Building → Storey → Space), `IfcRelContainedInSpatialStructure` |
| wall —corner/tee— wall | P3 wall join graph | `IfcRelConnectsPathElements` (ATSTART / ATEND / ATPATH) |
| space —bounded by— wall/slab | P4 (a polygon edge is a wall segment) | `IfcRelSpaceBoundary` |
| opening voids wall; door/window fills opening | P5 | `IfcRelVoidsElement`, `IfcRelFillsElement` |
| space —connects via door— space | P4 + P5 | Derived from the space boundaries plus the door (N3D's `connects-to`) |
| object on floor / on table | P8 Z constraints | No direct IFC equivalent; keep it as our own typed edge, export as a pset or `IfcRelNests` |
| near / adjacent / paired_with / group_member | #138 Stages 2–3 | Not IFC; our own edges (`IfcGroup` + `IfcRelAssignsToGroup` for seating groups) |

Two consequences:
- The wall graph and the `supports` relation are not extras. They are the data
  structures the reconstruction needs anyway: corner joins, and the floor snap for
  furniture.
- The room↔door↔room graph gives N3D-style room connectivity for free.

### 6.4 IFC types: tag ours, don't adopt the schema

- **Don't make the full IFC schema our internal model.** IFC4 has roughly 800
  entities, and its geometry and placement machinery is heavy.
- **Do this instead.** Every typed entity (building component, space, storey, object)
  carries an **IFC class** (`IfcWall`, `IfcSlab`, `IfcSpace`, `IfcDoor`, `IfcWindow`,
  `IfcColumn`, `IfcBeam`, `IfcFurnishingElement`, …) plus a **PredefinedType**
  (`PARTITIONING`, `FLOOR`, `CHAIR`, …). Store it in `building_components` or the
  scene-graph node table.
- **Map SAM3 prompt classes to IFC classes** in a single table next to
  `label_semantics`; Poux uses the same kind of class → IFC map. The prompt list then
  stays user-editable while the export stays well-typed.
- **The material passports already exist.** They become IFC property sets attached to
  the element, plus a `Scan_Evidence` pset with thickness CI, seen share, and so on.
  This is the reuse-inventory angle no one else has.

### 6.5 C++ IFC libraries

Surveyed 2026-10-13.

| Library | License | State | C++ authoring API | Geometry kernel | nixpkgs | Verdict |
|---|---|---|---|---|---|---|
| **[IfcOpenShell](https://github.com/IfcOpenShell/IfcOpenShell)** (`IfcParse` + `IfcGeom`) | LGPL-3.0 (fine with our GPL-3.0+) | Very active (daily commits; 0.8.x line); IFC2x3, IFC4, IFC4x3 | Yes. `IfcParse` creates entities and writes STEP; `IfcHierarchyHelper` adds helpers. Example: [`src/examples/IfcOpenHouse.cpp`](https://github.com/IfcOpenShell/IfcOpenShell/blob/v0.8.0/src/examples/IfcOpenHouse.cpp) builds walls, a slab and windows in C++ | OpenCASCADE **or CGAL**, now a separate kernel plugin. Pure writing needs **no kernel** | Python wheel only (`python3Packages.ifcopenshell` 0.8.0); **no C++ dev package**, so we need our own overlay | **Recommended** |
| [web-ifc](https://github.com/ThatOpen/engine_web-ifc) | MPL-2.0 | Active (0.0.78) | Thin native API; designed for JS/WASM *viewing* | Own tessellator | No | Reading only |
| [IFC++ / ifcplusplus](https://github.com/ifcquery/ifcplusplus) | MIT | Effectively dormant | Yes (C++ class model + STEP read/write) | Own (Carve) | No | Risky long-term dependency |
| [STEPcode](https://github.com/stepcode/stepcode) | NIST / BSD-ish (check per file) | Maintained (0.8.2) | Generic SDAI generated from the IFC EXPRESS schema; **no IFC helpers** (units, psets, GUIDs, placements) | None | **Yes** (`nixpkgs#stepcode`) | Plan B |
| ODA IFC SDK | Commercial (about $3k + $2.25k per year) | Active | Yes, complete | Own | — | Excluded (cost, licence model) |
| xBIM | — | — | C#/.NET only | — | — | Not applicable |

**Recommendation.**
- Build **IfcOpenShell's `IfcParse` only** (write path, schema IFC4, no geometry
  kernel) through a nix overlay, like OpenNURBS and HiGHS.
- Add the **CGAL kernel plugin** later for read-back validation: thickness equals the
  Qto, point-to-solid fit. That reuses our existing CGAL dependency and avoids adding
  OpenCASCADE.
- **Fallback:** a hand-written Part-21 writer for about 20 entity types is 1–2 weeks of
  work. The syntax is trivial; the risk is attribute order, relationship wiring and
  representation contexts that Revit, Solibri and Bonsai reject. It would still need
  IfcOpenShell (or `ifcopenshell.validate` in a Python test) to validate the output.
- **Module placement:** the IFC writer belongs in `io/` (`io/ifc.hpp`), next to
  `rhino.hpp`, and goes through the existing `io/export_scene.hpp` intermediate
  representation. `export_scene.hpp` needs to grow typed elements and relations
  (§6.3).

---

## 7. Open questions for review

1. **Direction.** This is a new workstream (scan-to-BIM / IFC). `docs/DIRECTION.md`
   currently ranks SLAM quality, GUI and 3DGS above it, and IFC isn't in the mission
   text. Does it go in, and at what priority?
2. **Fate of the MIP.** Do we keep it as Phase 7, or replace it outright once
   Phases 3–6 work?
3. **IFC dependency.** IfcOpenShell `IfcParse` via an overlay (recommended, §6.5), or a
   native minimal writer?
4. **Scene graph scope.** Does #138's `scene_relations` table become the solid model's
   relation store (§6.3), and does its staged plan merge into §5?
5. **Ground truth.** Is S3DIS acceptable (license is non-commercial research)? Which
   of our own scans get a hand-modelled reference?
6. **Manhattan assumption.** How common are non-orthogonal walls in our target
   buildings (Danish renovation stock)? This decides how much effort Phase 3 puts into
   *N*-family support.
7. **Data contract.** New ProjectDB tables (wall graph, rooms as polygons, storeys,
   openings), or does everything go through `building_components`? This needs a schema
   bump and a CONTRACTS.md update.
