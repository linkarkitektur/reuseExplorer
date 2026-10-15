# Creating a 3D Model from Scans: A Complete Walkthrough

This guide documents the complete end-to-end workflow for transforming raw SLAM scans and 360° images into semantically-rich, simplified 3D surface models using ReUseX. The resulting models are suitable for building reuse projects, architectural documentation, spatial analysis, and digital twin applications.

> **Note on screenshots:** the walkthrough previously linked screenshots from a `./images/` folder that was never committed. Those images are tracked to be re-captured and added back (see issue #251); until then, each step below carries a short textual description of what it produces instead of a broken image link.
>
> **Note on scope:** the software-facing commands below (phases 8-15) are kept in sync with [`docs/CONTRACTS.md`](../CONTRACTS.md), which is the authoritative source for what each pipeline stage consumes and produces. If a command here and CONTRACTS.md ever disagree, CONTRACTS.md wins — please file an issue.

## Prerequisites

### Hardware Equipment

The workflow described in this guide uses an iPad Pro with LiDAR for SLAM-based spatial mapping and positioning, paired with an Insta360 X4 camera for capturing 360° panoramic imagery. This combination allows us to create both accurate geometric representations and photorealistic textures from the same capture session.

### Software Requirements

You'll need a complete ReUseX installation with all dependencies, along with RTABMap for database viewing and loop closure management. The Hugin/PanoTools suite (`pto_gen`, `pto_template`, `hugin_executor`) handles the 360° image stitching, while ExifTool verifies timestamp alignment between the scanner and camera data.

### Pre-trained Models

ReUseX leverages deep learning for semantic segmentation, requiring pre-trained models to be available before processing. The current version uses SAM3 (Segment Anything Model 3) optimized with TensorRT for universal image segmentation, providing significant performance improvements over previous versions. Earlier versions of ReUseX utilized YOLO models for architectural element detection. Model files should be placed in your project root directory or their paths specified via command-line arguments.

*(Equipment used for this guide: an iPad Pro with LiDAR and an Insta360 X4 camera, mounted together for simultaneous capture.)*

## Phase 1: Preparation and Time Synchronization

Before beginning any scan, the most critical preparatory step is synchronizing the timestamps between your scanner and camera. This synchronization enables accurate spatial placement of 360° images along the scanning path that will be reconstructed later. Take the time to ensure both the iPad Pro and Insta360 X4 have their clocks synchronized with minimal offset—ideally less than one second. Consider using NTP or another reference time source to achieve precise synchronization. Document the exact time sync for reference, as you'll need to verify this alignment later in the process.

## Phase 2: Initial Scanning Pass

The scanning process begins with creating a foundational spatial map of the environment using the scanner alone. Walk through the building systematically with the iPad Pro, ensuring comprehensive coverage of all areas of interest. Maintain steady, deliberate movement and proper scanner orientation throughout the pass, allowing the device to capture sufficient feature points for accurate mapping. This initial pass establishes the baseline geometry and positional framework that all subsequent steps will build upon.

*(This pass produces the RTABMap `.db` scan database used as the input to Phase 4/8.)*

## Phase 3: Image Capture Pass

With the spatial map established, perform a second walkthrough—this time capturing 360° images at key locations throughout the space. The scanner should remain active during this pass, as it simultaneously records its position for each captured image. This positional data becomes crucial for geolocating the panoramic imagery within the 3D coordinate system established during the initial scan. Walk through the building with both devices, pausing to capture 360° images at locations where you want visual documentation or where complex spatial relationships need photographic reference.

*(This pass produces the set of dual-lens Insta360 `.insp` captures processed in Phase 5, each timestamped and geolocated by the concurrent scan.)*

## Phase 4: Scan Assembly

When multiple scanning sessions have been performed—perhaps covering different floors or areas of a building—these individual databases need to be merged into a single unified representation. RTABMap provides the `rtabmap-assemble` tool for this purpose:

```bash
rtabmap-assemble "./path/to/scan1.db;./path/to/scan2.db" -o ./path/to/merged.db
```

This command combines the separate scan databases while maintaining their relative positions and coordinate systems, creating a cohesive model of the entire scanned environment.

*(Skip this phase entirely if you only recorded a single scanning session — proceed straight to Phase 6/7 with that one `.db` file.)*

## Phase 5: 360° Image Processing

The raw 360° images captured by the Insta360 X4 exist in a proprietary format that requires reprojection and stitching before they can be used. This process transforms the dual-lens captures into usable equirectangular panoramic images.

### Insta360 X4 Hugin Template

First, create a template file for the Insta360 X4 lens configuration. Save the following as a `.pto` file (e.g., `insta360_x4_template.pto`):

```
# hugin project file
#hugin_ptoversion 2
p f2 w5422 h2711 v360  k1 E8.07729 R0 n"TIFF_m c:LZW r:CROP"
m i0

# image lines
#-hugin  cropFactor=1
i w5888 h2944 f2 v391 Ra0 Rb0 Rc0 Rd0 Re0 Eev8.07728723345422 Er1 Eb1 r9.67415383504551 p-15.4753436799109 y-151.183643070868 TrX0 TrY0 TrZ0 Tpy0 Tpp0 j0 a0 b0 c0 d1468.6 e0 g0 t0 Va1 Vb0 Vc0 Vd0 Vx0 Vy0  S2942,5881,0,2938 Vm5 n"IMG_20260213_101653_00_001.jpg"
#-hugin  cropFactor=1
i w5888 h2944 f2 v391 Ra0 Rb0 Rc0 Rd0 Re0 Eev8.07728723345422 Er1 Eb1 r-9.67415383504551 p15.4753436799108 y28.816356929132 TrX0 TrY0 TrZ0 Tpy0 Tpp0 j0 a0 b0 c0 d-1468.6 e0 g0 t0 Va1 Vb0 Vc0 Vd0 Vx0 Vy0  S0,2941,0,2941 Vm5 n"IMG_20260213_101653_00_001.jpg"


# specify variables that should be optimized
v Ra0
v Rb0
v Rc0
v Rd0
v Re0
v Eev0
v Vb0
v Vc0
v Vd0
v Ra1
v Rb1
v Rc1
v Rd1
v Re1
v Vb1
v Vc1
v Vd1
v


# control points
c n0 N1 x3225 y2138 X2645.63237348499 Y2132.45666117389 t0
c n0 N1 x5694 y1046 X182 Y1048 t0
c n0 N1 x5760 y1435 X108 Y1441 t0
c n0 N1 x5428.99981496392 y2409.00050462385 X489.912577366332 Y2282.31091125528 t0
c n0 N1 x3537 y2460 X2408 Y2506 t0
c n0 N1 x3380 y2284 X2575 Y2326 t0
c n0 N1 x3190 y2047 X2701 Y2040 t0
c n0 N1 x3132 y1463 X2882 Y1462 t0
c n0 N1 x3071.00001051711 y1192.00000987523 X2776.17131868548 Y1198.54417522873 t0
c n0 N1 x3253 y801 X2661 Y788 t0
c n0 N1 x3450 y546 X2471 Y516 t0
c n0 N1 x5715 y1212 X130 Y1212 t0
c n0 N1 x5685 y1188 X118 Y1173 t0
c n0 N1 x5372 y636 X395 Y554 t0

#hugin_optimizeReferenceImage 1
#hugin_blender enblend
#hugin_remapper nona
#hugin_enblendOptions
#hugin_enfuseOptions
#hugin_hdrmergeOptions -m avg -c
#hugin_verdandiOptions
#hugin_edgeFillMode 0
#hugin_edgeFillKeepInput false
#hugin_outputLDRBlended true
#hugin_outputLDRLayers true
#hugin_outputLDRExposureRemapped false
#hugin_outputLDRExposureLayers false
#hugin_outputLDRExposureBlended false
#hugin_outputLDRStacks false
#hugin_outputLDRExposureLayersFused false
#hugin_outputHDRBlended false
#hugin_outputHDRLayers false
#hugin_outputHDRStacks false
#hugin_outputLayersCompression LZW
#hugin_outputImageType jpg
#hugin_outputImageTypeCompression LZW
#hugin_outputJPEGQuality 90
#hugin_outputImageTypeHDR exr
#hugin_outputImageTypeHDRCompression LZW
#hugin_outputStacksMinOverlap 0.7
#hugin_outputLayersExposureDiff 0.5
#hugin_outputRangeCompression 0
#hugin_optimizerMasterSwitch 0
#hugin_optimizerPhotoMasterSwitch 21
```

This template defines the lens parameters, distortion correction, control points, and output settings specifically calibrated for the Insta360 X4's dual-lens configuration. The control points establish the correspondence between the front and back lens images, enabling accurate stitching along the seam.

### Automated Stitching Script

The following Fish shell function automates the stitching workflow using the Hugin toolchain:

```fish
function stitch_insp_folder
    set -l template_file $argv[1] # Path to the .pto template file
    set -l folder_path $argv[2] # Path to the folder containing .insp files

    if not test -f $template_file
        echo "Template file '$template_file' does not exist."
        return 1
    end

    if not test -d $folder_path
        echo "Folder '$folder_path' does not exist."
        return 1
    end

    for img1 in $folder_path/*.insp
        set base (basename $img1 .insp)
        set out_pto $folder_path/$base.pto
        set out_templated_pto $folder_path/$base"_templated.pto"
        set prefix $folder_path/$base

        echo "Processing $base ..."

        # 1. Generate minimal .pto
        pto_gen -o $out_pto $img1 $img1

        # 2. Apply template
        pto_template --template $template_file -o $out_templated_pto $out_pto

        # 3. Stitch
        hugin_executor --stitching --prefix=$prefix $out_templated_pto

        # 4. Clean up temporary files
        echo "Cleaning up temporary files for $base ..."
        rm -f $out_pto $out_templated_pto $folder_path/$base"_template.pto"
        rm -f $folder_path/$base*.tif

        echo "Finished $base"
    end
end
```

Call this function with `stitch_insp_folder /path/to/insta360_x4_template.pto /path/to/image/folder` to process an entire folder of captures. The script generates minimal panorama project files, applies the Insta360 X4 stitching template shown above, executes the stitch, and cleans up temporary artifacts—all automatically for each image pair.

*(Output: a stitched, equirectangular `.jpg`/`.tif` panorama per input `.insp` pair, named after the original capture.)*

## Phase 6: Timestamp Verification

After processing the images, verify that they retain correct EXIF timestamps that align with the scanner's recorded trajectory. Use ExifTool to examine the timestamp metadata:

```bash
exiftool ./path/to/image.* | grep Date
```

Compare these timestamps against the scanner's database to ensure proper temporal alignment. Any significant discrepancies here indicate synchronization problems that occurred during capture and may require manual correction or recapture of affected areas.

## Phase 7: Database Quality Control

Before proceeding with 3D reconstruction, carefully inspect the merged database to ensure loop closures have been correctly identified and the assembled point cloud exhibits accurate geometry. Open the RTABMap database viewer with `rtabmap-databaseViewer ./path/to/merged.db` and methodically review the assembled point cloud for inconsistencies or misalignments.

Pay particular attention to loop closure connections—the associations RTABMap makes when the scanner revisits previously mapped areas. Incorrect loop closures manifest as sudden spatial shifts, duplicated geometry, or warped structures. When you identify problematic closures, delete them within the viewer interface. Similarly, if you notice areas that should be connected but aren't, manually add correct loop closures to improve the reconstruction. This quality control step is crucial, as errors here propagate through all subsequent processing stages.

*(There is no screenshot for this step — the RTABMap Database Viewer is a third-party GUI application; consult its own documentation for what the loop-closure and point cloud review screens look like.)*

---

The remaining phases run entirely through the `rux` CLI against a single ReUseX project database (a `.rux` file). Every command below either takes an explicit `-p,--project <path>` global flag or falls back to `./project.rux` in the current directory — pick one project path and use it consistently for the whole session. The stage order and the data each stage reads/writes are defined in [`docs/CONTRACTS.md`](../CONTRACTS.md); the phases below follow that order:

```
import → (optimize | register) → create clouds → create annotate (+ create project)
       → create planes → create rooms → create instances → create mesh
```

## Phase 8: Import

Import extracts the raw sensor data (RGB images, depth maps, camera poses, camera intrinsics) from the RTABMap database into a new ReUseX project. No point cloud reconstruction happens at this stage — it only copies sensor frames.

```bash
rux -p building.rux import rtabmap ./path/to/merged.db
```

*Output: a `building.rux` project file containing one `sensor_frames` row per scanned frame (color, depth, confidence, pose, intrinsics), and nothing else yet.*

## Phase 9: Pose Optimization / Registration (optional)

RTABMap's own SLAM poses are usually good enough to reconstruct from directly, but if you see residual drift or misalignment in the merged cloud you can refine the stored per-frame poses in place before reconstructing geometry. Two independent refiners are available — use one, not both:

```bash
# Joint pairwise registration: minimizes point-to-plane residuals between
# overlapping frames, anchored back to the original RTABMap poses.
rux -p building.rux register --dry-run   # preview the residual first
rux -p building.rux register

# Plane-landmark pose graph: associates dominant planes across frames into
# shared landmarks and solves a global factor graph (better for drift that
# pairwise registration alone can't correct).
rux -p building.rux optimize --dry-run
rux -p building.rux optimize
```

Both commands overwrite the stored sensor poses in place (re-import the RTABMap database to recover the originals). Skip this phase if the scan already tracks cleanly.

*Output: updated poses in the existing `sensor_frames` rows — no new clouds yet.*

## Phase 10: Point Cloud Generation

`rux create clouds` back-projects each frame's depth image into 3D using its camera intrinsics and pose, merges all frames into one cloud, and applies depth filtering, per-pixel subsampling, and voxel-grid downsampling.

```bash
rux -p building.rux create clouds -g 0.02
```

*Output: `cloud` (`PointXYZRGB`) and `normals` (surface normals) point clouds saved to the project, index-aligned point for point.*

## Phase 11: Semantic Annotation

With geometry reconstructed, run ML inference on the stored RGB frames to label architectural elements, objects, and surfaces, then project those 2D labels onto the 3D cloud. This is two commands: `create annotate` runs the model over each sensor frame and stores per-frame segmentation images; `create project` back-projects those per-frame labels onto the existing `cloud` using the same camera poses/intrinsics used for reconstruction.

```bash
rux -p building.rux create annotate --net ./path/to/sam3.1_b.engine --cuda
rux -p building.rux create project
```

The annotation backend (TensorRT `.engine`, PyTorch `.pt`, or ONNX) is auto-detected from the model file's extension. This phase is orthogonal to the plane/room geometry pipeline below — plane and room segmentation only need `cloud`/`normals` — but it must run before Phase 14 (Instance Segmentation), which consumes the `labels` cloud this phase produces.

*Output: a `labels` point cloud (per-point semantic class, aligned with `cloud`/`normals`), plus the underlying per-frame `segmentation_images`.*

## Phase 12: Plane Segmentation

The plane segmentation phase extracts and identifies planar surfaces within the point cloud, forming the primary structural geometry (walls, floors, ceilings) that room segmentation and mesh generation build on.

```bash
rux -p building.rux create planes
```

The underlying algorithm is a noise-adaptive, multi-scale region-growing pass over `cloud`/`normals`: it estimates the cloud's local noise level and derives the plane-distance threshold (~3σ) and minimum cluster size (by point density) from that, unless you pin them explicitly with `-d`/`-m`. Region growing itself groups points by normal similarity (`-a, --angle-threshold`) and proximity (`-r, --radius`). Output is a `planes` label cloud plus one centroid and one normal per detected plane.

```bash
rux -p building.rux create planes -a 15 -d 0.05   # tighter angle/distance
```

*Output: `planes` (per-point plane label), `plane_centroids`, `plane_normals`.*

## Phase 13: Room Segmentation

With planes identified, ReUseX partitions the point cloud into individual rooms using the Leiden community-detection algorithm (via `igraph`) over a graph built from spatial and visual relationships between the detected planes.

```bash
rux -p building.rux create rooms
```

Raising the Leiden `-r, --resolution` parameter yields more, smaller rooms; lowering it merges rooms together. The mesh stage uses this room segmentation to decide which cells are "inside" the building, and it backs room-level architectural analysis; per-room mesh *output* is future work (WP9).

```bash
rux -p building.rux create rooms -r 1.5
```

*Output: a `rooms` label cloud, aligned with `cloud`/`planes`.*

## Phase 14: Instance Segmentation

Instance segmentation separates the semantic `labels` cloud from Phase 11 into distinct spatial instances via Euclidean clustering — for example, multiple points labeled "window" become separate `window` instances based on spatial separation, each with a stable GUID that survives re-running this stage.

```bash
rux -p building.rux create instances
rux -p building.rux create instances -t 0.3   # 30cm clustering tolerance
```

*Output: an `instances` label cloud plus an `instances` table (per-instance id, GUID, semantic class, point count).*

## Phase 15: Mesh Generation

The mesh generation phase transforms the segmented point cloud into a simplified surface mesh. It builds a cell complex from the detected planes, then solves a mixed-integer program (HiGHS by default, cuOpt optionally on GPU) to pick the best-fit set of cells, using the `rooms` labels only to weight which cells count as "inside" the building during that solve.

```bash
rux -p building.rux create mesh
rux -p building.rux create mesh -f 'rooms == 2'   # mesh only room 2
```

Large, multi-room buildings are solved section-by-section (per storey) by default to keep the MIP tractable; pass `!--no-sectioned` to force a single monolithic solve.

*Output: one watertight envelope mesh covering every selected cell, saved to the project's `meshes` table (default name `mesh`) — **not** split back into per-room meshes; the solver's per-cell room/wall labels are discarded. Per-room mesh output is tracked under WP9 (`reusex-z1v.11.2`).*

## Export and Visualization

Throughout the workflow you can visualize intermediate results with `rux -p building.rux view`, which opens an interactive PCL-based viewer for the point clouds, meshes, and label overlays currently stored in the project. When processing is complete, export the final model with `rux -p building.rux export <format>`. The formats actually implemented are:

- `rux export ply` — point cloud to binary PLY
- `rux export e57` — point cloud to E57 (point cloud exchange format)
- `rux export rhino` — full project (clouds, semantic layers, meshes, panoramas, material passports) to Rhino `.3dm` (OpenNURBS)
- `rux export speckle` — point cloud or mesh to a Speckle project (web platform)
- `rux export materialepas` — material passports to JSON
- `rux export csv` — building components and material passports to CSV
- `rux export semantic-images` — segmentation label images as Glasbey-colored PNGs
- `rux export colmap` — sensor frames as a COLMAP sparse model (for external MVS / Gaussian-splatting pipelines)

There is no PCD or HDF5 exporter, and no built-in IFC exporter as of this writing; run `rux export <subcommand> --help` for the authoritative, per-format option list.

## Conclusion

This workflow demonstrates ReUseX's complete pipeline for transforming raw SLAM scans and 360° imagery into semantically-rich, simplified 3D surface models optimized for building reuse and renovation projects. The process seamlessly integrates SLAM-based spatial mapping through RTABMap, deep learning-based semantic segmentation via SAM3 with TensorRT acceleration, advanced geometric processing leveraging CGAL and PCL, Leiden community-detection-based room segmentation (via `igraph`), and MIP-optimized mesh generation with architectural awareness.

The resulting models support diverse applications: building reuse projects benefit from accurate documentation of existing conditions for renovation planning; architects gain precise as-built records; space planning becomes data-driven with accurate measurements and spatial analysis; digital twin applications maintain synchronized representations of physical spaces; heritage preservation efforts document historical buildings with unprecedented detail; and construction teams track progress by comparing as-built versus as-planned geometry.

### Performance Considerations

Processing performance varies significantly based on hardware configuration. GPU acceleration with TensorRT provides dramatic speedups for semantic annotation—the optimized SAM3 implementation delivers significantly faster inference compared to previous YOLO-based approaches or non-optimized implementations. A CUDA-capable GPU is strongly recommended for the annotation phase. Consider batch processing when handling multiple scans, processing them in parallel to maximize hardware utilization. Save intermediate results after each phase to enable incremental workflows and avoid reprocessing if adjustments are needed. Finally, use SSD storage rather than traditional hard drives for faster data access during processing, particularly during the annotation and import phases which involve substantial random I/O.

### Troubleshooting

When encountering poor loop closures, return to the `rtabmap-databaseViewer` to review and correct the associations. Delete false positives that cause spatial distortions and manually add correct closures where the scanner obviously revisited the same location but failed to recognize it. Ensure sufficient feature overlap between scan passes—walking too quickly or with insufficient lighting can prevent reliable loop closure detection.

Segmentation issues often trace back to earlier stages. Check semantic annotation quality by reviewing the labeled images—poor lighting, motion blur, or unusual architectural elements can confuse the neural networks. Verify plane segmentation parameters match your environment's scale and characteristics (or let `--adaptive`, the default, derive them from the cloud's measured noise), and review point cloud density and coverage to ensure sufficient data for robust segmentation.

Mesh artifacts typically result from ambiguous room boundaries or outliers in the source data. Inspect room segmentation boundaries to identify areas where the algorithm struggled to determine spatial partitioning. Adjust plane fitting tolerance if minor surface variations are being over-segmented, and check for outlier points that should have been filtered during earlier processing stages. If `rux create mesh` reports an empty mesh, see its `--filter`/`--threshold` guidance in `rux create mesh --help` before re-running `create planes`/`create rooms`.

---

*For the authoritative stage-by-stage data contract, see [`docs/CONTRACTS.md`](../CONTRACTS.md). For detailed API documentation, refer to the generated Doxygen documentation. For questions or issues, submit an issue to the [ReUseX repository](https://github.com/pfmephisto/ReUseX).*
