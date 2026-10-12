<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX Architecture Overview

A map of the repository. This document is deliberately thin: the **normative**
detail lives elsewhere and is kept current there.

| Question | Authoritative document |
|---|---|
| What may depend on what? Layers, header hygiene, label contract, Definition of Done | [`docs/STANDARDS.md`](docs/STANDARDS.md) |
| What does each pipeline stage read and write in a `.rux` project? | [`docs/CONTRACTS.md`](docs/CONTRACTS.md) |
| Naming conventions, build options, CLI surface, TODO format | [`CLAUDE.md`](CLAUDE.md) |
| Per-symbol API reference | Doxygen: `cmake --build build --target docs` → `docs/api/html/index.html` |

## Repository Structure

```
ReUseX/
├── libs/reusex/                # The library
│   ├── include/                # Public headers, consumed as <reusex/...>
│   │   ├── core/               # ProjectDB, logging, stages, materials, validate
│   │   ├── geometry/           # geometry_common + forwarding shims from #222
│   │   ├── segmentation/       # planes, rooms, instances, reconstruct, filters
│   │   ├── reconstruction/     # CellComplex, Solidifier, mesh, texture, metrics
│   │   ├── slam/               # PlaneGraphOptimizer, JointPairwiseRegistration,
│   │   │                       #   PanoramaAlignment, loop closure
│   │   ├── io/                 # rtabmap, e57, ply, rhino, colmap, speckle, ...
│   │   ├── vision/             # ML models/backends/datasets (tensor_rt, onnx, ...)
│   │   ├── visualize/          # Optional PCL/Qt visualization
│   │   ├── utils/              # math, cv, tolerances, formatters
│   │   ├── types/              # point_types.hpp, eigen_types.hpp
│   │   └── types.hpp           # Umbrella over types/
│   ├── src/                    # Implementation, mirrors include/
│   ├── cmake/                  # reusexLibrary.cmake, Dependencies.cmake, ...
│   └── extern/                 # Vendored headers
├── apps/
│   ├── rux/                    # CLI tool (include/ + src/, grouped by command),
│   │                           #   plus the native Qt client (apps/rux/qt)
│   └── blender/reusex_panel/   # Blender add-on (standalone)
├── bindings/python/            # pybind11 bindings (read-only ProjectDB access)
├── python/                     # reusex_sam3: SAM 3.1 -> ONNX -> TensorRT export
│                               #   (standalone, not built by CMake)
├── models/                     # Model weights (gitignored; see models/README.md)
├── tests/                      # unit/ integration/ benchmarks/ support/ fixtures/
├── docs/                       # STANDARDS, CONTRACTS, guides/, design/, research/, api/
├── cmake/                      # Shared CMake utilities
├── overlays/ pkgs/ devshells/  # Nix packaging
└── tools/ scripts/ completions/
```

**Repo split (2026-10-09):** `ruxd` (the HTTP service worker / web GUI
server) and the web frontend each moved into their own repo — `ruxd` and
`rux-frontend` under the `ReUse-X` org. `ruxd` consumes this repo through its
installed CMake package (`find_package(ReUseX CONFIG REQUIRED)`). See
`docs/DIRECTION.md` for the changelog entry. A fourth repo, `reusex-fleet`,
holds the deployment (Docker Compose, Supabase auth, Caddy); deployment
questions belong there, not in this repo.

## Modules and layering

`libs/reusex/` builds **one static library per module** — `reusex_<module>` —
declared in `libs/reusex/cmake/reusexLibrary.cmake`. The layer graph is
**link-enforced** (#222): an illegal dependency is a link error, not a
convention.

```
Layer 4:  visualize                                    (optional, PCL/Qt/VTK)
Layer 3:  segmentation  reconstruction  slam  io  vision  (peers — MUST NOT link each other)
Layer 2:  core                                         (ProjectDB, logging, materials, stages)
Layer 1½: geometry_common                              (shared CGAL/PCL primitives)
Layer 1:  utils, types.hpp                             (no internal dependencies)
External: apps/rux, and ruxd (own repo, via find_package) (may use everything; keep logic thin)
```

See [`STANDARDS.md` §1](docs/STANDARDS.md#1-module-boundaries) for the rules and
the two documented cross-peer exceptions (`io -> reconstruction`,
`slam -> segmentation`) plus the tracked `core -> geometry_common` upward edge.

### Targets

| Target | Kind | Notes |
|---|---|---|
| `reusex_common` | INTERFACE | Public/external dependencies shared by all modules |
| `reusex_private_deps` | INTERFACE | Dependencies kept out of the public interface |
| `reusex_utils`, `reusex_geometry_common`, `reusex_core`, `reusex_io`, `reusex_vision`, `reusex_segmentation`, `reusex_reconstruction`, `reusex_slam` | STATIC | One per module |
| `reusex_visualize` | STATIC | Only when `src/visualize/` has sources |
| `reusex` | INTERFACE | Umbrella linking every module, for backward compatibility |
| `rux` | executable | `apps/rux` (`ruxd` is a separate executable in its own repo) |

The historical `ReUseX` and `ReUseX_visualization` target names no longer exist.

## Data flow

`ProjectDB` (`libs/reusex/include/core/ProjectDB.hpp`) is the single project
store — one sqlite3 file per project, conventionally `*.rux`. It replaced the
retired `RTABMapDatabase`; RTABMap is now only an *import* source in
`libs/reusex/src/io/rtabmap.cpp`.

```
external scan (RTABMap .db, MuSHRoom, ARKitScenes, E57/PLY, panoramas, photos)
        │  rux import …
        ▼
    ProjectDB  (sensor_frames, point_clouds, meshes, segmentation_images,
        │       building_components, material_passports, pipeline_log, …)
        ├──→ rux optimize / register        pose refinement, writes poses back
        ├──→ rux align 360                  360 panorama pose refinement
        ├──→ rux create clouds              → cloud, normals
        ├──→ rux create planes              → planes, plane_centroids, plane_normals
        ├──→ rux create rooms               → rooms
        ├──→ rux create annotate → project  → segmentation_images → labels
        ├──→ rux create annotate-360        SAM3 on panoramas (perspective-tiled)
        ├──→ rux create instances           → instances (+ instances table)
        ├──→ rux create mesh                → meshes
        └──→ rux export …                   PLY, E57, Rhino, COLMAP, Speckle, CSV
```

Vision datasets (`IDataset` and its implementations `TensorRTDataset`,
`LibTorchDataset`, `ONNXSam3Dataset`) read
frames from `ProjectDB` rather than owning storage.

The exact named clouds/tables per stage, and the checks behind
`rux validate --stage <name>`, are in [`docs/CONTRACTS.md`](docs/CONTRACTS.md).

## Key design patterns

**Subproject isolation** — each component has its own `CMakeLists.txt`; the
dependency direction is `apps`/`bindings` → `libs`.
See `docs/design/subproject-structure.md`.

**Pimpl** — `ProjectDB` and `Solidifier` hide sqlite3 / CGAL / solver headers
behind an opaque implementation pointer, so public headers stay light
(STANDARDS §2).

**Options structs** — every tunable parameter is defined once in a library
options struct (`SegmentPlanesOptions`, `SegmentRoomsOptions`,
`ReconstructionParams`, `SolidifierOptions`, `PlaneGraphOptions`, `JprParams`);
CLI flags mirror those
defaults rather than redefining them (STANDARDS §4).

**Named geometric tolerances** — `utils/tolerances.hpp` instead of ad-hoc
epsilon literals (STANDARDS §4).

## Build System

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Options (defaults in parentheses): `WITH_CUDA` (ON), `USE_CCACHE` (ON),
`BUILD_TESTS` (ON), `BUILD_DOCUMENTATION` (ON), `BUILD_PYTHON_BINDINGS` (ON),
`GUI_ENABLED` (OFF), `ENABLE_COVERAGE` (OFF), `ML_BACKENDS` (`AUTO`), and the
`LIN_ENABLE_*` sanitizer switches. See the table in [`CLAUDE.md`](CLAUDE.md) for
where each is defined. `rux` always links PCL visualization unconditionally —
there is no option to disable it.

## Version Management

The version is defined **once**, in the root `CMakeLists.txt`:

```cmake
project(reusex VERSION 0.0.5 ...)
```

- **C++**: `libs/reusex/include/core/version.hpp.in` is configured with
  `@PROJECT_VERSION@`; read it as `reusex::core::VERSION` (also
  `reusex::core::LICENSE_TEXT`). `rux --version` / `rux --license` print them.
- **Doxygen**: `PROJECT_NUMBER` in `docs/Doxyfile`.
- **Python**: `bindings/python/_version.py.in` is configured into the built package as `reusex/_version.py`, exposed as
  `__version__`; `pyproject.toml` uses `dynamic = ["version"]`.

## Conventions

**Namespaces** — root namespace is lowercase `reusex` (`reusex::core`,
`reusex::segmentation`, `reusex::vision::tensor_rt`). Keep nesting to three
levels or fewer; internal helpers go in a `detail` namespace.

**Labels** — storage `CV_16U` with a +1 offset (`0` = unlabeled), `ProjectDB`
API `CV_32S` with `-1`, in-memory `CloudL` uses `0` for unlabeled and valid
labels from `1`. Helpers in `core/label_semantics.hpp`. The full table is
[`STANDARDS.md` §3](docs/STANDARDS.md#3-label--identity-contract).

**Image orientation** — images, depth maps and label images are stored in their
original sensor orientation. The 90°-clockwise rotation applied by the old
RTABMap reader is gone; do not reintroduce it.

**Parallel clouds** — `cloud`, `normals`, `planes`, `rooms`, `instances`,
`labels` for one scan are index-aligned and must be filtered, downsampled or
reordered together (STANDARDS §3.2).

## Documentation

- **Engineering standards**: [`docs/STANDARDS.md`](docs/STANDARDS.md)
- **Pipeline stage contracts**: [`docs/CONTRACTS.md`](docs/CONTRACTS.md)
- **Docs index**: [`docs/README.md`](docs/README.md)
- **User guides**: [`docs/guides/`](docs/guides/)
- **Design notes** (historical in places): [`docs/design/`](docs/design/)
- **Research / benchmark notes**: [`docs/research/`](docs/research/)
- **SAM 3.1 export pipeline**: [`python/README.md`](python/README.md) and
  [`docs/sam3.1-tensorrt.md`](docs/sam3.1-tensorrt.md)
- **API reference**: `docs/api/` after `cmake --build build --target docs`
- **Contributing**: [`CONTRIBUTING.md`](CONTRIBUTING.md),
  [`CONTRIBUTING_AI.md`](CONTRIBUTING_AI.md)
- **Testing**: [`docs/guides/TESTING.md`](docs/guides/TESTING.md),
  [`tests/README.md`](tests/README.md)

## Questions?

- **Build issues**: [`docs/design/subproject-structure.md`](docs/design/subproject-structure.md)
- **Project database**: `libs/reusex/include/core/ProjectDB.hpp` (the design note
  in [`docs/design/database-design.md`](docs/design/database-design.md) documents
  the retired predecessor)
- **Report issues**: https://github.com/pfmephisto/ReUseX/issues
