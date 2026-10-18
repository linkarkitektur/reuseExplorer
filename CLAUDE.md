# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Direction

Project heading, priorities, and active workstreams live in
[`docs/DIRECTION.md`](docs/DIRECTION.md). Read it before planning, reviewing, or
filing issues. When work changes the project's direction, update it in the same
PR with a dated changelog entry.

## Agent Model Selection

The main session model is for **orchestration, guidance, review, and
synthesis only** — delegate substantial work to subagents, and always set
the `model:` parameter explicitly (never let an agent inherit the expensive
session default):

- **`model: "sonnet"`** — simple/mechanical execution: downloads, running
  scripts/benchmarks/test suites, file conversions, data collection,
  formatting fixes, report extraction.
- **`model: "opus"`** — reasoning-heavy work: debugging, root-cause
  analysis, design, code review, non-trivial implementation.

Pick by task complexity; when unsure whether a task needs judgment,
prefer sonnet for gathering and opus for deciding.

## Normative Documents

This file is the orientation/guidance layer. The **normative** rules live in:

- [`docs/STANDARDS.md`](docs/STANDARDS.md) — module boundaries (§1), header
  hygiene (§2), label & identity contract (§3), parameters (§4), error handling
  (§5), determinism (§6), testing (§7), performance (§8), Definition of Done (§9).
- [`docs/CONTRACTS.md`](docs/CONTRACTS.md) — what each pipeline stage consumes
  and produces in a `.rux` project database, enforced by `rux validate --stage`.

If this file and those two disagree, **they win** — and the disagreement is a bug
worth fixing here.

## Documentation Scope

`docs/` holds durable project documentation only: standards, contracts,
architecture, guides, research write-ups. It is **not** a home for
development specs or implementation plans — those belong in beads (an
epic's `--design`/`--notes`, a task's `--acceptance`), not in a committed
file that drifts from the issue tracker once the work ships.

This means `superpowers:brainstorming` and `superpowers:writing-plans`
must **not** use their default save paths (`docs/superpowers/specs/...`,
`docs/superpowers/plans/...`) in this repo — both skills honor a user
location override. Save specs and plans under the gitignored
`.superpowers/specs/` and `.superpowers/plans/` instead (sibling to the
already-gitignored `.superpowers/sdd/`), and carry anything durable —
architecture decisions, the step-level task breakdown — into the matching
beads issue before the plan's workspace is deleted.

## Project Overview

ReUseX is a C++20/CUDA project for processing 3D point cloud scans of building interiors. It combines geometric processing, deep learning, and computational geometry to create semantic 3D models for building reuse and renovation projects.

**Key capabilities:**
- Sensor-frame import from RTABMap SLAM databases, MuSHRoom and ARKitScenes
  captures, E57/PLY clouds, 360° panoramas and survey photos
- Pose refinement: plane-landmark pose-graph optimization (GTSAM) and joint
  pairwise registration
- Planar segmentation (noise-adaptive region growing) and room segmentation
  (2D band watershed per storey, default; or Leiden clustering over a plane
  graph, via igraph)
- Semantic segmentation via YOLO / SAM3 models (TensorRT, ONNX Runtime, LibTorch),
  including SAM3 on 360° panoramas via perspective tiling
- Cell complex 3D reconstruction with a MIP solve (HiGHS CPU / cuOpt GPU)
- Mesh generation with texture mapping; dense MVS clouds via OpenMVS
- Export to PLY, E57, OpenNURBS (.3dm), COLMAP, Speckle, CSV, MaterialEPAS

## Naming Conventions

- **Functions/methods**: snake_case — `save_point_cloud()`, `is_open()`
- **Getters**: No `get_` prefix — `path()`, `schema_version()`, `mesh()`
- **Boolean queries**: `is_`/`has_` prefix — `is_open()`, `has_mesh()`
- **Setters**: `set_` prefix — `set_log_level()`, `set_database_path()`
- **Classes/structs**: PascalCase — `CellComplex`, `ProjectDB`, `DetectionBox`
- **Enum classes**: PascalCase name, snake_case values — `Stage::mesh_generation`
- **Namespaces**: snake_case, root namespace is `reusex` (lowercase — `ReUseX::`
  appears nowhere in the source) — `reusex::core`, `reusex::segmentation`,
  `reusex::vision::tensor_rt`
- **Type aliases**: PascalCase — `CloudPtr`, `CloudNConstPtr`
- **Member variables**: snake_case with trailing `_` — `impl_`, `cloud_`
- **File naming**: PascalCase for class files, snake_case for function/algorithm files
- **Header guards**: `#pragma once`

## Build System

### Development Environment

The project uses **Nix flakes** for reproducible builds. Always enter the dev shell first:

```bash
nix develop
```

### Building

```bash
# Standard build (Release mode, all features)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Debug build
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# CPU-only build (no CUDA, no TensorRT)
cmake -B build -DWITH_CUDA=OFF
cmake --build build

# Build without tests
cmake -B build -DBUILD_TESTS=OFF
cmake --build build
```

**Build options that actually exist** (grep `option(` in `CMakeLists.txt`,
`cmake/`, `libs/reusex/cmake/`):

| Option | Default | Defined in | Effect |
|---|---|---|---|
| `WITH_CUDA` | `ON` | `CMakeLists.txt:38` | CUDA / NVIDIA GPU support. Also gates the TensorRT backend search and `cuOpt`. |
| `USE_CCACHE` | `ON` | `CMakeLists.txt:79` | Use ccache when available |
| `ENABLE_COVERAGE` | `OFF` | `cmake/Coverage.cmake:30` | Code coverage instrumentation |
| `BUILD_PYTHON_BINDINGS` | `ON` | `CMakeLists.txt:134` | Adds `bindings/python` |
| `BUILD_TESTS` | `ON` | `CMakeLists.txt:142` | Adds `tests/` and `enable_testing()` |
| `BUILD_DOCUMENTATION` | `ON` | `CMakeLists.txt:151`, `cmake/Documentation.cmake:9` | Defines the `docs` target |
| `BUILD_QT_CLIENT` | `ON` | `CMakeLists.txt:128` | Native Qt client `apps/rux/qt` (`rux_qt_core`, `rux_qt_lib`, `rux-qt-gallery`) and `tests/unit/rux_qt`; needs Qt6 OpenGLWidgets + VTK GUISupportQt |
| `GUI_ENABLED` | `OFF` | `libs/reusex/cmake/Dependencies.cmake:207` | CGAL Qt6 GUI components |
| `ML_BACKENDS` | `AUTO` | `libs/reusex/cmake/Dependencies.cmake:49` | Cache string, not a bool: `AUTO`, `NONE`, or a list like `TensorRT;LibTorch;ONNX;OpenVINO` |
| `LIN_ENABLE_ASAN` / `MSAN` / `UBSAN` / `TSAN` | `OFF` | `libs/reusex/cmake/CompilerOptions.cmake` | Sanitizers |
| `LIN_ENABLE_WERROR` | `OFF` | `libs/reusex/cmake/CompilerOptions.cmake:23` | `-Werror` |
| `REUSEX_PACKAGE_TEST` | `ON` | `tests/CMakeLists.txt:346` | Registers the `reusex_package_consumer` ctest (label `package`). `default.nix` turns it off and runs the same check in `installCheckPhase` instead. |

There is no option to make visualization optional: `rux` links
`pcl_visualization` unconditionally (`apps/rux/CMakeLists.txt`). Separately,
the `visualize` library module is built whenever
`libs/reusex/src/visualize/**` has sources (see
`libs/reusex/cmake/reusexLibrary.cmake`), and the umbrella `reusex` target picks
it up only `if(TARGET reusex_visualize)`.

**CMake auto-detection:** The build system uses `GLOB_RECURSE` with `CONFIGURE_DEPENDS`, so new .cpp/.cu files are automatically detected. No need to manually update CMakeLists.txt when adding source files.

### Installing / the ReUseX CMake package

`cmake --install build` installs the library as a CMake package
(`cmake/Installation.cmake`, `cmake/ReUseXConfig.cmake.in`), laid out like this:

- `include/reusex/...` holds the public headers, so `<reusex/...>` works unchanged.
- `include/reusex/extern/` holds the vendored CGAL/PCL extension headers.
- `lib/libreusex_<module>.a` holds the static libraries.
- `lib/cmake/ReUseX/` holds the package config.

Outside consumers (ruxd after the repo split) use
`find_package(ReUseX CONFIG REQUIRED)` and link `ReUseX::<module>`
(`ReUseX::core`, `ReUseX::pipeline`, …, umbrella `ReUseX::reusex`). The same
`ReUseX::` names are ALIAS targets in-tree. Everything the package needs is in
the install component `ReUseX_Development`:

```bash
cmake --install build --prefix /tmp/rx --component ReUseX_Development
```

Rules when touching the build:

- The modules are **static**, so every PRIVATE `target_link_libraries`
  dependency is exported as `$<LINK_ONLY:...>` and must resolve in the
  consumer. A new `find_package` in `libs/reusex/cmake/Dependencies.cmake`
  whose targets end up on a module's link line needs a matching
  `find_dependency` in `cmake/ReUseXConfig.cmake.in`.
- A third-party header included from a **public** header must be a PUBLIC
  dependency (range-v3 lives on `reusex_common` for that reason).
  `LINK_ONLY` carries no include directories.
- Public headers include siblings as `<reusex/...>`, never by the flat
  `core/...` path, which only resolves inside the build tree.
- Compile options on `reusex_common` are `$<BUILD_INTERFACE:...>`, so a
  consumer does not inherit our warnings or `-Werror`.

`ctest -L package` (`reusex_package_consumer`) runs `cmake --install` into
a temp prefix, then builds and runs `tests/package/consumer` against it.
That takes about 2.5 min, almost all of it the consumer's dependency lookups.
`nix build .#default` installs the same package into `$out`. The nix
`checks.tests` build runs the consumer against `$out` in `installCheckPhase`
(`doInstallCheck`).

### Testing

```bash
# Build and run all tests
cmake -B build -DBUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure --parallel $(nproc)

# Run with verbose output
ctest --verbose

# Run specific test
ctest -R test_name_pattern
```

**Always pass `--parallel`.** Nearly the entire suite's wall time is
per-process dynamic-loader overhead, not test work, so it scales almost
linearly with cores: serial is ~12 min, `-j8` about 1/6 of that.
`scripts/check.sh` already does this. Parallel ctest used to be flaky
because temp-file helpers derived names from object addresses; that was
fixed in #262 by `tests/support/temp_path.hpp`, which every test must use
for temp paths.

Tests live in `tests/`: `unit/` (per-module: `core`, `geometry`, `io`,
`utils`, `vision`, `visualize`, plus `pipeline`, `reconstruction`,
`segmentation`, `slam`; `rux` (`rux_core_lib`'s app logic — path parsing, filter
validation, stage prerequisites — short of a subcommand or the viewer),
`rux_app` (CLI subcommands, needs `rux_lib`; incl. "Kopiér som
rux-kommando"'s CLI round-trip) and `rux_qt` (the native Qt client's Qt-free
logic, `rux_qt_core`)), `integration/`, `benchmarks/`, `support/`,
`fixtures/`. Catch2 v3. `ruxd`'s own tests (formerly `unit/ruxd{,_api,_cli,_pg}`)
moved with it to the standalone `ruxd` repo (repo split, P4).

Unit tests link into **two** executables (`tests/CMakeLists.txt`):
`reusex_unit_tests` for most modules, and `reusex_unit_tests_vision` for
`unit/vision`, `unit/rux_app`, `unit/visualize` and `unit/gsplat/cuda`, which
need libtorch / TensorRT / `rux_lib` / the PCL-Qt viewer. `reusex_unit_tests`
(the light binary) links `rux_qt_core`
unconditionally whenever `BUILD_QT_CLIENT` builds the Qt client at all
(`apps/rux/qt/CMakeLists.txt`), so `unit/rux_qt` always runs there and costs
nothing extra on a build without Qt. Test names and `ctest -R` are unaffected
by the split. Put a new test in a heavy directory only if it really needs
those dependencies (#268).

`unit/gsplat/` splits inside the directory (#332): `unit/gsplat/cuda/` needs
torch or the rasterizer and is dropped from a build without
`REUSEX_HAVE_GSPLAT`, while tests directly under `unit/gsplat/` cover
`reusex_gsplat_common` and run everywhere — including the CPU-only variant CI
builds. Tests that need a CUDA *device* are tagged `[gpu]` and open with
`if (!gsplat::has_cuda_device()) SKIP(...)`, so a GPU-less machine reports
them as skipped rather than failed.

Benchmarks: `scripts/bench.sh` produces an XML report and
`scripts/bench-compare.py` diffs a baseline against a candidate, failing on a
regression beyond `--threshold` percent. The required before/after workflow is
[`docs/STANDARDS.md` §8.1](docs/STANDARDS.md#81-baseline-vs-candidate-workflow);
`scripts/bench-arkitscenes.sh` and `scripts/bench-mushroom.sh` drive
dataset-level runs. The solid-model benchmark is its own pair:
`scripts/bench-solid.sh` runs `rux analyze solid` over the rows of
`scripts/bench-solid.manifest` into one JSON report, and
`scripts/bench-solid-compare.py` gates two such reports per metric
([`docs/design/solid-model-json.md`](docs/design/solid-model-json.md)).

### Continuous integration

Two workflows, split by whether they need the nix closure:

| Workflow | Triggers | Jobs |
|---|---|---|
| `.github/workflows/lint.yml` | push (main) + PR | clang-format, REUSE/SPDX |
| `.github/workflows/ci.yml` | push (main) + PR + `workflow_dispatch` | `nix build .#ci` (CPU variant: package + ctest + install check), substitute-first: compiles only what nobody pushed |

**Build locally, let CI confirm** (reusex-37v, reusex-uk8). What CI realises
is one flake output, `.#ci`, and you build exactly that — no script. Build
and test are **two derivations**: `checks.cpu` (= `packages.cpu`) compiles
ReUseX once into `out` (the installed package ruxd builds against) and
`testsuite` (test binaries + ctest files); `checks.tests` (= `cpu.check`,
`default.nix`) only runs ctest and the package-consumer install check against
them and outputs a pass stamp with no references. `.#ci` links all three.
Before pushing a branch, on a clean, committed tree, from the dev shell
(it has `cachix`):

```bash
nix build .#ci --print-out-paths | cachix push reusex
```

CI checks out the **PR head commit** (not GitHub's merge commit), evaluates
the same derivations, logs them and nix's `--dry-run` plan, and tries
`nix build .#ci --max-jobs 0` (substitute only). On a miss it builds only
what is missing (a cached `.#cpu` means tests only, ~5 min; otherwise the full
compile, ~25-45 min) and a `::notice` says which. On `main` it also proves
that `github:…#cpu` (what downstream repos fetch) is the same derivation as
`.#cpu`. The derivations match locally and in CI because the sources are
`lib.fileset`-narrowed in `default.nix`: the **build** source holds only what
the compile reads (no `tests/fixtures`, so a GitHub tarball's LFS pointers
cannot change it), the **test** source adds what tests read at run time
(`tests/fixtures`, `docs/CONTRACTS.md`, …). Edits to `docs/`, `.github/`,
`CLAUDE.md`, most of `scripts/` change nothing; a fixture or `CONTRACTS.md`
edit reruns only the tests. A file a test reads must be in the test fileset
(it fails only under nix otherwise), and test binaries reach it through
`source_dir()` / `rux_executable()` (`tests/support/`), whose compiled-in
paths `checks.tests` overrides with `REUSEX_TEST_SOURCE_DIR` /
`REUSEX_TEST_RUX_EXECUTABLE`. The local nix build reuses `/var/cache/ccache`
inside the sandbox (`overlays/ccache.nix`), so a rebuild after a small change
is mostly linking plus the ~6 min test run. Pushing needs a Cachix write token
(`cachix authtoken <token>` once, or `CACHIX_AUTH_TOKEN`); in CI the token
is an **environment** secret on the `CI` environment (hence `environment:
CI`), and CI itself pushes only from `main`.

The build steps use `shell: bash` (pipefail): GitHub's default `bash -e`
let `nix build | tee` exit 0 on test failures until reusex-37v.

**ccache in the dev shell** is shared by the main checkout and every
`.worktrees/*`: `shell.nix` sets `CCACHE_BASEDIR` (the directory holding the
shared `.git`), `CCACHE_NOHASHDIR=1`, `CCACHE_COMPILERCHECK=content` and
`CCACHE_MAXSIZE=50G`. Keep absolute paths out of target-wide compile
definitions: ccache hashes the command line, so a `-DFOO="/abs/worktree/path"`
on a target makes every TU of it miss in every other worktree. Put such a
definition on the one TU that reads it (see `tests/support/source_dir.hpp`).

Full details — the flow, the derivation-hash proof, cache contents, size
budget and the 5 GB free-tier constraint:
[`docs/guides/ci-cache.md`](docs/guides/ci-cache.md).

### Documentation

The Doxygen target is named `docs` (not `doc`) and writes to `docs/api`
(`OUTPUT_DIRECTORY` in `docs/Doxyfile`).

```bash
# Generate Doxygen documentation
cmake -B build -DBUILD_DOCUMENTATION=ON
cmake --build build --target docs

# View generated docs
xdg-open docs/api/html/index.html
```

The Ressourcekortlægning PDF template (`apps/rux/resources/report.typ`, the
only copy — CMake embeds it) has its own light shell, `nix develop .#report`
(`report-watch` / `report-build` / `report-shots`); data contract and loop:
[`docs/guides/report-template.md`](docs/guides/report-template.md).

## Architecture

**Repo split (2026-10-09, see `docs/DIRECTION.md`):** this repo is the
library, the `rux` CLI and its native Qt client only. `ruxd` (the HTTP
service worker) and the web frontend each moved into their own repo under
the `ReUse-X` org:

| Repo | Contents |
|---|---|
| `ReUseX` (this repo) | library, `rux` CLI, Qt client (`apps/rux/qt`) |
| `ruxd` | the HTTP server, its tests, the API contract (`docs/gui/`), Postgres migrations, NixOS module, OCI image |
| `rux-frontend` | the React/Vite web GUI, its nix package, the web half of `design-studio` |

`ruxd` builds against this repo's installed CMake package
(`find_package(ReUseX CONFIG REQUIRED)`, see "Installing / the ReUseX CMake
package" above) and serves the web frontend's built bundle from `--assets` or
`$RUX_GUI_ASSETS`. The Qt client vendors a copy of the frontend's
`tokens.css` at `apps/rux/qt/theme/tokens.css` (`scripts/sync-tokens.sh`) so
it needs neither of those repos to build or run.

### Project Structure

```
ReUseX/
├── libs/reusex/                    # The library (one CMake target per module)
│   ├── include/                    # Public headers; consumers use <reusex/...>
│   │   ├── core/                   # ProjectDB, logging, stages, validate,
│   │   │                           #   MaterialPassport, guid, label_semantics
│   │   ├── geometry/               # geometry_common (Layer 1½); #222 forwarding
│   │   │                           #   shims removed in #248
│   │   ├── segmentation/           # planes, rooms, instances, reconstruct,
│   │   │                           #   depth_filters, downsample, surfels
│   │   ├── reconstruction/         # CellComplex, Solidifier, mesh, texture,
│   │   │                           #   regularization, SceneGraph, Registry
│   │   ├── slam/                   # PlaneGraphOptimizer, JointPairwiseRegistration
│   │   ├── io/                     # rtabmap, e57, ply, rhino, colmap, speckle,
│   │   │                           #   mushroom, exif, export_scene
│   │   ├── vision/                 # ML models, backends, datasets (tensor_rt/, onnx/,
│   │   │                           #   libtorch/, osd/, common/)
│   │   ├── visualize/              # scene builder (populate_scene), headless
│   │   │                           #   render_view, viewport layout (VTK)
│   │   ├── utils/                  # math, cv, tolerances, fmt_formatter
│   │   ├── types/                  # point_types.hpp, eigen_types.hpp
│   │   └── types.hpp               # Umbrella re-exporting types/*
│   ├── src/                        # Implementation, mirrors include/
│   ├── cmake/                      # reusexLibrary.cmake, Dependencies.cmake, ...
│   └── extern/                     # Vendored headers
├── apps/rux/                       # CLI application
│   ├── include/ + src/             # Subcommands, grouped in subdirs
│   └── qt/                         # Native Qt client (in progress): rux_qt_core,
│                                   #   rux_qt_lib, rux-qt-gallery; styles/app.qss;
│                                   #   theme/tokens.css (vendored from rux-frontend);
│                                   #   shell (Q1), Database (Q2), 3D, Rediger,
│                                   #   Pipeline, Log (Q3)
├── apps/blender/reusex_panel/      # Blender add-on
├── bindings/python/                # pybind11 bindings (read-only ProjectDB access)
├── python/                         # reusex_sam3: SAM 3.1 -> ONNX -> TensorRT export
├── models/                         # Model weights (gitignored; see models/README.md)
├── tests/                          # unit/ integration/ benchmarks/ support/ fixtures/
├── docs/                           # STANDARDS.md, CONTRACTS.md, guides/, design/, research/
├── cmake/                          # Shared CMake utilities
├── overlays/ pkgs/ devshells/      # Nix packaging
└── tools/ scripts/ completions/    # Dev tooling (bench.sh, bench-compare.py, check.sh)
```

Three distinct Python trees, easy to confuse:
`bindings/python/` (C++ bindings, built by CMake), `python/` (the standalone
`reusex_sam3` model-export pipeline, run in its own venv — **not** built by
CMake), and `apps/blender/reusex_panel/` (Blender add-on).

### Module targets

`libs/reusex/` builds **one static library per module**, named
`reusex_<module>`, wired in `libs/reusex/cmake/reusexLibrary.cmake`. The layer
graph is **link-enforced** — an illegal dependency is a link error, not a
convention. See [`docs/STANDARDS.md` §1](docs/STANDARDS.md#1-module-boundaries)
for the authoritative diagram and the documented exceptions.

Targets: `reusex_utils`, `reusex_geometry_common`, `reusex_core`, `reusex_io`,
`reusex_vision`, `reusex_segmentation`, `reusex_reconstruction`, `reusex_slam`,
`reusex_gsplat_common`, `reusex_pipeline`, `reusex_visualize` (conditional),
`reusex_gsplat` (conditional), plus two interface targets
(`reusex_common` = public deps, `reusex_private_deps`) and the umbrella
INTERFACE target `reusex` that links everything for backward compatibility.

Gaussian splatting is **two** targets (#332): `reusex_gsplat_common` is the
torch-free half (`GaussianCloud`, the `ProjectDB -> TrainingView` loader, the
view-sampling helpers) and is always built; `reusex_gsplat` adds the CUDA
training loop and only exists under `WITH_CUDA` + the vendored gsplat
rasterizer, linking `reusex_gsplat_common` PUBLIC. Both list their sources
**explicitly** in `reusexLibrary.cmake`, so a new `src/gsplat/*.cpp` needs a
CMake edit assigning it to the CPU or the CUDA half.

The old `ReUseX` / `ReUseX_visualization` target names no longer exist.

**Executables:** `rux` (`apps/rux`) is the only one built from this repo —
`ruxd` (the HTTP service worker and web GUI server) moved to its own repo
(repo split, P4). `rux` uses CLI11 for argument parsing and spdlog as the log
sink. The Morton tile index a GUI streams clouds by is built by the clouds
stage itself (`reusex/pipeline/tile_index.hpp`), whichever front end runs it —
`ruxd` is one consumer of that, from outside this tree.
Plain `rux` (no subcommand, optionally `-p x.rux`) opens the **native Qt
client** when there is a usable display (an existing X or Wayland socket, or
`QT_QPA_PLATFORM=offscreen`; `rux_qt/launch.hpp`) and prints help otherwise.
The shell lives in `rux_qt_lib` (`AppShell`, `ProjectSession`,
`CommandPalette`), linked into the `rux` **executable only**: `main.cpp` hands
`rux::run` a `GuiLauncher` (`rux_app.hpp`), so `rux_lib` and both test
binaries carry no Qt client code. Hidden dev flag:
`rux -p x.rux --quit-after-ms N`.
The Qt client's workspaces are Start, Oversigt (`DatabaseWorkspace`,
read-only), 3D (`Viewer3DWorkspace`), Rediger (`EditWorkspace`: `GraphPanel`,
`EdgeListPanel`, the match editor `PairImages` / `MatchCanvas` over the 3D
overlay — ORB matches between A and B, edited by hand, fitted with
`slam/frame_pair_matches.hpp` to place B — and the pair strip), Pipeline
(`PipelineWorkspace`) and Log. The 3D workspace and `rux render` share one
scene builder, `visualize::populate_scene()` (`visualize/scene.hpp`); its
Pipeline workspace runs stages in-process through `pipeline::JobRunner` with
`pipeline::stage_executor_with_optimize()` (which runs `optimize` and
`graph_discover`; handed over in `GuiLaunch`; `ruxd`
injects the same executor for the web GUI, from its own repo), and "Kopiér som
rux-kommando" (`rux_qt/cli_command.hpp`) is round-trip tested against the real
CLI (`tests/unit/rux_app/test_cli_command_roundtrip.cpp`, via
`rux::set_stage_params_sink`). `RUX_QT_PAGE=oversigt|3d|rediger|pipeline|log`
lands the smoke run on that page (`database` is still accepted as an alias
for `oversigt`; `posegraf` is gone).
`rux-qt-gallery` (`apps/rux/qt`) renders pages of the native Qt client
headless to PNG for design review; its theme is generated at run time from
`apps/rux/qt/theme/tokens.css`, a vendored copy of rux-frontend's tokens
(`scripts/sync-tokens.sh`). The loop and its gotchas:
`.claude/skills/design-studio/references/qt-client.md`.

### Type System (types.hpp)

Aliases are split so a TU pulls in only what it needs (STANDARDS §2):
`types/point_types.hpp` (PCL) and `types/eigen_types.hpp` (Eigen).
`types.hpp` is a backward-compatible umbrella over both — prefer the narrower
header in new code.

- `PointT` = `pcl::PointXYZRGB` - Point cloud points with color
- `NormalT` = `pcl::Normal` - Surface normals
- `LabelT` = `pcl::Label` - Segmentation labels
- `Cloud` / `CloudPtr` - Point cloud containers
- `CloudN` / `CloudL` / `CloudLoc` - Normals, labels, locations

### Segmentation / Reconstruction / SLAM modules

The former single `geometry` module was split by pipeline stage in #222.
`include/geometry/*.hpp` held one-line forwarding shims (e.g.
`geometry/CellComplex.hpp` including `reusex/reconstruction/CellComplex.hpp`)
so consumers didn't need to move at the same time; #248 migrated every
consumer to the new `<reusex/{segmentation,reconstruction,slam}/...>` paths
and deleted the shims. `include/geometry/` now holds only the real
`geometry_common` headers (next section) — there is nothing left to forward.

**segmentation** (`include/segmentation/`):
- `reconstruct.hpp`: pinhole back-projection of depth frames into clouds
- `segment_planes.hpp`: planar detection (noise-adaptive region growing;
  `noise_estimate.hpp` supplies the adaptive thresholds)
- `band_rooms.hpp`: room partitioning from a clutter-free height band per
  storey (2D free-space watershed); `rooms`' default method
  (`kDefaultRoomsMethod`, reusex-icg); needs the `storeys` table, and uses
  the `walls` / `openings` tables when present (wall graph blocks the grid,
  doors cut rooms; optional contract inputs, so run walls -> openings ->
  rooms)
- `split_level.hpp`: `floor_level_map()` — raised floor parts of one
  storey (house1's north block, +0.75 m) on the band grid; band rooms
  measure the wall band from each cell's own floor, and the spaces stage
  gives every space its own measured `z_floor` / `z_ceiling`
  (`measure_space_heights()`, reusex-546). A split level stays one storey
- `spaces.hpp`: `polygonise_spaces()` (W4.2) — band rooms as space
  polygons: CGAL arrangement of the wall faces (+ the interior outline)
  labelled by the band grid, raster fallback where the walls do not close
  a room; metrics, kind, space boundaries. Stored by the `spaces` stage
  (W4.3, `rux create spaces`, after rooms), which rebuilds the band grid
  with the `rooms` cloud's own parameters and refuses when it no longer
  gives back that cloud's labels (re-run rooms); parameters in one table,
  `pipeline/space_parameters.hpp`. `rux create rooms --spaces-json
  out.json` still writes the same spaces to a file (debug)
- `segment_rooms.hpp`: room partitioning via Leiden clustering (igraph);
  `rooms --method leiden`, selectable without storeys
- `segment_instances.hpp` / `reconcile_instances.hpp`: semantic → spatial instances
- `depth_filters.hpp`, `downsample.hpp`, `sync_downsample.hpp`, `densify.hpp`
- `Surfel.hpp` / `surfel_extraction.hpp`: surfels shared with `slam`
- `level.hpp`: `detect_storeys()` — floor/ceiling LSQ fits, the measured
  vertical (bisector of their normals) and storeys from a height histogram,
  as `core::StoreyRecord`s sharing one level frame
  (`geometry/level_frame.hpp`, header-only like `component_persistence.hpp`)
- `classify_planes.hpp`: `classify_planes()` — per plane: orientation,
  majority semantic class + vote share, role (wall/floor/ceiling/furniture/
  clutter/...) via the class -> role map `core/class_role_map.json`
  (`core/class_role_map.hpp`, embedded at build time) or geometric rules when
  the class is missing or contradicted, storey, area, z-range
  (`core::PlaneClassRecord`); stored by the `classify` stage in the
  `plane_classes` table

**reconstruction** (`include/reconstruction/`):
- `CellComplex.hpp`: Boost.Graph-based 3D spatial representation
  (Cell / Face / Vertex nodes); recenters to its bbox centroid internally to
  keep CGAL's inexact kernel usable on georeferenced input (STANDARDS §4)
- `Solidifier.hpp`: cell selection via MIP (HiGHS CPU / cuOpt GPU) → watertight mesh
- `mesh.hpp`, `texture_mesh.hpp`, `regularization.hpp`, `create_windows.hpp`
- `SceneGraph.hpp` / `Registry.hpp`: spatial relationships and object hierarchies
- `quality_metrics.hpp` / `accuracy_metrics.hpp`: backing `rux analyze`
- `walls.hpp`: walls with a measured thickness (WP3): axis families, faces
  from the offset profile, face pairing + Theil-Sen thickness with a
  bootstrap CI, the bounded wall graph (`detect_walls`, `wall_records`);
  stored by the `walls` stage, whose parameters (keys, CLI flags, bounds)
  all come from one table, `pipeline/wall_parameters.hpp`
- `openings.hpp`: doors, windows and openings in the wall faces (WP5): an
  (s, z) face raster with the occluder check (seen side only), rectangle
  fit, mullion merge, door / window / opening by sill and height, evidence
  (labels, instances, glass images, camera-path crossings), door leaf ->
  state and swing, local floor/ceiling per wall (`detect_openings`); stored
  by the `openings` stage, parameters in one table,
  `pipeline/opening_parameters.hpp`
- `solids.hpp`: typed solids (W6.1, `build_solids`): a wall box minus its
  openings (CGAL corefinement), a slab per storey (union of its spaces and
  wall footprints, down to the storey below) plus a roof, a space volume;
  split levels (reusex-546): one slab / roof region per floor / ceiling
  level of the spaces, walls from the lowest adjacent floor to the highest
  adjacent ceiling;
  each `ElementSolid` has a world mesh and the IFC-ready extrusion profile.
  Stored by the `solids` stage (`rux create solids`, after spaces) as the
  meshes `solid_walls` / `solid_slabs` / `solid_spaces`, the element index
  (type, source GUID, face range, profile, voids) in each mesh's
  `parameters` JSON (no schema change); parameters in one table,
  `pipeline/solid_parameters.hpp`; drawn by the `solids` render layer and
  the Qt 3D row "Solider (typede)"
- `solid_metrics.hpp`: the solid-model benchmark (`rux analyze solid`,
  `scripts/bench-solid.sh`): a detected `core::SolidModel` vs a reference plan
  in the solid-model JSON format (`io/solid_model_json.hpp`,
  [`docs/design/solid-model-json.md`](docs/design/solid-model-json.md))

**slam** (`include/slam/`):
- `PlaneGraphOptimizer.hpp`: plane-landmark pose graph (GTSAM), `rux optimize`.
  Solves the stored pose graph (`pose_graph_edges`, seeded from
  `sensor_frames.initial_transform`; `optimize_graph`); `slam/pose_graph.hpp`
  is `rux graph init`, `slam/loop_candidates.hpp` the ORB / panorama / file
  loop front-ends, `slam/graph_discover.hpp` (`rux graph discover`:
  front-ends vs the stored graph, PCM with confirmed edges as fixed inliers;
  also `JobStage::graph_discover`, run by `stage_executor_with_optimize()`),
  `slam/graph_prune.hpp` (`rux graph prune`).
- `pose_graph_conventions.hpp`: stored (`sensor_frames.transform`) ↔ optical
  edge transforms and information; `core/pose_graph.hpp` holds the schema
  vocabulary and SE(3) helpers.
- `JointPairwiseRegistration.hpp`: `rux register`
- `frame_pair_icp.hpp`: depth-cloud ICP between two stored frames, behind
  the Qt client's pair strip and (from the `ruxd` repo) the web GUI's
  `/posegraph/icp`
- `frame_pair_matches.hpp`: ORB keypoint matches between two stored frames
  (lifted to 3D through their depth) and the RANSAC rigid fit of B onto A,
  behind Rediger's match editor (Find match / Placér B fra match)
- `optimize_parameters.hpp`: the optimize stage's JSON parameters →
  `PlaneGraphOptions`, the one reader `rux optimize`, the Qt client and
  (from the `ruxd` repo) the web GUI share. The in-process `optimize` stage
  itself is the header-only `pipeline/optimize_stage.hpp`
  (`stage_executor_with_optimize()`), compiled into `rux` and, in its own
  repo, `ruxd`, because `reusex_pipeline` must not link GTSAM (#464)
- `PanoramaAlignment.hpp`: content-based 360 pose refinement, `rux align 360`
- `drift_diagnostics.hpp`: `rux analyze drift` (reusex-rcv.1.1): GT-free
  drift measures over the back-projected depth (pass-to-pass residual vs the
  noise floor, segment x segment matrix, per-segment rigid ICP suggestions,
  drift cells, through-wall crossings against a plan, doubled-wall patches);
  numbers depend on --frame-step / --depth-stride, compare like with like.
- `PanoramaLoopEdges.hpp`: wide-baseline `LoopEdge`s derived from 360
  panoramas (`rux graph discover --use-panoramas`, #236). Each panorama is
  resected INDEPENDENTLY against every frame it matches, in that frame's own
  optical coordinates — reusing the aligned panorama *pose* instead would be
  circular and carry no drift information. Does not require `rux align 360`.

**geometry_common** (`include/geometry/`, layer 1½): the real (non-shim) headers
there — `utils.hpp`, `cgal_utils.hpp`, `transform_utils.hpp`,
`CoplanarPolygon.hpp`, `BuildingComponent.hpp`, `unweld.hpp`,
`EquirectProjection.hpp` — are the CGAL/PCL/OpenCV primitives shared by the
peers above and by `core`. (`EquirectProjection` sits here because both `slam`
and `vision` need equirect↔perspective reprojection and peers may not link each
other.) Sources are listed **explicitly** (not globbed) in
`reusexLibrary.cmake`, so a new `src/geometry/*.cpp` **does** need a CMake edit;
every other module is globbed.

### Vision/ML Module

**Backend abstraction** (`vision/IMLBackend.hpp`, `vision/BackendFactory.hpp`):
- Pluggable backend system for ML inference. Which backends are compiled in is
  decided at configure time by `ML_BACKENDS` (`AUTO` probes for
  TensorRT / LibTorch / ONNX / OpenVINO; TensorRT is skipped unless
  `WITH_CUDA=ON`). See `libs/reusex/cmake/MLBackendConfig.cmake` — it also
  *excludes* the source files of disabled backends from the glob.
- Implemented today: `vision/tensor_rt/` and `vision/onnx/` (Yolo + Sam3),
  `vision/libtorch/` (dataset side).
- `BackendFactory::detect_model()` sniffs the path — `sam3`/`sam2` in the name,
  or a directory containing `vision-encoder.*`, means `Model::sam3`; otherwise
  `Model::yolo`. `BackendFactory::detect_backend()` picks the backend from the
  extension / directory contents.
- **Construct models via `reusex::vision::create_model_from_path()`**
  (`vision/model_factory.hpp`), not by reaching for a backend class directly:
  the `REUSEX_USE_*` backend defines are attached to `reusex_vision` by
  `configure_ml_backends()`, so they are not visible from the app layer.
- `enum class Backend { opencv, tensor_rt, libtorch, dnn, onnx_runtime,
  openvino, unknown }`.

**Model interfaces:**
- `IModel`: Base interface for all ML models
- `Yolo`: YOLO object detection/segmentation
- `Sam3`: SAM3 segmentation (`tensor_rt/Sam3.hpp`, `onnx/Sam3.hpp`)
- Models loaded from filesystem paths, backend auto-detected

**Dataset interfaces:**
- `IDataset`: Base dataset interface
- `LibTorchDataset` (`vision/libtorch/Dataset.hpp`): PyTorch-compatible dataset
- `TensorRTDataset` (`vision/tensor_rt/Dataset.hpp`): TensorRT-optimized dataset
- `ONNXSam3Dataset` (`vision/onnx/Sam3Dataset.hpp`): SAM3 under ONNX Runtime
- Datasets read frames from `ProjectDB`, not from an RTABMap database

**Key vision components:**
- `annotate.hpp`: Semantic annotation pipeline (`rux create annotate`)
- `project.hpp`: 2D-label → 3D-cloud projection (`rux create project`)
- `Dataloader.hpp`: Batch data loading
- `osd/`: On-screen display for visualization

### Core Module — ProjectDB

`core/ProjectDB.hpp` is the single project store (`*.rux`, sqlite3). It
**replaced** the old `RTABMapDatabase`, which no longer exists in the source
tree — if a doc mentions `RTABMapDatabase`, that doc is stale.

- Pimpl idiom keeps sqlite3 out of the public header; `cv::Mat` and the PCL mesh
  types are forward-declared, not included (STANDARDS §2)
- Stores point clouds (chunked), meshes + texture blobs, Gaussian splats
  (chunked, schema v12 — `rux create gsplat` writes here, #322), sensor frames
  (color/depth/confidence/pose/intrinsics), panoramic images,
  `segmentation_images`, building components, material passports,
  instance↔material links, and the pipeline log. Schema v25 turned
  materials into **resources** (one passport per survey part) and replaced
  `export_templates` with a `templates` table selecting resource keys by
  category or individually, read via `resource_templates()`
  (`core/resource_templates.hpp`). Schema v26 adds the tombstone table
  `survey_dismissed_instances`: a deleted scan-backed survey part's instance
  guid, which `sync_survey` skips so the part is not re-created. Schema v27
  adds the resource view record (`survey_parts.view_*`, key `sys:view`,
  `core/resource_view.hpp`): the one chosen image (sensor frame or panorama +
  normalised region) a resource is shown by, set by `sync_survey` / mask →
  resource and read with no fallback ranking; PATCHing a frame's `sys:view`
  without a region fills in the instance's region (`core::complete_view_region`).
  Schema v28 makes `pose_graph_edges` the stored pose graph (measurement, GTSAM-order
  information, source, status, metrics; canonical `from < to`,
  `UNIQUE(from, to, type)`; `upsert_pose_graph_edge` /
  `set_pose_graph_edge_status` / `update_pose_graph_edge_metrics`), adds
  `sensor_frames.initial_transform` and `project_settings` (`setting()` /
  `set_setting()`). Schema v29 makes semantic point label = API class + 1
  (`core::api_to_point_label`, STANDARDS §3.1): stored semantic clouds were
  shifted and stamped `parameters.label_encoding = "api_plus_one"`.
  Schema v30 adds the `storeys` table (`core::StoreyRecord`, `save_storeys`
  / `storeys()`; written by `rux create storeys`, `JobStage::storeys`) and
  `project_guid()` (project_settings key `project_guid`), the namespace of
  the solid-model rows' GUIDs (`core::deterministic_guid`; a re-detected
  storey keeps its stored GUID, `core/storey_identity.hpp`). It is
  write-once, so a file copy is the same project. Schema v31 adds the
  `plane_classes` table (`core::PlaneClassRecord`, `save_plane_classes` /
  `plane_classes()`, keyed by cloud + plane label, rows go with the cloud;
  written by `rux create classify`, `JobStage::classify`), which
  reads `oriented_normals` from the clouds'
  `normal_orientation` marker (`core/normal_orientation.hpp`, reusex-0se).
  Schema v32 adds `sample_part_links` (sample ↔ survey part, FK-cascading
  on both sides; `SampleRecord::part_codes`, `set_sample_part_links`): a
  part-linked sample bears on the type the part is filed under now, so
  `samples_for_type` / `core::environment_statuses` roll it up there.
  Schema v33 adds `walls` and `wall_joins` (`core::WallRecord` /
  `WallJoinRecord`, `save_walls` / `walls()` / `wall_joins()`; written by
  `rux create walls`, `JobStage::walls`; `storey` is the storey idx, a
  re-detected wall keeps the GUID of the stored wall it lies on,
  `core/wall_identity.hpp`). Schema v34 adds `openings`
  (`core::OpeningRecord`, `save_openings` / `openings()`; written by `rux
  create openings`, `JobStage::openings`; the host wall is named by GUID,
  not a foreign key, so openings survive a walls re-run;
  `core/opening_identity.hpp`). Schema v35 adds `spaces` and
  `space_boundaries` (`core::SpaceRecord` / `SpaceBoundaryRecord`,
  `save_spaces` / `spaces()` / `space_boundaries()`; written by `rux create
  spaces`, `JobStage::spaces`; the polygon is a blob,
  `core/space_polygon.hpp`; a boundary goes with its space but names its
  wall by GUID; a re-detected space keeps the GUID of the stored one whose
  centroid and area match, else uuid5 of storey + centroid at 5 cm,
  `core/space_identity.hpp`)
- Migrating schema; `LATEST_SCHEMA_VERSION` is defined in
  `src/core/ProjectDB.cpp` — read it there rather than trusting a doc (it moves
  most releases)
- **NOT thread-safe** (sqlite3): create a per-thread instance if needed
- `ProjectDB::OpenOptions::hardened` (DEFENSIVE, `trusted_schema=OFF`, no
  triggers or views) is for files from untrusted sources; `ruxd` makes it the
  process default (`set_hardened_by_default`) and refuses uploads and
  registrations that fail `ProjectDB::check_integrity()` (`quick_check`, no
  trigger or view). The `rux` CLI opens files plainly
- Every connection sets a 5 s sqlite busy timeout (`BUSY_TIMEOUT_MS`), and `ruxd --local`
  holds one idle read-write connection per open case for as long as it is open so the WAL index survives
  between per-request connections and the WAL is checkpointed into the .rux on
  exit (a read-only last closer cannot); a lock held past the timeout throws, never
  reads as "no table" / schema `-1`
- **No image rotation.** Images and labels are stored in their original
  orientation; the 90°-clockwise rotation the old RTABMap reader applied is gone
- Label encoding: `CV_16U` + 1 offset in storage (0 = unlabeled), `CV_32S` with
  `-1` in the API. The full contract, including the in-memory `CloudL`
  convention, is [`docs/STANDARDS.md` §3](docs/STANDARDS.md#3-label--identity-contract);
  helpers live in `core/label_semantics.hpp`
- **`probe()`**: a static, `noexcept` check of whether a path is a ReUseX
  project — opens sqlite read-only/immutable (no `-wal`/`-shm` created,
  works in a read-only directory), checks the tables `validate_schema()`
  requires and reads the schema version, all without constructing a full
  `ProjectDB`. Returns `ProbeResult{is_project, schema_version, error}`. Lets
  a GUI vet a foreign or empty sqlite file before a read-write open would
  stamp ReUseX tables onto it.
- **Raw table browsing** (the Qt client's Database workspace and `rux get`):
  `list_tables()` (every user table, sorted, with its row count),
  `table_columns()` (`PRAGMA table_info` per column), `table_rows(table,
  offset, limit, text_limit = 256)` (paged rows in rowid/PK order, one
  `TableCell` per column — a blob cell never reads its full bytes, only up to
  `text_limit` head bytes plus the real size). All three throw
  `std::invalid_argument` for a non-user table.

Other core pieces: `logging.hpp`, `stages.hpp` (`Stage` enum),
`validate.hpp` (`check_stage_inputs`, backing `rux validate --stage`),
`MaterialPassport.hpp` + `materialepas_*`, `guid.hpp`, `SensorIntrinsics.hpp`,
`processing_observer.hpp` / `visual_observer.hpp`, `filter_expression.hpp`.

### I/O Module

External-format adapters only — project state itself lives in `core/ProjectDB`.

- `rtabmap.hpp`: RTABMap SLAM database import (the only RTABMap consumer)
- `mushroom.hpp`: MuSHRoom RGB-D benchmark captures
- `e57.hpp`, `ply.hpp`: point cloud exchange
- `rhino.hpp`: OpenNURBS (.3dm) import/export
- `colmap.hpp`: COLMAP sparse model export
- `speckle.hpp`: Speckle export
- `exif.hpp`: photo metadata (exiv2)
- `export_scene.hpp`: serializes a reconstructed scene
- `solid_model_json.hpp`: the solid-model JSON document (reference plans and
  detected models, `core/solid_model.hpp` records)
- `reusex.hpp`: legacy custom format

There is no HDF5 dependency anywhere in the build.

### CLI Subcommands (apps/rux/src/)

Top-level commands, as registered in `apps/rux/src/rux.cpp`:

| Command | Sub-commands | Source |
|---|---|---|
| `import` | `rtabmap`, `mushroom`, `arkitscenes`, `e57`, `ply`, `gsplat`, `materialepas`, `csv`, `360`, `photos` | `src/import/` |
| `create` | `clouds`, `dense`, `annotate`, `annotate-360`, `material`, `project`, `planes`, `rooms`, `instances`, `storeys`, `classify`, `walls`, `openings`, `spaces`, `solids`, `materials`, `survey`, `mesh`, `texture`, `windows` (deprecated) | `src/create/` |
| `export` | `ply`, `e57`, `materialepas`, `csv`, `rhino`, `semantic-images`, `speckle`, `colmap`, `pdf` | `src/export/` |
| `align` | `360` (content-based panorama pose refinement) | `src/align/` |
| `edit` | `downsample`, `perturb-poses` (drift synthesis, #338) | `src/edit/` |
| `analyze` | `quality`, `accuracy`, `solid` (typed model vs a reference plan), `drift` (where the stored poses disagree: pass-to-pass residual vs noise floor, segment pairs + ICP suggestions, through-wall crossings) | `src/analyze/` |
| `optimize` | — (solves the stored pose graph; `--edits e.yaml` steers it with pose edits, `--edits e.yaml --watch` is the hot-reload pose loop: in-memory re-solve on every save, preview cloud, drift renders, metrics block with a shear indicator and paste-ready YAML suggestions, `optimize_watch.hpp`, [`docs/guides/pose-reference-authoring.md`](docs/guides/pose-reference-authoring.md)) | `src/optimize.cpp` |
| `graph` | `init`, `discover`, `prune` (stored pose graph) | `src/graph/` |
| `register` | — (joint pairwise registration) | `src/register.cpp` |
| `validate` | — (`--stage`, `--json`) | `src/validate.cpp` |
| `get` / `set` / `del` | path-based DB access (`src/database/*_router.cpp`) | `src/get.cpp`, `src/set.cpp`, `src/del.cpp` |
| `info` | — (project summary) | `src/info.cpp` |
| `log` | — (pipeline execution history) | `src/log.cpp` |
| `view` | — (interactive viewer, needs a display; `--plan gt.yaml --watch` = hot-reloading ground-truth workbench over `populate_scene`, `view/plan_workbench.hpp`) | `src/view/` |
| `render` | — (headless render to PNG: `--view top\|plan[:h]\|front\|back\|left\|right\|orbit:N\|frame:<id>`, `--section x=<m>\|y=<m>\|<x0>,<y0>:<x1>,<y1>`; layers incl. `frustums`, `panoramas`, `deviation` (cloud coloured by distance to the mesh), `walls` (the walls table: footprints on a sheet over the cut in plan/top, boxes in 3D; assumed or < 50 % seen dimmer — also a Qt 3D layer, "Vægge"), `openings` (the openings table: bars on the walls in plan/top, rectangles in both wall faces in 3D, coloured by kind — Qt 3D "Åbninger"), `spaces` (the spaces table: translucent polygons in their room's label colour with name and area, at floor height in 3D, a sheet under the walls in plan/top — Qt 3D "Rum (polygoner)"), `solids` (the `solid_*` meshes: walls blue with door / window reveals in their kind's colour, slabs grey, spaces translucent orange, cut with the scene — Qt 3D "Solider (typede)", on by default when present, walls / openings rows then off); label layers on the `--label-*` token scale; `--qa` writes the mesh-QA contact sheet, `visualize/qa_sheet.hpp`; `--plan gt.yaml --watch` re-renders and prints plan scores on every save, `render_watch.hpp`) | `src/render.cpp` |
| `assemble` | — (multi-scan assembly) | `src/assemble.cpp` |

With **no** subcommand, `rux` launches the Qt client (help without a display).
`create`, `import`, `export`, `edit`, `analyze`, `align`, `graph` all
`require_subcommand(1)`.
Global flags: `-v/-vv/-vvv`, `-V/--version`, `-L/--license`, `-D/--visualize`,
`-p/--project <path.rux>` (defaults to `./project.rux`).

There is **no `rux gui`** any more: the web GUI is served by `ruxd`, which
moved to its own repo (repo split, P4, 2026-10-09). `ruxd` builds against
this repo's installed CMake package (`find_package(ReUseX CONFIG REQUIRED)`)
and serves the `rux-frontend` repo's built bundle. Its own CLAUDE.md covers
`ruxd --local` vs. the multi-user server, auth, cases, the Postgres schema
and its API layout — none of that lives here any more.

## Development Patterns

### Adding New Source Files

**Usually no CMakeLists.txt update is required.** `reusexLibrary.cmake` globs
per module with `CONFIGURE_DEPENDS`:
- C++ sources: `libs/reusex/src/<module>/**/*.cpp` — where `<module>` is
  `utils`, `core`, `io`, `vision`, `segmentation`, `reconstruction`, `slam`,
  `visualize`
- CUDA sources: `libs/reusex/src/vision/**/*.cu`
- Headers: `libs/reusex/include/**/*.hpp`
- Tests: `tests/unit/**/*.cpp`

**Exceptions:** `src/geometry/` (the `geometry_common` module) and
`src/gsplat/` (split into `reusex_gsplat_common` / `reusex_gsplat`, #332) use
explicit source lists, so adding a `.cpp` to either needs a
`reusexLibrary.cmake` edit — for gsplat, one that says which half the file
belongs to.

Put the file in the module that matches its pipeline stage, and check
[`STANDARDS.md` §1](docs/STANDARDS.md#1-module-boundaries) first — a
cross-peer include will fail at link time.

### Creating New ML Backends

1. Inherit from `IMLBackend` (`vision/IMLBackend.hpp`)
2. Implement the model/dataset creation entry points
3. Teach `BackendFactory::detect_backend()` about the new extension/layout
4. Add the value to `reusex::vision::Backend`
5. Register the backend's packages and source-exclusion rules in
   `libs/reusex/cmake/Dependencies.cmake` (`ML_BACKEND_<name>_PACKAGES`) and
   `libs/reusex/cmake/MLBackendConfig.cmake`

### Adding New CLI Subcommands

1. Create `apps/rux/src/<group>/<name>.cpp` with a
   `setup_subcommand_<group>_<name>(CLI::App &parent, std::shared_ptr<RuxOptions>)`
2. Call it from the group's `setup_subcommand_<group>()` (e.g. `src/create.cpp`),
   or from `apps/rux/src/rux.cpp` for a new top-level command
3. Keep it thin: parse, validate, call one library entry point, report
   (STANDARDS §1). Templates: `src/create/planes.cpp`, `src/export/ply.cpp`
4. CLI flag defaults must mirror the library options struct, never redefine it
   (STANDARDS §4)

### Working with Point Clouds

- Always use the type aliases (PointT, Cloud, CloudPtr) — include
  `reusex/types/point_types.hpp` in new code, not the `types.hpp` umbrella
- Point clouds use `pcl::PointXYZRGB` (XYZ + RGB color)
- Use `pcl::Indices` for index vectors, not `std::vector<int>`
- `cloud` / `normals` / `planes` / `rooms` / `instances` / `labels` for one scan
  are index-aligned and must be filtered/reordered together — see
  [`docs/CONTRACTS.md`](docs/CONTRACTS.md) and STANDARDS §3.2
- Labels: `0` means unlabeled in a `CloudL`; never index with `label - 1`
  without checking `label >= 1`

### Logging

**Library code** uses the ReUseX logging API in
`libs/reusex/include/core/logging.hpp`:
- **Syntax**: `reusex::debug()`, `reusex::info()`, `reusex::warn()`,
  `reusex::error()`, `reusex::trace()`, `reusex::critical()`
- **Levels**: `reusex::core::LogLevel` — `trace, debug, info, warn, error,
  critical, off`
- **Formatting**: `fmt` compile-time format strings — `reusex::debug("Value: {}", x)`.
  Each function also has a plain `std::string_view` overload.
- **Filtering**: `should_log()` is checked before formatting, so a suppressed
  message costs nothing
- **Timing**: `reusex::stopwatch` (a `spdlog::stopwatch` replacement,
  `elapsed()` → seconds as `double`; has an `fmt` formatter)

**The `rux` CLI application** (`apps/rux`) uses `spdlog` directly (`ruxd`,
in its own repo, does the same):
- Syntax: `spdlog::debug()`, `spdlog::info()`, etc.
- `rux` installs spdlog as the library's log handler and keeps both levels in
  sync from `-v` (see `apps/rux/src/rux.cpp`)

**Implementation details**:
- Logging functions are defined in namespace `reusex::core`
- Explicit `using` declarations promote them into `reusex` for convenience
  (`critical, debug, error, info, log, LogLevel, stopwatch, trace, warn`)
- This avoids namespace conflicts with external libraries (e.g., CGAL's `debug` parameter)
- Handler configured via `reusex::core::set_log_handler()` (defaults to no-op in
  the library); `reset_log_handler()`, `set_log_level()`, `get_log_level()`
  are only reachable as `reusex::core::…`, not via the promoted names

### Error Handling

- Throw `std::runtime_error` for recoverable errors with descriptive messages
- Use RAII for resource management (smart pointers, custom destructors)
- Validate inputs early and fail fast with clear error messages
- **No silent failure**: a stage that produces empty/degenerate output must log
  at `warn` or above *with the reason and the numbers*. Full rules:
  [`docs/STANDARDS.md` §5](docs/STANDARDS.md#5-error-handling--diagnostics)

### RTABMap API Gotchas

RTABMap is now confined to `libs/reusex/src/io/rtabmap.cpp` (import only).

- `rtabmap::Rtabmap::init()` returns void (use try-catch for errors)
- `std::multimap` is in `<map>` header, not `<multimap>`
- Frames read out of RTABMap keep their original orientation — do **not**
  reintroduce the old 90° rotation on the way into `ProjectDB`

## TODO Comment Conventions

This project uses structured TODO/FIXME/BUG/HACK comments compatible with the
[tdg](https://gitlab.com/ribtoks/tdg) tool; the format is kept (tdg-compatible, greppable),
and TODO comments are tracked in beads (`bead=reusex-xxx`), with no automatic issue sync.
The full spec — markers, the required `category=`/`estimate=` metadata line, categories,
time-estimate bands, and worked examples — lives in the **`todo-comments` skill**
(`.claude/skills/todo-comments/SKILL.md`), which auto-triggers when you write or
review one. The short version:

```cpp
// [TODO|FIXME|BUG|HACK]: Brief imperative title (<70 chars)
// category=<CLI|I/O|Geometry|Vision|Visualization|Documentation> estimate=<Xm|Xh|Xd|Xw> [issue=N] [author=alias]
// Description: current behavior, why it matters, solution approach.
```

## Key Dependencies

Authoritative sources: `find_package` calls in
`libs/reusex/cmake/Dependencies.cmake`, and the `buildInputs` in `default.nix`.
Anything not found there is not a dependency.

**Geometry & Processing:**
- PCL (Point Cloud Library) - point cloud operations
- CGAL (`Core`, optionally `Qt6` under `GUI_ENABLED`) - computational geometry
- Eigen3 - linear algebra
- Embree - ray tracing
- OpenMVS (+ nanoflann, jsoncpp, Boost iostreams/program_options/serialization)
  - dense multi-view stereo for `rux create dense`
- TBB - parallel processing
- Boost (incl. Boost.Graph for `CellComplex`)

**Deep Learning:**
- LibTorch - neural network inference
- ONNX Runtime - portable inference
- TensorRT (via `trtsam3` + `tokenizers_cpp`) - optimized GPU inference; only
  searched for when `WITH_CUDA=ON`
- Protobuf / absl / utf8_range - pulled in by the above

**Graph Processing:**
- igraph - Leiden community detection for room segmentation
  (there is **no** GraphBLAS or LAGraph in this build)

**Optimization:**
- HiGHS - Mixed Integer Programming (MIP) solver, the CPU path for `Solidifier`
  - CUDA/GPU support is **disabled** (`CUPDLP_GPU=OFF` in `overlays/highs.nix`)
  - Reason: `Solidifier` solves a MIP with binary variables; HiGHS's GPU PDLP
    solver only handles continuous LP, so it would never be used
- cuOpt - optional NVIDIA GPU MIP backend (`find_package(cuOpt)` under
  `WITH_CUDA`); select with `rux create mesh --solver auto|cuopt|highs`
  (`auto` falls back to HiGHS on GPU error/OOM)
- GTSAM - factor-graph optimization for `slam` (pose-graph / registration)
- There is **no** SCIP dependency (`pkgs/cuOpt/package.nix` only references
  the `scipopt` GitHub org as an upstream source)

**I/O:**
- SQLite3 - the `.rux` project database backing `ProjectDB`
- RTABMap - SLAM database import
- E57Format - point cloud exchange format
- OpenNURBS - Rhino 3D (.3dm) files
- OpenCV (`core`, `imgproc`, `highgui`) - image processing
- VTK (`CommonCore`, `CommonDataModel`, `CommonExecutionModel`, `RenderingCore`,
  `RenderingOpenGL2`, `RenderingFreeType` for the plan overlay's labels) - the rendering backend behind `reusex_visualize`'s
  headless `render_view()`. Arrives in the closure via PCL's visualization
  module, but is `find_package`d directly because the module uses the VTK API
  and needs `vtk_module_autoinit`. Headless rendering depends on this VTK being
  built with `VTK_OPENGL_HAS_EGL` (the flake's is): with no X/Wayland session
  VTK falls back to `vtkEGLRenderWindow` and still renders on the GPU. With no
  rendering device at all, `render_view()` throws
  `visualize::OffscreenGlUnavailable` from an EGL probe run before the first
  `Render()` — VTK itself would segfault there (#313).
- exiv2 - photo EXIF metadata
- CURL + OpenSSL + nlohmann_json - Speckle / HTTP transport
- No HDF5

**Utilities:**
- CLI11 - command-line parsing
- spdlog - logging sink used by the `rux` app
- fmt - string formatting (the library's logging API is built on it)
- range-v3 - modern C++ ranges
- nlohmann_json - JSON
- Qt6 (`Core Widgets Gui OpenGL`, + `OpenGLWidgets` and VTK `GUISupportQt`
  for `apps/rux/qt`) - GUI components; the Qt client bundles Archivo, Oswald
  and JetBrains Mono TTFs (OFL-1.1, `LICENSES/OFL-1.1.txt`)
- Catch2 v3 - tests

## Pre-trained Models

Weights are **never committed**. `models/` is gitignored except for its README
— [`models/README.md`](models/README.md) is the reference for the expected
layout and where each file comes from.

Pass a path with `rux create annotate -n/--net`; it accepts either a single file
or a directory of sub-models. `BackendFactory::detect_model()` decides YOLO vs
SAM3 from the path:

- **SAM3/SAM2** — any path whose stem contains `sam3` or `sam2`, or a directory
  containing a `vision-encoder.*` file
- **YOLO** — everything else (e.g. `yolo11l.pt`, `yolo11l-seg.pt`)
- **TensorRT engines**: `.engine` files (or a directory of them) for optimized
  inference; `.onnx` selects the ONNX Runtime backend, `.pt` LibTorch

**SAM 3.1 engines** are produced by the `reusex_sam3` pipeline in `python/`
(eight engines + `tokenizer.json` + `tracker-meta.json` in one directory):
- [`python/README.md`](python/README.md) — run order / quickstart
- [`docs/sam3.1-tensorrt.md`](docs/sam3.1-tensorrt.md) — full write-up of the
  export, the engine-I/O contract, and how the C++ tracker consumes the engines
- [`docs/sam3.1-export-guide.md`](docs/sam3.1-export-guide.md) — export guide

**`ruxd --local` provisions SAM3 automatically** (code now in the `ruxd`
repo): its segment endpoints download the portable ONNX bundle and build
device-specific engines on first use, via this repo's `reusex_vision`
`EngineBuilder` (`vision/tensor_rt/common/EngineBuilder.hpp`) — no manual
export or engine placement is needed for GUI operation. The managed model
root resolves as: `--models-dir` flag > `$REUSEX_MODELS_DIR` >
`$XDG_CACHE_HOME/reusex/models`. Override the path with `--sam3-model <dir>`
or point at an existing ONNX export via `$REUSEX_SAM3_ONNX_DIR`. The build
recipe is `engine-build.json` (emitted by `build_engines.py --emit-profiles`
in `python/`), which is the shared source of truth consumed by both the
Python `trtexec` driver and the C++ `EngineBuilder`; it is embedded into a
generated header at build time (`python/reusex_sam3/engine-build.json`), so
it is the only link between the Python export pipeline and this repo's
build. Recipe v2 (`recipe_version: 2`) sizes the geometry encoder and decoder
so box and point prompts reach SAM3; managed engines built from an older
recipe are rebuilt once on first use (no download).
See [`models/README.md`](models/README.md) and
[`docs/sam3.1-tensorrt.md`](docs/sam3.1-tensorrt.md) §9 for the full layout,
cache-key scheme, status endpoint, and SAM License redistribution posture.

## Common Tasks

### Import and process a scan

Every command reads/writes one `.rux` project; select it with the global
`-p/--project` flag (default `./project.rux`). The named clouds and tables each
stage consumes and produces are specified in
[`docs/CONTRACTS.md`](docs/CONTRACTS.md).

```bash
# 1. Import raw sensor frames from an RTABMap SLAM database
rux -p scan.rux import rtabmap path/to/database.db

# 2. (optional) Refine the stored per-frame poses
rux -p scan.rux optimize                   # plane-landmark pose graph
rux -p scan.rux register                   # joint pairwise registration

# 3. Back-project depth frames into a fused cloud (-g = voxel size in m)
rux -p scan.rux create clouds -g 0.05

# 4. Geometry pipeline
rux -p scan.rux create planes
rux -p scan.rux create storeys          # rooms defaults to --method band, which needs this
rux -p scan.rux create rooms
rux -p scan.rux create spaces           # after walls -> openings -> rooms
rux -p scan.rux create solids           # typed solids: walls - openings, slabs, spaces
rux -p scan.rux create mesh --solver auto

# Check a stage's inputs before running it
rux -p scan.rux validate --stage mesh

# Inspect / view
rux -p scan.rux info
rux -p scan.rux log
rux -p scan.rux view

# See the result without a display (SSH, CI, an agent's worktree):
# writes a PNG via VTK's off-screen renderer, no X/Wayland needed.
rux -p scan.rux render -o plan.png --view plan --layers cloud   # cut floor plan
rux -p scan.rux render -o orbit.png --view orbit:8 --layers planes
```

### Run semantic annotation

```bash
# ML inference over the stored sensor frames
rux -p scan.rux create annotate -n models/sam3 --cuda

# Project the 2D labels onto the 3D cloud, then split into instances
rux -p scan.rux create project
rux -p scan.rux create instances
```

### Debug with verbosity

```bash
# Use -v, -vv, or -vvv for increasing detail (max 3)
rux -vvv -p scan.rux create planes
```

## Python Integration

Python bindings live in `bindings/python/` (pybind11 + scikit-build-core,
package name `reusex`). `BUILD_PYTHON_BINDINGS` defaults to **ON** in
`CMakeLists.txt:137`, so they are part of a default build — they are **not**
disabled.

Current scope is **read-only `.rux` inspection**, implemented in
`bindings/python/src/bindings.cpp` (~490 lines): `ProjectDB` plus the
`ProjectSummary` / `ProjectInfo` / `CloudInfo` / `MeshInfo` /
`SensorFrameInfo` / `PanoramicInfo` / `ComponentInfo` / `MaterialInfo` /
`PipelineLogEntry` value types. The native module is `reusex._reusex`;
`reusex/__init__.py` re-exports it and degrades to `__status__ = "Native module
not available: …"` if it is missing.

Writing/pipeline APIs are not exposed.

The top-level `python/` directory is **unrelated** to these bindings: it is the
standalone `reusex_sam3` package that exports Meta's SAM 3.1 checkpoint to ONNX
and then to TensorRT engines. It is not built by CMake — run it in its own venv
with prebuilt wheels (see [`python/README.md`](python/README.md); do not
nix-compile torch). `apps/blender/reusex_panel/` is a separate, standalone
Blender add-on.

## License and Copyright

- License: GPL-3.0-or-later
- Copyright: 2025 Povl Filip Sonne-Frederiksen
- Follows REUSE specification for license compliance
- All source files must have SPDX headers


<!-- BEGIN BEADS INTEGRATION v:1 profile:minimal hash:1105d646 -->
## Beads Issue Tracker

This project uses **bd (beads)** for issue tracking. Run `bd prime` to see full workflow context and commands.

### Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --claim  # Claim work
bd close <id>         # Complete work
```

### Rules

- Use `bd` for ALL task tracking — do NOT use TodoWrite, TaskCreate, or markdown TODO lists
- Run `bd prime` for detailed command reference and session close protocol
- Use `bd remember` for persistent knowledge — do NOT use MEMORY.md files

**Architecture in one line:** issues live in a local Dolt DB; sync uses `refs/dolt/data` on your git remote; `.beads/issues.jsonl` is a passive export. See https://github.com/gastownhall/beads/blob/main/docs/core-concepts/sync-concepts.md for details and anti-patterns.

## Agent Context Profiles

The managed Beads block is task-tracking guidance, not permission to override repository, user, or orchestrator instructions.

- **Conservative (default)**: Use `bd` for task tracking. Do not run git commits, git pushes, or Dolt remote sync unless explicitly asked. At handoff, report changed files, validation, and suggested next commands.
- **Minimal**: Keep tool instruction files as pointers to `bd prime`; use the same conservative git policy unless active instructions say otherwise.
- **Team-maintainer**: Only when the repository explicitly opts in, agents may close beads, run quality gates, commit, and push as part of session close. A current "do not commit" or "do not push" instruction still wins.

## Session Completion

This protocol applies when ending a Beads implementation workflow. It is subordinate to explicit user, repository, and orchestrator instructions.

1. **File issues for remaining work** - Create beads for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **Handle git/sync by active profile**:
   ```bash
   # Conservative/minimal/default: report status and proposed commands; wait for approval.
   git status

   # Team-maintainer opt-in only, unless current instructions forbid it:
   git pull --rebase
   git push
   git status
   ```
5. **Hand off** - Summarize changes, validation, issue status, and any blocked sync/commit/push step

**Critical rules:**
- Explicit user or orchestrator instructions override this Beads block.
- Do not commit or push without clear authority from the active profile or the current user request.
- If a required sync or push is blocked, stop and report the exact command and error.
<!-- END BEADS INTEGRATION -->

## Beads: git hooks

bd's git-hook sections have to coexist with the pre-commit framework hooks
that git-hooks.nix installs: clang-format, reuse, alejandra, statix, deadnix
and the check-* hooks on pre-commit, and its git-lfs hooks on pre-push,
post-checkout, post-merge and post-commit. They do, in both places
core.hooksPath can point:

- **Shared git hooks dir** (`$(git rev-parse --git-common-dir)/hooks`, which
  shell.nix pins as an absolute path on every entry, #317). `bd hooks install`
  injects bd's marked section into each existing hook, above pre-commit's
  final `exec`, and keeps the rest of the file. shell.nix runs it on every
  entry, right after git-hooks.nix (re)installs its hooks and the #317 pin,
  which also undoes a `core.hooksPath=.beads/hooks` left behind by `bd init`
  or `bd hooks install --beads`. Without nix, run `bd hooks install` once.
- **`.beads/hooks/`** (committed; used only if core.hooksPath points there).
  Each file opens with a repo-local `HOOK CHAIN` block, outside bd's markers
  so bd keeps it on reinstall. It runs the same hook from the shared dir
  first, and execs it outright when it already carries bd's section, so bd
  runs once either way. Keep that block if you regenerate the hooks.

Both states were checked with a throwaway commit: the pre-commit framework's
checks (reuse, clang-format, ...) and git-lfs's post-commit ran, and so did
`bd hooks run pre-commit` and `prepare-commit-msg`, each once. Every bd
section is guarded by `command -v bd` and skips when the checkout has no
beads database, so CI and clones without bd are unaffected. A linked
worktree uses the main checkout's database; run `bd bootstrap` there, not in
the worktree.
