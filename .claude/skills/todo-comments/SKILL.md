---
name: todo-comments
description: Write or review structured TODO/FIXME/BUG/HACK comments in ReUseX C++ code. Use whenever adding, editing, or auditing an in-code TODO-style marker, or when asked about the project's TODO/tdg conventions (required category=/estimate= metadata, markers, categories, time estimates). The structured format is kept (tdg-compatible, greppable), and TODO comments are tracked in beads, so format matters.
---

# TODO Comment Conventions

This project uses structured TODO comments compatible with the [tdg (TODO Generator)](https://gitlab.com/ribtoks/tdg) tool. The structured format is kept (tdg-compatible, greppable), and TODO comments are tracked in beads (`bead=reusex-xxx`, see Optional Metadata). There is no automatic GitHub issue sync; GitHub issues are disabled on ReUse-X/ReUseX.

## Format Specification

All TODO-style comments must follow this format:

```cpp
// [MARKER]: Brief imperative title (50-70 chars max)
// category=ModuleName estimate=Xh [issue=N] [author=alias]
// Multi-line description providing context and rationale.
// Additional details about current behavior, impact, or solution approach.
```

**Key formatting rules:**
- First line: Marker keyword followed by colon, then brief imperative title
- Second line: Metadata fields as `key=value` pairs, space-separated
- Subsequent lines: Detailed description (no metadata, just explanation)
- All lines must start with `//` (C++ line comment style)

## Supported Markers

- **TODO:** Future improvements, missing features, enhancements, or refactoring opportunities (adding functionality, improving performance, enhancing UX)
- **FIXME:** Known limitations / suboptimal-but-functional code, edge cases not fully handled, workarounds in place
- **BUG:** Active bugs — crashes, segfaults, memory/resource leaks, race conditions, incorrect results
- **HACK:** Temporary workarounds, shortcuts bypassing proper abstractions, knowingly-violated design patterns

## Required Metadata (second line)

Every comment **must** include:

1. **`category=X`** — Module or area (see Categories below)
2. **`estimate=Xh`** — Time estimate. Suffixes: `30m`/`1h`/`2h`/`4h` (minutes/hours), `1d`/`2d` (days), `1w`/`2w` (weeks)

```cpp
// TODO: Add input validation for mesh generation parameters
// category=CLI estimate=30m
// Currently accepts any values which can lead to crashes with extreme inputs.
```

## Optional Metadata

- **`bead=reusex-xxx`** — the beads issue tracking this comment's removal, when one exists. Example: `bead=reusex-hbi`
- **`issue=N`** — a GitHub issue number from before the switch to beads (pfmephisto/ReUseX or an older point in this repo's history). Do not add new ones; `bead=` replaces it going forward.
- **`author=alias`** — Creator's alias, for handoffs (git blame is preferred otherwise). Example: `author=pfils`

## Categories

| Category | Scope | Examples |
|----------|-------|----------|
| `CLI` | Command-line interface, argument parsing, rux subcommands | Argument validation, help text, new subcommand |
| `I/O` | File I/O, database access, format conversions | ProjectDB schema, RTABMap import, E57 import, Rhino/Speckle export |
| `Geometry` | `geometry_common` / `segmentation` / `reconstruction` / `slam` modules | Plane detection, room segmentation, CGAL algorithms, pose-graph optimization |
| `Vision` | ML models, inference backends, semantic annotation | YOLO integration, TensorRT optimization, SAM3 |
| `Visualization` | PCL visualization, on-screen display, rendering | Point cloud viewer, debug overlays, Qt widgets |
| `Documentation` | Code comments, Doxygen, guides, examples | API docs, tutorials, inline comments |

Pick the primary module affected even if the change touches several; use `CLI` for user-facing interface changes and the module name for internal changes.

## Time Estimate Guidelines

| Estimate | Complexity | Description | Example |
|----------|-----------|-------------|---------|
| `30m-1h` | TRIVIAL | Quick config change, single-line fix, simple parameter addition | Adding a CLI flag, fixing typo logic |
| `1h-4h` | EASY | Single-session task, isolated change, well-understood problem | Input validation, error message improvement |
| `1d-2d` | MEDIUM | Multi-session work, moderate complexity, some research needed | Refactoring a module, adding medium feature |
| `3d-1w` | HARD | Significant refactor, architectural changes, complex debugging | Redesigning subsystem, fixing deep bugs |
| `>1w` | VERY_HARD | Major feature, fundamental redesign, multi-component changes | New backend support, SLAM integration |

Account for implementation + testing + documentation and for unknowns/edge cases; when uncertain, estimate higher.

## Writing Good Comments

**Titles:** imperative mood ("Add validation", not "Validation needed"), specific ("Validate mesh vertex count", not "Fix validation"), under 70 chars, focused on the *what*.

**Descriptions:** line 1 = current behavior/problem; line 2 = why it matters (impact/rationale); line 3+ = solution approach or steps; include references (related TODOs, docs, issue numbers).

```cpp
// TODO: Add comprehensive input size validation with detailed error messages
// category=CLI estimate=30m
// Current validation only checks a subset of input files. Should validate all:
// 1. Check cloud, rooms, normals, plane_labels all have same size
// 2. Verify plane_normals and plane_centroids have expected dimensions
// 3. Provide specific error message showing actual vs expected sizes
// 4. Add early validation before heavy processing to fail fast
```

## Real Examples from the Codebase

Simple TODO with a reference (`libs/reusex/include/vision/nms.hpp:22`):
```cpp
// TODO: Replace custom NMS with torchvision library implementation
// category=Vision estimate=4h
// Current implementation is custom-written. Consider using official torchvision
// NMS: Reference:
// https://github.com/pytorch/vision/blob/main/torchvision/csrc/ops/cpu/nms_kernel.cpp
```

Large TODO with an `issue=` link (`libs/reusex/src/core/ProjectDB.cpp:13`):
```cpp
// TODO: Remove core -> geometry dependency in BuildingComponent persistence
// category=I/O estimate=1d issue=222
// ProjectDB (Layer 2, core) persists geometry::BuildingComponent and calls
// geometry::CoplanarPolygon (de)serialization — a Layer-3 type. This is a
// documented layering exception enforced via an explicit reusex_core ->
// ...
```

HACK documenting a workaround (`libs/reusex/src/io/speckle.cpp:883`):
```cpp
// HACK: Wrap material InstanceProxies in a "Cameras" sub-collection and
// rename each one "Camera N" to match the legacy Grasshopper layout.
// category=I/O estimate=2h
// The reuse-x webapp's loadImages composable hardcodes
//     speckleRoot.elements.find(c => c.name === 'Cameras')
// so without this wrapper our material tags never get discovered.
```

## Finding TODOs Locally

```bash
grep -r "// TODO:" libs/ apps/ --include="*.cpp" --include="*.hpp"        # all TODOs
grep -r "category=Geometry" libs/ apps/ --include="*.cpp" --include="*.hpp"  # by category
grep -r "estimate=[0-9]*[dw]" libs/ apps/ --include="*.cpp" --include="*.hpp" # high-estimate (1d+)
```

Run tdg itself (install from https://gitlab.com/ribtoks/tdg):
```bash
tdg --path libs/ --path apps/ --output TODO.json
```
