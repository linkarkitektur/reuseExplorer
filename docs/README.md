# ReUseX Documentation

Welcome to the ReUseX documentation! This directory contains comprehensive documentation for users and developers.

## For Users

### Getting Started
- **Installation Guide**: See main [README.md](../README.md) for build instructions
- **Process Walkthrough**: [guides/Process walk through.md](guides/Process%20walk%20through.md)
- **CLI Usage**: Run `rux --help` for command-line interface documentation

### User Guides
- [Data Loading](guides/DataLoader.md) - Working with datasets
- [OpenCV Integration](guides/OpenCV.md) - Computer vision operations
- [Testing Guide](guides/TESTING.md) - Running and writing tests
- [Documentation Guide](guides/DOCUMENTATION.md) - Building API docs
- [AprilTag 36h11 Printing](guides/apriltag-36h11-printing.md) - Fiducial targets

### Vision / ML
- [SAM 3.1 TensorRT](sam3.1-tensorrt.md) - Full write-up: model teardown, ONNX
  export blockers, engine-I/O contract, how the C++ tracker consumes the engines
- [SAM 3.1 Export Guide](sam3.1-export-guide.md) - Export walkthrough
- [python/README.md](../python/README.md) - Run order / quickstart for the
  standalone `reusex_sam3` export pipeline
- [models/README.md](../models/README.md) - Expected model-weight layout

### Research & benchmarks
- [ARKitScenes Benchmark](research/arkitscenes-benchmark.md)
- [Registration Improvements](research/registration-improvements.md)
- [360 Panorama Integration](research/panorama-integration.md)
- Benchmark workflow: [STANDARDS.md §8.1](STANDARDS.md#81-baseline-vs-candidate-workflow)
  (`scripts/bench.sh` + `scripts/bench-compare.py`)

## For Developers

### Normative
- [Engineering Standards](STANDARDS.md) - The bar every change must meet:
  module boundaries, header hygiene, label contract, parameters, error handling,
  determinism, testing, performance, Definition of Done
- [Pipeline Stage Contracts](CONTRACTS.md) - What each `rux` stage consumes and
  produces in a `.rux` project, enforced by `rux validate --stage`

### Architecture
- [Architecture Overview](../ARCHITECTURE.md) - High-level repository structure
- [Subproject Structure](design/subproject-structure.md) - Build system organization
- [cuOpt Integration](CUOPT_INTEGRATION.md) - GPU MIP solver backend

### Historical design notes
These record earlier designs and are **not** kept in sync with the code:
- [Detailed Architecture](design/architecture.md) - Predates the #222 module split
- [API Design Review](design/api-design-review.md) - Public API audit and refactoring recommendations
- [Database Design](design/database-design.md) - The **retired** `RTABMapDatabase`; the current store is `core/ProjectDB.hpp`

### Contributing
- [Contributing Guide](../CONTRIBUTING.md) - Code standards and workflow
- [AI Assistant Guide](../CONTRIBUTING_AI.md) - Guidance for AI coding assistants
- [CLAUDE.md](../CLAUDE.md) - Naming conventions, build/CLI orientation, TODO format
- [Testing Guide](../tests/README.md) - Writing and running tests
- Coverage tools: `./tools/coverage/generate_coverage.sh`

### API Reference

Build the API documentation with:
```bash
cmake --build build --target docs
xdg-open docs/api/html/index.html
```

Or view online: [GitHub Pages](https://pfmephisto.github.io/ReUseX/) (published by
`.github/workflows/doxy.yml`)

## Repository Structure

```
ReUseX/
├── libs/reusex/        # The library (one target per module)
├── apps/rux/           # CLI application, plus the native Qt client (apps/rux/qt)
├── apps/blender/       # Blender add-on
├── bindings/python/    # pybind11 bindings (read-only ProjectDB access)
├── python/             # reusex_sam3 SAM 3.1 export pipeline (standalone)
├── models/             # Model weights (gitignored)
├── tests/              # unit/ integration/ benchmarks/ support/ fixtures/
├── docs/               # This directory
│   ├── api/           # Doxygen output (generated)
│   ├── guides/        # User guides
│   ├── research/      # Benchmark / research notes
│   └── design/        # Historical design notes
├── cmake/             # Shared CMake utilities
├── overlays/ pkgs/    # Nix packaging
└── tools/ scripts/    # Development tools
```

`ruxd` (the HTTP service worker / web GUI server) and the web frontend each
live in their own repo — `ruxd` and `rux-frontend` — not in this tree
(repo split, 2026-10-09; see `docs/DIRECTION.md`). Deploying them (Docker
images, Supabase auth, Caddy) is covered by the `reusex-fleet` repo.

## Quick Links

- **Main README**: [../README.md](../README.md)
- **Architecture**: [../ARCHITECTURE.md](../ARCHITECTURE.md)
- **Engineering Standards**: [STANDARDS.md](STANDARDS.md)
- **Pipeline Contracts**: [CONTRACTS.md](CONTRACTS.md)
- **Contributing**: [../CONTRIBUTING.md](../CONTRIBUTING.md)
- **Test Documentation**: [../tests/README.md](../tests/README.md)
- **Python Bindings**: [../bindings/python/README.md](../bindings/python/README.md)

## Building Documentation

### API Documentation (Doxygen)
```bash
cmake -B build -DBUILD_DOCUMENTATION=ON
cmake --build build --target docs
```

Output: `docs/api/html/index.html`

### User Guides

User guides are written in Markdown and located in `docs/guides/`. They are human-readable and don't require building.

## Getting Help

- **Issues**: [GitHub Issues](https://github.com/pfmephisto/ReUseX/issues)
- **Discussions**: [GitHub Discussions](https://github.com/pfmephisto/ReUseX/discussions)
- **Questions**: Open an issue with the `question` label

## Documentation Standards

When contributing documentation:
- Use clear, concise language
- Include code examples where appropriate
- Keep guides focused on one topic
- Update this README when adding new documents
- Follow Markdown best practices

## License

Documentation is licensed under the same terms as the project: GPL-3.0-or-later

See [LICENSE.md](../LICENSE.md) for details.
