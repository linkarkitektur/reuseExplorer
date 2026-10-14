# Report template workflow (sub-project B)

Date: 2026-10-13 · Status: draft for review · Repos: ReUseX only

Part of the PDF report rework (Ressourcekortlægning, #456 follow-up):

| # | Sub-project | Spec |
|---|---|---|
| A | View record on resources | `2026-10-13-resource-view-record-design.md` |
| **B** | **Report template workflow (this spec)** | this file |
| C | Report redesign (cover, cards with images, non-overflowing tables) | written after A + B merge |

## 1. Intent

The report must become an official document (attached to a demolition-permit
application to the municipality). Getting there takes many design rounds on
the Typst template, by the user and by Claude. Today that loop is slow: the
template lives twice (`apps/rux/resources/report.typ` and a pasted copy in
`libs/reusex/src/core/report_generator.cpp`), and seeing a change means a C++
rebuild plus ruxd. Success = edit `report.typ`, see the PDF update within a
second, against either committed sample data or a real project's data, with
no C++ build in the loop.

## 2. Single source for the template

- `apps/rux/resources/report.typ` is the only copy. CMake embeds it into a
  generated header at build time (same mechanism as `engine-build.json` →
  generated header in `reusex_vision`; reuse that CMake helper if it is
  generic, else write a small `file(READ)` + `configure_file` one). The
  pasted `kTypstTemplate` and the `ReportTemplate_CopiesInSync` test are
  removed. The `.typ` file is a build dependency (`CMAKE_CONFIGURE_DEPENDS`
  or a custom command), so editing it rebuilds the header.
- `report_generator.hpp` gains an options struct; the existing two-argument
  call keeps compiling (ruxd uses it):

  ```cpp
  struct ReportOptions {
    std::optional<std::int64_t> resource_template_id;
    /// Typst template to use instead of the embedded one. Empty: the
    /// $REUSEX_REPORT_TEMPLATE environment variable, else embedded.
    std::filesystem::path template_path;
  };
  std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(ProjectDB &, const ReportOptions &);
  /// Write data.json + images/ into dir exactly as the PDF run sees them.
  void dump_report_data(ProjectDB &, const ReportOptions &, const std::filesystem::path &dir);
  ```

  The existing `(db, optional<int64_t>)` overload forwards to the new one.
- A template path that does not exist throws `std::runtime_error` naming it.

## 3. Data contract

`data.json` + an `images/` directory, both relative to the Typst `--root`.
The shape is documented once, in `docs/guides/report-template.md` (new), and
the template's header comment points there instead of repeating it. B keeps
today's fields unchanged (C extends them); the dump/sample layout is what
C will build on, so it is fixed here:

```
<dir>/data.json
<dir>/images/<name>.jpg
```

## 4. Committed sample data

`apps/rux/resources/report-sample/` — `data.json` + ~8 small JPEGs (≤ 60 kB
each, our own captures from a project we may redistribute, or synthetic; no
third-party photos). Hand-maintained to exercise the template: ~15
resources over ~5 types, a few without an image, at least one very long
name, one template with many (≥ 15) fields, one draft-state type
(`blocking > 0`), Danish characters everywhere. SPDX: add a `REUSE.toml`
annotation for the directory (JSON cannot carry a header; images need it
anyway).

## 5. `nix develop .#report`

New `devshells/report.nix` (auto-discovered by `flake.nix`). Light: no
`inputsFrom` the C++ package — `typst`, `poppler-utils` (pdftoppm), fonts
the template uses (provide them via `TYPST_FONT_PATHS`; the template must
not depend on system fonts so output is identical in the devshell, CI, ruxd
and the nix package). Commands (each a `writeShellScriptBin`, `menu`
prints them, motd like `devshells/completions.nix`):

| Command | Does |
|---|---|
| `report-watch [DATA_DIR]` | `typst watch apps/rux/resources/report.typ <out>/report.pdf --root DATA_DIR` (default: the sample dir). Copies/links the template into the root as needed so `json("data.json")` resolves. |
| `report-build [DATA_DIR]` | one-shot compile, non-zero exit on Typst error |
| `report-shots [DATA_DIR]` | build, then `pdftoppm -r 110 -png` every page into `shots/report/page-NN.png` |

Outputs go to `shots/report/` (already gitignored? verify; add if not).
Typst resolves paths against `--root`, so the scripts must keep the
template and data in one root — the implementation picks the simplest way
(temporary root with symlinks, or `--root` at the data dir and template
copied in) and documents it.

## 6. `rux export pdf`

New `apps/rux/src/export/pdf.cpp`, registered under `export`:

```
rux -p x.rux export pdf -o report.pdf [--resource-template ID] [--template FILE.typ]
rux -p x.rux export pdf --dump-data DIR [--resource-template ID]
```

Thin (STANDARDS §1): parse, call `generate_ressourcekortlaegning_pdf` or
`dump_report_data`, report. `--dump-data` output is directly usable as
`report-watch DIR`. Also offered by the Qt client's "Kopiér som
rux-kommando" only if the Qt client grows a report action (not in scope).

## 7. design-studio skill

Add a "PDF report" section to `.claude/skills/design-studio/SKILL.md` (and
broaden the skill description so it triggers on report/PDF/Typst work), plus
`references/pdf-report.md`: the loop (`report-shots` → Read every PNG →
critique → fix), the data contract pointer, how to get real data
(`rux export pdf --dump-data`), and a print critique checklist: nothing
wider than the text block, no table whose column count comes from data, no
card split across pages, no orphaned headings, readable in greyscale, image
resolution ≥ 150 dpi at printed size, Danish number/date formatting, every
page has header/footer, file size sane (< 10 MB for 150 resources).

## 8. Testing

- Unit (`tests/unit/core`): embedded template equals the file on disk (one
  read, guards the CMake embedding); `template_path` override is used;
  missing override throws; `dump_report_data` writes `data.json` that
  parses and references only files that exist.
- `rux_app` test: `export pdf --dump-data` round-trip produces a directory
  that `typst compile` accepts (skip when `typst` not on PATH, as the
  existing PDF test does).
- Sample data compiles: a ctest that runs `typst compile` on
  `report-sample/` (skip without typst) so the sample never rots.

## 9. Out of scope

Any visual change to the report (C). Resource images in `data.json` (C, on
top of A's view record).
