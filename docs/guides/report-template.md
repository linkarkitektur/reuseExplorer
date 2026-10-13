# PDF report template (Ressourcekortlægning)

`apps/rux/resources/report.typ` is the only copy of the template. CMake
embeds it into `reusex/core/report_template_default.hpp` at build time;
`generate_ressourcekortlaegning_pdf()` compiles that embedded copy unless a
template file is named by `ReportOptions::template_path`
(`rux export pdf --template FILE.typ`) or `$REUSEX_REPORT_TEMPLATE`. A named
file that does not exist is an error, never a silent fallback.

## Data contract

A report data directory holds:

```
<dir>/data.json
<dir>/images/<name>.jpg      (or .png / .webp / .gif, from the stored MIME type)
```

An image is named `images/<guid>.<ext>` when the guid is a safe file name
(`[A-Za-z0-9][A-Za-z0-9._-]*`, no `..`); otherwise it is
`images/material-NNNN.<ext>`, `NNNN` being the material's zero-padded index.

The template reads it from `sys.inputs.at("data-dir", default: ".")`. The
generator writes it into a temp dir next to the template (default `"."`);
the preview scripts pass `--root / --input data-dir=<abs dir>`.
`rux export pdf --dump-data DIR` writes exactly what the PDF run sees. `DIR`
may be missing, empty, or a *previous* dump — a directory holding nothing but
`data.json` and `images/`, with `data.json` parsing as an object that has
`project_name`, `materials` and `survey` keys; its `images/` is then replaced
wholesale. Any other non-empty directory (an unrelated `data.json`, stray
files, a `data.json` that fails to parse or is missing those keys) is
refused and left untouched, rather than guessed at.

`data.json`:

| Field | Type | Meaning |
|---|---|---|
| `project_name` | string | First project's name, else the `.rux` file stem |
| `generated_at` | string | UTC ISO 8601 (`2026-10-13T09:30:00Z`) |
| `columns[]` | `{id, name, type}` | User-defined passport property columns |
| `materials[]` | `{guid, has_thumbnail, thumbnail_path, properties}` | One per material passport; `thumbnail_path` is relative to the data dir (`images/<guid>.jpg`) or `""`; `properties` is keyed by **column name** |
| `survey.rows[]` | `{name, bim7aa, eak, quantity, mass, treatment, environment}` | Reportable (approved, not withheld) survey types; all values preformatted Danish strings (`"620 m²"`, `"190 t"`, `"—"`) |
| `survey.circularity[]` | `{label, tonnes}` | Tonnes per treatment over exactly those rows |
| `survey.blocking` | int | Types keeping the report a draft (> 0 = draft) |
| `resources` | `null` or `{name, tables[]}` | Ressourcetabel through a resource template; each table `{headers[], rows[][]}`, at most 8 columns, each led by `Betegnelse` |

Every value the template prints comes from data, so names containing Typst
markup (`#panic(...)`, `*x*`) print literally.

## Fonts

Typst runs with `--ignore-system-fonts` everywhere (generator and preview
scripts), so the PDF does not depend on the host. Today the template uses
only fonts embedded in the typst binary (Libertinus Serif, New Computer
Modern, DejaVu Sans Mono). The `.#report` shell also puts the Qt client's
bundled TTFs (`apps/rux/qt/resources/fonts`: Archivo, Oswald, JetBrains Mono,
OFL-1.1) on `TYPST_FONT_PATHS`, but the generator inside ruxd / the nix
package does not see them yet — a template that uses them must first make
the generator pass a font path, or ruxd's PDF silently falls back.

## Preview loop: `nix develop .#report`

A light shell (typst, pdftoppm, ImageMagick; no C++ toolchain):

```bash
nix develop .#report
report-watch                 # live: shots/report/report.pdf on every save of report.typ
report-build [DATA_DIR]      # one-shot; non-zero exit on a Typst error
report-shots [DATA_DIR]      # build + shots/report/page-NN.png at 110 dpi
```

`DATA_DIR` defaults to `apps/rux/resources/report-sample/` (committed,
synthetic). The scripts compile the repo's `report.typ` in place with
`--root / --input data-dir=<abs DATA_DIR>`, so edits are picked up without a
build; `$REUSEX_REPORT_TEMPLATE` points them (and the generator) at another
template file. Outputs under `shots/` are gitignored.

The sample is hand-maintained: edit `data.json` directly; regenerate the
JPEGs with `scripts/report-sample-images.sh`. `ReportSample_*` tests keep it
compiling and keep it exercising long names, missing images, a 16-field
template, a draft state and Danish characters.

## Real project data

```bash
rux -p x.rux export pdf --dump-data shots/report-data [--resource-template ID]
nix develop .#report -c report-watch shots/report-data
```

`--dump-data` writes into a missing or empty directory, or replaces a
previous dump (its `images/` is cleared first); any other non-empty
directory is refused. A real project's photos stay out of git: dump under
`shots/` (gitignored), never into `apps/rux/resources/`.

`rux -p x.rux export pdf -o report.pdf [--resource-template ID] [--template FILE.typ]`
renders the PDF the same way ruxd does.
