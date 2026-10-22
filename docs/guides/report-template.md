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
| `materials[]` | `{guid, has_thumbnail, thumbnail_path, properties}` | One per material passport; `thumbnail_path` is relative to the data dir (`images/<guid>.jpg`) or `""`; `properties` is keyed by **column name** and holds only the `columns[]` (the template prints nothing else) |
| `survey.rows[]` | `{name, bim7aa, eak, quantity, mass, treatment, environment}` | Reportable (approved, not withheld) survey types; all values preformatted Danish strings (`"620 m²"`, `"190 t"`, `"—"`) |
| `survey.circularity[]` | `{label, tonnes}` | Tonnes per treatment over exactly those rows |
| `survey.blocking` | int | Types keeping the report a draft (> 0 = draft) |
| `resources` | `null` or `{name, tables[]}` | Ressourcetabel through a resource template; each table `{headers[], rows[][]}`, at most 8 columns, each led by `Betegnelse` |

Sections added with the Forside / Sagsdata layout (reusex-ufcm). A
`data.json` from an older generator lacks them and the template still
compiles. Every value is a preformatted Danish display string, `null` when
not filled in:

| Field | Type | Meaning |
|---|---|---|
| `project` | object | `id`, `name`, `building_address`, `year_of_construction`, `survey_date` (`14.09.2026`), `survey_organisation`, `notes`, plus the BEK 496 Sagsdata fields `bbr_number`, `cadastral`, `client`, `client_contact`, `coordinator`, `municipality`, `demolition_start`, `demolished_floor_area_m2` (`"2.480 m²"`), `screening_approved_at`. The first project of the file; the Sagsdata fields live in project_settings under `sagsdata/<project id>/<field>` (`core/sagsdata.hpp`) |
| `sagsdata[]` | `{label, value}` | The Sagsdata page's rows, in the GUI's order |
| `types[]` | `{id, name, approved, eak, treatment, quantity, mass, mass_needed, environment}` | Every non-rejected survey type (Ressourcekortlægning), approved or not; `mass` null = no tonnes (`mass_needed` false for bevaring) |
| `types_summary` | `{count, approved, total, line}` | `line` = `"7 typer · 406,3 t i alt · 5 godkendt"` |
| `samples[]` | `{code, title, what, types, stage, result, contaminated}` | Miljøkortlægning; `result` null until answered |
| `appendices[]` | `{letter, text, status}` | Prøver & bilag: one per sample, then the photo documentation; `status` null = afventer |
| `demolition_steps[]` | `{title, items[]}` | Nedrivningsplan: Miljøsanering first, then by the waste hierarchy |
| `sections[]` | `{id, title, ready, missing[]}` | Readiness per section (forside, sagsdata, ressourcer, miljoe, bilag, nedrivningsplan), as `PreparedReport::sections()` returns it |
| `report` | `{version, status, date}` | The version this run would be stored as (`list_report_pdfs().size() + 1`), `"kladde"` / `"komplet"` (blocking > 0 is kladde), the date (`22.10.2026`) |
| `cover_path` | string | `images/cover.jpg` / `.png`, or `""` |
| `cover_source` | string | `frame`, `panorama`, `upload` (the project's choice, `core/report_cover.hpp`), `render` (the caller's `ReportOptions::cover_fallback`), or `none` |

The cover image is the project setting `report/cover_image`: a sensor
frame or a 360° panorama plus a normalised region (the resource view
record's shape), or an uploaded JPEG/PNG kept in `report_assets` (schema
v38). Unset, the report uses `ReportOptions::cover_fallback` — core cannot
render, so `rux export pdf` and ruxd pass an orbit render of the cloud.

Every section starts on a new page and carries a `<section>` metadata
label; `PreparedReport::section_pages()` maps them to their first page with
`typst eval --in report.typ 'query(<section>)…'`.

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

## PNG pages (the GUI preview)

The web GUI's Rapport preview shows the real template, not a mock-up:
ruxd builds a `reusex::PreparedReport` (data dir + template in a temp dir),
looks its `digest()` up in a cache, and only on a miss runs
`compile_png(ppi)` (`typst compile --format png --ppi N`, one PNG per page)
off the request thread. The digest covers the template, `data.json` without
`generated_at` and every image, so an unchanged project is never compiled
twice. `sections()` gives the per-section readiness and `section_pages()`
the first page of each section.
