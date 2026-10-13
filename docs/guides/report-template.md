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

The template reads it from `sys.inputs.at("data-dir", default: ".")`. The
generator writes it into a temp dir next to the template (default `"."`);
the preview scripts pass `--root / --input data-dir=<abs dir>`.
`rux export pdf --dump-data DIR` writes exactly what the PDF run sees.

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
