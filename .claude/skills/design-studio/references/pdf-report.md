# PDF report — reference

## Where things are

| What | Where |
|---|---|
| Template (only copy) | `apps/rux/resources/report.typ` |
| Data contract | `docs/guides/report-template.md` |
| Sample data (synthetic, hand-maintained) | `apps/rux/resources/report-sample/` (`data.json`, `images/`) |
| Sample images | `scripts/report-sample-images.sh` |
| Generator (C++) | `libs/reusex/src/core/report_generator.cpp` |
| Tests that keep it compiling | `ReportSample_*`, `ReportTemplate_*`, `ReportPdf_*`, `ReportDump_*` (`tests/unit/core/`), `ExportPdf_*` (`tests/unit/rux_app/`) |

## The loop

```bash
nix develop .#report
report-shots                          # sample data
report-shots shots/report-data        # a dump: rux -p x.rux export pdf --dump-data shots/report-data
```

Read every `shots/report/page-NN.png`, critique, fix, repeat. `report-build`
exits non-zero on a Typst error — read its message before guessing.

Gotchas:
- The template reads its data via `sys.inputs.at("data-dir", default: ".")`;
  every path to data or images must go through `data_dir`. A bare
  `json("data.json")` or `image(mat.thumbnail_path)` works in the generator
  and breaks the preview (or the other way round).
- System fonts are ignored everywhere. A font not embedded in typst must be
  shipped to the generator too (see the guide's "Fonts" section), or ruxd's
  PDF silently differs from the preview.
- When the sample stops exercising a case, extend `data.json` by hand; the
  `ReportSample_DataJson_ExercisesTheTemplate` test lists the minimums.

## Print critique checklist

- Nothing wider than the text block (tables, images, long names, URLs).
- No table whose column count comes from data — a data-driven column set is
  split or turned into cards (the resource tables already cap at 8).
- No card or table row split across pages; no orphaned headings at a page
  bottom.
- Readable in greyscale: no meaning carried by colour alone.
- Images ≥ 150 dpi at their printed size; `fit: "contain"`, never stretched.
- Danish formatting: decimal comma (`1.234,5`), dates `13. oktober 2026` or
  `13.10.2026`, units with a space (`620 m²`, `190 t`).
- Every page has the header and footer (project, title, page X / Y).
- Draft state (`survey.blocking > 0`) is unmistakable on the first page.
- File size sane: < 10 MB for 150 resources (check `ls -l shots/report/report.pdf`).
