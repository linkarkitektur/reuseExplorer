// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Ressourcekortlægning (Material Resource Mapping) report template.
//
// Invoked by ruxd via:
//   typst compile report.typ out.pdf --root <tmpdir>
//
// data.json must be present in <tmpdir> with the structure:
//   {
//     "project_name": "...",
//     "generated_at": "...",
//     "columns": [{"id": "...", "name": "...", "type": "..."}],
//     "materials": [{
//       "guid": "...",
//       "has_thumbnail": true,
//       "thumbnail_path": "thumbnails/guid.jpg",
//       "properties": {"col-name": "value"}
//     }],
//     "survey": {
//       "rows": [{"name", "bim7aa", "eak", "quantity", "mass", "treatment", "environment"}],
//       "circularity": [{"label": "Genanvendelse", "tonnes": "196,8 t"}],
//       "blocking": 7
//     }
//     "resources": null | {
//       "name": "template name",
//       "tables": [{"headers": ["Betegnelse", ...], "rows": [["...", ...]]}]
//     }
//   }
//
// This file is the only copy: CMake embeds it into the generated header
// reusex/core/report_template_default.hpp at build time.

#let data = json("data.json")
#let cols = data.columns
#let mats = data.materials

// ── Page layout ──────────────────────────────────────────────────────────────

#set page(
  paper: "a4",
  margin: (top: 2.5cm, bottom: 2.8cm, left: 2cm, right: 2cm),
  header: context [
    #set text(size: 8pt, fill: luma(140))
    #data.project_name
    #h(1fr)
    Ressourcekortlægning
  ],
  footer: context [
    #set text(size: 8pt, fill: luma(140))
    #data.generated_at
    #h(1fr)
    Side #counter(page).display() / #counter(page).final().at(0)
  ],
)

#set text(size: 10pt)
#set par(justify: false)

// ── Title block ───────────────────────────────────────────────────────────────

#v(0.5cm)
#align(center)[
  #text(size: 22pt, weight: "bold")[Ressourcekortlægning]
  #v(0.3cm)
  #text(size: 13pt)[#data.project_name]
  #v(0.2cm)
  #text(size: 9pt, fill: luma(120))[Genereret: #data.generated_at]
]
#v(0.6cm)
#line(length: 100%, stroke: 0.4pt + luma(180))
#v(0.5cm)

// ── Kortlægning: approved survey types ───────────────────────────────────────

#let survey = data.survey

#text(size: 13pt, weight: "bold")[Kortlægning]
#v(0.2cm)
#if survey.blocking > 0 [
  #text(size: 9pt, style: "italic")[Udkast — #survey.blocking type(r) afventer gennemsyn eller prøvesvar, eller mangler tons, og indgår ikke i mængderne.]
  #v(0.2cm)
]
#if survey.rows.len() == 0 [
  _Ingen godkendte typer endnu._
] else {
  table(
    columns: (1.9fr, 1.5fr, 0.9fr, 0.9fr, 0.7fr, 1.35fr, 1.1fr),
    stroke: 0.3pt + luma(190),
    inset: (x: 5pt, y: 5pt),
    fill: (col, row) => if row == 0 { luma(215) } else { white },
    table.header([*Type*], [*BIM7AA*], [*EAK*], [*Mængde*], [*Tons*], [*Behandling*], [*Miljø*]),
    ..survey.rows.map(r => (r.name, r.bim7aa, r.eak, r.quantity, r.mass, r.treatment, r.environment)).flatten(),
  )
}
#if survey.circularity.len() > 0 [
  #v(0.2cm)
  #text(size: 9pt)[Cirkularitet (godkendte typer): #survey.circularity.map(c => c.label + " " + c.tonnes).join(" · ")]
]
#v(0.6cm)

// ── Ressourcetabel: resources through a chosen template ──────────────────────

#let res = data.at("resources", default: none)
#if res != none [
  #text(size: 13pt, weight: "bold")[Ressourcetabel — #res.name]
  #v(0.2cm)
  #if res.tables.len() == 0 [
    _Ingen ressourcer i projektet._
  ] else {
    for t in res.tables {
      table(
        columns: t.headers.len(),
        stroke: 0.3pt + luma(190),
        inset: (x: 5pt, y: 5pt),
        fill: (col, row) => if row == 0 { luma(215) } else { white },
        table.header(..t.headers.map(h => [*#h*])),
        ..t.rows.flatten(),
      )
      v(0.3cm)
    }
  }
  #v(0.6cm)
]

#text(size: 13pt, weight: "bold")[Materialepas]
#v(0.2cm)

// ── Material table ────────────────────────────────────────────────────────────

#if mats.len() == 0 [
  #align(center)[_Ingen materialer i projektet._]
] else {
  // Build header cells: thumbnail + one cell per user-defined column.
  let header_cells = (
    table.cell(fill: luma(215), align: center)[*Billede*],
  ) + cols.map(col =>
    table.cell(fill: luma(215), align: center)[*#col.name*]
  )

  // Build body cells: one thumbnail + one property cell per column per row.
  let body_cells = ()
  for mat in mats {
    let thumb_cell = if mat.has_thumbnail {
      table.cell(align: center)[
        #image(mat.thumbnail_path, width: 2.8cm, height: 2.2cm, fit: "contain")
      ]
    } else {
      table.cell(fill: luma(245), align: center)[—]
    }
    body_cells = body_cells + (thumb_cell,)
    for col in cols {
      let val = mat.properties.at(col.name, default: "")
      body_cells = body_cells + (table.cell[#val],)
    }
  }

  // Column widths: fixed thumbnail + 1fr per user column.
  let col_widths = (2.9cm,) + cols.map(_ => 1fr)

  table(
    columns: col_widths,
    stroke: 0.3pt + luma(190),
    inset: (x: 5pt, y: 6pt),
    fill: (col, row) => {
      if row == 0 { luma(215) }
      else if calc.odd(row) { luma(250) }
      else { white }
    },
    ..header_cells,
    ..body_cells,
  )
}
