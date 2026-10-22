// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Ressourcekortlægning og nedrivningsplan (BEK 496/2024) report template.
//
// Data contract (data.json + images/): docs/guides/report-template.md.
//
// The data directory comes from `--input data-dir=DIR`; it defaults to the
// directory holding this file, which is how the generator runs it:
//   typst compile report.typ out.pdf --root <tmpdir>                    (generator)
//   typst compile report.typ out.pdf --root / --input data-dir=<dir>    (report-build)
// Typst refuses a source file outside --root (symlinks are resolved first),
// so the preview scripts use --root / rather than linking this file into
// the data directory.
//
// Sections, in the GUI preview's order (rux-frontend src/bygning/pages.ts):
// Forside, Sagsdata, Ressourcekortlægning, Miljøkortlægning, Prøver & bilag,
// Nedrivningsplan, then the Materialepas appendix. Each section starts on a
// new page and carries a `<section>` metadata label, so `typst query` can
// map sections to pages.
//
// This file is the only copy: CMake embeds it into the generated header
// reusex/core/report_template_default.hpp at build time.

#let data_dir = sys.inputs.at("data-dir", default: ".")
#let data = json(data_dir + "/data.json")
#let cols = data.columns
#let mats = data.materials
#let survey = data.survey

// Fields added with the sectioned layout (reusex-ufcm). A data.json from an
// older generator lacks them; the template still compiles.
#let project = data.at("project", default: (name: data.project_name))
#let pget(key) = project.at(key, default: none)
#let report = data.at("report", default: (version: none, status: none, date: none))
#let types = data.at("types", default: ())
#let samples = data.at("samples", default: ())
#let appendices = data.at("appendices", default: ())
#let steps = data.at("demolition_steps", default: ())
#let sagsdata = data.at("sagsdata", default: ())
#let cover_path = data.at("cover_path", default: "")
#let is_draft = survey.blocking > 0

// ── Look ─────────────────────────────────────────────────────────────────────

#let ink = luma(25)
#let muted = luma(110)
#let rule = luma(190)
#let head_fill = luma(228)
#let zebra = luma(247)

#let project_title = {
  let n = pget("name")
  if n == none { data.project_name } else { n }
}

#set document(title: "Ressourcekortlægning — " + project_title)
#set text(size: 10pt, lang: "da", fill: ink)
#set par(justify: false, leading: 0.62em)
#set table(stroke: 0.3pt + rule, inset: (x: 5pt, y: 4.5pt))

/// A table's repeating header row, bold.
#let th(..cells) = table.header(..cells.pos().map(c => text(weight: "bold", size: 9pt, c)))
/// Wide, data-driven tables (Ressourcetabel, Materialepas): smaller type,
/// hyphenated, so long Danish compounds wrap instead of overflowing a cell.
#let dense(body) = {
  set text(size: 8.5pt, hyphenate: true)
  body
}

#set page(
  paper: "a4",
  margin: (top: 2.4cm, bottom: 2.6cm, left: 2cm, right: 2cm),
  header: context {
    if counter(page).get().first() > 1 {
      set text(size: 8pt, fill: muted)
      project_title
      h(1fr)
      [Ressourcekortlægning og nedrivningsplan]
      if is_draft [ · *UDKAST*]
      v(-0.5em)
      line(length: 100%, stroke: 0.3pt + rule)
    }
  },
  footer: context {
    set text(size: 8pt, fill: muted)
    if report.version != none [Version #report.version · #report.date] else [#data.generated_at]
    h(1fr)
    [Side #counter(page).display() / #counter(page).final().at(0)]
  },
)

#show heading.where(level: 1): it => {
  set text(size: 17pt, weight: "bold")
  block(above: 0pt, below: 0.7em, it.body)
}
#show heading.where(level: 2): it => {
  set text(size: 12pt, weight: "bold")
  block(above: 1.3em, below: 0.6em, sticky: true, it.body)
}

/// A missing value: unmistakable on paper and in greyscale.
#let missing(body: "mangler") = text(fill: muted, style: "italic")[#body]
#let or_missing(v, body: "mangler") = if v == none or v == "" { missing(body: body) } else { v }
#let note(body) = text(size: 8.5pt, fill: muted, body)

/// Start a section on a new page and label it for `typst query`.
#let section(id, title, first: false) = {
  if not first { pagebreak(weak: true) }
  [#metadata((id: id, title: title)) <section>]
  heading(level: 1, title)
}

#let header_fill(col, row) = if row == 0 { head_fill } else if calc.even(row) { zebra } else { none }

// ── Forside ──────────────────────────────────────────────────────────────────

#[#metadata((id: "forside", title: "Forside")) <section>]
#v(1.2cm)
#text(size: 9pt, weight: "bold", tracking: 0.08em, fill: muted)[#upper[Ressourcekortlægning og nedrivningsplan]]
#v(0.35cm)
#text(size: 26pt, weight: "bold")[#project_title]
#v(0.2cm)
#{
  let address = pget("building_address")
  let bbr = pget("bbr_number")
  let place = (address, if bbr != none { "BBR " + bbr }).filter(x => x != none)
  if place.len() == 0 { missing(body: "adresse og BBR mangler") } else {
    text(size: 12pt, place.join(" · "))
  }
}
#v(0.7cm)
#if is_draft {
  block(width: 100%, inset: (x: 10pt, y: 8pt), stroke: 1.2pt + ink)[
    #text(weight: "bold")[UDKAST] — #survey.blocking #if survey.blocking == 1 [type] else [typer] afventer
    gennemsyn eller prøvesvar, eller mangler tons, og indgår ikke i mængderne.
  ]
  v(0.4cm)
}
#block(width: 100%, height: 10.5cm, fill: luma(240), clip: true)[
  #if cover_path != "" {
    image(data_dir + "/" + cover_path, width: 100%, height: 100%, fit: "cover")
  } else {
    align(center + horizon, missing(body: "Intet forsidebillede — vælg et foto eller 360° i Rapport"))
  }
]
#v(0.8cm)
#grid(
  columns: (auto, 1fr),
  column-gutter: 1.2em,
  row-gutter: 0.6em,
  text(fill: muted)[Udarbejdet af],
  {
    let by = pget("coordinator")
    if by == none { by = pget("survey_organisation") }
    or_missing(by)
  },
  text(fill: muted)[Bygherre], or_missing(pget("client")),
  text(fill: muted)[Version],
  if report.version != none [
    Version #report.version · #if report.status == "komplet" [komplet] else [kladde] · #report.date
  ] else [#data.generated_at],
)
#v(1fr)
#line(length: 100%, stroke: 0.3pt + rule)
#text(size: 9pt)[
  Jf. BEK 496/2024 om selektiv nedrivning.
  #if pget("demolished_floor_area_m2") != none [Nedrevet etageareal #pget("demolished_floor_area_m2").]
]

// ── Sagsdata ─────────────────────────────────────────────────────────────────

#section("sagsdata", "Sagsdata")
#if sagsdata.len() == 0 [
  #missing(body: "Ingen sagsdata i data.json")
] else {
  table(
    columns: (6.2cm, 1fr),
    stroke: (x, y) => (bottom: 0.3pt + rule),
    inset: (x: 4pt, y: 6pt),
    ..sagsdata.map(r => (text(fill: muted, r.label), or_missing(r.value))).flatten(),
  )
}
#if pget("notes") != none [
  == Noter
  #pget("notes")
]

// ── Ressourcekortlægning ─────────────────────────────────────────────────────

#section("ressourcer", "Ressourcekortlægning")
#{
  let s = data.at("types_summary", default: none)
  if s != none { text(s.line) }
}
#if types.len() == 0 [
  #missing(body: "Ingen typer i kortlægningen endnu.")
] else {
  v(0.2cm)
  table(
    columns: (1.1cm, 1fr, 1.95cm, 2.6cm, 2cm, 1.7cm),
    fill: header_fill,
    align: (x, y) => if x >= 4 and y > 0 { right } else { left },
    th([ID], [Type], [EAK], [Behandling], [Mængde], [Masse]),
    ..types.map(t => (
      text(font: "DejaVu Sans Mono", size: 8pt)[\##t.id],
      if t.approved { t.name } else [#t.name #missing(body: "ikke godkendt")],
      if t.eak == none { missing() } else { text(font: "DejaVu Sans Mono", size: 8pt, t.eak) },
      t.treatment,
      t.quantity,
      if t.mass != none { t.mass } else if t.mass_needed { missing() } else [—],
    )).flatten(),
  )
  note[Kun godkendte mængder indgår i kortlægningen nedenfor og i cirkularitetsopgørelsen.]
}

== Kortlægning — godkendte typer
#if is_draft [
  #note[Udkast — #survey.blocking #if survey.blocking == 1 [type] else [typer] afventer gennemsyn eller prøvesvar, eller mangler tons, og indgår ikke i mængderne.]
  #v(0.15cm)
]
#if survey.rows.len() == 0 [
  #missing(body: "Ingen godkendte typer endnu.")
] else {
  table(
    columns: (1.9fr, 1.5fr, 0.9fr, 0.9fr, 0.7fr, 1.35fr, 1.1fr),
    fill: header_fill,
    th([Type], [BIM7AA], [EAK], [Mængde], [Tons], [Behandling], [Miljø]),
    ..survey.rows.map(r => (r.name, r.bim7aa, r.eak, r.quantity, r.mass, r.treatment, r.environment)).flatten(),
  )
}
#if survey.circularity.len() > 0 [
  #v(0.15cm)
  #text(size: 9pt)[*Cirkularitet* (godkendte typer): #survey.circularity.map(c => c.label + " " + c.tonnes).join(" · ")]
]

#let res = data.at("resources", default: none)
#if res != none [
  == Ressourcetabel — #res.name
  #if res.tables.len() == 0 [
    #missing(body: "Ingen ressourcer i projektet.")
  ] else {
    for t in res.tables {
      dense(table(
        columns: t.headers.len(),
        fill: header_fill,
        th(..t.headers.map(h => [#h])),
        ..t.rows.flatten(),
      ))
      v(0.3cm)
    }
  }
]

// ── Miljøkortlægning ─────────────────────────────────────────────────────────

#section("miljoe", "Miljøkortlægning")
Mistanke om miljøfarlige stoffer og udtagne prøver.
#v(0.2cm)
#if samples.len() == 0 [
  Der er ikke registreret prøver i sagen.
] else {
  table(
    columns: (1.4cm, 1fr, 1fr, 2.4cm, 2.2cm),
    fill: header_fill,
    th([Prøve], [Stof / materiale], [Typer], [Status], [Resultat]),
    ..samples.map(s => (
      text(font: "DejaVu Sans Mono", size: 8pt, s.code),
      [#s.title #if s.what != "" [\ #note(s.what)]],
      or_missing(s.types, body: "ingen type"),
      s.stage,
      if s.result == none { missing(body: "afventer") } else if s.contaminated { text(weight: "bold")[#s.result] } else { s.result },
    )).flatten(),
  )
}

// ── Prøver & bilag ───────────────────────────────────────────────────────────

#section("bilag", "Prøver & bilag")
Laboratorierapporterne vedlægges som bilag.

== Bilag
#if appendices.len() == 0 and mats.len() == 0 [
  Ingen bilag endnu.
] else {
  table(
    columns: (1cm, 1fr, auto),
    stroke: (x, y) => (bottom: 0.3pt + rule),
    inset: (x: 4pt, y: 6pt),
    ..appendices.map(a => (
      text(font: "DejaVu Sans Mono", weight: "bold", a.letter),
      a.text,
      if a.status == none { missing(body: "afventer") } else { text(fill: muted, a.status) },
    )).flatten(),
    ..if mats.len() > 0 {
      (text(font: "DejaVu Sans Mono", weight: "bold")[MP], [Materialepas · #mats.len() #if mats.len() == 1 [materiale] else [materialer]], if cols.len() > 0 { text(fill: muted)[Bagerst i rapporten] } else { text(fill: muted)[Ingen felter valgt til rapporten] })
    } else { () },
  )
}

// ── Nedrivningsplan ──────────────────────────────────────────────────────────

#section("nedrivningsplan", "Nedrivningsplan")
== Rækkefølge
#if steps.len() == 0 [
  Ingen typer i kortlægningen endnu.
] else {
  for (i, s) in steps.enumerate() {
    block(breakable: false, below: 0.8em)[
      #grid(
        columns: (0.9cm, 1fr),
        text(size: 13pt, weight: "bold", fill: muted)[#(i + 1).],
        [*#s.title* \ #text(size: 9pt, s.items.join(", "))],
      )
    ]
  }
}
== Modtagere
#missing(body: "Modtagere af affaldet registreres ikke i ReUseX endnu — angiv dem i et bilag.")

// ── Materialepas ─────────────────────────────────────────────────────────────

// Without a user-defined column there is nothing to tabulate but the
// thumbnails; the Bilag list says how many passports there are instead.
#if mats.len() > 0 and cols.len() > 0 {
  section("materialepas", "Bilag · Materialepas")

  // Thumbnail + one column per user-defined property column.
  let header_cells = ([Billede],) + cols.map(col => [#col.name])
  let body_cells = ()
  for mat in mats {
    let thumb_cell = if mat.has_thumbnail {
      table.cell(align: center)[
        #image(data_dir + "/" + mat.thumbnail_path, width: 2.8cm, height: 2.2cm, fit: "contain")
      ]
    } else {
      table.cell(fill: luma(245), align: center + horizon)[—]
    }
    body_cells = body_cells + (thumb_cell,)
    for col in cols {
      let val = mat.properties.at(col.name, default: "")
      body_cells = body_cells + (table.cell[#val],)
    }
  }
  dense(table(
    columns: (2.9cm,) + cols.map(_ => 1fr),
    fill: header_fill,
    th(..header_cells),
    ..body_cells,
  ))
}
