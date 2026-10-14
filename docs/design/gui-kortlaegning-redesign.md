<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# GUI redesign: prototype v2 + Kortlægning workbench

Status: **implemented** (2026-10-02; phases 1–6 complete; proposed 2026-09-30). Anchor: #265 (GUI application).

> **2026-10-02 (resources/templates redesign):** this doc is kept as the
> record of what shipped in Phases 1–6, but two of its screens were since
> folded into others — see the resources/templates design doc (§8,
> removed 2026-10-14, see git history and `docs/DIRECTION.md`'s
> 2026-10-02 changelog entry).
> **On-site** was removed from `rux gui` and moved to the mobile-app track
> (same §8, and the note under "Sager and On-site screens" below).
> **Materialedata** and **Eksport** were retired: their content now lives in
> Kortlægning (resources replace materials) and Rapport (CSV + PDF
> Ressourcetabel export) respectively.

## Source

A colleague's clickable prototype, *ReUseX Overblik — prototype v2*
(claude.ai artifact `At7n5is3zp7cYX54faGJkn`). It defines one committed visual
identity and six screens:

| # | Screen | What it is |
|---|---|---|
| 01 | **Sager** | Case list: one card per building survey |
| 02a | **Overblik** | Case dashboard: hero, KPIs, circularity bar, quick links, BBR line |
| 02b | **Kortlægning** | The review workbench — the focus of this redesign |
| 02c | **Miljø & prøver** | Environmental samples that gate approval |
| 02d | **Rapport** | Ressourcekortlægning report versions |
| 02e | **Indberetning** | Waste fractions per EAK code for bygningsaffald.dk |
| 03 | **On-site** | A phone "interruption" sheet during capture |

Screenshots of the prototype, rendered headlessly, are kept in
`docs/gui/images/prototype-v2/` for reference. `rapport.png` and
`indberetning.png` were added in Phase 5, rendered the same way from the
artifact. `onsite.png` was added in Phase 6, rendered at the prototype's
phone size (390×844) from the artifact's `03 On-site` screen.

## Decisions (agreed with the maintainer, 2026-09-30)

1. **Scope: everything in the prototype**, delivered in phases (below).
2. **Themes: light is the default**, the prototype's palette. A navy-derived
   **dark theme is kept** and the Light / Dark / System toggle keeps working.
   The 3D viewport canvas stays near-black in both themes (the prototype does
   the same — its point-cloud stage is `#171C24`).
3. **Language: Danish**, using the prototype's copy, on every screen this
   redesign touches. Technical tool screens not redesigned here (Pipeline,
   Frames, Graph View, …) stay English until they are.
4. **Data: backend fields too.** Kortlægning, Miljø & prøver and Indberetning
   get typed storage in `ProjectDB`, library entry points in `libs/reusex`
   (Library-first, DIRECTION.md), REST endpoints in `rux gui`, and entries in
   `docs/gui/openapi.yaml`. Nothing is smuggled into free-form passport
   properties.

## Visual identity

The token system stays the mechanism: the prototype's values go into
`src/tokens.css`, components keep using `var(--…)` only.

- **Palette (light, default)** — navy chrome `#1D2D3D` / `#24374B` /
  `#35495E`, workbench `#E9E9E7`, surfaces `#FFFFFF` / `#F2F2F0`, ink
  `#1C2530` / `#5A6470` / `#8B939D`, line `#D8D8D4`, accent `#5980A6` /
  `#3E5F80` / `#E4EBF2`.
- **New token roles** the old system could not express (names owned by the
  repo, values by the design project):
  - `--color-chrome`, `--color-chrome-raised`, `--color-chrome-border`,
    `--color-on-chrome`, `--color-on-chrome-muted` — the navy sidebar/title bar.
  - `--color-accent-deep` — accent text on light accent fills.
  - Tone pairs for pills/notices: `--tone-good-bg|ink`, `--tone-warn-bg|ink`,
    `--tone-wait-bg|ink`, `--tone-crit-bg|ink`, `--tone-accent-bg|ink`.
  - `--color-star` — the "vigtig" marker.
  - Waste hierarchy (affaldshierarki), a categorical scale:
    `--circ-bevaring`, `--circ-genbrug`, `--circ-genanvendelse`,
    `--circ-nyttiggoerelse`, `--circ-bortskaffelse`.
  - `--font-display` — condensed display face for headings and big figures.
- **Type** — display: Oswald 500/600/700, uppercase headings; body: Archivo
  400–700 at 13.5px base. **Self-hosted** via `@fontsource/*` (OFL-1.1), never
  from Google Fonts: `rux gui` is local-first and must render offline.
- **Chrome** — a 44px navy title bar and a 13.5rem navy sidebar: `PROJEKT`
  label, project name, the case navigation with count badges, a "Værktøjer"
  group for the existing technical screens, and "← Alle sager" at the bottom.
- The light theme is the `:root` block; `[data-theme='dark']` overrides it.
  With no stored preference the app now starts **light**.
- Correctness constraints kept: `--color-canvas` near-black in both themes;
  `--label-0..7` stays the Okabe-Ito set.

## Information architecture

| Route | Screen | Replaces |
|---|---|---|
| `/sager` | Sager | — (new) |
| `/` | Overblik | Dashboard (Overview) |
| `/kortlaegning` | Kortlægning | Materials as the primary materials view |
| `/viewport` | Viewport | moved from the Værktøjer group (2026-10-02) |
| `/miljoe` | Miljø & prøver | — (new) |
| `/rapport` | Rapport | report part of Export |
| `/indberetning` | Indberetning | — (new) |
| `/skabeloner` | Skabeloner | template CRUD, split out of Export (2026-10-02) |
| `/on-site` | On-site | — (new, phone layout) (removed 2026-10-02 → mobile app) |
| `/graph-view`, `/pipeline`, `/pipeline/log`, `/frames`, `/geometry`, `/instances`, `/labels` | Værktøjer group | unchanged, restyled by the tokens only |

> **2026-10-02:** `/materials` and `/export` were retired by the
> resources/templates spec (§3) — both now redirect (`/materials` to
> Kortlægning, `/export` to Rapport) rather than rendering their own page.
> The free-form MaterialEPAS passport spreadsheet `/materials` once offered
> no longer exists as a distinct tool now that resources carry the passport
> fields. `Viewport` moved out of Værktøjer into the Sag group, alongside the
> new `/skabeloner` — both rows are above.

> **2026-10-06 (Kortlægning fixes, A1–A2):** `/projektdata` is retired too —
> its content (the technical inventory: point clouds, meshes, components,
> schema version, project path, **Seneste aktivitet**, and the Punktskyer /
> Meshes / Komponenter pr. type tables) moved into a closed-by-default
> **"Projektdata"** disclosure at the foot of Overblik (`ProjectData.tsx`).
> `Dashboard.tsx` is deleted; `/projektdata` now redirects to `/`
> (`navigation.ts` `REDIRECTS`). Separately, the **Værktøjer** nav group is
> now collapsible (closed by default, auto-opens on a tools route, choice
> remembered per viewer in localStorage) rather than a flat always-open list.

**Sager and one project per server.** `ruxd --local` (formerly `rux gui`)
serves one `.rux`. The Sager screen lists the open project as its card and
says how to open another (`ruxd --local <fil>.rux`). Listing and switching
between many cases is the multi-case ruxd's job (the ruxd-multiuser design
doc, phase S2, removed 2026-10-14, see git history and `docs/DIRECTION.md`'s
2026-10-08 changelog entries); the screen is built so a longer list drops in
without layout change.

## Kortlægning domain model

The prototype's two-level table:

- **Type** (group row) = a new **survey type** entity. It groups many parts —
  "Vinduespartier, aluminium" with RX-008 and RX-009. It cannot be a material
  passport: `rux create materials` writes one passport *per instance*, so a
  passport-as-type would always have exactly one part.
- **Bygningsdel** (child row, `RX-###`) = one **instance** (`instances`), with
  a per-part quantity and room. Its material passport, when the instance has
  one (`instance_materials`), is shown alongside.

New storage (schema v22):

- `survey_types` — name, EAK code, BIM7AA code, unit, treatment (bevaring |
  genbrug | genanvendelse | nyttiggoerelse | bortskaffelse), review status
  (queue | approved | rejected), AI confidence (nullable), mass in tonnes
  (nullable), process note, starred, and the semantic class it was seeded from.
- `survey_parts` — code `RX-###` (primary key), type, instance (by its stable
  GUID — survives `rux create instances` re-runs; nullable so a manually
  added part needs no instance), room id + name, quantity, starred, note.
- `samples` — code `P-##`, title, what was sampled, stage (planlagt |
  udtaget | sendt | svar), result (null | ren | forurenet), and — from
  schema v24 — part code: the bygningsdel it was taken at, set when it is
  registered on site (not a foreign key; it may outlive the part).
- `sample_links` — many-to-many sample ↔ survey type. (Originally written as
  "passport"; miljøstatus and the approval gate are properties of a type, so
  the link is to the type — as implemented in Phase 2.)

A dangling `samples.part_code` — left behind when a survey type's parts are
cascade-deleted — is a deliberate exception to STANDARDS §3.2 (no silent
orphaning): the sample keeps the code rather than being rewritten or
rejected, and Miljø shows it as plain text instead of a link when the part no
longer exists.

The same holds when the part outlives its link: a sample's type links stay
user-editable in Miljø, so unlinking the type the part belongs to is allowed.
`Udtaget ved` then still names the part — the code records where the sample
was taken, not which types it currently covers.

Derived, never stored (pure library functions, unit-tested):

- **Miljøstatus** of a type: no linked samples → *ren (screening)*; any linked
  sample with result *forurenet* → *forurenet*; else any sample before stage
  *svar* → *afventer prøve*; else *ren (prøvesvar)*.
- **Approval gate**: a type whose miljøstatus is *afventer prøve* cannot be
  approved — the library throws `SamplePendingError`, the API answers
  `422 Unprocessable Entity`. (Not 409: in this API 409 already means "a
  pipeline job holds the writer lock, retry later", and this is not retryable.)
- **Quantity redistribution**: setting a type's quantity scales its parts
  proportionally (equal split when they are all zero), two decimals, with the
  rounding remainder on the last part.
- **Circularity breakdown**: tonnes per treatment over non-rejected types.
- **Fractions** (Indberetning): approved tonnes per (EAK code, behandling,
  contaminated). *Bevaring* never counts: it stays in the building, so it is
  not waste. A type awaiting a sample is withheld even when approved.
  Contaminated tonnes get their own row. The **blocking list** holds every
  non-rejected type still in the queue (`review`), awaiting a sample
  (`sample`), or approved without tonnes (`mass`), and the report is ready
  when that list is empty and there is at least one fraction row (an empty
  survey has nothing to report).

Rejecting a type ("Afvis — fejldetektion") sets status *rejected*; it is hidden
from Til gennemsyn / Godkendt / Alle, kept visible in its own **Afvist** tab
(2026-10-06, spec A3), and not deleted, so it can be restored ("Genåbn", back
to the queue) or, separately, deleted outright ("Slet" — a destructive,
two-click-armed action distinct from reject, available in every tab for a
whole type or a single part).

**Populating the survey.** A new idempotent library entry point
`sync_survey(ProjectDB&)` creates a `survey_parts` row for every instance that
lacks one, filed under a survey type per semantic class (created on first use,
named from the semantic cloud's label definitions), with the next `RX-###`
code and the instance's room (from the `rooms` label cloud, majority vote over
the instance's points). It never overwrites user edits and never moves a part
a user re-filed. It is exposed as `rux create survey` and
`POST /api/v1/survey/sync`.

**Evidence renders.** Plan / Punktsky / Rum-model images come from the existing
headless `render_view()` with a new instance-highlight option. The API
library (`rux_gui_lib` then, `ruxd_api_lib` now) must not link VTK (same rule
as for SAM3), so the app injects an
`IViewRenderer`, exactly like `IFrameSegmenter`; without one the endpoint
answers 503.

## Kortlægning screen (the prototype, component by component)

- **Header** — `KORTLÆGNING` + sub "Ressourcekortlægning · bygningsdele pr. rum,
  grupperet pr. type" + `Eksport (XLS)` (the existing CSV export endpoint,
  labelled for spreadsheet use).
- **Coverage notice** (warn tone) — unclassified points from the instance
  label cloud and rooms without scan coverage, from `GET /survey/summary`.
- **Workbench** — table panel left, 21.5rem right column (evidence + detail);
  stacks below 1080px.
- **Tabs** — Til gennemsyn (n) · Godkendt (n) · Alle (n) · **Afvist (n)**
  (2026-10-06, spec A3; rejected types are excluded from the other three tabs
  as before, but are no longer hidden outright — they live here, with
  "Genåbn" back to the queue and, separately, "Slet").
- **Tools** — search (Søg bygningsdel…), room filter (Alle rum), miljø filter
  (Al miljøstatus / Ren / Afventer prøve / Forurenet).
- **Keyboard bar** — ↑↓ naviger · →← fold ud/ind · Enter åbn redigering ·
  G godkend · A afvis · V vigtig · 1–5 evidens · Esc tilbage (2026-10-06: a
  fifth evidence key, see below).
- **Table** — columns Betegnelse · Mængde · EAK · BIM7AA · Behandling · Miljø ·
  Status. Group rows: chevron, ★, name, "n dele", quantity + unit + tonnes.
  Child rows: `RX-### · Rum`, quantity, EAK, photo count. Treatment as a
  circularity-coloured pill, miljø as a tone pill, status as a confidence bar
  or "Godkendt ✓". Sticky header, keyboard-focusable, selected row with an
  accent inset bar.
- **Evidence panel** — tabs **Plan · 360° · Foto · Punktsky · Rum** (keys 1–5,
  2026-10-06, spec A5 — a real 360° tab replaces the earlier Foto substitute):
  - *Plan*: server-rendered floor plan (`render_view`, `plan` preset) with the
    selected instance highlighted.
  - *360°*: the nearest placeable panorama (`GET /instances/<cloud>/<id>/panoramas`),
    preferring a **resected** one (heading measured by `rux align 360`) within
    range over a merely levelled one (position borrowed from a matched frame,
    heading unknown); shown as a pannable equirect strip centred on the
    part's `u`, with a marker at `(u,v)` only for a resected panorama — a
    levelled one is captioned "<rum> · 360° · retning ukendt" with no marker
    — and a link "Åbn i viewport". Empty state: "Ingen 360°-optagelse nær
    denne ressource".
  - *Foto*: a regular sensor-frame photo ("Bedste foto") of the selected
    part's instance — or of the first linked part's instance, for a type row.
  - *Punktsky*: server-rendered orbit view of the cloud, instance highlighted
    (the "Åbn i viewport" link is only wired for 360° so far — see
    follow-ups).
  - *Rum*: server-rendered view of the `rooms` layer.
- **Detail panel** — title, BIM7AA pill, miljø pill, ★ pill; Mængde (editable;
  on a type it is redistributed proportionally over the parts), EAK, Behandling
  (select), Sikkerhed (AI); sample line, each sample a link to its card in
  Miljø & prøver (or `Registrér prøve` when none is linked); Proces /
  håndtering note;
  ☆ Markér vigtig, Afvis, Godkend mængde ✓ (disabled with a gate note while a
  sample is pending), Genåbn, and (2026-10-06) **Slet** — a separate,
  destructive, two-click-armed action ("Slet type" / "Slet ressource", then
  "Bekræft: …") available in every tab, for a whole type (and all its parts)
  or for any single part.
- **Edit dialog** (Enter / double-click) — navy header with prev/next/close;
  left form (part chips, quantity, EAK, behandling, sikkerhed, sample line,
  note, photos = the instance's best frames, ☆); right evidence with five
  numbered thumbnails (2026-10-06: was four, now includes 360°) and a large
  stage; footer shortcuts, gate note, Afvis, Godkend & næste ✓. ⌘/Ctrl+Enter
  approve-and-next, PgUp/PgDn move, 1–5 switch view, Esc close.
- **Toast** after approve: "✓ <type> godkendt · n tilbage i køen". After
  reject (2026-10-06): "Afvist som fejldetektion — flyttet til Afvist".

## Miljø & prøver screen (the prototype, component by component)

- **Chrome** — the same navy title bar and sidebar; Miljø & prøver carries a
  neutral count badge (not the "hot" accent Kortlægning's review queue uses).
- **View head** — `MILJØ & PRØVER` + sub "Prøver styrer miljøstatus på de
  koblede bygningsdele", and `+ Ny prøve`.
- **Sample cards** — a vertical stack, one per sample: a title row (code +
  title, a stage/result pill, "Koblet: <types>"), then what was sampled on
  its own line, a
  stage chain (Planlagt — Udtaget — Sendt til lab — Svar modtaget) and an
  action row that depends on stage: *sendt* offers `Registrér svar: Ren` /
  `Registrér svar: Forurenet`; *udtaget* offers `Næste trin →`; *svar* with a
  result shows only a note that miljøstatus has been updated.
- **Footnote** — factual, not a sketch note: a prøvesvar updates miljøstatus
  on every linked type at once; lab integration (e.g. Milva) is a follow-up.

v1 adds, beyond the prototype:

- an inline `Rediger` editor (title, what, links) opened from the `Koblet:`
  line, since the prototype has no list/detail split or link editor;
- `Fortryd svar`, the one backward step the screen offers: it returns a
  sample from *svar* to *sendt* with no result — not to *svar* with no
  result, which would otherwise count as clean and silently un-gate its
  types;
- recording a result as one combined patch, `{stage: 'svar', result}`,
  checked by the backend against the merged state;
- a toast after every sample change reporting the approval-gate effect (how
  many types were un-gated or re-gated by the change);
- deep links both ways: `?sample=<id>` scrolls to and highlights a card,
  `?ny=<typeId>` opens the create form pre-linked to that type, and a linked
  type's name on a card opens `/kortlaegning?type=<id>`;
- the badge is `pending_samples` (from `GET /survey/summary`) — samples not
  yet at stage *svar* — never a client-side count.

## Overblik, Rapport and Indberetning screens (the prototype, component by component)

**Overblik** — the case hero (name, address, year, registration date,
organisation — only the fields the record has, R5), its in-place editor
(click the hero to open `ProjectMetaForm`; name/address/notes commit on
blur, the year field validates and reverts on an invalid draft, Esc reverts
without saving); a five-tile KPI row (Komponenter, Klassificeret, Bevaring /
genbrug, Til gennemsyn, Prøver afventer); the circularity bar; and quick
links to Kortlægning, Miljø & prøver, Rapport and Indberetning, each with a
live sub-line. v1 changes from the prototype:

- the KPI **Klassificeret** (share of the instance cloud's points that carry
  an instance label) stands in for the prototype's **Scanningsdækning**,
  which nothing in the project measures (R4; a real scan-coverage measure is
  a follow-up, `.github/issue-drafts/35-scan-coverage-kpi.md`);
- Indberetning's quick-link sub-line says "n typer blokerer" from
  `GET /survey/fractions` whenever anything blocks — approved types can still
  block (a pending sample, no tonnes) — and "n af m typer godkendt" otherwise;
- no aerial photo, sync chip, case number, MRK or demolition deadline — none
  of that is stored yet (R5, a follow-up); correspondingly no BBR line (R6,
  shown only once a BFE number can be stored);
- the hero's in-place editor is new: the prototype's hero is static;
- the old Dashboard/inventory screen moved from `/` to `/projektdata`, a
  `Værktøjer` entry, so Overblik could take the landing route — and (2026-10-06,
  spec A2) `/projektdata` was retired in turn: its content is now the
  **"Projektdata"** disclosure at the foot of Overblik itself (see the
  information-architecture note above), so the content that started at `/`
  has, after two moves, landed back on it.

**Rapport** — Ressourcekortlægning report versions, newest first: each one
numbered `v<n>` with its generation date, file size and a `Komplet`/`Udkast`
pill; `Generér ny version` (disabled while another version is generating;
while a pipeline job holds the writer lock it stays enabled, and a click gets
the server's 409 and a toast saying to try again); a draft notice naming how
many types still block completeness; and an Inventarliste download. v1
changes:

- versions are numbered `v<n>` and marked `Komplet`/`Udkast` from the
  **stored** blocking-type count at generation time (schema v23,
  `ReportPdfVersion.blocking_types`), not inferred client-side — a version
  generated before that column existed shows "Ukendt status" instead of
  guessing;
- the PDF itself opens with the approved survey as its first section (the
  kortlægning data `Create survey` populates), ahead of the material-passport
  columns Export already produced;
- Inventarliste is **not** a stored version: it is the live CSV export
  (`/exports/csv`), downloaded fresh every time, since a bygningsdel list
  changes between report generations;
- the hero's figures are the whole survey (every non-rejected type), while
  the PDF reports approved, reportable types only; the hero is captioned
  "Hele kortlægningen (inkl. ikke-godkendte)" so the two are not confused;
- no MRK signature or approval workflow on a version — nothing models
  approval yet (R7, a follow-up, `.github/issue-drafts/34-report-approval-xls-version.md`).

Known edge, not a defect: generating a version from a survey with no
reportable rows (no types at all, or none approved yet with nothing blocking)
records `blocking_types: 0`, so the version is marked `Komplet` though its
survey section is empty. Nothing blocks, so it is not a draft either; the
Indberetning send gate, which needs at least one fraction, stays shut.

**Indberetning** — a fraction table (EAK code, behandling, mass in tonnes,
contaminated flag), a blocking-list notice while types are still in the
queue or awaiting a sample, a CSV download of the ready fractions, and
`Send til bygningsaffald.dk`. v1 changes:

- the send button is gated like the prototype's (disabled while any type
  blocks the report, and also while there is no fraction to report) but,
  clicked, **posts nothing** — it shows a notice saying so, since no
  bygningsaffald.dk integration exists (R9, out of scope below;
  `.github/issue-drafts/33-bygningsaffald-submission.md`);
- the CSV download is new: the prototype has no export affordance on this
  screen.

## Sager and On-site screens (the prototype, component by component)

> **2026-10-02:** On-site was removed from `rux gui` and its backend
> (`samples.part_code` dropped in schema v25) and moved to the mobile-app
> track — see the resources/templates design doc §8 (removed 2026-10-14,
> see git history).
> The description below is kept as the design record for that app.

**Sager** — a case list: one card per building survey, the fixture name and
address, the survey's figures, a status pill and a deadline. **On-site** — a
phone "interruption" sheet during capture: a live camera viewfinder with a
reticle on the detected object, a detection chip, and a bottom sheet with
★ / note / `+ Tilføj ekstra foto` / `Registrér prøve` actions plus
`Videre → RX-###`.

v1 changes:

- **Sager:**
  - one card for the project the server was started with, linking to
    Overblik;
  - a plan render as its thumb (the striped status thumb when the render
    fails);
  - the survey's figures, and a status derived from the summary and the
    fractions (`Kladde`, `Gennemgang`, `Klar til indberetning`,
    `Gennemgået`);
  - the registration date in place of the deadline, which is not stored;
  - no `+ Nyt projekt` — instead an `Åbn en anden sag` panel with
    `ruxd --local <fil>.rux` and, for a phone, the
    `--bind 0.0.0.0 --auth-token <token>` recipe (open
    `http://<host>:8420/?token=<token>`);
  - the prototype's top tabs are not built; `← Alle sager` and an `On-site`
    sidebar entry replace them.
- **On-site:**
  - a walk through the stored bygningsdele (rooms in Danish order, then
    code; `?del=RX-###`);
  - the part's best sensor-frame photo with the reticle on its instance in
    place of a live camera;
  - the sheet writes ★ and the note to the part and registers a sample
    there (`POST /samples` with `part_code`, `stage: udtaget`; the part's
    type is always linked);
  - `Tilføj ekstra foto` is not drawn (no blob storage);
  - `Videre → RX-###` moves on, replacing the history entry;
  - Kortlægning gains a `Kun vigtige ★` filter and ★/✎ part markers, and a
    Miljø card says `Udtaget ved RX-### · <rum>`.
- **Shell:** below 900px the sidebar is a drawer behind `Menu`, the title
  bar drops its meta, and no route overflows at 390px.
- **Writes:** one app-wide chain. The case screens' first loads wait for
  it; Rapport keeps its own.

## Phases

Each phase is a separate PR that leaves the app working.

1. **Identity & shell** — tokens, fonts, theme default, navy title bar and
   sidebar, routes, Danish labels; existing screens restyled by tokens.
2. **Survey backend** — schema v22 (survey types, parts, samples, links),
   library model + derivations + approval gate, `sync_survey`,
   `rux create survey`, REST endpoints, render endpoint with instance
   highlight, openapi.yaml, and the frontend contract layer (types + client).
3. **Kortlægning frontend** — the workbench, detail panel, edit dialog,
   keyboard flow, evidence panel.
4. **Miljø & prøver** — the samples screen (stage chain, links, result entry)
   on the Phase 2 API.
5. **Overblik, Rapport, Indberetning** — KPIs, circularity, report versions on
   the existing report endpoints, fraction table and send gate. (done in
   Phase 5)
6. **Sager & On-site** — case list (single-project), phone capture sheet
   writing ★ / note / sample against a bygningsdel. (done in Phase 6) —
   later removed from rux gui; On-site moved to the mobile-app track
   (2026-10-02)

## Out of scope / follow-up issues

- Multi-project case list and switching, and creating a project from the GUI
  (`ruxd`, #265 Phase 6; `.github/issue-drafts/37-multi-case-list-ruxd.md`).
- Pairing or authentication for opening `rux gui` from a phone on the LAN;
  v1 documents `--bind` + `--allow-origin` and warns
  (`.github/issue-drafts/38-lan-pairing-auth.md`).
- bygningsaffald.dk API submission (v1 produces the numbers in the portal's
  structure; the send button is gated but posts nowhere;
  `.github/issue-drafts/33-bygningsaffald-submission.md`).
- BBR lookup (the prototype marks it "Demodata"; shown only when project
  metadata carries a BFE number).
- Lab integration (e.g. Milva) for automatic sample results.
- Pushing the new token values to the Claude Design project ("ReUseX GUI") with
  `/design-sync` after Phase 1 merges — a maintainer-initiated step.
- ~~A nearest-panorama endpoint, so the evidence panel's Foto tab can become a
  true 360° tab instead of substituting a sensor-frame photo~~ — done
  2026-10-06 (spec A5): `GET /instances/<cloud>/<id>/panoramas` and the
  evidence panel's 360° tab.
- ~~Linking the detail panel's sample line to the Miljø & prøver screen~~ —
  done in Phase 4: the sample line is now a link to each linked sample's card
  (or `Registrér prøve` when none is linked).
- A survey-specific export — "Eksport (XLS)" currently downloads the existing
  material-passport CSV (`/exports/csv`), not a Kortlægning-shaped spreadsheet.
- Two pieces of the structure above that v1 did not draw yet: ~~the child
  rows' photo count~~ — done 2026-10-06 (spec A4: "n fotos" per part row,
  `photoCountText`) — and the Punktsky tab's "Åbn i viewport" link (still
  missing; only the 360° tab has it so far).
- A `--border-width` token: the tab underline offset in `SurveyTable.module.css`
  and `EvidencePanel.module.css` currently computes it as `calc(-1 * 1px)`
  because no border-width token exists yet.
- Uploading the lab's miljørapport (PDF) to a sample — needs blob storage and
  an endpoint; the prototype's `Upload miljørapport (PDF)` button is not drawn
  until then.
- Extra photos from the phone, stored against a part and shown in the
  evidence panel — needs the same blob storage as the miljørapport upload
  (`.github/issue-drafts/36-onsite-part-photos.md`, with
  `22-miljoerapport-pdf-upload.md`).
- On-site during a live capture: the sheet as an interruption while the scan
  runs, with a detection chip from live segmentation
  (`.github/issue-drafts/39-onsite-live-capture.md`).
- An offline outbox for On-site, so writes made without signal are kept and
  sent later (`.github/issue-drafts/40-onsite-offline-outbox.md`).
- Rewinding a sample's stage beyond `Fortryd svar` (back to *sendt*)
  (`.github/issue-drafts/23-sample-stage-rewind.md`).
- Kortlægning's selection is not in the URL, so Back after a cross-screen link
  returns to row 0. Keep `?type=<selected>` updated with `replace`
  (`.github/issue-drafts/31-kortlaegning-selection-not-in-url.md`).
- Case identity in project metadata: BFE number, case number
  (`RX-2026-0047`), MRK and the demolition deadline — schema, `PATCH
  /projects`, `rux set` — then Overblik's hero line and the BBR line with a
  verified BBR link. Until then BBR is not drawn
  (`.github/issue-drafts/24-case-identity-metadata-bbr.md`).
- The client (bygherre) in case metadata, for the Sager card and the
  Overblik hero (`.github/issue-drafts/41-case-client-bygherre.md`).
- Esc in the Kortlægning edit dialog still closes *and saves* (the closing
  blur commits); every other editor drops the draft (Phase 5 R10)
  (`.github/issue-drafts/26-editdialog-esc-commits.md`).
- Report approval (`Godkendt` / MRK signature on a version) and a stored XLS
  inventory version (`.github/issue-drafts/34-report-approval-xls-version.md`).
- A scan-coverage measure for the building, so Overblik can show the
  prototype's `Scanningsdækning` instead of `Klassificeret`
  (`.github/issue-drafts/35-scan-coverage-kpi.md`).
- The Phase 6 regression checks for the responsive shell (R5), the app-wide
  write chain (R11) and the cross-link style (R12) were run as uncommitted
  Playwright scripts, not from the repo. Committing them as a suite is a
  follow-up; the draft lists what they cover and the gaps they found
  (`.github/issue-drafts/42-commit-playwright-suite.md`).
