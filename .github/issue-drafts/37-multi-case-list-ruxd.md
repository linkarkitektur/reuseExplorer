title: Sager: show a case's running job on its card
labels: gui, enhancement, frontend

## Background
Most of this draft's original scope shipped with the ruxd server work (the
ruxd-multiuser design doc, phases S2 and S3, removed 2026-10-14 — see git
history and `docs/DIRECTION.md`'s 2026-10-08 changelog entries):
- `GET /api/v1/cases` lists cases (id, name, file name, size, archived, the
  caller's role) with each card's survey figures read without opening the
  case, and a server-rendered plan thumbnail (`/cases/{cid}/renders`).
- Every project route is case-scoped (`/api/v1/cases/{cid}/…`), and the
  frontend's case screens live under `/sager/:cid/…`, so switching a case is
  navigation.
- `/sager` creates a case and uploads a `.rux` (chunked `/api/v1/uploads`),
  in `ruxd --local <dir>` and in server mode alike.

## Problem
What is left is the prototype's job-driven status (`Scanning i gang`,
`Afventer upload`). The card's pill is derived from the survey only (Kladde /
Gennemgang / Klar til indberetning / Gennemgået, `src/sager/model.ts`), so a
case whose pipeline is running reads the same as an idle one.

## Proposed fix
- [ ] Carry the case's active job (stage, state, progress) in `GET /cases`,
      from the job store, without opening the case.
- [ ] Show it on the card next to the survey-derived pill, and keep it live
      (events are per case today: a server-level job channel, or polling the
      list while a job runs).
- [ ] An upload in progress (an open `/api/v1/uploads` session of the
      caller's) as a placeholder card.

category=CLI estimate=2d
