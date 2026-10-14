// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Server-side PDF generation for Ressourcekortlægning (#456).
//
// The generator assembles material passports, column definitions and thumbnail
// blobs from a ProjectDB, writes them to a temporary directory alongside the
// embedded Typst template, and invokes `typst compile` as a subprocess. The
// resulting PDF bytes are returned so the caller can store them in the DB and
// serve them over HTTP.
//
// Requires `typst` to be present in PATH at runtime. The devshells gain it via
// `pkgs.typst` (shell.nix, devshells/report.nix); in production supply it in
// the process's PATH. Typst runs with --ignore-system-fonts, so the PDF does
// not depend on the host's fonts.

#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace reusex {

class ProjectDB;

/// Options for generate_ressourcekortlaegning_pdf() and dump_report_data().
struct ReportOptions {
  /// Add a Ressourcetabel: every non-rejected resource through this
  /// template's keys (core::resource_report_section).
  std::optional<std::int64_t> resource_template_id;
  /// Typst template to use instead of the embedded one. Empty: the
  /// $REUSEX_REPORT_TEMPLATE environment variable, else embedded.
  std::filesystem::path template_path;
};

/// Generate a Ressourcekortlægning PDF for the given project.
///
/// Writes data.json + images/ (see dump_report_data) and the Typst template
/// into a temporary directory and executes:
///   typst compile report.typ out.pdf --root <tmpdir> --ignore-system-fonts
///
/// The template is ReportOptions::template_path, else the file named by
/// $REUSEX_REPORT_TEMPLATE, else apps/rux/resources/report.typ as embedded
/// at build time. A named template is copied into the temporary directory,
/// so it must be self-contained (no #import of sibling files).
///
/// @returns Raw PDF bytes on success.
/// @throws std::out_of_range when resource_template_id names no template.
/// @throws std::runtime_error when the named template file does not exist
///         or is not a regular file (the message names the path and whether
///         it came from template_path or $REUSEX_REPORT_TEMPLATE). Both are
///         checked before any typst work.
/// @throws std::runtime_error if typst is not in PATH, data assembly fails,
///         or the compilation exits non-zero (typst's error text included).
std::vector<std::uint8_t>
generate_ressourcekortlaegning_pdf(ProjectDB &db, const ReportOptions &options);

/// Backward-compatible form (ruxd): forwards to the ReportOptions overload
/// with only @p resource_template_id set — so $REUSEX_REPORT_TEMPLATE still
/// applies.
std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(
    ProjectDB &db,
    std::optional<std::int64_t> resource_template_id = std::nullopt);

/// How many survey types keep the report from being complete right now: the
/// length of core::fractions_by_eak's blocking list (review, sample or mass).
/// Stored with each PDF version (ProjectDB::add_report_pdf) so the GUI can
/// tell a complete version from a draft.
int report_blocking_types(const ProjectDB &db);

/// Write data.json + images/ into dir exactly as the PDF run sees them.
///
/// The result is what `report-watch DIR` (nix develop .#report) and
/// `rux export pdf --dump-data DIR` work with; the format is documented in
/// docs/guides/report-template.md. ReportOptions::template_path is ignored.
///
/// @p dir may be missing (it is created), empty, or a previous dump: a
/// directory holding nothing but data.json and images/ as top-level
/// entries, where data.json parses as a JSON object with "project_name",
/// "materials" and "survey" keys. A previous dump's images/ is replaced,
/// never merged. Anything else non-empty (an unrelated data.json, extra
/// files, a parse failure) is refused rather than guessed at.
/// @throws std::out_of_range when resource_template_id names no template
///         (nothing is written, @p dir is not created).
/// @throws std::runtime_error when @p dir is a file, a non-empty directory
///         that is not a previous dump (left untouched), or on I/O errors.
void dump_report_data(ProjectDB &db, const ReportOptions &options,
                      const std::filesystem::path &dir);

} // namespace reusex
