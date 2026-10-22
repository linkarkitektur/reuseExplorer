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
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
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
  /// The cover image when the project has none chosen
  /// (core/report_cover.hpp): PNG or JPEG bytes, empty for none. Called at
  /// most once per run, and only then. Core cannot render the model itself
  /// (`visualize` is Layer 4), so rux and ruxd pass a visualize::render_view
  /// orbit shot here; without it the Forside shows a placeholder.
  using CoverFallback =
      std::function<std::vector<std::uint8_t>(const ProjectDB &)>;
  CoverFallback cover_fallback;
};

/// One section (≈ page group) of the report and whether its data is
/// complete — the readiness the GUI's page strip shows next to the preview.
/// Mirrors the "Før indberetning" checklist rows each section rests on.
struct ReportSection {
  std::string id;    ///< forside, sagsdata, ressourcer, miljoe, bilag,
                     ///< nedrivningsplan
  std::string title; ///< "Forside", "Sagsdata", ...
  bool ready = false;
  std::vector<std::string> missing; ///< why not, in Danish; empty when ready
};

/// The report's input, assembled once into an owned temporary directory
/// (data.json, images/, report.typ) and removed with the object. Compiling
/// needs no ProjectDB, so a server can prepare on the request thread (a
/// fraction of a second) and compile elsewhere. Move-only.
class PreparedReport {
    public:
  /// Assembles the data and resolves the template.
  /// @throws as generate_ressourcekortlaegning_pdf does before typst runs.
  PreparedReport(ProjectDB &db, const ReportOptions &options);
  ~PreparedReport();
  PreparedReport(PreparedReport &&) noexcept;
  PreparedReport &operator=(PreparedReport &&) noexcept;
  PreparedReport(const PreparedReport &) = delete;
  PreparedReport &operator=(const PreparedReport &) = delete;

  /// 16 hex digits over the template text, data.json without its
  /// `generated_at` stamp, and every image: equal digests compile to the
  /// same pages, so it is a cache key for compiled output.
  [[nodiscard]] const std::string &digest() const;
  [[nodiscard]] const std::vector<ReportSection> &sections() const;
  /// The data directory (data.json + images/).
  [[nodiscard]] const std::filesystem::path &dir() const;

  /// typst compile to PDF. @throws std::runtime_error with typst's error.
  [[nodiscard]] std::vector<std::uint8_t> compile_pdf() const;
  /// typst compile --format png --ppi @p ppi: one PNG per page, in order.
  /// Safe to call concurrently with compile_pdf() on the same object.
  /// @throws std::invalid_argument for ppi outside 10..600;
  ///         std::runtime_error with typst's error.
  [[nodiscard]] std::vector<std::vector<std::uint8_t>>
  compile_png(int ppi = 96) const;

  /// The first page (1-based) of each section the template labels
  /// `<section>`, in document order ({"forside", 1}, {"sagsdata", 2}, ...),
  /// from `typst eval` over the laid-out document (≈ one more layout pass).
  /// Empty, with a warning, when typst cannot answer (an older typst
  /// without `eval --in`, or a template without the labels).
  [[nodiscard]] std::vector<std::pair<std::string, int>> section_pages() const;

    private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
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
