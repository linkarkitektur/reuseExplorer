// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/report_generator.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/core/report_template_default.hpp>
#include <reusex/core/resource_export.hpp>
#include <reusex/core/survey_service.hpp>

#include <fmt/format.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace reusex {

namespace {

// ── helpers ──────────────────────────────────────────────────────────────────

std::string now_iso8601_rg() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return std::string(buf);
}

/// 1234.5 -> "1234,5"; whole numbers lose the decimal ("640"). The report is
/// Danish, so the decimal mark is a comma.
std::string da_number(double v) {
  std::string s = fmt::format("{:.1f}", v);
  if (s.size() > 2 && s.compare(s.size() - 2, 2, ".0") == 0)
    s.resize(s.size() - 2);
  std::replace(s.begin(), s.end(), '.', ',');
  return s;
}

std::string ext_for_mime(const std::string &mime) {
  if (mime == "image/png")
    return ".png";
  if (mime == "image/webp")
    return ".webp";
  if (mime == "image/gif")
    return ".gif";
  return ".jpg";
}

// ── RAII temp directory ───────────────────────────────────────────────────

struct TempDir {
  std::filesystem::path path;

  explicit TempDir() {
    const auto tmp = std::filesystem::temp_directory_path();
    const auto pid = static_cast<long>(::getpid());
    const auto ts = std::chrono::steady_clock::now().time_since_epoch().count();
    path = tmp /
           ("reusex_report_" + std::to_string(pid) + "_" + std::to_string(ts));
    std::filesystem::create_directories(path);
  }

  ~TempDir() noexcept {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }

  TempDir(const TempDir &) = delete;
  TempDir &operator=(const TempDir &) = delete;
};

nlohmann::json resources_json(const core::ResourceReportSection &s) {
  nlohmann::json tables = nlohmann::json::array();
  for (const auto &t : s.tables)
    tables.push_back({{"headers", t.headers}, {"rows", t.rows}});
  return {{"name", s.template_name}, {"tables", std::move(tables)}};
}

// ── data assembly ─────────────────────────────────────────────────────────

nlohmann::json assemble_report_data(ProjectDB &db,
                                    const std::filesystem::path &tmpdir) {
  const auto summary = db.project_summary();
  const auto defs = db.list_property_definitions();

  std::string project_name;
  if (!summary.projects.empty() && !summary.projects.front().name.empty())
    project_name = summary.projects.front().name;
  else
    project_name = summary.path.stem().string();

  nlohmann::json data;
  data["project_name"] = project_name;
  data["generated_at"] = now_iso8601_rg();

  nlohmann::json cols = nlohmann::json::array();
  for (const auto &d : defs)
    cols.push_back({{"id", d.id}, {"name", d.name}, {"type", d.type}});
  data["columns"] = std::move(cols);

  const auto thumb_dir = tmpdir / "thumbnails";
  std::filesystem::create_directories(thumb_dir);

  nlohmann::json mats = nlohmann::json::array();
  for (const auto &info : summary.materials) {
    const auto &guid = info.guid;

    auto thumb_opt = db.material_thumbnail(guid);
    bool has_thumb = thumb_opt.has_value();
    std::string thumb_path;

    if (has_thumb) {
      const auto &[blob, mime] = *thumb_opt;
      const std::string fname = "thumbnails/" + guid + ext_for_mime(mime);
      std::ofstream f(tmpdir / fname, std::ios::binary);
      f.write(reinterpret_cast<const char *>(blob.data()),
              static_cast<std::streamsize>(blob.size()));
      if (f.good())
        thumb_path = fname;
      else
        has_thumb = false;
    }

    const auto props_map = db.passport_stored_properties(guid);
    nlohmann::json props = nlohmann::json::object();
    for (const auto &[k, v] : props_map)
      props[k] = v;

    mats.push_back({{"guid", guid},
                    {"has_thumbnail", has_thumb},
                    {"thumbnail_path", thumb_path},
                    {"properties", std::move(props)}});
  }
  data["materials"] = std::move(mats);

  // Survey (Kortlægning): reportable types only — "kun godkendte mængder
  // indgår i rapporten" (GUI Phase 5, R8). report_survey_rows already
  // withholds types awaiting a sample or missing tonnes, so the circularity
  // line below sums exactly the rows the table shows.
  nlohmann::json rows = nlohmann::json::array();
  std::vector<core::TypeTotals> approved;
  for (const auto &r : core::report_survey_rows(db)) {
    rows.push_back(
        {{"name", r.name},
         {"bim7aa", r.bim7aa_code},
         {"eak", r.eak_code},
         {"quantity", da_number(r.quantity) + " " + r.unit},
         {"mass", r.mass_t ? da_number(*r.mass_t) + " t" : std::string("—")},
         {"treatment", std::string(core::treatment_label_da(r.treatment))},
         {"environment",
          std::string(core::environment_label_da(r.environment))}});
    approved.push_back({r.treatment, core::ReviewStatus::approved, r.mass_t,
                        r.eak_code, r.environment});
  }
  const auto breakdown = core::circularity_breakdown(approved);
  nlohmann::json circ = nlohmann::json::array();
  for (std::size_t i = 0; i < core::kTreatmentCount; ++i)
    if (breakdown[i] > 0.0)
      circ.push_back({{"label", std::string(core::treatment_label_da(
                                    static_cast<core::Treatment>(i)))},
                      {"tonnes", da_number(breakdown[i]) + " t"}});
  data["survey"] = {{"rows", std::move(rows)},
                    {"circularity", std::move(circ)},
                    {"blocking", report_blocking_types(db)}};
  return data;
}

// ── Typst invocation ──────────────────────────────────────────────────────

std::vector<std::uint8_t> run_typst(const std::filesystem::path &tmpdir) {
  const auto typ_path = tmpdir / "report.typ";
  const auto out_path = tmpdir / "out.pdf";
  const auto err_path = tmpdir / "typst.stderr";

  {
    std::ofstream f(typ_path);
    f << core::detail::kDefaultReportTemplate;
    if (!f.good())
      throw std::runtime_error(
          "report_generator: failed to write template to " + typ_path.string());
  }

  const std::string cmd = "typst compile '" + typ_path.string() + "' '" +
                          out_path.string() + "' --root '" + tmpdir.string() +
                          "' 2>'" + err_path.string() + "'";

  const int status = std::system(cmd.c_str()); // NOLINT(cert-env33-c)

  if (status != 0) {
    std::string err_msg;
    std::ifstream ef(err_path);
    if (ef)
      err_msg.assign(std::istreambuf_iterator<char>(ef),
                     std::istreambuf_iterator<char>{});
    throw std::runtime_error("typst compile failed (exit " +
                             std::to_string(WEXITSTATUS(status)) + "):\n" +
                             err_msg);
  }

  std::ifstream pf(out_path, std::ios::binary);
  if (!pf)
    throw std::runtime_error("report_generator: typst produced no output at " +
                             out_path.string());
  return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(pf),
                                   std::istreambuf_iterator<char>{});
}

} // namespace

// ── public API ────────────────────────────────────────────────────────────

int report_blocking_types(const ProjectDB &db) {
  return static_cast<int>(
      core::fractions_by_eak(core::type_totals(db)).blocking_types);
}

std::vector<std::uint8_t>
generate_ressourcekortlaegning_pdf(ProjectDB &db,
                                   std::optional<std::int64_t> template_id) {
  // Resolve the template first: an unknown id fails fast, typst or not.
  std::optional<core::ResourceReportSection> resources;
  if (template_id)
    resources = core::resource_report_section(db, *template_id);

  TempDir tmpdir;

  auto data = assemble_report_data(db, tmpdir.path);
  data["resources"] =
      resources ? resources_json(*resources) : nlohmann::json(nullptr);
  {
    std::ofstream f(tmpdir.path / "data.json");
    f << data.dump();
    if (!f.good())
      throw std::runtime_error("report_generator: failed to write data.json");
  }

  reusex::info(
      "generate_ressourcekortlaegning_pdf: invoking typst ({} materials, "
      "{} columns, {} resource table(s))",
      data.at("materials").size(), data.at("columns").size(),
      resources ? resources->tables.size() : 0);

  const auto pdf = run_typst(tmpdir.path);

  reusex::info("generate_ressourcekortlaegning_pdf: {} bytes", pdf.size());
  return pdf;
}

} // namespace reusex
