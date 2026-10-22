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
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
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

/// True when @p guid is safe as a file name stem: starts alphanumeric, only
/// [A-Za-z0-9._-] after that, and no "..". The guid comes from the .rux,
/// which may be untrusted, so anything else must not reach a path.
bool is_safe_file_stem(const std::string &guid) {
  if (guid.empty() || !std::isalnum(static_cast<unsigned char>(guid[0])))
    return false;
  for (const char c : guid) {
    const auto u = static_cast<unsigned char>(c);
    if (!(std::isalnum(u) || c == '.' || c == '_' || c == '-'))
      return false;
  }
  return guid.find("..") == std::string::npos;
}

constexpr const char *kTemplateEnvVar = "REUSEX_REPORT_TEMPLATE";

/// The Typst source this run compiles: options.template_path, else the file
/// named by $REUSEX_REPORT_TEMPLATE, else the copy of
/// apps/rux/resources/report.typ embedded at build time. A named file that
/// is missing is an error — never a silent fallback to the embedded copy.
std::string resolve_template(const ReportOptions &options) {
  std::filesystem::path path = options.template_path;
  std::string origin = "ReportOptions::template_path";
  if (path.empty()) {
    const char *env =
        std::getenv(kTemplateEnvVar); // NOLINT(concurrency-mt-unsafe)
    if (env == nullptr || *env == '\0')
      return core::detail::kDefaultReportTemplate;
    path = env;
    origin = std::string("$") + kTemplateEnvVar;
  }
  std::error_code ec;
  if (!std::filesystem::is_regular_file(path, ec))
    throw std::runtime_error(fmt::format(
        "report template {} (from {}) does not exist or is not a regular file",
        path.string(), origin));
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error(fmt::format(
        "report template {} (from {}) cannot be read", path.string(), origin));
  reusex::info("report template: {} (from {})", path.string(), origin);
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>{}};
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

/// True if @p dir is a previous dump_report_data() output: nothing at the
/// top level but data.json and images/, and data.json parses as a JSON
/// object carrying the keys every dump writes. Anything else (an unrelated
/// data.json, extra files, a parse failure) is not a previous dump, so the
/// caller refuses rather than guessing.
bool is_previous_report_dump(const std::filesystem::path &dir) {
  namespace fs = std::filesystem;
  for (const auto &entry : fs::directory_iterator(dir)) {
    const auto name = entry.path().filename().string();
    if (name != "data.json" && name != "images")
      return false;
  }

  std::ifstream f(dir / "data.json", std::ios::binary);
  if (!f)
    return false;
  nlohmann::json data;
  try {
    f >> data;
  } catch (const nlohmann::json::parse_error &) {
    return false;
  }
  return data.is_object() && data.contains("project_name") &&
         data.contains("materials") && data.contains("survey");
}

// ── data assembly ─────────────────────────────────────────────────────────

/// Write the report's data directory: data.json plus images/<guid><ext> for
/// every material with a thumbnail. Shared by the PDF run (into a temp dir)
/// and dump_report_data, so both see byte-identical input.
nlohmann::json
write_report_data(ProjectDB &db,
                  const std::optional<core::ResourceReportSection> &resources,
                  const std::filesystem::path &dir) {
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

  const auto image_dir = dir / "images";
  std::filesystem::create_directories(image_dir);

  // A dismissed part (survey_parts.dismissed, schema v36) is a misdetection:
  // its passport is not delivered, unless a counted part shares it.
  std::set<std::string> dismissed_passports, counted_passports;
  for (const auto &p : db.survey_parts())
    if (p.material_guid)
      (p.dismissed ? dismissed_passports : counted_passports)
          .insert(*p.material_guid);
  for (const auto &g : counted_passports)
    dismissed_passports.erase(g);

  nlohmann::json mats = nlohmann::json::array();
  for (std::size_t index = 0; index < summary.materials.size(); ++index) {
    const auto &guid = summary.materials[index].guid;
    if (dismissed_passports.count(guid) != 0)
      continue;

    auto thumb_opt = db.material_thumbnail(guid);
    bool has_thumb = thumb_opt.has_value();
    std::string thumb_path;

    if (has_thumb) {
      const auto &[blob, mime] = *thumb_opt;
      std::string stem = guid;
      if (!is_safe_file_stem(guid)) {
        stem = fmt::format("material-{:04d}", index);
        reusex::warn("Material guid '{}' is not a safe file name; writing its "
                     "thumbnail as images/{}{}",
                     guid, stem, ext_for_mime(mime));
      }
      const std::string fname = "images/" + stem + ext_for_mime(mime);
      std::ofstream f(dir / fname, std::ios::binary);
      f.write(reinterpret_cast<const char *>(blob.data()),
              static_cast<std::streamsize>(blob.size()));
      if (f.good()) {
        thumb_path = fname;
      } else {
        reusex::warn("Could not write the thumbnail of material '{}' to {}; "
                     "the report shows it without an image",
                     guid, (dir / fname).string());
        has_thumb = false;
      }
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

  data["resources"] =
      resources ? resources_json(*resources) : nlohmann::json(nullptr);
  {
    std::ofstream f(dir / "data.json");
    f << data.dump(2);
    if (!f.good())
      throw std::runtime_error("report_generator: failed to write " +
                               (dir / "data.json").string());
  }
  return data;
}

// ── Typst invocation ──────────────────────────────────────────────────────

std::vector<std::uint8_t> run_typst(const std::filesystem::path &tmpdir,
                                    const std::string &template_text) {
  const auto typ_path = tmpdir / "report.typ";
  const auto out_path = tmpdir / "out.pdf";
  const auto err_path = tmpdir / "typst.stderr";

  {
    std::ofstream f(typ_path, std::ios::binary);
    f << template_text;
    if (!f.good())
      throw std::runtime_error(
          "report_generator: failed to write template to " + typ_path.string());
  }

  // --ignore-system-fonts: the PDF must come out the same in the devshell,
  // CI, ruxd and the nix package, so only fonts embedded in typst (or named
  // via $TYPST_FONT_PATHS) are used.
  const std::string cmd = "typst compile --ignore-system-fonts '" +
                          typ_path.string() + "' '" + out_path.string() +
                          "' --root '" + tmpdir.string() + "' 2>'" +
                          err_path.string() + "'";

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
                                   const ReportOptions &options) {
  // Everything bad input can break is checked before any typst work: an
  // unknown resource template first, then a missing template file.
  std::optional<core::ResourceReportSection> resources;
  if (options.resource_template_id)
    resources =
        core::resource_report_section(db, *options.resource_template_id);
  const std::string template_text = resolve_template(options);

  TempDir tmpdir;

  const auto data = write_report_data(db, resources, tmpdir.path);

  reusex::info(
      "generate_ressourcekortlaegning_pdf: invoking typst ({} materials, "
      "{} columns, {} resource table(s))",
      data.at("materials").size(), data.at("columns").size(),
      resources ? resources->tables.size() : 0);

  const auto pdf = run_typst(tmpdir.path, template_text);

  reusex::info("generate_ressourcekortlaegning_pdf: {} bytes", pdf.size());
  return pdf;
}

std::vector<std::uint8_t>
generate_ressourcekortlaegning_pdf(ProjectDB &db,
                                   std::optional<std::int64_t> template_id) {
  ReportOptions options;
  options.resource_template_id = template_id;
  return generate_ressourcekortlaegning_pdf(db, options);
}

void dump_report_data(ProjectDB &db, const ReportOptions &options,
                      const std::filesystem::path &dir) {
  namespace fs = std::filesystem;
  // An unknown template fails before anything touches the disk.
  std::optional<core::ResourceReportSection> resources;
  if (options.resource_template_id)
    resources =
        core::resource_report_section(db, *options.resource_template_id);

  std::error_code ec;
  const bool exists = fs::exists(dir, ec);
  if (ec)
    throw std::runtime_error(fmt::format("dump_report_data: cannot stat {}: {}",
                                         dir.string(), ec.message()));

  if (exists) {
    const bool is_dir = fs::is_directory(dir, ec);
    if (ec)
      throw std::runtime_error(fmt::format(
          "dump_report_data: cannot stat {}: {}", dir.string(), ec.message()));
    if (!is_dir)
      throw std::runtime_error(fmt::format(
          "dump_report_data: {} exists and is not a directory", dir.string()));

    const bool empty = fs::is_empty(dir, ec);
    if (ec)
      throw std::runtime_error(fmt::format(
          "dump_report_data: cannot stat {}: {}", dir.string(), ec.message()));

    if (!empty && !is_previous_report_dump(dir))
      throw std::runtime_error(fmt::format(
          "dump_report_data: {} is not empty and is not a previous report "
          "dump; refusing to write a report dump into it",
          dir.string()));

    // A previous dump's images: replace, never mix in stale files.
    fs::remove_all(dir / "images", ec);
    if (ec)
      throw std::runtime_error(
          fmt::format("dump_report_data: cannot clear {}: {}",
                      (dir / "images").string(), ec.message()));
  }
  fs::create_directories(dir);

  const auto data = write_report_data(db, resources, dir);
  reusex::info("dump_report_data: wrote {} ({} materials, {} resource "
               "table(s))",
               dir.string(), data.at("materials").size(),
               resources ? resources->tables.size() : 0);
}

} // namespace reusex
