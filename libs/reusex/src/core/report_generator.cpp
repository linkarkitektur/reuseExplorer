// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/report_generator.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/core/report_cover.hpp>
#include <reusex/core/report_template_default.hpp>
#include <reusex/core/resource_export.hpp>
#include <reusex/core/sagsdata.hpp>
#include <reusex/core/survey_service.hpp>

#include <fmt/format.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
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

/// 1234.5 -> "1.234,5"; whole numbers lose the decimal ("640"). The report
/// is Danish: comma decimal mark, '.' groups thousands (as da-DK
/// toLocaleString does in the GUI).
std::string da_number(double v) {
  std::string s = fmt::format("{:.1f}", v);
  if (s.size() > 2 && s.compare(s.size() - 2, 2, ".0") == 0)
    s.resize(s.size() - 2);
  const auto dot = s.find('.');
  std::string frac = dot == std::string::npos ? "" : "," + s.substr(dot + 1);
  std::string whole = s.substr(0, dot);
  const bool neg = !whole.empty() && whole[0] == '-';
  if (neg)
    whole.erase(0, 1);
  std::string grouped;
  for (std::size_t i = 0; i < whole.size(); ++i) {
    if (i > 0 && (whole.size() - i) % 3 == 0)
      grouped += '.';
    grouped += whole[i];
  }
  return (neg ? "-" : "") + grouped + frac;
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
    static std::atomic<unsigned> counter{0};
    path =
        tmp / ("reusex_report_" + std::to_string(pid) + "_" +
               std::to_string(ts) + "_" + std::to_string(counter.fetch_add(1)));
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

// ── report sections (project, Sagsdata, types, samples, plan) ───────────

/// A trimmed non-empty string, else null.
nlohmann::json text_or_null(const std::string &s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return nullptr;
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

nlohmann::json opt_text(const std::optional<std::string> &s) {
  return s ? text_or_null(*s) : nlohmann::json(nullptr);
}

std::string sample_stage_label_da(core::SampleStage s) {
  switch (s) {
  case core::SampleStage::planlagt:
    return "Planlagt";
  case core::SampleStage::udtaget:
    return "Udtaget";
  case core::SampleStage::sendt:
    return "Sendt til lab";
  case core::SampleStage::svar:
    return "Svar modtaget";
  }
  return "";
}

std::string sample_result_label_da(core::SampleResult r) {
  return r == core::SampleResult::forurenet ? "Forurenet"
         : r == core::SampleResult::ren     ? "Ren"
                                            : "";
}

/// "A", "B", ..., "Z", "AA", ... (the GUI's appendix letters).
std::string appendix_letter(std::size_t i) {
  std::string out;
  long n = static_cast<long>(i);
  do {
    out.insert(out.begin(), static_cast<char>('A' + n % 26));
    n = n / 26 - 1;
  } while (n >= 0);
  return out;
}

/// `project`: ProjectMetadata + the Sagsdata fields, every value a display
/// string (Danish dates, "1.240 m²") or null when unset.
nlohmann::json project_json(const ProjectDB &db,
                            const std::string &fallback_name) {
  const auto pid = core::primary_project_id(db);
  ProjectDB::ProjectMetadata meta;
  if (!pid.empty())
    meta = db.get_project_metadata(pid);
  nlohmann::json p;
  p["id"] = pid;
  p["name"] = meta.name.empty() ? nlohmann::json(fallback_name)
                                : text_or_null(meta.name);
  p["building_address"] = text_or_null(meta.building_address);
  p["year_of_construction"] =
      meta.year_of_construction > 0
          ? nlohmann::json(std::to_string(meta.year_of_construction))
          : nlohmann::json(nullptr);
  p["survey_date"] = meta.survey_date.empty()
                         ? nlohmann::json(nullptr)
                         : nlohmann::json(core::danish_date(meta.survey_date));
  p["survey_organisation"] = text_or_null(meta.survey_organisation);
  p["notes"] = text_or_null(meta.notes);
  for (const auto &field : core::sagsdata_fields()) {
    const auto v =
        pid.empty() ? std::nullopt : core::sagsdata_value(db, pid, field);
    if (!v) {
      p[field] = nullptr;
    } else if (field == "demolished_floor_area_m2") {
      const auto n = nlohmann::json::parse(*v, nullptr, false);
      p[field] = n.is_number() && n.get<double>() > 0.0
                     ? nlohmann::json(da_number(n.get<double>()) + " m²")
                     : nlohmann::json(nullptr);
    } else if (field == "demolition_start" ||
               field == "screening_approved_at") {
      p[field] = core::danish_date(*v);
    } else {
      p[field] = opt_text(v);
    }
  }
  return p;
}

/// The Sagsdata table, in the GUI's row order; value null = not filled in.
nlohmann::json sagsdata_rows_json(const nlohmann::json &p) {
  static const std::vector<std::pair<const char *, const char *>> rows{
      {"Adresse", "building_address"},
      {"BBR-nummer", "bbr_number"},
      {"Matrikel", "cadastral"},
      {"Bygherre", "client"},
      {"Kontaktperson", "client_contact"},
      {"Miljø- og ressourcekoordinator", "coordinator"},
      {"Kommune", "municipality"},
      {"Nedrivning starter", "demolition_start"},
      {"Nedrevet etageareal", "demolished_floor_area_m2"},
      {"Opførelsesår", "year_of_construction"},
      {"Kortlægningsdato", "survey_date"},
      {"Kortlagt af", "survey_organisation"},
      {"Screening godkendt af bygherre", "screening_approved_at"},
  };
  nlohmann::json out = nlohmann::json::array();
  for (const auto &[label, key] : rows)
    out.push_back({{"label", label}, {"value", p.at(key)}});
  return out;
}

/// The Sagsdata fields BEK 496 cannot do without (the GUI's
/// SAGSDATA_REQUIRED), by label, still blank.
std::vector<std::string> missing_sagsdata(const nlohmann::json &p) {
  static const std::vector<std::pair<const char *, const char *>> required{
      {"Adresse", "building_address"},
      {"BBR-nummer", "bbr_number"},
      {"Matrikel", "cadastral"},
      {"Bygherre", "client"},
      {"Kommune", "municipality"},
      {"Nedrivningsstart", "demolition_start"},
      {"Nedrevet etageareal", "demolished_floor_area_m2"}};
  std::vector<std::string> out;
  for (const auto &[label, key] : required)
    if (p.at(key).is_null())
      out.emplace_back(label);
  return out;
}

/// Every non-rejected survey type (Ressourcekortlægning), in id order. Mass
/// comes from type_totals, the one source of a type's tonnes.
struct TypeFacts {
  nlohmann::json rows = nlohmann::json::array();
  std::size_t count = 0, approved = 0, lacking_eak_or_tons = 0;
  double total_t = 0.0;
  /// Nedrivningsplan, in order.
  nlohmann::json steps = nlohmann::json::array();
};

TypeFacts type_facts(const ProjectDB &db) {
  TypeFacts f;
  const auto totals = core::type_totals(db);
  std::map<std::int64_t, const core::TypeTotals *> by_id;
  for (const auto &t : totals)
    by_id[t.type_id] = &t;
  std::map<std::int64_t, double> quantity;
  for (const auto &p : db.survey_parts())
    if (!p.dismissed)
      quantity[p.type_id] += p.quantity;

  std::vector<std::string> sanering;
  std::array<std::vector<std::string>, core::kTreatmentCount> by_treatment;
  for (const auto &t : db.survey_types()) {
    if (t.review_status == core::ReviewStatus::rejected)
      continue;
    const auto it = by_id.find(t.id);
    const auto mass =
        it != by_id.end() ? it->second->mass_t : std::optional<double>{};
    const auto env = it != by_id.end() ? it->second->environment
                                       : core::EnvironmentStatus::ren_screening;
    const bool approved = t.review_status == core::ReviewStatus::approved;
    const bool mass_needed = t.treatment != core::Treatment::bevaring;
    const auto eak = text_or_null(t.eak_code);
    ++f.count;
    f.approved += approved ? 1 : 0;
    if (eak.is_null() || (!mass && mass_needed))
      ++f.lacking_eak_or_tons;
    f.total_t += mass.value_or(0.0);
    f.rows.push_back(
        {{"id", t.id},
         {"name", t.name},
         {"approved", approved},
         {"eak", eak},
         {"treatment", std::string(core::treatment_label_da(t.treatment))},
         {"quantity", da_number(quantity[t.id]) + " " + t.unit},
         {"mass", mass ? nlohmann::json(da_number(*mass) + " t")
                       : nlohmann::json(nullptr)},
         {"mass_needed", mass_needed},
         {"environment", std::string(core::environment_label_da(env))}});
    if (env == core::EnvironmentStatus::forurenet)
      sanering.push_back(t.name);
    else if (env == core::EnvironmentStatus::afventer)
      sanering.push_back(t.name + " (afventer prøvesvar)");
    by_treatment[static_cast<std::size_t>(t.treatment)].push_back(t.name);
  }
  static const std::array<const char *, core::kTreatmentCount> step_title{
      "Bevares i bygningen", "Udtag til genbrug", "Nedtages til genanvendelse",
      "Anden nyttiggørelse", "Bortskaffelse"};
  if (!sanering.empty())
    f.steps.push_back({{"title", "Miljøsanering"}, {"items", sanering}});
  for (std::size_t i = 0; i < core::kTreatmentCount; ++i)
    if (!by_treatment[i].empty())
      f.steps.push_back({{"title", step_title[i]}, {"items", by_treatment[i]}});
  return f;
}

struct SampleFacts {
  nlohmann::json rows = nlohmann::json::array();
  nlohmann::json appendices = nlohmann::json::array();
  std::size_t pending = 0; ///< not yet answered (stage != svar)
};

SampleFacts sample_facts(const ProjectDB &db) {
  SampleFacts f;
  std::map<std::int64_t, std::string> names;
  for (const auto &t : db.survey_types())
    names[t.id] = t.name;
  const auto samples = db.samples();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto &s = samples[i];
    std::string types;
    for (const auto id : s.type_ids)
      if (const auto it = names.find(id); it != names.end())
        types += (types.empty() ? "" : ", ") + it->second;
    const bool answered = s.stage == core::SampleStage::svar;
    f.pending += answered ? 0 : 1;
    const auto result = sample_result_label_da(s.result);
    f.rows.push_back(
        {{"code", s.code},
         {"title", s.title},
         {"what", s.what},
         {"types",
          types.empty() ? nlohmann::json(nullptr) : nlohmann::json(types)},
         {"stage", sample_stage_label_da(s.stage)},
         {"result", answered && !result.empty() ? nlohmann::json(result)
                                                : nlohmann::json(nullptr)},
         {"contaminated", s.result == core::SampleResult::forurenet}});
    f.appendices.push_back(
        {{"letter", appendix_letter(i)},
         {"text", s.code + " · " + s.title},
         {"status",
          answered ? nlohmann::json("Svar: " + (result.empty()
                                                    ? std::string("modtaget")
                                                    : result))
                   : nlohmann::json(nullptr)}});
  }
  std::size_t photographed = 0;
  for (const auto &p : db.survey_parts())
    if (!p.dismissed && p.view)
      ++photographed;
  if (photographed > 0)
    f.appendices.push_back(
        {{"letter", appendix_letter(samples.size())},
         {"text", "Fotodokumentation · " +
                      (photographed == 1
                           ? std::string("1 bygningsdel")
                           : std::to_string(photographed) + " bygningsdele") +
                      " med foto"},
         {"status", "Fra scanningen"}});
  return f;
}

nlohmann::json section_json(const std::string &id, const std::string &title,
                            std::vector<std::string> missing) {
  return {{"id", id},
          {"title", title},
          {"ready", missing.empty()},
          {"missing", std::move(missing)}};
}

std::string plural(std::size_t n, const char *one, const char *many) {
  return std::to_string(n) + " " + (n == 1 ? one : many);
}

/// Per-section readiness (the GUI checklist's rows, by the sections they
/// rest on). "Alle typer screenet" needs the resource keys and is left to
/// the GUI's checklist.
nlohmann::json sections_json(const nlohmann::json &project,
                             const TypeFacts &types,
                             const SampleFacts &samples) {
  std::vector<std::string> sagsdata;
  if (const auto m = missing_sagsdata(project); !m.empty()) {
    std::string list;
    for (const auto &l : m)
      list += (list.empty() ? "" : ", ") + l;
    sagsdata.push_back("Sagsdata mangler: " + list);
  }
  std::vector<std::string> approval;
  if (types.count == 0)
    approval.emplace_back("Ingen typer i kortlægningen endnu");
  else if (types.approved < types.count)
    approval.push_back(plural(types.count - types.approved, "type", "typer") +
                       " ikke godkendt (" + std::to_string(types.approved) +
                       " af " + std::to_string(types.count) + ")");
  if (types.lacking_eak_or_tons > 0)
    approval.push_back(plural(types.lacking_eak_or_tons, "type", "typer") +
                       " mangler EAK eller tons");
  std::vector<std::string> sampled;
  if (samples.pending > 0)
    sampled.push_back(plural(samples.pending, "prøve", "prøver") +
                      " afventer svar");
  auto sagsdata_page = sagsdata;
  if (project.at("screening_approved_at").is_null())
    sagsdata_page.emplace_back("Bygherrens godkendelse af screeningen er "
                               "ikke registreret");
  nlohmann::json out = nlohmann::json::array();
  out.push_back(section_json("forside", "Forside", sagsdata));
  out.push_back(section_json("sagsdata", "Sagsdata", sagsdata_page));
  out.push_back(section_json("ressourcer", "Ressourcekortlægning", approval));
  out.push_back(section_json("miljoe", "Miljøkortlægning", sampled));
  out.push_back(section_json("bilag", "Prøver & bilag", sampled));
  out.push_back(section_json("nedrivningsplan", "Nedrivningsplan", approval));
  return out;
}

/// "image/png" -> ".png", anything else (JPEG) ".jpg".
std::string cover_ext(const std::string &mime) {
  return mime == "image/png" ? ".png" : ".jpg";
}

bool looks_like_png(const std::vector<std::uint8_t> &b) {
  return b.size() >= 4 && b[0] == 0x89 && b[1] == 'P' && b[2] == 'N' &&
         b[3] == 'G';
}

/// Write images/cover.* and return {cover_path, cover_source}: the
/// project's chosen image, else the caller's fallback render, else none.
std::pair<std::string, std::string>
write_cover(const ProjectDB &db, const ReportOptions &options,
            const std::filesystem::path &dir) {
  std::string source;
  core::EncodedImage img;
  if (auto chosen = core::report_cover_image(db)) {
    const auto c = core::report_cover(db);
    source = c ? std::string(core::to_string(c->source)) : "upload";
    img = std::move(*chosen);
  } else if (options.cover_fallback) {
    try {
      img.bytes = options.cover_fallback(db);
    } catch (const std::exception &e) {
      reusex::warn("report cover: the fallback render failed ({}); the "
                   "Forside shows no image",
                   e.what());
      img.bytes.clear();
    }
    img.mime_type = looks_like_png(img.bytes) ? "image/png" : "image/jpeg";
    source = "render";
  }
  if (img.bytes.empty())
    return {"", "none"};
  const std::string rel = "images/cover" + cover_ext(img.mime_type);
  std::ofstream f(dir / rel, std::ios::binary);
  f.write(reinterpret_cast<const char *>(img.bytes.data()),
          static_cast<std::streamsize>(img.bytes.size()));
  if (!f.good()) {
    reusex::warn("report cover: could not write {}; the Forside shows no "
                 "image",
                 (dir / rel).string());
    return {"", "none"};
  }
  return {rel, source};
}

// ── data assembly ─────────────────────────────────────────────────────────

/// Write the report's data directory: data.json plus images/<guid><ext> for
/// every material with a thumbnail. Shared by the PDF run (into a temp dir)
/// and dump_report_data, so both see byte-identical input.
nlohmann::json write_report_data(
    ProjectDB &db, const std::optional<core::ResourceReportSection> &resources,
    const ReportOptions &options, const std::filesystem::path &dir) {
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
  std::set<std::string> column_names;
  for (const auto &d : defs) {
    cols.push_back({{"id", d.id}, {"name", d.name}, {"type", d.type}});
    column_names.insert(d.name);
  }
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

    // Only the user-defined columns: the template prints nothing else, and
    // a full passport is ~75 kB of JSON (126 MB for 1672 passports), which
    // the preview would otherwise write, hash and parse on every request.
    const auto props_map = db.passport_stored_properties(guid);
    nlohmann::json props = nlohmann::json::object();
    for (const auto &[k, v] : props_map)
      if (column_names.count(k) != 0)
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

  // The preview's sections (Forside .. Nedrivningsplan), reusex-ufcm.
  auto project = project_json(db, project_name);
  const auto types = type_facts(db);
  const auto samples = sample_facts(db);
  data["sagsdata"] = sagsdata_rows_json(project);
  data["types"] = types.rows;
  data["types_summary"] = {
      {"count", types.count},
      {"approved", types.approved},
      {"total", da_number(types.total_t) + " t"},
      {"line", std::to_string(types.count) +
                   (types.count == 1 ? " type · " : " typer · ") +
                   da_number(types.total_t) + " t i alt · " +
                   std::to_string(types.approved) + " godkendt"}};
  data["samples"] = samples.rows;
  data["appendices"] = samples.appendices;
  data["demolition_steps"] = types.steps;
  data["sections"] = sections_json(project, types, samples);
  data["project"] = std::move(project);
  {
    const int blocking = data["survey"]["blocking"].get<int>();
    const auto now = data["generated_at"].get<std::string>();
    data["report"] = {
        {"version", static_cast<int>(db.list_report_pdfs().size()) + 1},
        {"status", blocking > 0 ? "kladde" : "komplet"},
        {"date", core::danish_date(now)}};
  }
  const auto [cover_path, cover_source] = write_cover(db, options, dir);
  data["cover_path"] = cover_path;
  data["cover_source"] = cover_source;
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

void write_template(const std::filesystem::path &tmpdir,
                    const std::string &template_text) {
  const auto typ_path = tmpdir / "report.typ";
  std::ofstream f(typ_path, std::ios::binary);
  f << template_text;
  if (!f.good())
    throw std::runtime_error("report_generator: failed to write template to " +
                             typ_path.string());
}

std::vector<std::uint8_t> slurp(const std::filesystem::path &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("report_generator: typst produced no output at " +
                             path.string());
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>{}};
}

/// Run typst on tmpdir/report.typ. @p out is the output path or, for PNG,
/// a `{p}` page pattern; @p extra holds the format flags.
void run_typst_cmd(const std::filesystem::path &tmpdir,
                   const std::filesystem::path &out, const std::string &extra,
                   const std::string &tag) {
  const auto typ_path = tmpdir / "report.typ";
  const auto err_path = tmpdir / ("typst-" + tag + ".stderr");
  // --ignore-system-fonts: the PDF must come out the same in the devshell,
  // CI, ruxd and the nix package, so only fonts embedded in typst (or named
  // via $TYPST_FONT_PATHS) are used.
  const std::string cmd = "typst compile --ignore-system-fonts " + extra +
                          " '" + typ_path.string() + "' '" + out.string() +
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
}

std::vector<std::uint8_t> run_typst_pdf(const std::filesystem::path &tmpdir) {
  const auto out_path = tmpdir / "out.pdf";
  run_typst_cmd(tmpdir, out_path, "--format pdf", "pdf");
  return slurp(out_path);
}

std::vector<std::vector<std::uint8_t>>
run_typst_png(const std::filesystem::path &tmpdir, int ppi) {
  namespace fs = std::filesystem;
  // One directory per call, so concurrent PNG runs never mix pages.
  static std::atomic<unsigned> counter{0};
  const auto out_dir =
      tmpdir / fmt::format("png-{}-{}", ppi, counter.fetch_add(1));
  fs::create_directories(out_dir);
  run_typst_cmd(tmpdir, out_dir / "page-{0p}.png",
                fmt::format("--format png --ppi {}", ppi),
                out_dir.filename().string());
  std::vector<fs::path> files;
  for (const auto &e : fs::directory_iterator(out_dir))
    if (e.path().extension() == ".png")
      files.push_back(e.path());
  std::sort(files.begin(), files.end()); // zero-padded: name order = page order
  std::vector<std::vector<std::uint8_t>> pages;
  pages.reserve(files.size());
  for (const auto &f : files)
    pages.push_back(slurp(f));
  std::error_code ec;
  fs::remove_all(out_dir, ec);
  if (pages.empty())
    throw std::runtime_error("report_generator: typst wrote no PNG pages");
  return pages;
}

/// FNV-1a 64: a cache key, not a security boundary.
struct Fnv64 {
  std::uint64_t h = 0xcbf29ce484222325ULL;
  void add(const void *p, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i) {
      h ^= b[i];
      h *= 0x100000001b3ULL;
    }
  }
  void add(const std::string &s) {
    const std::uint64_t n = s.size();
    add(&n, sizeof n);
    add(s.data(), s.size());
  }
};

std::string report_digest(const std::string &template_text, nlohmann::json data,
                          const std::filesystem::path &dir) {
  namespace fs = std::filesystem;
  Fnv64 h;
  h.add(std::string("reusex-report-digest-v1"));
  h.add(template_text);
  data.erase("generated_at");
  // The date shown on the Forside is part of the page, so it stays in.
  h.add(data.dump());
  std::vector<fs::path> images;
  if (fs::is_directory(dir / "images"))
    for (const auto &e : fs::directory_iterator(dir / "images"))
      images.push_back(e.path());
  std::sort(images.begin(), images.end());
  for (const auto &p : images) {
    h.add(p.filename().string());
    const auto bytes = slurp(p);
    h.add(std::string(bytes.begin(), bytes.end()));
  }
  return fmt::format("{:016x}", h.h);
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
  const PreparedReport prepared(db, options);
  const auto pdf = prepared.compile_pdf();

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

  const auto data = write_report_data(db, resources, options, dir);
  reusex::info("dump_report_data: wrote {} ({} materials, {} resource "
               "table(s))",
               dir.string(), data.at("materials").size(),
               resources ? resources->tables.size() : 0);
}

// ── PreparedReport ────────────────────────────────────────────────────────

struct PreparedReport::Impl {
  TempDir tmp;
  std::string digest;
  std::vector<ReportSection> sections;
};

PreparedReport::PreparedReport(ProjectDB &db, const ReportOptions &options)
    : impl_(std::make_unique<Impl>()) {
  // Everything bad input can break is checked before any typst work: an
  // unknown resource template first, then a missing template file.
  std::optional<core::ResourceReportSection> resources;
  if (options.resource_template_id)
    resources =
        core::resource_report_section(db, *options.resource_template_id);
  const std::string template_text = resolve_template(options);

  const auto data = write_report_data(db, resources, options, impl_->tmp.path);
  write_template(impl_->tmp.path, template_text);
  impl_->digest = report_digest(template_text, data, impl_->tmp.path);
  for (const auto &s : data.at("sections"))
    impl_->sections.push_back(
        {s.at("id").get<std::string>(), s.at("title").get<std::string>(),
         s.at("ready").get<bool>(),
         s.at("missing").get<std::vector<std::string>>()});
  reusex::info("report prepared: {} materials, {} columns, {} resource "
               "table(s), {} types, cover {}, digest {}",
               data.at("materials").size(), data.at("columns").size(),
               resources ? resources->tables.size() : 0,
               data.at("types").size(),
               data.at("cover_source").get<std::string>(), impl_->digest);
}

PreparedReport::~PreparedReport() = default;
PreparedReport::PreparedReport(PreparedReport &&) noexcept = default;
PreparedReport &PreparedReport::operator=(PreparedReport &&) noexcept = default;

const std::string &PreparedReport::digest() const { return impl_->digest; }
const std::vector<ReportSection> &PreparedReport::sections() const {
  return impl_->sections;
}
const std::filesystem::path &PreparedReport::dir() const {
  return impl_->tmp.path;
}

std::vector<std::uint8_t> PreparedReport::compile_pdf() const {
  return run_typst_pdf(impl_->tmp.path);
}

std::vector<std::vector<std::uint8_t>>
PreparedReport::compile_png(int ppi) const {
  if (ppi < 10 || ppi > 600)
    throw std::invalid_argument(
        fmt::format("compile_png: ppi must be 10..600, got {}", ppi));
  return run_typst_png(impl_->tmp.path, ppi);
}

std::vector<std::pair<std::string, int>> PreparedReport::section_pages() const {
  const auto &dir = impl_->tmp.path;
  static std::atomic<unsigned> counter{0};
  const auto n = counter.fetch_add(1);
  const auto out = dir / fmt::format("sections-{}.json", n);
  const auto err = dir / fmt::format("sections-{}.stderr", n);
  const std::string cmd = "typst eval --ignore-system-fonts --root '" +
                          dir.string() + "' --in '" +
                          (dir / "report.typ").string() +
                          "' 'query(<section>).map(m => (id: m.value.id, page: "
                          "m.location().page()))' >'" +
                          out.string() + "' 2>'" + err.string() + "'";
  const int status = std::system(cmd.c_str()); // NOLINT(cert-env33-c)
  std::vector<std::pair<std::string, int>> pages;
  try {
    if (status != 0)
      throw std::runtime_error("typst eval exited " +
                               std::to_string(WEXITSTATUS(status)));
    std::ifstream f(out);
    const auto j = nlohmann::json::parse(f);
    for (const auto &e : j)
      pages.emplace_back(e.at("id").get<std::string>(),
                         e.at("page").get<int>());
  } catch (const std::exception &e) {
    std::string why;
    std::ifstream ef(err);
    if (ef)
      why.assign(std::istreambuf_iterator<char>(ef),
                 std::istreambuf_iterator<char>{});
    reusex::warn("report section pages unavailable ({}): {}", e.what(), why);
    pages.clear();
  }
  std::error_code ec;
  std::filesystem::remove(out, ec);
  std::filesystem::remove(err, ec);
  return pages;
}

} // namespace reusex
