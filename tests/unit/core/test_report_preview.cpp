// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The sectioned report (reusex-ufcm): the cover image choice (schema v38),
// the data.json fields of the Forside .. Nedrivningsplan sections, and
// PreparedReport (digest, readiness, PNG pages).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <core/ProjectDB.hpp>
#include <core/report_cover.hpp>
#include <core/report_generator.hpp>
#include <core/sagsdata.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"
#include "../../support/typst.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using Catch::Matchers::ContainsSubstring;
using reusex::ProjectDB;
using reusex::test_support::TempDir;
using reusex::test_support::TempPath;
namespace core = reusex::core;

namespace {

std::vector<std::uint8_t> encode(const cv::Mat &m, const char *ext) {
  std::vector<std::uint8_t> out;
  REQUIRE(cv::imencode(ext, m, out));
  return out;
}

/// A 200x100 frame: left half red, right half blue (BGR).
cv::Mat two_tone() {
  cv::Mat m(100, 200, CV_8UC3, cv::Scalar(0, 0, 255));
  m(cv::Rect(100, 0, 100, 100)).setTo(cv::Scalar(255, 0, 0));
  return m;
}

nlohmann::json read_json(const fs::path &p) {
  std::ifstream f(p);
  REQUIRE(f.good());
  return nlohmann::json::parse(f);
}

void seed_project(ProjectDB &db) {
  ProjectDB::ProjectMetadata meta;
  meta.id = "p1";
  meta.name = "Skolevej 12";
  meta.building_address = "Skolevej 12, 6000 Kolding";
  meta.survey_date = "2026-09-14";
  db.update_project_metadata(meta);
  db.set_setting(core::sagsdata_setting_key("p1", "bbr_number"),
                 "621-123456-1");
  db.set_setting(core::sagsdata_setting_key("p1", "demolished_floor_area_m2"),
                 "2480");
  db.set_setting(core::sagsdata_setting_key("p1", "demolition_start"),
                 "2027-02-02");
}

} // namespace

TEST_CASE("ReportAssets_RoundTripAndDelete", "[report][cover]") {
  TempPath tmp("test_report_preview");
  ProjectDB db(tmp.path);
  CHECK_FALSE(db.report_asset("cover").has_value());
  db.set_report_asset("cover", {1, 2, 3}, "image/png");
  const auto a = db.report_asset("cover");
  REQUIRE(a);
  CHECK(a->first == std::vector<std::uint8_t>{1, 2, 3});
  CHECK(a->second == "image/png");
  CHECK(db.delete_report_asset("cover"));
  CHECK_FALSE(db.delete_report_asset("cover"));
  CHECK_THROWS_AS(db.set_report_asset("cover", {}, "image/png"),
                  std::invalid_argument);
}

TEST_CASE("ReportCover_Unset_ReadsNullAndNoImage", "[report][cover]") {
  TempPath tmp("test_report_preview");
  ProjectDB db(tmp.path);
  CHECK_FALSE(core::report_cover(db).has_value());
  CHECK(core::report_cover_json(db).at("source").is_null());
  CHECK_FALSE(core::report_cover_image(db).has_value());
}

TEST_CASE("ReportCover_Frame_CropsToRegion", "[report][cover]") {
  TempPath tmp("test_report_preview");
  ProjectDB db(tmp.path);
  db.save_sensor_frame(7, two_tone());

  CHECK_THROWS_AS(core::set_report_cover_view(db, {core::ViewKind::frame, 8}),
                  std::out_of_range);
  CHECK_THROWS_AS(
      core::set_report_cover_view(
          db, {core::ViewKind::frame, 7, core::ViewRegion{0.8, 0.0, 0.5, 1.0}}),
      std::invalid_argument); // x + w > 1 on a frame

  core::set_report_cover_view(
      db, {core::ViewKind::frame, 7, core::ViewRegion{0.5, 0.0, 0.5, 1.0}});
  const auto j = core::report_cover_json(db);
  CHECK(j.at("source") == "frame");
  CHECK(j.at("id") == 7);
  CHECK(j.at("region").size() == 4);

  const auto img = core::report_cover_image(db);
  REQUIRE(img);
  CHECK(img->mime_type == "image/jpeg");
  const cv::Mat m = cv::imdecode(img->bytes, cv::IMREAD_COLOR);
  REQUIRE(m.cols == 100);
  CHECK(m.rows == 100);
  const auto px = m.at<cv::Vec3b>(50, 50);
  CHECK(px[0] > 200); // the blue half
  CHECK(px[2] < 60);
}

TEST_CASE("ReportCover_Panorama_RegionWrapsAcrossTheSeam", "[report][cover]") {
  TempPath tmp("test_report_preview");
  ProjectDB db(tmp.path);
  db.save_panoramic_image("pano.jpg", encode(two_tone(), ".jpg"));
  const int id = db.list_panoramic_images().front().id;
  // x = 0.75 .. 1.25: the right quarter (blue) then the left quarter (red).
  core::set_report_cover_view(db, {core::ViewKind::panorama, id,
                                   core::ViewRegion{0.75, 0.0, 0.5, 1.0}});
  CHECK(core::report_cover_json(db).at("source") == "panorama");
  const auto img = core::report_cover_image(db);
  REQUIRE(img);
  const cv::Mat m = cv::imdecode(img->bytes, cv::IMREAD_COLOR);
  REQUIRE(m.cols == 100);
  CHECK(m.at<cv::Vec3b>(50, 10)[0] > 200); // blue first
  CHECK(m.at<cv::Vec3b>(50, 90)[2] > 200); // then red
}

TEST_CASE("ReportCover_Upload_StoresBytesAndClearRemovesThem",
          "[report][cover]") {
  TempPath tmp("test_report_preview");
  ProjectDB db(tmp.path);
  CHECK_THROWS_AS(core::set_report_cover_upload(db, {'G', 'I', 'F', '8'}),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::set_report_cover_upload(
                      db, {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A, 0}),
                  std::invalid_argument); // sniffs as PNG, does not decode
  const auto png = encode(two_tone(), ".png");
  CHECK(core::set_report_cover_upload(db, png) == "image/png");
  CHECK(core::report_cover_json(db).at("source") == "upload");
  CHECK(core::report_cover_json(db).at("mime_type") == "image/png");
  const auto img = core::report_cover_image(db);
  REQUIRE(img);
  CHECK(img->bytes == png);

  // Choosing a frame afterwards drops the upload.
  db.save_sensor_frame(1, two_tone());
  core::set_report_cover_view(db, {core::ViewKind::frame, 1});
  CHECK_FALSE(db.report_asset(core::kReportCoverAsset).has_value());

  core::clear_report_cover(db);
  CHECK_FALSE(core::report_cover(db).has_value());
}

TEST_CASE("ReportDump_SectionFields_ProjectSagsdataTypesSamplesCover",
          "[report][dump]") {
  TempPath tmp("test_report_preview");
  TempDir dir("test_report_preview_dump");
  ProjectDB db(tmp.path);
  seed_project(db);
  using namespace reusex::test_support;
  const auto t1 = make_type(db, "Mursten", core::Treatment::genbrug);
  make_type(db, "Gelænder", core::Treatment::bevaring);
  make_part(db, "RX-001", t1, std::nullopt, 1240.0);
  db.add_sample("PCB i fugemasse", "Vinduesfuge", {t1});

  reusex::ReportOptions opt;
  int fallback_calls = 0;
  opt.cover_fallback = [&](const ProjectDB &) {
    ++fallback_calls;
    return encode(two_tone(), ".png");
  };
  reusex::dump_report_data(db, opt, dir.path / "d");
  CHECK(fallback_calls == 1);
  const auto data = read_json(dir.path / "d/data.json");

  const auto &p = data.at("project");
  CHECK(p.at("name") == "Skolevej 12");
  CHECK(p.at("bbr_number") == "621-123456-1");
  CHECK(p.at("demolished_floor_area_m2") == "2.480 m²");
  CHECK(p.at("demolition_start") == "02.02.2027");
  CHECK(p.at("survey_date") == "14.09.2026");
  CHECK(p.at("cadastral").is_null());

  CHECK(data.at("sagsdata").size() == 13);
  CHECK(data.at("sagsdata").at(0).at("label") == "Adresse");

  REQUIRE(data.at("types").size() == 2);
  CHECK(data.at("types").at(0).at("quantity") == "1.240 stk");
  CHECK(data.at("types").at(0).at("approved") == false);
  CHECK(data.at("types").at(1).at("mass_needed") == false);
  CHECK(data.at("types_summary").at("line") ==
        "2 typer · 0 t i alt · 0 godkendt");

  REQUIRE(data.at("samples").size() == 1);
  CHECK(data.at("samples").at(0).at("code") == "P-01");
  CHECK(data.at("samples").at(0).at("types") == "Mursten");
  CHECK(data.at("samples").at(0).at("result").is_null());
  CHECK(data.at("appendices").at(0).at("letter") == "A");

  // Miljøsanering first: the sample keeps Mursten "afventer".
  REQUIRE_FALSE(data.at("demolition_steps").empty());
  CHECK(data.at("demolition_steps").at(0).at("title") == "Miljøsanering");

  CHECK(data.at("report").at("version") == 1);
  CHECK(data.at("cover_source") == "render");
  CHECK(data.at("cover_path") == "images/cover.png");
  CHECK(fs::is_regular_file(dir.path / "d/images/cover.png"));

  const auto &sections = data.at("sections");
  REQUIRE(sections.size() == 6);
  CHECK(sections.at(0).at("id") == "forside");
  CHECK(sections.at(0).at("ready") == false); // Matrikel, Bygherre, ...
  CHECK_THAT(sections.at(0).at("missing").at(0).get<std::string>(),
             ContainsSubstring("Matrikel"));
  CHECK(sections.at(3).at("id") == "miljoe");
  CHECK_THAT(sections.at(3).at("missing").at(0).get<std::string>(),
             ContainsSubstring("1 prøve afventer svar"));
}

TEST_CASE("ReportDump_ChosenCover_BeatsTheFallback", "[report][dump]") {
  TempPath tmp("test_report_preview");
  TempDir dir("test_report_preview_dump");
  ProjectDB db(tmp.path);
  db.save_sensor_frame(3, two_tone());
  core::set_report_cover_view(db, {core::ViewKind::frame, 3});
  reusex::ReportOptions opt;
  opt.cover_fallback = [](const ProjectDB &) -> std::vector<std::uint8_t> {
    FAIL("the fallback must not run when a cover is chosen");
    return {};
  };
  reusex::dump_report_data(db, opt, dir.path / "d");
  const auto data = read_json(dir.path / "d/data.json");
  CHECK(data.at("cover_source") == "frame");
  CHECK(data.at("cover_path") == "images/cover.jpg");
}

TEST_CASE("ReportDump_FallbackThrows_ReportHasNoCover", "[report][dump]") {
  TempPath tmp("test_report_preview");
  TempDir dir("test_report_preview_dump");
  ProjectDB db(tmp.path);
  reusex::ReportOptions opt;
  opt.cover_fallback = [](const ProjectDB &) -> std::vector<std::uint8_t> {
    throw std::runtime_error("no GL");
  };
  reusex::dump_report_data(db, opt, dir.path / "d");
  const auto data = read_json(dir.path / "d/data.json");
  CHECK(data.at("cover_source") == "none");
  CHECK(data.at("cover_path") == "");
}

TEST_CASE("PreparedReport_Digest_StableAcrossRunsAndTracksData",
          "[report][preview]") {
  TempPath tmp("test_report_preview");
  ProjectDB db(tmp.path);
  seed_project(db);
  const reusex::ReportOptions opt;
  const reusex::PreparedReport a(db, opt);
  const reusex::PreparedReport b(db, opt);
  CHECK(a.digest().size() == 16);
  CHECK(a.digest() == b.digest()); // generated_at differs, digest does not
  CHECK(a.dir() != b.dir());
  REQUIRE(a.sections().size() == 6);
  CHECK(a.sections().at(1).id == "sagsdata");

  db.set_setting(core::sagsdata_setting_key("p1", "client"), "Kolding Kommune");
  const reusex::PreparedReport c(db, opt);
  CHECK(c.digest() != a.digest());
}

TEST_CASE("PreparedReport_CompilePng_OnePngPerPage",
          "[report][preview][typst]") {
  if (!reusex::test_support::have_typst())
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempPath tmp("test_report_preview");
  ProjectDB db(tmp.path);
  seed_project(db);
  const reusex::PreparedReport r(db, {});
  CHECK_THROWS_AS(r.compile_png(5), std::invalid_argument);
  const auto pages = r.compile_png(30);
  REQUIRE(pages.size() >= 6); // one page per section at least
  for (const auto &p : pages) {
    REQUIRE(p.size() > 8);
    CHECK(p[0] == 0x89);
    CHECK(p[1] == 'P');
  }
  const auto pdf = r.compile_pdf();
  CHECK(std::string(pdf.begin(), pdf.begin() + 4) == "%PDF");
}

TEST_CASE("PreparedReport_SectionPages_InDocumentOrder",
          "[report][preview][typst]") {
  if (!reusex::test_support::have_typst())
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempPath tmp("test_report_preview");
  ProjectDB db(tmp.path);
  seed_project(db);
  const reusex::PreparedReport r(db, {});
  const auto pages = r.section_pages();
  REQUIRE(pages.size() >= 6);
  CHECK(pages.front() == std::pair<std::string, int>{"forside", 1});
  CHECK(pages.at(1).first == "sagsdata");
  for (std::size_t i = 1; i < pages.size(); ++i)
    CHECK(pages[i].second > pages[i - 1].second); // each on its own page
}
