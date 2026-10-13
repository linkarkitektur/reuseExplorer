// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// ProjectDB::set_survey_part_view and the view a survey part reads back
// (spec 2026-10-13-resource-view-record §2): validation, existence of the
// source, clearing, and a dangling view reading as none.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/logging.hpp>
#include <core/resource_view.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using namespace reusex::test_support;

namespace {
struct TempDB : TempPath {
  TempDB() : TempPath("test_survey_view") {}
};
std::vector<std::uint8_t> tiny_jpeg() {
  std::vector<std::uint8_t> out;
  REQUIRE(
      cv::imencode(".jpg", cv::Mat(8, 16, CV_8UC3, cv::Scalar(1, 2, 3)), out));
  return out;
}
/// A part RX-001, sensor frame 5 and panorama 1.
void seed(ProjectDB &db) {
  make_part(db, "RX-001", make_type(db, "Døre"));
  db.save_sensor_frame(5, cv::Mat(8, 8, CV_8UC3, cv::Scalar(1, 2, 3)));
  db.save_panoramic_image("a.jpg", tiny_jpeg());
}
struct WarnCapture {
  std::vector<std::string> lines;
  LogLevel saved = get_log_level();
  WarnCapture() {
    set_log_handler([this](LogLevel l, std::string_view m) {
      if (l == LogLevel::warn)
        lines.emplace_back(m);
    });
    set_log_level(LogLevel::info);
  }
  ~WarnCapture() {
    reset_log_handler();
    set_log_level(saved);
  }
};
} // namespace

TEST_CASE("SetSurveyPartView_FrameAndPanorama_RoundTripAndClear",
          "[ProjectDB][survey][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  const ResourceView frame{ViewKind::frame, 5, ViewRegion{0.1, 0.2, 0.3, 0.4}};
  db.set_survey_part_view("RX-001", frame);
  CHECK(db.survey_part("RX-001")->view == frame);
  CHECK(db.survey_parts().at(0).view == frame);
  const ResourceView pano{ViewKind::panorama, 1,
                          ViewRegion{0.9, 0.3, 0.2, 0.25}};
  db.set_survey_part_view("RX-001", pano); // wraps the seam: allowed
  CHECK(db.survey_part("RX-001")->view == pano);
  db.set_survey_part_view("RX-001",
                          ResourceView{ViewKind::frame, 5, std::nullopt});
  CHECK_FALSE(db.survey_part("RX-001")->view->region);
  db.set_survey_part_view("RX-001", std::nullopt);
  CHECK_FALSE(db.survey_part("RX-001")->view);
}

TEST_CASE("SetSurveyPartView_BadInput_ThrowsAndWritesNothing",
          "[ProjectDB][survey][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  CHECK_THROWS_AS(db.set_survey_part_view(
                      "RX-404", ResourceView{ViewKind::frame, 5, std::nullopt}),
                  std::out_of_range);
  CHECK_THROWS_AS(db.set_survey_part_view(
                      "RX-001", ResourceView{ViewKind::frame, 6, std::nullopt}),
                  std::out_of_range); // no such frame
  CHECK_THROWS_AS(
      db.set_survey_part_view(
          "RX-001", ResourceView{ViewKind::panorama, 9, std::nullopt}),
      std::out_of_range); // no such panorama
  CHECK_THROWS_AS(db.set_survey_part_view(
                      "RX-001", ResourceView{ViewKind::frame, 5,
                                             ViewRegion{0.9, 0, 0.2, 0.5}}),
                  std::invalid_argument); // a frame cannot wrap
  CHECK_THROWS_AS(
      db.set_survey_part_view(
          "RX-001", ResourceView{ViewKind::frame, 5, ViewRegion{0, 0, 0, 0.5}}),
      std::invalid_argument);
  CHECK_FALSE(db.survey_part("RX-001")->view);
}

TEST_CASE("SetSurveyPartView_RoundedEdgeRegion_IsAccepted",
          "[ProjectDB][survey][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  const auto r = pad_region({0.1, 0.7, 0.5, 0.3}); // clamped at the bottom
  CHECK_NOTHROW(
      db.set_survey_part_view("RX-001", ResourceView{ViewKind::frame, 5, r}));
  CHECK_NOTHROW(db.set_survey_part_view(
      "RX-001", ResourceView{ViewKind::frame, 5,
                             ViewRegion{0.0, 0.3, 1.0, 0.7000000000000001}}));
}

TEST_CASE("SurveyPartView_DanglingPanorama_ReadsNullAndWarns",
          "[ProjectDB][survey][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  db.set_survey_part_view("RX-001",
                          ResourceView{ViewKind::panorama, 1, std::nullopt});
  db.delete_panoramic_image(1);
  WarnCapture warns;
  CHECK_FALSE(db.survey_part("RX-001")->view);
  REQUIRE(warns.lines.size() == 1);
  CHECK(warns.lines[0].find("RX-001") != std::string::npos);
}

TEST_CASE("HasPanoramicImage_ById_TrueOnlyForStoredIds",
          "[ProjectDB][panorama]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  CHECK(db.has_panoramic_image(1));
  CHECK_FALSE(db.has_panoramic_image(2));
  CHECK(db.has_panoramic_image("a.jpg")); // the filename overload still works
}
