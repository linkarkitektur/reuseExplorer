// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// sensor_frames.initial_transform (the snapshot `rux optimize` seeds from) and
// project_settings (spec §3.2, §3.3).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <stdexcept>

using reusex::ProjectDB;
using namespace reusex::test_support;

namespace {
void seed(ProjectDB &db, int id, std::array<double, 16> pose) {
  db.save_sensor_frame(id, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
                       pose, make_intrinsics(8, 8, 4, 4, 8, 8), 0.0);
}
} // namespace

TEST_CASE("InitialPose_SetAndRead_IsIndependentOfTransform",
          "[projectdb][initial_pose]") {
  TempPath tmp("test_initial_pose");
  ProjectDB db(tmp.path);
  seed(db, 3, translation_pose(1, 2, 3));
  CHECK_FALSE(db.has_initial_pose(3));
  CHECK_FALSE(db.initial_pose(3).has_value());

  db.set_initial_pose(3, translation_pose(7, 8, 9));
  REQUIRE(db.has_initial_pose(3));
  CHECK((*db.initial_pose(3))[3] == 7.0);
  CHECK(db.sensor_frame_pose(3)[3] == 1.0); // transform untouched

  db.update_sensor_frame_pose(3, translation_pose(4, 4, 4));
  CHECK((*db.initial_pose(3))[3] == 7.0); // and vice versa
}

TEST_CASE("InitialPose_InvalidInput_Throws", "[projectdb][initial_pose]") {
  TempPath tmp("test_initial_pose_invalid");
  ProjectDB db(tmp.path);
  seed(db, 3, identity16());
  CHECK_THROWS_AS(db.set_initial_pose(99, identity16()), std::runtime_error);
  CHECK_THROWS_AS(db.set_initial_pose(3, zero_pose()), std::invalid_argument);
  CHECK_THROWS_AS(db.set_initial_pose(3, nan_translation_pose()),
                  std::invalid_argument);
}

TEST_CASE("InitialPose_ReadOnlyV26_ReadsAsAbsent",
          "[projectdb][initial_pose]") {
  TempPath tmp("test_initial_pose_v26");
  {
    ProjectDB db(tmp.path);
    seed(db, 3, identity16());
  }
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
  REQUIRE(sqlite3_exec(raw,
                       "ALTER TABLE sensor_frames DROP COLUMN "
                       "initial_transform; DROP TABLE project_settings; "
                       "DELETE FROM schema_version WHERE version >= 27;",
                       nullptr, nullptr, nullptr) == SQLITE_OK);
  sqlite3_close(raw);
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_FALSE(ro.has_initial_pose(3));
  CHECK_FALSE(ro.initial_pose(3).has_value());
  CHECK_FALSE(ro.setting("view.yaw_deg").has_value());
}

TEST_CASE("ProjectSettings_RoundTripAndOverwrite", "[projectdb][settings]") {
  TempPath tmp("test_project_settings");
  {
    ProjectDB db(tmp.path);
    CHECK_FALSE(db.setting("view.yaw_deg").has_value());
    db.set_setting("view.yaw_deg", "12.5");
    db.set_setting("view.yaw_deg", "-30");
    db.set_setting("other", "x");
  }
  ProjectDB db(tmp.path, /*readOnly=*/true);
  CHECK(db.setting("view.yaw_deg") == std::optional<std::string>("-30"));
  CHECK(db.setting("other") == std::optional<std::string>("x"));
  CHECK_THROWS_AS(db.set_setting("k", "v"), std::runtime_error);
}
