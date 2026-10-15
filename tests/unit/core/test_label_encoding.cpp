// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core/label_encoding.hpp (reusex-z1v.2.1 review): which label clouds are
// semantic, the encoding stamp, and the loud path for an unstamped semantic
// cloud in a v29+ file (written by a pre-v29 binary after the migration ran).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/label_encoding.hpp>
#include <reusex/core/label_semantics.hpp>
#include <reusex/core/validate.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

using reusex::CloudL;
using reusex::ProjectDB;
using reusex::test_support::TempPath;
using namespace reusex::core;

namespace {

CloudL two_labels() {
  CloudL c;
  c.resize(2);
  c[0].label = 0;
  c[1].label = 1;
  return c;
}

bool has_check(const std::vector<ValidationIssue> &issues,
               const std::string &check, const std::string &needle) {
  return std::any_of(issues.begin(), issues.end(), [&](const auto &i) {
    return i.check == check && i.message.find(needle) != std::string::npos;
  });
}

} // namespace

TEST_CASE("IsSemanticLabelCloud_NameAndStage_ClassifiesStructuralAsNot",
          "[label_semantics][label_encoding]") {
  CHECK(is_semantic_label_cloud("labels", ""));
  CHECK(is_semantic_label_cloud("labels", "sync_downsample"));
  CHECK(is_semantic_label_cloud("labels", "segment_resource"));
  CHECK(is_semantic_label_cloud("other", "reconstruct"));
  CHECK(is_semantic_label_cloud("other", "project_labels"));
  CHECK_FALSE(is_semantic_label_cloud("planes", "segment_planes"));
  CHECK_FALSE(is_semantic_label_cloud("rooms", "segment_rooms"));
  CHECK_FALSE(is_semantic_label_cloud("instances", "segment_instances"));
  CHECK_FALSE(is_semantic_label_cloud("scan_labels", "import_ply"));
  // A stage that only writes structural clouds wins over the name.
  CHECK_FALSE(is_semantic_label_cloud("labels", "segment_instances"));
}

TEST_CASE("StampLabelEncoding_VariousParams_AddsKeyKeepsRest",
          "[label_semantics][label_encoding]") {
  CHECK(has_api_plus_one_encoding(stamp_label_encoding("")));
  CHECK(has_api_plus_one_encoding(stamp_label_encoding("{}")));
  const auto j = nlohmann::json::parse(stamp_label_encoding(R"({"a":1})"));
  CHECK(j.at("a") == 1);
  CHECK(j.at(kLabelEncodingKey) == kLabelEncodingApiPlusOne);
  CHECK_FALSE(has_api_plus_one_encoding(""));
  CHECK_FALSE(has_api_plus_one_encoding("not json"));
  CHECK_FALSE(has_api_plus_one_encoding(R"({"label_encoding":"api"})"));
}

TEST_CASE("UnstampedSemanticCloud_InV29File_IsReportedAndFlaggedByValidate",
          "[label_semantics][label_encoding][validate]") {
  TempPath tmp("test_label_encoding_unstamped");
  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() >= 29);
  // What a pre-v29 binary writes into an already-migrated file.
  db.save_point_cloud("labels", two_labels(), "project_labels");
  db.save_point_cloud("planes", two_labels(), "segment_planes");

  CHECK(is_unstamped_semantic_cloud(db, "labels"));
  CHECK_FALSE(is_unstamped_semantic_cloud(db, "planes"));
  CHECK_FALSE(is_unstamped_semantic_cloud(db, "missing"));
  CHECK(warn_if_unstamped_semantic_cloud(db, "labels"));

  std::vector<ValidationIssue> issues;
  check_unstamped_semantic_clouds(db, issues);
  CHECK(has_check(issues, "semantic_cloud_unstamped", "labels"));
  CHECK_FALSE(has_check(issues, "semantic_cloud_unstamped", "planes"));
  CHECK(validate_project(db).warning_count() >= 1);

  db.save_point_cloud("labels", two_labels(), "project_labels",
                      stamp_label_encoding(""));
  CHECK_FALSE(is_unstamped_semantic_cloud(db, "labels"));
  CHECK_FALSE(warn_if_unstamped_semantic_cloud(db, "labels"));
  issues.clear();
  check_unstamped_semantic_clouds(db, issues);
  CHECK(issues.empty());
}
