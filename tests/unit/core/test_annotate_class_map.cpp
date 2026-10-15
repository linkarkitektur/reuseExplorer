// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// core::annotated_label_definitions() (reusex-z1v.2.2): the class names of
// the LATEST `annotate_class_map` pipeline-log entry, re-keyed from API class
// id to point label. Shared by `rux create clouds` and `rux create project`.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/annotate_class_map.hpp>

#include <map>
#include <string>

using reusex::ProjectDB;
using reusex::core::annotated_label_definitions;

namespace {
void log_class_map(ProjectDB &db, const std::string &json) {
  const int id = db.log_pipeline_start("annotate_class_map", json);
  db.log_pipeline_end(id, true);
}
} // namespace

TEST_CASE("AnnotatedLabelDefinitions_NoAnnotateRun_IsEmpty",
          "[label_semantics][annotate_class_map]") {
  ProjectDB db(":memory:");
  CHECK(annotated_label_definitions(db).empty());
}

TEST_CASE("AnnotatedLabelDefinitions_KeyedByPointLabel",
          "[label_semantics][annotate_class_map]") {
  ProjectDB db(":memory:");
  log_class_map(db, R"({"0":"wall","1":"floor"})");
  CHECK(annotated_label_definitions(db) ==
        std::map<int, std::string>{{1, "wall"}, {2, "floor"}});
}

TEST_CASE("AnnotatedLabelDefinitions_SeveralRuns_LatestWins",
          "[label_semantics][annotate_class_map]") {
  ProjectDB db(":memory:");
  log_class_map(db, R"({"0":"door"})");
  // An unrelated stage in between must not matter.
  const int other = db.log_pipeline_start("project_labels", "{}");
  db.log_pipeline_end(other, true);
  log_class_map(db, R"({"0":"wall","1":"floor"})");

  CHECK(annotated_label_definitions(db) ==
        std::map<int, std::string>{{1, "wall"}, {2, "floor"}});
}

TEST_CASE("AnnotatedLabelDefinitions_UnparsableLatest_IsEmpty",
          "[label_semantics][annotate_class_map]") {
  // A broken latest entry must not silently fall back to an older run's names
  // (they could belong to a different prompt list).
  ProjectDB db(":memory:");
  log_class_map(db, R"({"0":"door"})");
  log_class_map(db, "not json");
  CHECK(annotated_label_definitions(db).empty());
}
