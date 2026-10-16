// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/validate.hpp>
#include <types.hpp>

#include "../../support/temp_path.hpp"

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

using namespace reusex;
using reusex::core::MaterialPassport;
using reusex::core::ValidationSeverity;
namespace fs = std::filesystem;

namespace {

struct TempDB : reusex::test_support::TempPath {
  TempDB() : TempPath("test_validate") {}
};

CloudLPtr makeLabelCloud(size_t n) {
  auto c = std::make_shared<CloudL>();
  c->width = static_cast<uint32_t>(n);
  c->height = 1;
  c->is_dense = false;
  c->points.resize(n);
  for (size_t i = 0; i < n; ++i)
    c->points[i].label = static_cast<uint32_t>((i % 2) + 1);
  return c;
}

CloudPtr makeRgbCloud(size_t n) {
  auto c = std::make_shared<Cloud>();
  c->width = static_cast<uint32_t>(n);
  c->height = 1;
  c->is_dense = false;
  c->points.resize(n);
  return c;
}

CloudNPtr makeNormalCloud(size_t n) {
  auto c = std::make_shared<CloudN>();
  c->width = static_cast<uint32_t>(n);
  c->height = 1;
  c->is_dense = false;
  c->points.resize(n);
  return c;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> makeXyzCloud(size_t n) {
  auto c = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  c->width = static_cast<uint32_t>(n);
  c->height = 1;
  c->is_dense = false;
  c->points.resize(n);
  return c;
}

// Populate a project with the per-point and per-plane clouds the mesh stage
// consumes, all consistently sized so the stage contract is satisfied.
void seedMeshInputs(ProjectDB &db, size_t points, size_t planes) {
  db.save_point_cloud("cloud", *makeRgbCloud(points), "create_clouds");
  db.save_point_cloud("normals", *makeNormalCloud(points), "create_clouds");
  db.save_point_cloud("planes", *makeLabelCloud(points), "segment_planes");
  db.save_point_cloud("rooms", *makeLabelCloud(points), "segment_rooms");
  db.save_point_cloud("plane_centroids", *makeXyzCloud(planes),
                      "segment_planes");
  db.save_point_cloud("plane_normals", *makeNormalCloud(planes),
                      "segment_planes");
}

void makePassport(ProjectDB &db, const std::string &guid) {
  MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "test-project");
}

bool hasCheck(const std::vector<reusex::core::ValidationIssue> &issues,
              const std::string &check) {
  return std::any_of(issues.begin(), issues.end(),
                     [&](const auto &i) { return i.check == check; });
}

} // namespace

TEST_CASE("ValidateProject_CleanProject_ReportsNoIssues", "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeLabelCloud(4), "segment_instances");
  std::vector<ProjectDB::InstanceRecord> recs = {{1, "g1", 1, 2},
                                                 {2, "g2", 1, 2}};
  db.save_instances("instances", recs);
  db.save_label_definitions("instances",
                            {{1, "SM1-1 (2p)"}, {2, "SM1-2 (2p)"}});
  makePassport(db, "mat-1");
  db.set_instance_material("instances", 1, "mat-1");

  auto report = reusex::core::validate_project(db);
  // Passport mat-1 is linked; g? passports linked. No errors expected.
  REQUIRE(report.error_count() == 0);
  REQUIRE(report.ok());
}

TEST_CASE("CheckOrphanedPassports_UnlinkedPassport_FlagsWarning",
          "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  makePassport(db, "lonely-passport");

  std::vector<reusex::core::ValidationIssue> issues;
  reusex::core::check_orphaned_passports(db, issues);
  REQUIRE(hasCheck(issues, "orphaned_passport"));
  REQUIRE(std::all_of(issues.begin(), issues.end(), [](const auto &i) {
    return i.severity == ValidationSeverity::warning;
  }));
  // Warnings do not make the project invalid.
  auto report = reusex::core::validate_project(db);
  REQUIRE(report.ok());
}

TEST_CASE("CheckInstancesWithoutLabelDefs_MissingDefinition_FlagsError",
          "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeLabelCloud(4), "segment_instances");
  std::vector<ProjectDB::InstanceRecord> recs = {{1, "g1", 1, 2},
                                                 {2, "g2", 1, 2}};
  db.save_instances("instances", recs);
  // Only define label 1; instance 2 has no definition.
  db.save_label_definitions("instances", {{1, "SM1-1 (2p)"}});

  std::vector<reusex::core::ValidationIssue> issues;
  reusex::core::check_instances_without_label_defs(db, issues);
  REQUIRE(hasCheck(issues, "instance_without_label_def"));

  auto report = reusex::core::validate_project(db);
  REQUIRE_FALSE(report.ok());
}

TEST_CASE("CheckSiblingCloudSizes_MismatchedSizes_FlagsError",
          "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(10), "create_clouds");
  db.save_point_cloud("labels", *makeLabelCloud(7), "annotate"); // mismatch

  std::vector<reusex::core::ValidationIssue> issues;
  reusex::core::check_sibling_cloud_sizes(db, issues);
  REQUIRE(hasCheck(issues, "sibling_size_mismatch"));

  auto report = reusex::core::validate_project(db);
  REQUIRE_FALSE(report.ok());
}

TEST_CASE("CheckSiblingCloudSizes_MatchingSizes_PassesCheck",
          "[core][validate]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(8), "create_clouds");
  db.save_point_cloud("labels", *makeLabelCloud(8), "annotate");

  std::vector<reusex::core::ValidationIssue> issues;
  reusex::core::check_sibling_cloud_sizes(db, issues);
  REQUIRE_FALSE(hasCheck(issues, "sibling_size_mismatch"));
}

// ── Stage input contracts (#222) ───────────────────────────────────────────

using reusex::core::PipelineStage;

TEST_CASE("ParsePipelineStage_NameStringsAndAliases_RoundTrips",
          "[core][validate][stage]") {
  using reusex::core::parse_pipeline_stage;
  using reusex::core::to_string;
  REQUIRE(parse_pipeline_stage("mesh") == PipelineStage::mesh);
  REQUIRE(parse_pipeline_stage("planes") == PipelineStage::planes);
  // "register" is an alias for "optimize".
  REQUIRE(parse_pipeline_stage("register") == PipelineStage::optimize);
  REQUIRE(parse_pipeline_stage("optimize") == PipelineStage::optimize);
  REQUIRE_FALSE(parse_pipeline_stage("bogus").has_value());
  REQUIRE(to_string(PipelineStage::mesh) == "mesh");
}

TEST_CASE("ValidateStage_PlanesMissingInputs_ReportsErrors",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path); // empty project

  auto report = reusex::core::validate_stage(db, PipelineStage::planes);
  REQUIRE_FALSE(report.ok());
  REQUIRE(hasCheck(report.issues, "missing_stage_input"));
}

TEST_CASE("ValidateStage_PlanesInputsAligned_Passes",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(16), "create_clouds");
  db.save_point_cloud("normals", *makeNormalCloud(16), "create_clouds");

  auto report = reusex::core::validate_stage(db, PipelineStage::planes);
  REQUIRE(report.ok());
  REQUIRE(report.error_count() == 0);
}

TEST_CASE("ValidateStage_PlanesInputsMisaligned_ReportsError",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(16), "create_clouds");
  db.save_point_cloud("normals", *makeNormalCloud(15), "create_clouds");

  auto report = reusex::core::validate_stage(db, PipelineStage::planes);
  REQUIRE_FALSE(report.ok());
  REQUIRE(hasCheck(report.issues, "stage_input_size_mismatch"));
}

TEST_CASE("ValidateStage_MeshFullInputSet_Passes", "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seedMeshInputs(db, /*points=*/32, /*planes=*/5);

  auto report = reusex::core::validate_stage(db, PipelineStage::mesh);
  REQUIRE(report.ok());
  REQUIRE(report.error_count() == 0);
}

TEST_CASE("ValidateStage_MeshMissingRooms_FlagsMissingInput",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seedMeshInputs(db, 32, 5);
  // Remove one required sibling to break the contract.
  db.delete_point_cloud("rooms");

  auto report = reusex::core::validate_stage(db, PipelineStage::mesh);
  REQUIRE_FALSE(report.ok());
  REQUIRE(hasCheck(report.issues, "missing_stage_input"));
}

TEST_CASE("ValidateStage_ImportEmptyProject_Passes",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path); // empty project
  auto report = reusex::core::validate_stage(db, PipelineStage::import);
  REQUIRE(report.ok());
}

// ── Stages added when the contract was consolidated (#246) ─────────────────

TEST_CASE("ParsePipelineStage_EveryTableEntry_Parses",
          "[core][validate][stage]") {
  for (const auto &name : reusex::core::pipeline_stage_names())
    REQUIRE(reusex::core::parse_pipeline_stage(name).has_value());
  // The four stages the CLI validated privately before #246.
  REQUIRE(reusex::core::parse_pipeline_stage("texture") ==
          PipelineStage::texture);
  REQUIRE(reusex::core::parse_pipeline_stage("windows") ==
          PipelineStage::windows);
  REQUIRE(reusex::core::parse_pipeline_stage("annotate") ==
          PipelineStage::annotate);
  REQUIRE(reusex::core::parse_pipeline_stage("project") ==
          PipelineStage::project);
}

TEST_CASE("ValidateStage_TextureMissingMesh_ReportsError",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(8), "create_clouds");

  auto report = reusex::core::validate_stage(db, PipelineStage::texture);
  REQUIRE_FALSE(report.ok());
  REQUIRE(hasCheck(report.issues, "missing_stage_input"));
}

TEST_CASE("ValidateStage_WindowsMissingInputs_ReportsErrors",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(8), "create_clouds");

  auto report = reusex::core::validate_stage(db, PipelineStage::windows);
  REQUIRE_FALSE(report.ok());
  REQUIRE(hasCheck(report.issues, "missing_stage_input"));
}

TEST_CASE("ValidateStage_InstancesPlanesFallback_Passes",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(12), "create_clouds");
  db.save_point_cloud("planes", *makeLabelCloud(12), "segment_planes");

  // No `labels` cloud, but the contract accepts the geometric fallback.
  auto report = reusex::core::validate_stage(db, PipelineStage::instances);
  REQUIRE(report.ok());
}

TEST_CASE("ValidateStage_InstancesSemanticCloudOverride_DropsFallback",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", *makeRgbCloud(12), "create_clouds");
  db.save_point_cloud("planes", *makeLabelCloud(12), "segment_planes");

  // `--semantic-cloud custom` is an instruction: the missing cloud must be
  // refused even though the contract's fallback happens to be present.
  auto refused = reusex::core::validate_stage(db, PipelineStage::instances,
                                              {{"labels", "custom"}});
  REQUIRE_FALSE(refused.ok());

  db.save_point_cloud("custom", *makeLabelCloud(12), "annotate");
  auto accepted = reusex::core::validate_stage(db, PipelineStage::instances,
                                               {{"labels", "custom"}});
  REQUIRE(accepted.ok());
}

TEST_CASE("ValidateStage_MissingInput_HintNamesProducingCommand",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path); // empty project

  auto report = reusex::core::validate_stage(db, PipelineStage::mesh);
  REQUIRE_FALSE(report.ok());

  // The hint is derived from the table, so it must name the whole chain of
  // stages that has to run — not just the immediate producer.
  bool found = false;
  for (const auto &issue : report.issues) {
    if (issue.hint.empty())
      continue;
    found = true;
    INFO(issue.hint);
    CHECK(issue.hint.find("rux create clouds") != std::string::npos);
  }
  REQUIRE(found);
}

TEST_CASE("ValidateStage_PlanesUnsatisfiedChain_HintOrdersPrerequisites",
          "[core][validate][stage]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  // `cloud` exists, `normals` does not — but regenerating normals means
  // re-running `create clouds`, which itself has no sensor frames to work
  // from, so the hint has to name the import too rather than send the user
  // round a command that would also fail.
  db.save_point_cloud("cloud", *makeRgbCloud(8), "create_clouds");

  auto report = reusex::core::validate_stage(db, PipelineStage::planes);
  REQUIRE_FALSE(report.ok());
  REQUIRE(report.issues.size() == 1);
  const auto &hint = report.issues.front().hint;
  INFO(hint);
  CHECK(hint.find("rux import") != std::string::npos);
  CHECK(hint.find("rux create clouds") != std::string::npos);
  // Ordered: import comes before the stage that consumes what it produces.
  CHECK(hint.find("rux import") < hint.find("rux create clouds"));
}

TEST_CASE("ValidateStage_RoomsWithoutOptionalInputs_Passes",
          "[core][validate][stage]") {
  // W5.4: walls / openings / storeys are optional inputs of `rooms`; a
  // project that never ran those stages still validates, and the missing
  // ones are no issue (they would be, as required inputs).
  TempDB tmp;
  ProjectDB db(tmp.path);
  seedMeshInputs(db, 32, 5);
  db.delete_point_cloud("rooms");
  auto report = reusex::core::validate_stage(db, PipelineStage::rooms);
  INFO(report.issues.size() << " issue(s)");
  REQUIRE(report.ok());
  CHECK(report.error_count() == 0);
  for (const auto &issue : report.issues) {
    INFO(issue.message);
    CHECK(issue.message.find("walls") == std::string::npos);
    CHECK(issue.message.find("openings") == std::string::npos);
  }
}

TEST_CASE("ValidateStage_RoomsOptionalNormalsMisaligned_ReportsError",
          "[core][validate][stage]") {
  // An optional input that IS present is checked like any other: a
  // `normals` cloud of the wrong size is an alignment error.
  TempDB tmp;
  ProjectDB db(tmp.path);
  seedMeshInputs(db, 32, 5);
  db.save_point_cloud("normals", *makeNormalCloud(31), "create_clouds");
  auto report = reusex::core::validate_stage(db, PipelineStage::rooms);
  CHECK_FALSE(report.ok());
}
