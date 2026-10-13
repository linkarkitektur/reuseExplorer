// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux validate --stage X` and `rux create X` must never disagree (#246).
//
// Before the contract was consolidated they were two separate implementations,
// so a stage could validate clean and then be refused (or, worse, validate
// dirty and run anyway). Both now read the same table; these tests assert the
// observable consequence rather than the shared call, so a future front end
// that re-grows its own copy fails here.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/stage_contract.hpp>
#include <core/validate.hpp>
#include <pipeline/stages.hpp>
#include <types/point_types.hpp>

#include "../../support/temp_path.hpp"

#include <string>
#include <vector>

using namespace reusex;
using reusex::test_support::TempPath;

namespace {

CloudPtr rgb_cloud(size_t n) {
  auto cloud = std::make_shared<Cloud>();
  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->points.resize(n);
  return cloud;
}

CloudNPtr normal_cloud(size_t n) {
  auto cloud = std::make_shared<CloudN>();
  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->points.resize(n);
  return cloud;
}

CloudLPtr label_cloud(size_t n) {
  auto cloud = std::make_shared<CloudL>();
  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->points.resize(n);
  for (size_t i = 0; i < n; ++i)
    cloud->points[i].label = static_cast<uint32_t>((i % 2) + 1);
  return cloud;
}

/// The core::PipelineStage each runnable JobStage validates against.
core::PipelineStage contract_of(pipeline::JobStage stage) {
  switch (stage) {
  case pipeline::JobStage::clouds:
    return core::PipelineStage::clouds;
  case pipeline::JobStage::planes:
    return core::PipelineStage::planes;
  case pipeline::JobStage::rooms:
    return core::PipelineStage::rooms;
  case pipeline::JobStage::instances:
    return core::PipelineStage::instances;
  case pipeline::JobStage::mesh:
    return core::PipelineStage::mesh;
  case pipeline::JobStage::optimize:
    return core::PipelineStage::optimize;
  case pipeline::JobStage::annotate:
    return core::PipelineStage::annotate;
  case pipeline::JobStage::graph_discover:
    return core::PipelineStage::graph_discover;
  }
  FAIL("JobStage has no contract mapping");
  return core::PipelineStage::clouds;
}

const std::vector<pipeline::JobStage> &runnable_stages() {
  static const std::vector<pipeline::JobStage> stages = {
      pipeline::JobStage::clouds, pipeline::JobStage::planes,
      pipeline::JobStage::rooms, pipeline::JobStage::instances,
      pipeline::JobStage::mesh};
  return stages;
}

} // namespace

TEST_CASE("ValidateAndRunStage_EmptyProject_AgreeOnRefusal",
          "[pipeline][stages][contract]") {
  for (auto stage : runnable_stages()) {
    TempPath project("test_contract_agreement");
    ProjectDB db(project.path);

    INFO("stage " << pipeline::to_string(stage));

    const auto report = core::validate_stage(db, contract_of(stage));
    REQUIRE_FALSE(report.ok());

    pipeline::StageContext ctx;
    ctx.project = project.path;
    ctx.stage = stage;
    const auto result = pipeline::run_stage(db, ctx);

    // Refused, and refused as bad input rather than as an internal failure.
    CHECK_FALSE(result.ok);
    CHECK(result.invalid_input);

    // The refusal must carry the same reason the validator reported, so a user
    // who ran `rux validate --stage` first recognises the message.
    for (const auto &issue : report.issues) {
      if (issue.severity != core::ValidationSeverity::error)
        continue;
      INFO("expected reason: " << issue.message);
      CHECK(result.message.find(issue.message) != std::string::npos);
    }
  }
}

TEST_CASE("ValidateAndRunStage_SatisfiedContract_AgreeNotRefused",
          "[pipeline][stages][contract]") {
  TempPath project("test_contract_agreement");
  ProjectDB db(project.path);
  db.save_point_cloud("cloud", *rgb_cloud(24), "cloud_reconstruction");
  db.save_point_cloud("normals", *normal_cloud(24), "cloud_reconstruction");

  REQUIRE(core::validate_stage(db, core::PipelineStage::planes).ok());

  pipeline::StageContext ctx;
  ctx.project = project.path;
  ctx.stage = pipeline::JobStage::planes;
  const auto result = pipeline::run_stage(db, ctx);

  // The stage may or may not find planes in a degenerate synthetic cloud, but
  // it must not be REFUSED — that is the drift this issue was about.
  CHECK_FALSE(result.invalid_input);
}

TEST_CASE("ValidateAndRunStage_MisalignedSiblingClouds_AgreeOnRefusal",
          "[pipeline][stages][contract]") {
  TempPath project("test_contract_agreement");
  ProjectDB db(project.path);
  db.save_point_cloud("cloud", *rgb_cloud(24), "cloud_reconstruction");
  db.save_point_cloud("normals", *normal_cloud(23), "cloud_reconstruction");

  const auto report = core::validate_stage(db, core::PipelineStage::planes);
  REQUIRE_FALSE(report.ok());

  pipeline::StageContext ctx;
  ctx.project = project.path;
  ctx.stage = pipeline::JobStage::planes;
  const auto result = pipeline::run_stage(db, ctx);
  CHECK(result.invalid_input);
  CHECK(result.message.find("index-aligned") != std::string::npos);
}

TEST_CASE("RunStage_SemanticCloudOverride_RefusedLikeValidator",
          "[pipeline][stages][contract]") {
  TempPath project("test_contract_agreement");
  ProjectDB db(project.path);
  db.save_point_cloud("cloud", *rgb_cloud(16), "cloud_reconstruction");
  db.save_point_cloud("planes", *label_cloud(16), "segment_planes");

  // The contract's fallback (`planes`) is present, but the caller explicitly
  // asked for a cloud that is not — both doors must refuse.
  const auto report = core::validate_stage(db, core::PipelineStage::instances,
                                           {{"labels", "missing_cloud"}});
  REQUIRE_FALSE(report.ok());

  pipeline::StageContext ctx;
  ctx.project = project.path;
  ctx.stage = pipeline::JobStage::instances;
  ctx.parameters = R"({"semantic_cloud":"missing_cloud"})";
  const auto result = pipeline::run_stage(db, ctx);
  CHECK_FALSE(result.ok);
  CHECK(result.invalid_input);
}
