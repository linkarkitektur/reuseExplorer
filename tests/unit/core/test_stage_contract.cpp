// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Integrity tests for the single stage-contract table (#246).
//
// The table is data, so these tests are what keeps it honest: every enumerator
// covered, every artifact it names registered, every stage producing something
// (in the project or, for a leaf like `gsplat`, outside it), the stage order a
// real DAG, and docs/CONTRACTS.md — a prose mirror, not a second source of
// truth — saying the same thing the table does.

#include <catch2/catch_test_macros.hpp>

#include <core/stage_contract.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "../../support/source_dir.hpp"

using reusex::core::Alignment;
using reusex::core::ArtifactKind;
using reusex::core::find_artifact;
using reusex::core::parse_pipeline_stage;
using reusex::core::pipeline_artifacts;
using reusex::core::pipeline_stage_names;
using reusex::core::PipelineStage;
using reusex::core::producing_stage;
using reusex::core::stage_contract;
using reusex::core::stage_contracts;
using reusex::core::to_string;

namespace {

// Every enumerator of PipelineStage. C++ has no reflection, so this list is
// maintained by hand — but it cannot silently fall behind: the size assertion
// below fails the moment the table grows or shrinks without it.
const std::vector<PipelineStage> &all_stages() {
  static const std::vector<PipelineStage> stages = {
      PipelineStage::import,         PipelineStage::graph_init,
      PipelineStage::graph_discover, PipelineStage::optimize,
      PipelineStage::clouds,         PipelineStage::annotate,
      PipelineStage::project,        PipelineStage::planes,
      PipelineStage::storeys,        PipelineStage::rooms,
      PipelineStage::instances,      PipelineStage::classify,
      PipelineStage::walls,          PipelineStage::mesh,
      PipelineStage::texture,        PipelineStage::windows,
      PipelineStage::materials,      PipelineStage::survey,
      PipelineStage::attributes,     PipelineStage::gsplat,
  };
  return stages;
}

size_t stage_index(PipelineStage stage) {
  const auto &contracts = stage_contracts();
  for (size_t i = 0; i < contracts.size(); ++i)
    if (contracts[i].stage == stage)
      return i;
  FAIL("stage is not in the contract table");
  return 0;
}

} // namespace

TEST_CASE("StageContractTable_AllPipelineStages_HaveExactlyOneRow",
          "[core][contract]") {
  const auto &contracts = stage_contracts();
  REQUIRE(contracts.size() == all_stages().size());

  for (auto stage : all_stages()) {
    const auto matches =
        std::count_if(contracts.begin(), contracts.end(),
                      [&](const auto &c) { return c.stage == stage; });
    INFO("stage " << to_string(stage));
    REQUIRE(matches == 1);
  }
}

TEST_CASE("StageContractTable_RowOrder_MatchesEnumOrder", "[core][contract]") {
  // The interpreter's hint recursion and the DAG rule below both rely on table
  // position meaning "pipeline position", so the two orders must not diverge.
  const auto &contracts = stage_contracts();
  const auto &stages = all_stages();
  for (size_t i = 0; i < contracts.size(); ++i)
    CHECK(contracts[i].stage == stages[i]);
}

TEST_CASE("StageContractTable_NamesAndAliases_AreUniqueAndParseable",
          "[core][contract]") {
  std::set<std::string> seen;
  for (const auto &contract : stage_contracts()) {
    INFO("stage " << contract.name);
    CHECK_FALSE(contract.name.empty());
    CHECK_FALSE(contract.command.empty());
    CHECK_FALSE(contract.summary.empty());

    REQUIRE(seen.insert(std::string(contract.name)).second);
    REQUIRE(parse_pipeline_stage(contract.name) == contract.stage);
    REQUIRE(to_string(contract.stage) == contract.name);

    for (const auto &alias : contract.aliases) {
      REQUIRE(seen.insert(std::string(alias)).second);
      REQUIRE(parse_pipeline_stage(alias) == contract.stage);
    }
  }

  // pipeline_stage_names() drives `rux validate --help`, so it must list every
  // token the parser accepts and nothing else.
  const auto names = pipeline_stage_names();
  REQUIRE(names.size() == seen.size());
  for (const auto &name : names)
    CHECK(seen.count(name) == 1);

  CHECK_FALSE(parse_pipeline_stage("bogus").has_value());
}

TEST_CASE("PipelineStageNames_GsplatToken_ParsesAndRoundTrips",
          "[core][contract]") {
  // The generic case above derives the expected spelling from the row it is
  // checking, so it stays green if the row is renamed to "splat" — the table
  // would simply agree with itself. This one pins the literal token, which is
  // what `rux validate --stage gsplat` accepts and what the gsplat section
  // heading in docs/CONTRACTS.md spells. Renaming it is a CLI break, and this
  // is what would catch one.
  REQUIRE(parse_pipeline_stage("gsplat") == PipelineStage::gsplat);
  CHECK(to_string(PipelineStage::gsplat) == "gsplat");

  const auto names = pipeline_stage_names();
  CHECK(std::find(names.begin(), names.end(), "gsplat") != names.end());
}

TEST_CASE("StageContractTable_NamedArtifacts_AreAllRegistered",
          "[core][contract]") {
  for (const auto &contract : stage_contracts()) {
    INFO("stage " << contract.name);
    for (const auto &input : contract.inputs) {
      REQUIRE_FALSE(input.any_of.empty());
      for (const auto &name : input.any_of) {
        INFO("input " << name);
        REQUIRE(find_artifact(name) != nullptr);
      }
    }
    for (const auto &output : contract.outputs) {
      INFO("output " << output);
      REQUIRE(find_artifact(output) != nullptr);
    }
  }
}

TEST_CASE("ArtifactRegistry_Entries_HaveNoDuplicates", "[core][contract]") {
  std::set<std::string> seen;
  for (const auto &artifact : pipeline_artifacts()) {
    INFO("artifact " << artifact.name);
    CHECK_FALSE(artifact.description.empty());
    REQUIRE(seen.insert(std::string(artifact.name)).second);
    // Only point clouds are index-aligned with anything.
    if (artifact.kind != ArtifactKind::point_cloud)
      CHECK(artifact.alignment == Alignment::none);
  }
}

TEST_CASE("StageContractTable_Inputs_ProducedByEarlierStages",
          "[core][contract]") {
  // This is the "outputs of stage N are the inputs of stage N+1" rule,
  // generalised so the annotate -> project -> instances branch does not have to
  // be a straight line. It is also what makes the resolution-hint recursion in
  // check_stage_inputs() terminate.
  for (const auto &contract : stage_contracts()) {
    const size_t consumer = stage_index(contract.stage);
    for (const auto &input : contract.inputs) {
      for (const auto &name : input.any_of) {
        INFO("stage " << contract.name << " input " << name);
        const auto producer = producing_stage(name);
        REQUIRE(producer.has_value());
        CHECK(stage_index(*producer) < consumer);
      }
    }
  }
}

TEST_CASE("StageContractTable_Outputs_HaveFirstProducer", "[core][contract]") {
  for (const auto &contract : stage_contracts()) {
    for (const auto &output : contract.outputs) {
      INFO("stage " << contract.name << " output " << output);
      const auto producer = producing_stage(output);
      REQUIRE(producer.has_value());
      // `optimize` rewrites sensor_frames in place, so the reported producer is
      // the earliest one — never a later stage than the one declaring it.
      CHECK(stage_index(*producer) <= stage_index(contract.stage));
    }
  }
}

TEST_CASE("StageContractTable_EveryStage_ProducesInProjectOrExternalArtifact",
          "[core][contract]") {
  // A stage that produces nothing at all is either a half-filled table row or a
  // stage that does not belong in the pipeline. The invariant is "produces
  // something", and `external_outputs` carries the other half of "something"
  // for a stage whose whole product is a file on disk.
  //
  // There is no such stage today: `gsplat` was the only one, and #322 moved its
  // model into the project. The branch is kept rather than deleted because the
  // *reason* it exists has not gone away — a future stage whose product cannot
  // be a `ProjectDB` row still needs somewhere to say so, and `outputs` is not
  // it (see the field's comment). So this asserts the invariant and the
  // in-project witness, and deliberately does not require an external-only one.
  bool saw_in_project_producer = false;
  for (const auto &contract : stage_contracts()) {
    INFO("stage " << contract.name);
    CHECK((!contract.outputs.empty() || !contract.external_outputs.empty()));

    if (!contract.outputs.empty())
      saw_in_project_producer = true;
  }
  CHECK(saw_in_project_producer);
}

TEST_CASE("StageContractTable_InProjectProducers_DeclareNoExternalOutputs",
          "[core][contract]") {
  // The two fields are alternatives, not a pair. A stage whose product lands in
  // the `.rux` is fully described by `outputs`; docs/CONTRACTS.md renders an
  // `| External |` row only for a stage that has nothing there, and the doc
  // mirror below asserts exactly that correspondence. A row filling in both
  // would make "Produces: nothing in the project" a half-truth for some stage
  // and leave the reader with two places to look for the same answer.
  for (const auto &contract : stage_contracts()) {
    INFO("stage " << contract.name);
    if (!contract.outputs.empty())
      CHECK(contract.external_outputs.empty());
  }
}

TEST_CASE("StageContractTable_StageWithNoOutputs_IsNotAnArtifactProducer",
          "[core][contract]") {
  // The consequence of keeping external products out of the artifact registry:
  // a file on disk has no name `ProjectDB` can be asked about, so a stage that
  // writes only such files must be invisible to producing_stage(). If one ever
  // became the answer for some artifact, check_stage_inputs() would start
  // handing users a "run `rux create gsplat` first" hint for a prerequisite
  // that stage cannot possibly satisfy — and on a CPU-only build that command
  // does not even exist.
  for (const auto &contract : stage_contracts()) {
    if (!contract.outputs.empty())
      continue;
    INFO("leaf stage " << contract.name);
    for (const auto &artifact : pipeline_artifacts()) {
      INFO("artifact " << artifact.name);
      const auto producer = producing_stage(artifact.name);
      if (producer.has_value())
        CHECK(*producer != contract.stage);
    }
  }
}

TEST_CASE("StageContract_UnknownEnumerator_ThrowsLogicError",
          "[core][contract]") {
  const auto beyond =
      static_cast<PipelineStage>(static_cast<int>(stage_contracts().size()));
  REQUIRE_THROWS_AS(stage_contract(beyond), std::logic_error);
}

// ── docs/CONTRACTS.md is a mirror, and this proves it ──────────────────────

namespace {

/// Backticked tokens of `line` that name a registered pipeline artifact.
/// Type names and prose (`PointXYZRGB`, `meshes` table, …) are not artifacts
/// and drop out, which is what lets the prose stay readable.
std::set<std::string> artifact_tokens(const std::string &line) {
  std::set<std::string> found;
  size_t pos = 0;
  while ((pos = line.find('`', pos)) != std::string::npos) {
    const size_t end = line.find('`', pos + 1);
    if (end == std::string::npos)
      break;
    const std::string token = line.substr(pos + 1, end - pos - 1);
    if (find_artifact(token))
      found.insert(token);
    pos = end + 1;
  }
  return found;
}

struct DocStage {
  std::set<std::string> consumes;
  std::set<std::string> produces;
  /// Whether the section's table carries an `| External |` row. Its *content*
  /// is deliberately not compared: `external_outputs` is prose, and requiring
  /// the doc to repeat it word for word would make the mirror a copy rather
  /// than a check. Presence is the part that can silently drift.
  bool has_external = false;
};

/// Parse the `### \`stage\`` sections of docs/CONTRACTS.md into their Consumes
/// and Produces artifact sets, plus whether an External row is present.
std::map<std::string, DocStage> parse_contracts_doc(const std::string &text) {
  std::map<std::string, DocStage> stages;
  std::istringstream in(text);
  std::string line;
  std::string current;
  while (std::getline(in, line)) {
    if (line.rfind("### ", 0) == 0) {
      const size_t open = line.find('`');
      const size_t close = line.find('`', open + 1);
      current = (open == std::string::npos || close == std::string::npos)
                    ? std::string{}
                    : line.substr(open + 1, close - open - 1);
      if (!current.empty())
        stages[current];
      continue;
    }
    if (current.empty())
      continue;
    if (line.rfind("| Consumes ", 0) == 0)
      stages[current].consumes = artifact_tokens(line);
    else if (line.rfind("| Produces ", 0) == 0)
      stages[current].produces = artifact_tokens(line);
    else if (line.rfind("| External ", 0) == 0)
      stages[current].has_external = true;
  }
  return stages;
}

} // namespace

TEST_CASE("ContractsDoc_StageSections_MatchContractTable",
          "[core][contract][docs]") {
  const std::filesystem::path doc =
      reusex::test_support::source_dir() / "docs" / "CONTRACTS.md";
  REQUIRE(std::filesystem::exists(doc));

  std::ifstream in(doc);
  REQUIRE(in.good());
  const std::string text((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());

  const auto documented = parse_contracts_doc(text);
  REQUIRE_FALSE(documented.empty());

  for (const auto &contract : stage_contracts()) {
    const std::string name(contract.name);
    INFO("docs/CONTRACTS.md section '### `" << name << "`'");
    const auto it = documented.find(name);
    REQUIRE(it != documented.end());

    std::set<std::string> consumes;
    for (const auto &input : contract.inputs)
      for (const auto &artifact : input.any_of)
        consumes.insert(std::string(artifact));
    std::set<std::string> produces;
    for (const auto &output : contract.outputs)
      produces.insert(std::string(output));

    CHECK(it->second.consumes == consumes);
    CHECK(it->second.produces == produces);

    // A stage with an external product must document it, and only such a stage
    // may carry an `| External |` row. Without this, a stage could acquire an
    // empty `Produces` row in the doc and leave the reader believing it does
    // nothing — the one reading of an empty Produces that is never true.
    CHECK(it->second.has_external == !contract.external_outputs.empty());
  }

  // No stage section in the doc that the table does not know about.
  for (const auto &[name, unused] : documented) {
    (void)unused;
    INFO("documented stage '" << name << "'");
    CHECK(parse_pipeline_stage(name).has_value());
  }
}
