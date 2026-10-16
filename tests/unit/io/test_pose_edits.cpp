// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The pose-edits document, reusex.pose_edits.v1 (reusex-rcv.1.2): every kind
// round-trips (JSON exactly, YAML semantically), malformed input names the
// edit and the field, and the examples in docs/design/pose-edits.md are valid
// against both the reader and docs/design/pose-edits.schema.json.

#include "../../support/json_schema_lite.hpp"
#include "../../support/temp_path.hpp"

#include <reusex/core/pose_edits.hpp>
#include <reusex/io/pose_edits.hpp>

#include "io/yaml_json.hpp" // reusex_io-private: the YAML -> JSON front end

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Geometry>

#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>

using namespace reusex;
using Catch::Matchers::ContainsSubstring;
using core::PoseEdit;
using core::PoseEditKind;

namespace {

std::string slurp(const std::string &path) {
  std::ifstream in(path);
  REQUIRE(in.good());
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

/// One edit of every kind, exercising every optional field.
core::PoseEdits all_kinds() {
  core::PoseEdits d;
  d.project_guid = "9a3f0c2e-0000-5000-8000-000000000001";
  d.frame = "storey:2b1e6d8a-0000-5000-8000-000000000002";

  PoseEdit a;
  a.id = "a-1203";
  a.kind = PoseEditKind::anchor;
  a.nodes.first = a.nodes.last = 1203;
  a.pose.source = core::PoseSpec::Source::given;
  a.pose.value.xyz = {1.25, -3.5, 0.1};
  a.pose.value.yaw_deg = 91.0 / 3.0; // not exactly representable in decimal
  a.dofs = core::Dof::x | core::Dof::y | core::Dof::yaw;
  a.sigma_trans_m = 0.02;
  a.sigma_rot_deg = 0.3;
  a.note = "plan.png: corner on the plan";
  d.edits.push_back(a);

  PoseEdit a2;
  a2.id = "a-matrix";
  a2.kind = PoseEditKind::anchor;
  a2.nodes.first = a2.nodes.last = 7;
  a2.pose.source = core::PoseSpec::Source::given;
  Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
  M.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.2, 0.3, 1.0).normalized())
          .toRotationMatrix();
  M.topRightCorner<3, 1>() = Eigen::Vector3d(0.1, 0.2, 0.3);
  std::array<double, 16> m{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m[static_cast<size_t>(4 * r + c)] = M(r, c);
  a2.pose.value.matrix = m;
  d.edits.push_back(a2);

  PoseEdit mv;
  mv.id = "m-late-pass";
  mv.kind = PoseEditKind::move;
  mv.nodes.first = 358;
  mv.nodes.last = 389;
  mv.delta.xyz = {0.15, -0.11, 0.0};
  mv.delta.yaw_deg = 1.2;
  mv.delta.pivot = std::array<double, 3>{-2.0, 4.5, 1.0};
  mv.sigma_trans_m = 0.03;
  mv.note = "seg 358-389: 65 -> 31 mm";
  d.edits.push_back(mv);

  PoseEdit mv2;
  mv2.id = "m-list";
  mv2.kind = PoseEditKind::move;
  mv2.nodes.list = {3, 5, 8};
  mv2.delta.matrix = m;
  d.edits.push_back(mv2);

  PoseEdit rel;
  rel.id = "r-412-980";
  rel.kind = PoseEditKind::relative;
  rel.nodes.first = rel.nodes.last = 412;
  rel.nodes_b.first = rel.nodes_b.last = 980;
  rel.pose.source = core::PoseSpec::Source::initial;
  d.edits.push_back(rel);

  PoseEdit ss;
  ss.id = "s-hall-west";
  ss.kind = PoseEditKind::same_surface;
  ss.nodes.first = 1;
  ss.nodes.last = 33;
  ss.nodes_b.first = 358;
  ss.nodes_b.last = 389;
  ss.region = std::array<double, 4>{-5.2, -3.4, -4.8, -2.0};
  ss.region_b = std::array<double, 4>{-5.3, -3.4, -4.7, -2.0};
  ss.sigma_m = 0.005;
  d.edits.push_back(ss);

  PoseEdit w;
  w.id = "w-x-north";
  w.kind = PoseEditKind::on_plan_wall;
  w.plan_path = "tests/fixtures/solid_model/newoffice.plan.yaml";
  w.wall_guid = "x-north";
  w.nodes.first = 2300;
  w.nodes.last = 2400;
  w.region = std::array<double, 4>{0.0, 4.0, 6.0, 4.6};
  d.edits.push_back(w);

  PoseEdit x;
  x.id = "x-orb-17";
  x.kind = PoseEditKind::reject_edge;
  x.nodes.first = x.nodes.last = 17;
  x.nodes_b.first = x.nodes_b.last = 3611;
  x.edge_type = "loop_closure";
  d.edits.push_back(x);
  return d;
}

/// A one-edit YAML document around @p edit (already indented as a list item).
std::string doc_with(const std::string &edit,
                     const std::string &head = "schema: "
                                               "reusex.pose_edits.v1\n") {
  return head + "edits:\n" + edit;
}

void require_error(const std::string &yaml, const std::string &edit_id,
                   const std::string &field) {
  INFO(yaml);
  try {
    (void)io::pose_edits_from_yaml(yaml);
    FAIL("expected a throw");
  } catch (const std::runtime_error &e) {
    const std::string msg = e.what();
    INFO(msg);
    CHECK_THAT(msg, ContainsSubstring("edit '" + edit_id + "'"));
    CHECK_THAT(msg, ContainsSubstring(field));
  }
}

/// The ```yaml blocks of the design doc that are whole documents.
std::vector<std::string> doc_examples() {
  const std::string md =
      slurp(std::string(REUSEX_SOURCE_DIR) + "/docs/design/pose-edits.md");
  std::vector<std::string> out;
  const std::regex block("```yaml\\n([\\s\\S]*?)```");
  for (std::sregex_iterator it(md.begin(), md.end(), block), end; it != end;
       ++it) {
    const std::string body = (*it)[1].str();
    if (body.rfind("schema: reusex.pose_edits.v1", 0) == 0 &&
        body.find("- id:") != std::string::npos)
      out.push_back(body);
  }
  return out;
}

test_support::JsonSchemaLite schema() {
  return test_support::JsonSchemaLite(nlohmann::json::parse(slurp(
      std::string(REUSEX_SOURCE_DIR) + "/docs/design/pose-edits.schema.json")));
}

} // namespace

TEST_CASE("PoseEdits_EveryKind_RoundTripsJsonBitIdentical",
          "[io][pose_edits]") {
  const auto d = all_kinds();
  const std::string text = io::pose_edits_to_json(d);
  const auto back = io::pose_edits_from_json(text);
  CHECK(back == d);
  CHECK(io::pose_edits_to_json(back) == text);
}

TEST_CASE("PoseEdits_EveryKind_RoundTripsYamlSemantically",
          "[io][pose_edits]") {
  const auto d = all_kinds();
  const std::string yaml = io::pose_edits_to_yaml(d);
  INFO(yaml);
  CHECK(io::pose_edits_from_yaml(yaml) == d);
}

TEST_CASE("PoseEdits_FileRoundTrip_ByExtension", "[io][pose_edits]") {
  const auto d = all_kinds();
  for (const char *ext : {".yaml", ".json"}) {
    test_support::TempPath tmp("test_pose_edits_file", ext);
    io::write_pose_edits(d, tmp.path);
    CHECK(io::read_pose_edits(tmp.path) == d);
    CHECK(io::read_pose_edits(tmp.path, d.project_guid) == d);
  }
}

TEST_CASE("PoseEdits_Defaults_OmittedFieldsTakeRecordDefaults",
          "[io][pose_edits]") {
  const auto d = io::pose_edits_from_yaml(doc_with(R"(  - id: a
    kind: anchor
    node: 4
    pose: current
  - id: m
    kind: move
    nodes: [10, 20]
    delta: {yaw_deg: 2}
)"));
  REQUIRE(d.edits.size() == 2);
  CHECK(d.frame == "world");
  CHECK(d.edits[0].dofs == core::Dof::all);
  CHECK(d.edits[0].sigma_trans_m == PoseEdit{}.sigma_trans_m);
  CHECK(d.edits[0].pose.source == core::PoseSpec::Source::current);
  CHECK(d.edits[1].nodes.contains(15));
  CHECK_FALSE(d.edits[1].nodes.contains(21));
  CHECK(d.edits[1].delta.xyz == std::array<double, 3>{0.0, 0.0, 0.0});
}

TEST_CASE("PoseEdits_RigidTransform_RotatesAboutPivot", "[io][pose_edits]") {
  core::RigidTransform t;
  t.yaw_deg = 90.0;
  const Eigen::Vector3d pivot(2.0, 0.0, 0.0);
  const Eigen::Matrix4d T = t.to_matrix(pivot);
  // The pivot itself does not move; a point 1 m east of it swings north.
  CHECK((T.topLeftCorner<3, 3>() * pivot + T.topRightCorner<3, 1>() - pivot)
            .norm() < 1e-12);
  const Eigen::Vector3d p(3.0, 0.0, 0.0);
  CHECK((T.topLeftCorner<3, 3>() * p + T.topRightCorner<3, 1>() -
         Eigen::Vector3d(2.0, 1.0, 0.0))
            .norm() < 1e-12);
}

TEST_CASE("PoseEdits_UnknownKind_NamesEditAndField", "[io][pose_edits]") {
  require_error(doc_with("  - {id: k1, kind: teleport, node: 3}\n"), "k1",
                "kind: unknown kind 'teleport'");
}

TEST_CASE("PoseEdits_MissingNodes_NamesEditAndField", "[io][pose_edits]") {
  require_error(doc_with("  - {id: m1, kind: move, delta: {yaw_deg: 1}}\n"),
                "m1", "nodes: missing required field");
  require_error(doc_with("  - {id: a1, kind: anchor, pose: current}\n"), "a1",
                "node: missing required field");
  require_error(
      doc_with("  - id: s1\n    kind: same_surface\n    a: {nodes: [1, 2], "
               "region: [0, 0, 1, 1]}\n    b: {region: [0, 0, 1, 1]}\n"),
      "s1", "b.nodes: missing required field");
}

TEST_CASE("PoseEdits_NaN_NamesEditAndField", "[io][pose_edits]") {
  require_error(doc_with("  - {id: m2, kind: move, nodes: [1, 9], delta: "
                         "{xyz: [nan, 0, 0]}}\n"),
                "m2", "delta.xyz");
  require_error(doc_with("  - {id: m3, kind: move, nodes: [1, 9], delta: "
                         "{yaw_deg: .nan}}\n"),
                "m3", "delta.yaw_deg");
}

TEST_CASE("PoseEdits_NonRigidPose_NamesEditAndField", "[io][pose_edits]") {
  require_error(
      doc_with("  - {id: a2, kind: anchor, node: 3, pose: {matrix: "
               "[2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]}}\n"),
      "a2", "pose.matrix: is not a rigid transform");
}

TEST_CASE("PoseEdits_BadDofName_NamesEditAndField", "[io][pose_edits]") {
  require_error(doc_with("  - {id: a3, kind: anchor, node: 3, pose: current, "
                         "dofs: [x, heading]}\n"),
                "a3", "dofs: unknown DoF 'heading'");
}

TEST_CASE("PoseEdits_ProjectGuidMismatch_IsRefused", "[io][pose_edits]") {
  auto d = all_kinds();
  test_support::TempPath tmp("test_pose_edits_guid", ".yaml");
  io::write_pose_edits(d, tmp.path);
  try {
    (void)io::read_pose_edits(tmp.path, "some-other-project");
    FAIL("expected a throw");
  } catch (const std::runtime_error &e) {
    const std::string msg = e.what();
    CHECK_THAT(msg, ContainsSubstring("project_guid"));
    CHECK_THAT(msg, ContainsSubstring(d.project_guid));
    CHECK_THAT(msg, ContainsSubstring("some-other-project"));
  }
  d.project_guid.clear(); // an unguarded file applies anywhere
  CHECK_NOTHROW(io::check_pose_edits_project(d, "some-other-project"));
}

TEST_CASE("PoseEdits_OtherMistakes_AreRefused", "[io][pose_edits]") {
  require_error(doc_with("  - {id: d, kind: anchor, node: 1, pose: current}\n"
                         "  - {id: d, kind: anchor, node: 2, pose: current}\n"),
                "d", "duplicate id");
  require_error(doc_with("  - {id: r, kind: move, nodes: [9, 1], delta: {}}\n"),
                "r", "nodes: first (9) > last (1)");
  require_error(
      doc_with("  - {id: t, kind: anchor, node: 1, pose: current, typo: 1}\n"),
      "t", "typo: unknown key");
  require_error(doc_with("  - {id: e, kind: reject_edge, a: 1, b: 2, "
                         "edge_type: wormhole}\n"),
                "e", "edge_type");
  CHECK_THROWS_WITH(io::pose_edits_from_yaml("schema: reusex.pose_edits.v2\n"
                                             "edits: []\n"),
                    ContainsSubstring("schema"));
}

TEST_CASE("PoseEdits_DesignDocExamples_PassReaderAndSchema",
          "[io][pose_edits]") {
  const auto examples = doc_examples();
  REQUIRE(examples.size() >= 6); // one per kind
  const auto s = schema();
  std::set<PoseEditKind> kinds;
  for (const auto &ex : examples) {
    INFO(ex);
    const auto d = io::pose_edits_from_yaml(ex);
    for (const auto &e : d.edits)
      kinds.insert(e.kind);
    // The example as written, and the reader's canonical form.
    CHECK(s.validate(io::detail::yaml_text_to_json(ex)) == "");
    CHECK(s.validate(nlohmann::json::parse(io::pose_edits_to_json(d))) == "");
  }
  CHECK(kinds.size() == 6);
  // And the schema is not vacuous.
  CHECK(s.validate(nlohmann::json::parse(
            R"({"schema": "reusex.pose_edits.v1", "edits": [{"id": "x",
                "kind": "anchor", "node": 1, "pose": "somewhere"}]})")) != "");
  CHECK(s.validate(
            nlohmann::json::parse(io::pose_edits_to_json(all_kinds()))) == "");
}
