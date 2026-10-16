// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// "Kopiér som rux-kommando" (Qt client, Stream Q extra b) must print a
// command that really does what the GUI run did. The key -> flag table lives
// in rux_qt/cli_command.cpp, far from the CLI it describes; this test parses
// every generated command back through the REAL CLI11 setup of each stage
// subcommand (setup_subcommand_create / _optimize / _graph) and checks
// that the stage receives the same parameters — key by key, for every
// parameter pipeline::stage_parameters() lists, and all at once.
//
// rux::set_stage_params_sink() keeps the subcommands from running: their
// callbacks hand the parameter JSON over and return before opening a project.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <create.hpp>
#include <create/stage_bridge.hpp>
#include <global-params.hpp>
#include <graph.hpp>
#include <optimize.hpp>

#include <reusex/pipeline/mesh_parameters.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/slam/optimize_parameters.hpp>
#include <rux_qt/cli_command.hpp>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace pl = reusex::pipeline;
using nlohmann::json;
using rux::qt::CliParam;
using rux::qt::CliValueKind;

namespace {

/// Keys the CLI has no flag for: the command runs them at their default and
/// the Qt client says so next to the command.
const std::set<std::string> kNoFlag = {"rooms.propagate_k"};

/// Keys with a CLI flag that the CLI only sends when the user actually
/// passed it (like the #214 pinned fields), but that are NOT marked
/// `presence_sensitive` in the shared descriptor table: unlike a pinned
/// field, the Qt dropdown stays freely selectable (no pin checkbox) and
/// sends the key whenever its value differs from the default, which already
/// gives the GUI the same "default degrades gracefully" behaviour. Only the
/// CLI needs the finer "asked for band by name" vs. "defaulted to band"
/// distinction (reusex-icg), via a CLI11 `->count()` flag mirroring
/// `dist_explicit`/`min_explicit` without going through
/// `ParameterDescriptor::presence_sensitive`.
const std::set<std::string> kSentOnlyWhenExplicit = {"rooms.method"};

/// Stages known to have no `rux create`/`rux <stage>` CLI mapping at all —
/// a job-only stage the Qt client cannot offer as "Kopiér som rux-kommando".
/// Explicit by name (final-review.md finding 4) rather than "whatever
/// build_cli_command() currently refuses", so a stage that loses its real
/// CLI mapping by accident fails this test instead of silently being
/// skipped: "annotate" has no mapping yet (gpu-worker W0); "gsplat" is not
/// even a pipeline::JobStage (core::PipelineStage only) and never appears in
/// job_stage_names(), but is listed for the same reason
/// CliCommand_UnknownStage_IsUnsupported (test_workspace_logic.cpp) uses it:
/// a byword for "a stage name with no CLI mapping".
const std::set<std::string> kNoCliMapping = {
    "gsplat",
    "annotate",
};

struct Captured {
  std::string stage;
  json params;
};

/// Parse @p args (the words after `rux`) through the real subcommand setup.
Captured parse_through_cli(const std::vector<std::string> &args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_create(app, global);
  setup_subcommand_optimize(app, global);
  setup_subcommand_graph(app, global);

  Captured got;
  rux::set_stage_params_sink(
      [&](std::string_view stage, const std::string &params) {
        got.stage = std::string(stage);
        got.params = json::parse(params);
      });
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  try {
    app.parse(reversed);
  } catch (const CLI::ParseError &e) {
    rux::set_stage_params_sink({});
    FAIL("CLI rejected the generated command: " << e.what());
  }
  rux::set_stage_params_sink({});
  return got;
}

CliValueKind kind_of(pl::ParameterType t) {
  switch (t) {
  case pl::ParameterType::number:
    return CliValueKind::number;
  case pl::ParameterType::integer:
    return CliValueKind::integer;
  case pl::ParameterType::boolean:
    return CliValueKind::boolean;
  case pl::ParameterType::string:
    return CliValueKind::string;
  case pl::ParameterType::integer_list:
    return CliValueKind::integer_list;
  }
  return CliValueKind::string;
}

/// A value for @p d that differs from its default and passes its bounds.
std::string changed_value(const pl::ParameterDescriptor &d) {
  const double lo = d.minimum.value_or(-1e9);
  const double hi = d.maximum.value_or(1e9);
  switch (d.type) {
  case pl::ParameterType::number: {
    const double def = std::get<double>(d.default_value);
    const double step =
        d.minimum && d.maximum ? (hi - lo) * 0.1 : 0.25 * std::abs(def) + 0.5;
    double v = def + step;
    if (v > hi)
      v = def - step;
    return rux::qt::format_number(static_cast<float>(v));
  }
  case pl::ParameterType::integer: {
    const long long def = std::get<long long>(d.default_value);
    return std::to_string(def + 1 <= hi ? def + 1 : def - 1);
  }
  case pl::ParameterType::boolean:
    return std::get<bool>(d.default_value) ? "false" : "true";
  case pl::ParameterType::string:
    if (d.key == "filter")
      return "planes in [1, 2]";
    if (d.key == "solver")
      return "highs";
    if (d.key == "method")
      return "leiden"; // differs from the "band" default (reusex-icg)
    if (d.key == "loop_proposal")
      return "spatial";
    return "x_" + d.key;
  case pl::ParameterType::integer_list:
    return "1,4";
  }
  return {};
}

/// The captured JSON value as the canonical text CliParam uses.
std::string canonical(const json &v, pl::ParameterType t) {
  switch (t) {
  case pl::ParameterType::number:
    return rux::qt::format_number(static_cast<float>(v.get<double>()));
  case pl::ParameterType::integer:
    return std::to_string(v.get<long long>());
  case pl::ParameterType::boolean:
    return v.get<bool>() ? "true" : "false";
  case pl::ParameterType::string:
    return v.get<std::string>();
  case pl::ParameterType::integer_list: {
    std::string s;
    for (const auto &e : v) {
      if (!s.empty())
        s += ',';
      s += std::to_string(e.get<long long>());
    }
    return s;
  }
  }
  return {};
}

std::string default_text(const pl::ParameterDescriptor &d) {
  if (std::holds_alternative<double>(d.default_value))
    return rux::qt::format_number(
        static_cast<float>(std::get<double>(d.default_value)));
  if (std::holds_alternative<long long>(d.default_value))
    return std::to_string(std::get<long long>(d.default_value));
  if (std::holds_alternative<bool>(d.default_value))
    return std::get<bool>(d.default_value) ? "true" : "false";
  if (std::holds_alternative<std::string>(d.default_value))
    return std::get<std::string>(d.default_value);
  return {};
}

} // namespace

TEST_CASE("CliCommandRoundTrip_EveryParameter_ReachesTheStageUnchanged",
          "[rux_app][cli][qt]") {
  for (const std::string &stage : pl::job_stage_names()) {
    // See kNoCliMapping: a stage with no CLI mapping at all (annotate,
    // gpu-worker W0) has nothing below this point in the loop that is
    // meaningful. Any OTHER stage must have a mapping — REQUIRE rather than
    // skip, so a real mapping regressing to unsupported fails loudly instead
    // of quietly dropping out of this test (final-review.md finding 4).
    if (kNoCliMapping.count(stage))
      continue;
    REQUIRE(rux::qt::build_cli_command(stage, "p.rux", {}).supported);
    const auto parsed = pl::parse_job_stage(stage);
    REQUIRE(parsed);
    for (const auto &d : pl::stage_parameters(*parsed)) {
      const std::string id = stage + "." + d.key;
      INFO(id);
      if (rux::qt::cli_flag_for(stage, d.key).empty()) {
        CHECK(kNoFlag.count(id) == 1); // a new key needs a flag (or a ruling)
        continue;
      }
      const CliParam p{d.key, kind_of(d.type), changed_value(d)};
      const auto cmd =
          rux::qt::build_cli_command(stage, "/tmp/kopi af projekt.rux", {p});
      REQUIRE(cmd.supported);
      REQUIRE(cmd.unmapped.empty());
      INFO(cmd.text);

      const Captured got = parse_through_cli(cmd.args);
      CHECK(got.stage == stage);
      REQUIRE(got.params.contains(d.key));
      CHECK(canonical(got.params.at(d.key), d.type) == p.value);
    }
  }
}

TEST_CASE("CliCommandRoundTrip_AllParametersAtOnce_AndTheShellLine",
          "[rux_app][cli][qt]") {
  for (const std::string &stage : pl::job_stage_names()) {
    INFO(stage);
    // See kNoCliMapping and the comment in
    // CliCommandRoundTrip_EveryParameter_ReachesTheStageUnchanged: a stage
    // with no CLI mapping at all (annotate, gpu-worker W0) has nothing to
    // round-trip — build_cli_command() hands back an unsupported, empty
    // command, which split_shell_words()/words.front() below is not meant
    // to parse. Any other stage must be supported.
    if (kNoCliMapping.count(stage))
      continue;
    REQUIRE(rux::qt::build_cli_command(stage, "p.rux", {}).supported);
    std::vector<CliParam> params;
    for (const auto &d : pl::stage_parameters(*pl::parse_job_stage(stage)))
      if (!rux::qt::cli_flag_for(stage, d.key).empty())
        params.push_back({d.key, kind_of(d.type), changed_value(d)});
    const auto cmd = rux::qt::build_cli_command(stage, "/tmp/a b.rux", params);
    // What the user pastes is the text: split it like a shell would.
    auto words = rux::qt::split_shell_words(cmd.text);
    REQUIRE(words.front() == "rux");
    words.erase(words.begin());
    CHECK(words == cmd.args);
    const Captured got = parse_through_cli(words);
    for (const auto &p : params) {
      INFO(p.key);
      const auto &d = *std::find_if(
          pl::stage_parameters(*pl::parse_job_stage(stage)).begin(),
          pl::stage_parameters(*pl::parse_job_stage(stage)).end(),
          [&](const auto &x) { return x.key == p.key; });
      REQUIRE(got.params.contains(p.key));
      CHECK(canonical(got.params.at(p.key), d.type) == p.value);
    }
  }
}

TEST_CASE("CliCommandRoundTrip_NoParameters_MeansTheStageDefaults",
          "[rux_app][cli][qt]") {
  // An untouched GUI form sends no keys, and its command passes no flags; the
  // CLI must then hand the stage the same defaults the form showed — and
  // must not send a presence-sensitive key (#214) that would pin it.
  for (const std::string &stage : pl::job_stage_names()) {
    const auto cmd = rux::qt::build_cli_command(stage, "p.rux", {});
    // See kNoCliMapping and the comment in
    // CliCommandRoundTrip_EveryParameter_ReachesTheStageUnchanged.
    if (kNoCliMapping.count(stage))
      continue;
    REQUIRE(cmd.supported);
    const Captured got = parse_through_cli(cmd.args);
    for (const auto &d : pl::stage_parameters(*pl::parse_job_stage(stage))) {
      const std::string id = stage + "." + d.key;
      INFO(id);
      if (d.presence_sensitive ||
          std::holds_alternative<std::monostate>(d.default_value) ||
          kSentOnlyWhenExplicit.count(id)) {
        CHECK_FALSE(got.params.contains(d.key));
        continue;
      }
      if (kNoFlag.count(id))
        continue;
      REQUIRE(got.params.contains(d.key));
      CHECK(canonical(got.params.at(d.key), d.type) == default_text(d));
    }
  }
}

TEST_CASE("CliCommandRoundTrip_MeshAndOptimize_ShareTheStageReader",
          "[rux_app][cli][qt]") {
  // `rux create mesh` and `rux optimize` drive their solvers themselves, but
  // read their options through the same functions the in-process stages use
  // (pipeline::mesh_options_from_parameters,
  // geometry::apply_optimize_parameters). So
  // the parameters a command line produces become the same options a GUI run
  // with that JSON gets.
  {
    const auto cmd = rux::qt::build_cli_command(
        "mesh", "p.rux",
        {{"solver", CliValueKind::string, "highs"},
         {"time_limit_seconds", CliValueKind::number, "90"},
         {"sectioned", CliValueKind::boolean, "false"},
         {"output_name", CliValueKind::string, "mesh2"}});
    const Captured got = parse_through_cli(cmd.args);
    const auto parsed = pl::mesh_options_from_parameters(got.params.dump());
    CHECK(parsed.output_name == "mesh2");
    CHECK(parsed.options.time_limit_seconds == 90.0);
    CHECK_FALSE(parsed.options.sectioned);
    CHECK(parsed.options.solver ==
          reusex::geometry::parse_solver_choice("highs"));
  }
  {
    // Structural plane selection (reusex-8l3) reaches StructuralPlaneOptions.
    const auto cmd = rux::qt::build_cli_command(
        "mesh", "p.rux",
        {{"structural_filter", CliValueKind::boolean, "false"},
         {"structural_min_wall_area", CliValueKind::number, "2.5"},
         {"structural_collinear_offset", CliValueKind::number, "0.2"},
         {"structural_storey_clearance", CliValueKind::number, "1.5"},
         {"structural_footprint_overlap", CliValueKind::number, "0.4"}});
    const Captured got = parse_through_cli(cmd.args);
    const auto s =
        pl::mesh_options_from_parameters(got.params.dump()).options.structural;
    CHECK_FALSE(s.enabled);
    CHECK(s.min_wall_area_m2 == Catch::Approx(2.5));
    CHECK(s.collinear_offset_m == Catch::Approx(0.2));
    CHECK(s.min_storey_clearance_m == Catch::Approx(1.5));
    CHECK(s.min_footprint_overlap == Catch::Approx(0.4));
  }
  {
    // Plane classification (reusex-z1v.4.3): off by default, on by flag.
    const auto off =
        parse_through_cli(rux::qt::build_cli_command("mesh", "p.rux", {}).args);
    CHECK_FALSE(
        pl::mesh_options_from_parameters(off.params.dump()).use_plane_classes);
    const auto on = parse_through_cli(
        rux::qt::build_cli_command(
            "mesh", "p.rux", {{"plane_classes", CliValueKind::boolean, "true"}})
            .args);
    CHECK(pl::mesh_options_from_parameters(on.params.dump()).use_plane_classes);
  }
  {
    const auto cmd = rux::qt::build_cli_command(
        "optimize", "p.rux",
        {{"min_observations", CliValueKind::integer, "7"},
         {"no_gnc", CliValueKind::boolean, "true"}});
    const Captured got = parse_through_cli(cmd.args);
    auto from_cli = plane_graph_options(SubcommandOptimizeOptions{});
    bool dry = false;
    reusex::geometry::apply_optimize_parameters(from_cli, dry,
                                                got.params.dump());
    CHECK(from_cli.min_landmark_observations == 7);
    CHECK_FALSE(from_cli.use_gnc);
    CHECK_FALSE(dry);
  }
}

namespace {

/// Everything differing between two PlaneGraphOptions, field by field,
/// including nested surfel — a readable mismatch
/// report for CHECK's INFO, without hand-listing fields a second time (that
/// hand list is exactly the bug the integration review's finding 4 flagged).
///
/// Written against `operator==` rather than reading fields itself: every
/// field compared here is also a field the `==` just below folded in, so the
/// two cannot drift apart the way a hand-maintained list and the struct did.
template <typename T>
std::string describe(std::string_view name, const T &cli, const T &lib) {
  return cli == lib ? std::string()
                    : std::string(name) + " differs from the library default";
}

} // namespace

TEST_CASE("CliCommandRoundTrip_OptimizeFlagDefaults_MatchTheStageBase",
          "[rux_app][cli][qt]") {
  // The in-process optimize stage (pipeline/optimize_stage.hpp: the Qt client
  // and ruxd's web GUI) starts from PlaneGraphOptions{}; `rux optimize`
  // starts from its flag defaults. Both then apply the same stage
  // parameters, so they solve the same problem only while every flag default
  // mirrors the library default (STANDARDS §4).
  //
  // Integration review finding 4: this used to be ~65 individual CHECKs,
  // hand-copied from the struct. That list had already fallen behind —
  // LoopClosureOptions gained exhaustive_budget, vocab_size,
  // vocab_sample_per_frame, vocab_iterations, its own ransac_iterations,
  // min_depth, max_depth, pcm_trans_threshold, pcm_rot_threshold,
  // pcm_max_edges and seed with no matching CHECK, so a CLI default
  // diverging on any of them would have passed silently. PlaneGraphOptions
  // and its nested `surfel` struct now each carry a defaulted,
  // memberwise `operator==` (`= default`, C++20) right next to their field
  // list, so a newly added field is compared automatically — by the
  // compiler, not by whoever remembers to update this test.
  const reusex::geometry::PlaneGraphOptions lib;
  const auto cli = plane_graph_options(SubcommandOptimizeOptions{});

  INFO(describe("surfel", cli.surfel, lib.surfel));
  CHECK(cli == lib);
}
