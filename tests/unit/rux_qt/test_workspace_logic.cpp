// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Qt-free logic of the Q3 workspaces: the "Kopiér som rux-kommando" text, the
// Log filter, Posegraf hit testing and the log tap. The round trip of the
// generated commands through the real CLI lives in
// tests/unit/rux_app/test_cli_command_roundtrip.cpp (it needs rux_lib).

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/cli_command.hpp>
#include <rux_qt/workspace_logic.hpp>

#include <algorithm>
#include <string>
#include <vector>

using namespace rux::qt;

TEST_CASE("CliCommand_Planes_ShowsOnlyThePassedKeys", "[rux_qt][cli]") {
  const auto cmd =
      build_cli_command("planes", "/data/kontor.rux",
                        {{"angle_threshold", CliValueKind::number, "20"},
                         {"min_inliers", CliValueKind::integer, "800"},
                         {"adaptive", CliValueKind::boolean, "false"},
                         {"job_id", CliValueKind::string, "abc"}});
  REQUIRE(cmd.supported);
  CHECK(cmd.unmapped.empty());
  CHECK(cmd.text == "rux -p /data/kontor.rux create planes --angle-threshold "
                    "20 --min-cluster-size 800 --no-adaptive");
}

TEST_CASE("CliCommand_DefaultBooleans_EmitNothing", "[rux_qt][cli]") {
  CHECK(build_cli_command("planes", "",
                          {{"adaptive", CliValueKind::boolean, "true"}})
            .text == "rux create planes");
  CHECK(build_cli_command("mesh", "",
                          {{"sectioned", CliValueKind::boolean, "true"}})
            .text == "rux create mesh");
  CHECK(build_cli_command("optimize", "",
                          {{"no_gnc", CliValueKind::boolean, "true"},
                           {"dry_run", CliValueKind::boolean, "false"}})
            .text == "rux optimize --no-gnc");
}

TEST_CASE("CliCommand_QuotesFiltersAndPaths", "[rux_qt][cli]") {
  const auto cmd =
      build_cli_command("rooms", "/home/pfs/sager/Kontorhus Valby/project.rux",
                        {{"filter", CliValueKind::string, "planes in [1, 2]"},
                         {"propagate_k", CliValueKind::integer, "12"}});
  CHECK(cmd.text == "rux -p '/home/pfs/sager/Kontorhus Valby/project.rux' "
                    "create rooms --filter 'planes in [1, 2]'");
  REQUIRE(cmd.unmapped.size() == 1);
  CHECK(cmd.unmapped.front() == "propagate_k");
  // The line splits back into exactly the argument vector.
  auto words = split_shell_words(cmd.text);
  REQUIRE(words.size() == cmd.args.size() + 1);
  for (std::size_t i = 0; i < cmd.args.size(); ++i)
    CHECK(words[i + 1] == cmd.args[i]);
}

TEST_CASE("CliCommand_EmptyFilterAndList_AreNotSet", "[rux_qt][cli]") {
  CHECK(build_cli_command("instances", "",
                          {{"labels", CliValueKind::integer_list, ""},
                           {"filter", CliValueKind::string, ""}})
            .args.size() == 2);
  CHECK(build_cli_command("instances", "",
                          {{"labels", CliValueKind::integer_list, "1,4"}})
            .text == "rux create instances --labels 1,4");
}

TEST_CASE("CliCommand_UnknownStage_IsUnsupported", "[rux_qt][cli]") {
  CHECK_FALSE(build_cli_command("gsplat", "x.rux", {}).supported);
  CHECK(cli_flag_for("clouds", "resolution") == "--grid");
  CHECK(cli_flag_for("planes", "adaptive") == "--no-adaptive");
  CHECK(cli_flag_for("rooms", "propagate_k").empty());
}

TEST_CASE("ShellQuote_And_Split_RoundTrip", "[rux_qt][cli]") {
  for (const std::string w :
       {"plain", "", "two words", "it's", "a\"b", "$HOME", "x\\y", "æøå"}) {
    const auto words = split_shell_words("cmd " + shell_quote(w));
    REQUIRE(words.size() == 2);
    CHECK(words[1] == w);
  }
  CHECK(split_shell_words("a  \"b c\" d\\ e") ==
        std::vector<std::string>{"a", "b c", "d e"});
}

TEST_CASE("WrapShellCommand_BreaksBetweenFlagsAndStillPastes",
          "[rux_qt][cli]") {
  const std::string line =
      "rux -p '/tmp/a b/office_corridor.rux' create planes --angle-threshold "
      "20 --plane-dist-threshold 0.04 --filter 'rooms in [1, 2]'";
  const std::string w = wrap_shell_command(line, 40);
  // Never inside a flag or a quoted word, and every line within the width
  // unless one group alone is wider.
  CHECK(w.find("--angle-threshold 20") != std::string::npos);
  CHECK(w.find("'rooms in [1, 2]'") != std::string::npos);
  CHECK(w.find("--\n") == std::string::npos);
  // A shell joins the continuations back into the same words.
  std::string joined;
  for (std::size_t i = 0; i < w.size(); ++i) {
    if (w[i] == '\\' && i + 1 < w.size() && w[i + 1] == '\n') {
      ++i;
      continue;
    }
    joined += w[i];
  }
  CHECK(split_shell_words(joined) == split_shell_words(line));
  CHECK(wrap_shell_command(line, 1000) == line);

  // A path longer than a line is never cut mid-word (a bare continuation
  // inside "project.rux" would paste back as a different filename, e.g.
  // "offi\ce_corridor.rux" — final review finding 9): it is placed on its
  // own line and allowed to overflow the width instead, and still joins
  // back into the same words.
  const std::string long_line =
      "rux -p /tmp/a-very-long-directory-name/and-another-one/project.rux "
      "create planes --angle-threshold 20";
  const std::string lw = wrap_shell_command(long_line, 24);
  CHECK(lw.find("/tmp/a-very-long-directory-name/and-another-one/"
                "project.rux") != std::string::npos);
  std::string lj;
  for (std::size_t i = 0; i < lw.size(); ++i) {
    if (lw[i] == '\\' && i + 1 < lw.size() && lw[i + 1] == '\n') {
      ++i;
      continue;
    }
    lj += lw[i];
  }
  CHECK(split_shell_words(lj) == split_shell_words(long_line));
}

TEST_CASE("FormatNumber_IsShortestAndLocaleFree", "[rux_qt][cli]") {
  CHECK(format_number(0.05) == "0.05");
  CHECK(format_number(25.0) == "25");
  CHECK(format_number(0.1 + 0.2) == "0.30000000000000004");
  CHECK(format_number(0.07F) == "0.07");
  CHECK(format_number(static_cast<float>(0.07)) == "0.07");
}

TEST_CASE("LogFilter_StageStatusAndText", "[rux_qt][log]") {
  const std::vector<LogRow> rows = {
      {"segment_planes", "success", true, "", R"({"radius":0.5})"},
      {"segment_rooms", "failed", true, "Leiden diverged", ""},
      {"segment_planes", "running", false, "", ""},
      {"cloud_reconstruction", "running", true, "", ""},
      {"segment_planes", "cancelled", true, "cancelled (user request)", ""},
  };
  auto count = [&](const LogFilter &f) {
    int n = 0;
    for (const auto &r : rows)
      n += log_row_matches(r, f);
    return n;
  };
  CHECK(count({}) == 5);
  CHECK(count({"segment_planes", LogStatusFilter::all, ""}) == 3);
  CHECK(count({"", LogStatusFilter::failed, ""}) == 1);
  CHECK(count({"", LogStatusFilter::success, ""}) == 1);
  // A cancel is its own outcome, never counted as a failure.
  CHECK(count({"", LogStatusFilter::cancelled, ""}) == 1);
  // "running" with a finish time is not unfinished (a crashed writer).
  CHECK(count({"", LogStatusFilter::unfinished, ""}) == 1);
  CHECK(count({"", LogStatusFilter::all, "LEIDEN"}) == 1);
  CHECK(count({"", LogStatusFilter::all, "radius"}) == 1);
  CHECK(count({"segment_rooms", LogStatusFilter::success, ""}) == 0);
  CHECK(log_stages(rows) == std::vector<std::string>{"cloud_reconstruction",
                                                     "segment_planes",
                                                     "segment_rooms"});
}

TEST_CASE("LogStatus_LabelAndTone_CancelledDistinctFromFailedAndRunning",
          "[rux_qt][log]") {
  CHECK(log_status_label("success", true) == "Gennemført");
  CHECK(log_status_label("failed", true) == "Fejlet");
  CHECK(log_status_label("cancelled", true) == "Annulleret");
  // A "running" row with no finished_at means the process died, not that it
  // is still active in a reopened project (#2 of the integration review).
  // Same term the Pipeline workspace's stage pill uses for the equivalent
  // case, so the two surfaces no longer name the same state differently
  // (#9: "Afbrudt" vs "Ikke afsluttet").
  CHECK(log_status_label("running", false) == "Afbrudt");

  CHECK(log_status_tone_key("success", true) == "good");
  CHECK(log_status_tone_key("failed", true) == "crit");
  CHECK(log_status_tone_key("cancelled", true) == "wait");
  CHECK(log_status_tone_key("running", false) == "wait");
  // A cancel is never coloured as a failure.
  CHECK(log_status_tone_key("cancelled", true) !=
        log_status_tone_key("failed", true));
}

TEST_CASE("Posegraf_NearestNodeAndEdge", "[rux_qt][posegraph]") {
  const std::vector<GraphNode> nodes = {
      {10, 0.0, 0.0}, {11, 1.0, 0.0}, {12, 1.0, 1.0}};
  CHECK(nearest_node(nodes, 0.9, 0.1, 0.5) == 1);
  CHECK(nearest_node(nodes, 5.0, 5.0, 0.5) == -1);
  CHECK(nearest_node(nodes, 0.5, 0.0, 0.5) == 0); // tie -> earlier

  const std::vector<GraphEdge> edges = {{10, 12}, {11, 12}, {10, 99}};
  CHECK(nearest_edge(nodes, edges, 0.5, 0.45, 0.2) == 0);
  CHECK(nearest_edge(nodes, edges, 1.05, 0.5, 0.2) == 1);
  CHECK(nearest_edge(nodes, edges, 3.0, 3.0, 0.2) == -1);
  CHECK(segment_distance(0, 1, -1, 0, 1, 0) == 1.0);
  CHECK(segment_distance(3, 0, -1, 0, 1, 0) == 2.0);
}

TEST_CASE("LogTap_DeliversToListenersUntilRemoved", "[rux_qt][log]") {
  std::vector<std::string> got;
  const auto token = add_log_listener([&](int level, std::string_view m) {
    got.push_back(std::to_string(level) + ":" + std::string(m));
  });
  publish_log(2, "planes: 49 planes");
  remove_log_listener(token);
  publish_log(2, "after");
  CHECK(got == std::vector<std::string>{"2:planes: 49 planes"});
}

TEST_CASE("LogTailAccepts_WarnAndAboveFromAnyThread", "[rux_qt][log]") {
  // trace (0) / debug (1): dropped regardless of thread.
  CHECK_FALSE(log_tail_accepts(0, true));
  CHECK_FALSE(log_tail_accepts(0, false));
  CHECK_FALSE(log_tail_accepts(1, true));
  CHECK_FALSE(log_tail_accepts(1, false));
  // info (2): only the job's own thread — an OMP/TBB worker's info line is
  // not the job's narrative.
  CHECK(log_tail_accepts(2, true));
  CHECK_FALSE(log_tail_accepts(2, false));
  // warn (3) and above: kept from any thread, since STANDARDS §5 requires a
  // warning reach the user even when it is logged from a fan-out worker
  // (e.g. segment_instances' OMP-thread cancellation warning).
  CHECK(log_tail_accepts(3, true));
  CHECK(log_tail_accepts(3, false));
  CHECK(log_tail_accepts(4, false));
  CHECK(log_tail_accepts(5, false));
}

TEST_CASE("BalancedCanvasSplit_SharesTheSpareEvenlyOverBothFloors",
          "[rux_qt][rediger]") {
  // 44 px of toolbar on the images pane, 10 on the overlay's.
  const auto s = balanced_canvas_split(560, 54, 10);
  CHECK(s[0] + s[1] == 560);
  const int images_canvas = s[0] - 54, overlay_canvas = s[1] - 10;
  CHECK(images_canvas - kMatchCanvasFloorPx ==
        overlay_canvas - kOverlayCanvasFloorPx);
  CHECK(images_canvas == 223);
}

TEST_CASE("BalancedCanvasSplit_OddSpare_TheImagesPaneGetsThePixel",
          "[rux_qt][rediger]") {
  const auto s = balanced_canvas_split(515, 0, 0);
  CHECK(s[0] == 200 + 33);
  CHECK(s[1] == 250 + 32);
}

TEST_CASE("BalancedCanvasSplit_TooLittleRoom_SharesTheShortfall",
          "[rux_qt][rediger]") {
  const auto s = balanced_canvas_split(430, 0, 0);
  CHECK(s[0] + s[1] == 430);
  CHECK(s[0] == 190);
  CHECK(s[1] == 240);
  const auto z = balanced_canvas_split(20, 50, 50);
  CHECK(z[0] >= 0);
  CHECK(z[1] >= 0);
  CHECK(z[0] + z[1] == 20);
}
