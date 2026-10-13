// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the Qt client's app-shell logic (Stream Q, phase Q1): the
// command palette's fuzzy ranking, the recent-projects list, what plain `rux`
// launches, and how a failed project open is explained. All of it lives in
// `rux_qt_core` (no Qt), so it runs in the light test binary.

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/fuzzy.hpp>
#include <rux_qt/launch.hpp>
#include <rux_qt/palette_table.hpp>
#include <rux_qt/recent.hpp>

#include <algorithm>
#include <set>
#include <string>
#include <vector>

using namespace rux::qt;

namespace {

std::vector<std::string> titles_of(const std::vector<PaletteMatch> &m,
                                   const std::vector<PaletteCandidate> &c) {
  std::vector<std::string> out;
  for (const auto &x : m)
    out.push_back(c[x.index].title);
  return out;
}

const std::vector<PaletteCandidate> kCommands = {
    {"Start", "forside velkomst"},
    {"Database", "tabeller billeder"},
    {"3D", "punktsky viewer"},
    {"Posegraf", "pose graph"},
    {"Pipeline", "kør trin"},
    {"Log", "kørselslog"},
    {"Åbn projekt…", "open fil"},
    {"Luk projekt", "close"},
    {"Vis eller skjul inspektør", "panel"},
    {"Skift til lyst tema", "theme"},
    {"Ændr billede", ""},
};

} // namespace

// ------------------------------------------------------------------ fuzzy --

TEST_CASE("Palette_EmptyQuery_ReturnsEveryCandidateInOrder",
          "[rux_qt][palette]") {
  for (const char *q : {"", "   "}) {
    const auto m = rank_palette(q, kCommands);
    REQUIRE(m.size() == kCommands.size());
    for (std::size_t i = 0; i < m.size(); ++i) {
      CHECK(m[i].index == i);
      CHECK(m[i].score == 0);
      CHECK(m[i].positions.empty());
    }
  }
}

TEST_CASE("Palette_NonSubsequence_IsLeftOut", "[rux_qt][palette]") {
  CHECK(fuzzy_score("xyz", "Database") == -1);
  CHECK(fuzzy_score("esab", "Database") == -1); // order matters
  const auto m = rank_palette("qqq", kCommands);
  CHECK(m.empty());
}

TEST_CASE("Palette_IsCaseInsensitive_AndIgnoresQuerySpaces",
          "[rux_qt][palette]") {
  CHECK(fuzzy_score("DATA", "Database") > 0);
  CHECK(fuzzy_score("luk pro", "Luk projekt") > 0);
  CHECK(fuzzy_score("lukpro", "Luk projekt") ==
        fuzzy_score("luk pro", "Luk projekt"));
}

TEST_CASE("Palette_PrefixAndWordStarts_BeatScatteredMatches",
          "[rux_qt][palette]") {
  // "db": Database's D is the title's first letter (the biggest bonus);
  // "Ændr billede" only has a word-start b. Database must win.
  const auto m = rank_palette("db", kCommands);
  REQUIRE_FALSE(m.empty());
  CHECK(kCommands[m.front().index].title == "Database");

  // A prefix beats the same letters later in a title.
  CHECK(fuzzy_score("log", "Log") > fuzzy_score("log", "Vis log for trin"));
  // Consecutive beats scattered.
  CHECK(fuzzy_score("pose", "Posegraf") >
        fuzzy_score("pose", "Pipeline og scener"));
  // Word starts: "vsi" = Vis Skjul Inspektør beats an in-word scatter.
  CHECK(fuzzy_score("vsi", "Vis eller skjul inspektør") >
        fuzzy_score("vsi", "avsi"));
}

TEST_CASE("Palette_Positions_PointAtTheMatchedCodePoints",
          "[rux_qt][palette]") {
  std::vector<int> pos;
  REQUIRE(fuzzy_score("pg", "Posegraf", &pos) > 0);
  CHECK(pos == std::vector<int>{0, 4});

  // Multi-byte letters count as one position each (QString indices).
  pos.clear();
  REQUIRE(fuzzy_score("ab", "Åbn projekt…", &pos) > 0);
  CHECK(pos == std::vector<int>{0, 1});
}

TEST_CASE("Palette_DanishLetters_FoldToTheirBaseLetter", "[rux_qt][palette]") {
  CHECK(fuzzy_score("korsel", "Kørselslog") > 0);
  CHECK(fuzzy_score("abn", "Åbn projekt…") > 0);
  CHECK(fuzzy_score("aendr", "Ændr billede") ==
        -1); // æ is one letter, not "ae"
  CHECK(fuzzy_score("andr", "Ændr billede") > 0);
  // The folded letters themselves, in either case, match too.
  CHECK(fuzzy_score("ø", "Kørselslog") > 0);
  CHECK(fuzzy_score("Å", "åbn") > 0);
  // An exact accented hit outranks the folded one.
  CHECK(fuzzy_score("ø", "kø") > fuzzy_score("o", "kø"));
}

TEST_CASE("Palette_KeywordOnlyMatch_RanksBelowTitleMatch_WithoutPositions",
          "[rux_qt][palette]") {
  // "theme" only appears in the keywords of "Skift til lyst tema".
  auto m = rank_palette("theme", kCommands);
  REQUIRE(m.size() == 1);
  CHECK(kCommands[m[0].index].title == "Skift til lyst tema");
  CHECK(m[0].positions.empty());

  // "log": a title match (Log) outranks a keyword match (kørselslog).
  m = rank_palette("log", kCommands);
  REQUIRE(m.size() >= 1);
  CHECK(kCommands[m[0].index].title == "Log");
  CHECK_FALSE(m[0].positions.empty());
}

TEST_CASE("Palette_KeywordMatch_NeedsAContiguousRun", "[rux_qt][palette]") {
  const std::vector<PaletteCandidate> c = {{"Afslut", "quit exit"},
                                           {"Log", "pipeline kørsel"}};
  // q-t: a subsequence of "quit", not a run -> no match.
  CHECK(rank_palette("qt", c).empty());
  CHECK(rank_palette("quit", c).size() == 1);
  // Folding applies inside keywords too.
  CHECK(rank_palette("korsel", c).size() == 1);
}

TEST_CASE("Palette_Ties_AreBrokenByLengthThenOrder", "[rux_qt][palette]") {
  const std::vector<PaletteCandidate> c = {
      {"Projekt B lang", ""}, {"Projekt A", ""}, {"Projekt C", ""}};
  const auto t = titles_of(rank_palette("projekt", c), c);
  CHECK(t ==
        std::vector<std::string>{"Projekt A", "Projekt C", "Projekt B lang"});
}

TEST_CASE("Palette_LongTexts_AreMatchedOnlyWithinTheCap", "[rux_qt][palette]") {
  // Texts are matched within their first 512 code points: a hit inside the
  // cap is found, one only past it is deliberately dropped (a pathological
  // keyword string must not make every keystroke quadratic).
  std::string long_text(600, 'a');
  CHECK(fuzzy_score("aaa", long_text) > 0);
  std::string late(600, 'x');
  late += "project";
  CHECK(fuzzy_score("project", late) == -1);
  std::string early = "project" + std::string(600, 'x');
  CHECK(fuzzy_score("project", early) > 0);
  const std::vector<PaletteCandidate> c = {{"x", late}};
  CHECK(rank_palette("project", c).empty());
}

// ------------------------------------------------- the real palette table --

namespace {

PaletteState open_state() {
  PaletteState s;
  s.project_open = true;
  s.project_path = "/home/u/sager/NewOffice/project.rux";
  s.current_page = 0;
  s.recent = {{"/home/u/sager/NewOffice/project.rux", false},
              {"/home/u/sager/Kontorhus Valby/project.rux", true},
              {"/home/u/sager/Skolen 2025/scan-02.rux", true}};
  return s;
}

std::vector<std::string> ranked_titles(const std::string &q,
                                       const std::vector<PaletteEntry> &t) {
  std::vector<std::string> out;
  for (const auto &m : rank_palette(q, palette_candidates(t)))
    out.push_back(t[m.index].title);
  return out;
}

} // namespace

TEST_CASE("PaletteTable_ListsPagesActionsAndRecents", "[rux_qt][palette]") {
  const auto t = build_palette(open_state());
  REQUIRE(t.size() == 6 + 8 + 4 + 3);
  CHECK(t[0].title == "Start");
  CHECK(t[0].badge == "Her");
  CHECK(t[1].title == "Oversigt");
  CHECK(t[1].shortcut == "Alt+2");
  CHECK(t[3].title == "Rediger");
  // Without a project there is nothing to close, reload, copy or show.
  PaletteState none;
  for (const auto &e : build_palette(none))
    CHECK((e.id != "close" && e.id != "reload" && e.id != "copy-path" &&
           e.id.rfind("pane:", 0) != 0));
  // The Rediger panes, after the project actions.
  CHECK(t[14].id == "pane:graph");
  CHECK(t[17].id == "pane:overlay");
  CHECK(t[17].title == "Vis eller skjul 3D-overlay");
  // Recents: generic project.rux shows its folder; missing ones are disabled.
  CHECK(t[18].title == "NewOffice");
  CHECK(t[18].badge == "Åben");
  CHECK(t[19].title == "Kontorhus Valby");
  CHECK_FALSE(t[19].enabled);
  CHECK(t[19].badge == "Mangler");
  CHECK(action_shortcut("open") == "Ctrl+O");
}

TEST_CASE("PaletteTable_Db_FindsOversigtFirst", "[rux_qt][palette]") {
  const auto r = ranked_titles("db", build_palette(open_state()));
  REQUIRE_FALSE(r.empty());
  CHECK(r.front() == "Oversigt");
}

TEST_CASE("PaletteTable_Posegraf_FindsRediger", "[rux_qt][palette]") {
  const auto r = ranked_titles("posegraf", build_palette(open_state()));
  REQUIRE_FALSE(r.empty());
  CHECK(r.front() == "Rediger");
}

TEST_CASE("SmokePage_NamesMapToRailIndices", "[rux_qt][shell]") {
  CHECK(smoke_page_index("start") == 0);
  CHECK(smoke_page_index("oversigt") == 1);
  CHECK(smoke_page_index("database") == 1); // the old name keeps working
  CHECK(smoke_page_index("3d") == 2);
  CHECK(smoke_page_index("rediger") == 3);
  CHECK(smoke_page_index("pipeline") == 4);
  CHECK(smoke_page_index("log") == 5);
  CHECK(smoke_page_index("posegraf") == -1);
  CHECK(smoke_page_index("") == -1);
}

TEST_CASE("EditPane_KeysRoundTrip", "[rux_qt][shell]") {
  for (int i = 0; i < kEditPaneCount; ++i) {
    const auto p = static_cast<EditPane>(i);
    CHECK(parse_edit_pane(edit_pane_key(p)) == p);
  }
  CHECK(edit_pane_name_da(EditPane::edges) == "Kantliste");
  CHECK_FALSE(parse_edit_pane("nope").has_value());
}

TEST_CASE("PaletteTable_Pro_RanksTheProjectActionsFirst", "[rux_qt][palette]") {
  const auto r = ranked_titles("pro", build_palette(open_state()));
  REQUIRE(r.size() >= 5);
  const std::set<std::string> top4(r.begin(), r.begin() + 4);
  CHECK(top4 == std::set<std::string>{"Luk projekt", "Åbn projekt…",
                                      "Kopiér projektets sti",
                                      "Genindlæs projekt"});
  // The scattered K-o-p…r…o hit comes after every "pro…" word.
  CHECK(r[4] == "Kopiér som rux-kommando");
}

TEST_CASE("PaletteTable_RecentFolder_IsSearchable_ButPathsAreNot",
          "[rux_qt][palette]") {
  const auto t = build_palette(open_state());
  auto r = ranked_titles("valby", t);
  REQUIRE_FALSE(r.empty());
  CHECK(r.front() == "Kontorhus Valby");
  // A path fragment every row shares matches none of them.
  CHECK(ranked_titles("home", t).empty());
  CHECK(ranked_titles("sager", t).empty());
}

// ----------------------------------------------------------------- recent --

TEST_CASE("Recent_Normalise_IsLexicalAndAbsolute", "[rux_qt][recent]") {
  CHECK(normalise_project_path("a/./b/../c.rux", "/home/u") ==
        "/home/u/a/c.rux");
  CHECK(normalise_project_path("/x//y/z.rux", "/home/u") == "/x/y/z.rux");
  CHECK(normalise_project_path("", "/home/u").empty());
}

TEST_CASE("Recent_Push_MovesToFront_DeDuplicates_AndCaps", "[rux_qt][recent]") {
  std::vector<std::string> l;
  for (int i = 0; i < 12; ++i)
    l = recent_push(l, "/p/" + std::to_string(i) + ".rux");
  REQUIRE(l.size() == kMaxRecent);
  CHECK(l.front() == "/p/11.rux");
  CHECK(l.back() == "/p/2.rux");

  l = recent_push(l, "/p/5.rux");
  CHECK(l.size() == kMaxRecent);
  CHECK(l.front() == "/p/5.rux");
  CHECK(std::count(l.begin(), l.end(), "/p/5.rux") == 1);

  CHECK(recent_push({}, "").empty());
  CHECK(recent_push({"/a", "", "/b"}, "/c") ==
        std::vector<std::string>{"/c", "/a", "/b"});
  CHECK(recent_push({"/a", "/b", "/c"}, "/d", 2) ==
        std::vector<std::string>{"/d", "/a"});
}

TEST_CASE("Recent_Remove_DropsEveryCopy", "[rux_qt][recent]") {
  CHECK(recent_remove({"/a", "/b", "/a"}, "/a") ==
        std::vector<std::string>{"/b"});
  CHECK(recent_remove({"/a"}, "/zzz") == std::vector<std::string>{"/a"});
}

TEST_CASE("Recent_Sanitise_CleansAStoredList", "[rux_qt][recent]") {
  std::vector<std::string> stored = {"/a/b.rux", "", "/a/./b.rux", "rel.rux"};
  for (int i = 0; i < 20; ++i)
    stored.push_back("/many/" + std::to_string(i));
  const auto l = recent_sanitise(stored, "/cwd");
  REQUIRE(l.size() == kMaxRecent);
  CHECK(l[0] == "/a/b.rux");
  CHECK(l[1] == "/cwd/rel.rux");
  CHECK(l[2] == "/many/0");
}

TEST_CASE("Recent_Annotate_MarksMissingFiles", "[rux_qt][recent]") {
  const std::set<std::string> on_disk = {"/here.rux"};
  const auto e =
      recent_annotate({"/here.rux", "/gone.rux"}, [&](const std::string &p) {
        return on_disk.count(p) > 0;
      });
  REQUIRE(e.size() == 2);
  CHECK_FALSE(e[0].missing);
  CHECK(e[1].missing);
  CHECK(e[1].path == "/gone.rux");
}

// ----------------------------------------------------------------- launch --

TEST_CASE("Launch_Subcommand_AlwaysRunsTheCli", "[rux_qt][launch]") {
  LaunchInputs in;
  in.has_subcommand = true;
  in.display = ":0";
  CHECK(decide_launch(in) == LaunchAction::run_subcommand);
  in.display = "";
  CHECK(decide_launch(in) == LaunchAction::run_subcommand);
}

TEST_CASE("Launch_NoSubcommand_OpensTheAppOnlyWithADisplay",
          "[rux_qt][launch]") {
  const std::set<std::string> sockets = {"/run/user/1000/wayland-1",
                                         "/tmp/.X11-unix/X0"};
  LaunchInputs in;
  in.xdg_runtime_dir = "/run/user/1000";
  in.path_exists = [&](const std::string &p) { return sockets.count(p) > 0; };
  CHECK(decide_launch(in) == LaunchAction::print_help);

  in.display = ":0";
  CHECK(decide_launch(in) == LaunchAction::open_app);
  in.display = "unix:0.0";
  CHECK(decide_launch(in) == LaunchAction::open_app);

  in.display = "";
  in.wayland_display = "wayland-1";
  CHECK(decide_launch(in) == LaunchAction::open_app);

  // A desktop profile's platform list leaks into SSH sessions: not a display.
  in.wayland_display = "";
  in.qpa_platform = "wayland;xcb";
  CHECK(decide_launch(in) == LaunchAction::print_help);

  // Explicit offscreen (tests, screenshots) counts, with options too.
  in.qpa_platform = "offscreen";
  CHECK(decide_launch(in) == LaunchAction::open_app);
  in.qpa_platform = "offscreen:fontengine=freetype";
  CHECK(decide_launch(in) == LaunchAction::open_app);

  // No GUI launcher linked: help, never a crash.
  in.qt_client_built = false;
  in.display = ":0";
  CHECK(decide_launch(in) == LaunchAction::print_help);
}

TEST_CASE("Launch_StaleOrMismatchedDisplays_PrintHelp", "[rux_qt][launch]") {
  const std::set<std::string> sockets = {"/run/user/1000/wayland-1",
                                         "/tmp/.X11-unix/X0"};
  auto base = [&] {
    LaunchInputs in;
    in.xdg_runtime_dir = "/run/user/1000";
    in.path_exists = [&](const std::string &p) { return sockets.count(p) > 0; };
    return in;
  };

  // A stale WAYLAND_DISPLAY (tmux/ssh) whose socket is gone.
  auto in = base();
  in.wayland_display = "wayland-9";
  CHECK_FALSE(has_display(in));
  // A relative name with no XDG_RUNTIME_DIR cannot be resolved.
  in = base();
  in.wayland_display = "wayland-1";
  in.xdg_runtime_dir = "";
  CHECK_FALSE(has_display(in));
  // An absolute socket path works without XDG_RUNTIME_DIR.
  in.wayland_display = "/run/user/1000/wayland-1";
  CHECK(has_display(in));

  // A leaked local DISPLAY with no X socket.
  in = base();
  in.display = ":7";
  CHECK_FALSE(has_display(in));
  // A remote (ssh -X) display is trusted.
  in.display = "localhost:10.0";
  CHECK(has_display(in));
  // Garbage.
  in.display = "nonsense";
  CHECK_FALSE(has_display(in));

  // QT_QPA_PLATFORM names one platform: its variable must be usable.
  in = base();
  in.qpa_platform = "xcb";
  in.wayland_display = "wayland-1"; // no XWayland DISPLAY
  CHECK_FALSE(has_display(in));
  in = base();
  in.qpa_platform = "wayland";
  in.display = ":0"; // X only
  CHECK_FALSE(has_display(in));
  in.wayland_display = "wayland-1";
  CHECK(has_display(in));
  // A list needs any member.
  in = base();
  in.qpa_platform = "wayland;xcb";
  in.display = ":0";
  CHECK(has_display(in));
  // An explicit other platform is trusted.
  in = base();
  in.qpa_platform = "vnc";
  CHECK(has_display(in));
}

// ------------------------------------------------------------ open errors --

TEST_CASE("OpenError_Classify_MapsSqliteAndProjectDbMessages",
          "[rux_qt][open]") {
  CHECK(classify_open_error("Cannot open database: database is locked") ==
        OpenErrorKind::locked);
  CHECK(classify_open_error("Failed to create table: database is locked") ==
        OpenErrorKind::locked);
  CHECK(classify_open_error("SQL exec failed: database table is locked") ==
        OpenErrorKind::locked);
  CHECK(classify_open_error("SQL exec failed: file is not a database") ==
        OpenErrorKind::not_a_project);
  CHECK(
      classify_open_error("Required table 'projects' not found in database") ==
      OpenErrorKind::not_a_project);
  CHECK(classify_open_error(
            "Cannot open database: unable to open database file") ==
        OpenErrorKind::permission);
  CHECK(classify_open_error("attempt to write a readonly database") ==
        OpenErrorKind::permission);
  CHECK(classify_open_error("database disk image is malformed") ==
        OpenErrorKind::corrupt);
  CHECK(classify_open_error("Migration to v12 failed: no such column") ==
        OpenErrorKind::other);
}

TEST_CASE("OpenError_EveryKind_HasDanishTitleAndHint", "[rux_qt][open]") {
  for (auto k : {OpenErrorKind::not_found, OpenErrorKind::not_a_file,
                 OpenErrorKind::permission, OpenErrorKind::locked,
                 OpenErrorKind::not_a_project, OpenErrorKind::corrupt,
                 OpenErrorKind::other}) {
    CHECK_FALSE(open_error_title_da(k).empty());
    CHECK_FALSE(open_error_hint_da(k).empty());
  }
  CHECK(open_error_title_da(OpenErrorKind::locked).find("låst") !=
        std::string::npos);
  CHECK(open_error_title_da(OpenErrorKind::none).empty());
}

TEST_CASE("OpenError_WalInReadOnlyDirectory_HasItsOwnDanishText",
          "[rux_qt][open_error]") {
  const auto k = OpenErrorKind::wal_read_only_dir;
  CHECK(open_error_title_da(k).find("skrivebeskyttet") != std::string::npos);
  CHECK(open_error_hint_da(k).find("-wal") != std::string::npos);
}
