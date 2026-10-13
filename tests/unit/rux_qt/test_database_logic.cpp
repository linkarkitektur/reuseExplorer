// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The Qt-free logic of the Database workspace (Stream Q, Q2): the A/B frame
// pair and its keys, the pending pose-graph edits, the table paging, and the
// formatters the widgets show. All in rux_qt_core, so it runs in the light
// test binary.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <rux_qt/database_logic.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace rux::qt;
using Catch::Approx;

// ------------------------------------------------------------ frame pair --

TEST_CASE("FramePair_Reset_SortsIdsAndPicksTheFirstTwo",
          "[rux_qt][database][pair]") {
  FramePair p;
  CHECK(p.empty());
  CHECK(p.a_id() == -1);
  CHECK(p.b_id() == -1);
  p.reset({30, 10, 20, 10});
  REQUIRE(p.size() == 3);
  CHECK(p.ids() == std::vector<int>{10, 20, 30});
  CHECK(p.a_id() == 10);
  CHECK(p.b_id() == 20);

  p.reset({7});
  CHECK(p.a_id() == 7);
  CHECK(p.b_id() == 7);
  p.reset({});
  CHECK(p.a_index() == -1);
}

TEST_CASE("FramePair_Steps_AreClampedAndReportMovement",
          "[rux_qt][database][pair]") {
  FramePair p;
  p.reset({1, 2, 3, 4, 5});
  CHECK_FALSE(p.step_a(-1)); // already first
  CHECK(p.step_a(1));
  CHECK(p.a_id() == 2);
  CHECK(p.step_a(100));
  CHECK(p.a_id() == 5);
  CHECK_FALSE(p.step_a(1));
  CHECK(p.step_b(-100));
  CHECK(p.b_id() == 1);
  CHECK(p.a_id() == 5); // B never drags A along
}

TEST_CASE("FramePair_SetById_RejectsUnknownIds", "[rux_qt][database][pair]") {
  FramePair p;
  p.reset({100, 200, 300});
  CHECK(p.set_b_id(300));
  CHECK(p.b_index() == 2);
  CHECK_FALSE(p.set_a_id(150));
  CHECK(p.a_id() == 100);
  CHECK_FALSE(p.set_a_index(3));
  CHECK_FALSE(p.set_a_index(-1));
  CHECK(p.index_of(200) == 1);
  CHECK(p.index_of(201) == -1);
}

TEST_CASE("FramePair_NearestIndex_FindsTheClosestId",
          "[rux_qt][database][pair]") {
  FramePair p;
  CHECK(p.nearest_index(5) == -1);
  p.reset({10, 20, 40});
  CHECK(p.nearest_index(0) == 0);
  CHECK(p.nearest_index(14) == 0);
  CHECK(p.nearest_index(15) == 0); // tie -> lower id
  CHECK(p.nearest_index(16) == 1);
  CHECK(p.nearest_index(31) == 2);
  CHECK(p.nearest_index(999) == 2);
}

TEST_CASE("BrowserKey_ArrowsMoveAAndShiftArrowsMoveB",
          "[rux_qt][database][keys]") {
  CHECK(browser_key(ArrowKey::left, false, false) == BrowserKey::a_prev);
  CHECK(browser_key(ArrowKey::right, false, false) == BrowserKey::a_next);
  CHECK(browser_key(ArrowKey::left, true, false) == BrowserKey::b_prev);
  CHECK(browser_key(ArrowKey::right, true, false) == BrowserKey::b_next);
  CHECK(browser_key(ArrowKey::other, false, false) == BrowserKey::none);
  // A focused text field or spin box keeps its own cursor keys.
  CHECK(browser_key(ArrowKey::left, false, true) == BrowserKey::none);
  CHECK(browser_key(ArrowKey::right, true, true) == BrowserKey::none);
  CHECK(browser_step(false) == 1);
  CHECK(browser_step(true) == 10);
}

// --------------------------------------------------------- pending edits --

namespace {

EdgeRecord edge(int from, int to, const char *type = "loop_closure",
                const char *status = "auto", double residual = 0.0,
                double weight = 1.0) {
  EdgeRecord r;
  r.key = {from, to, type};
  r.residual = residual;
  r.weight = weight;
  r.status = status;
  return r;
}

using R = PendingEdgeEdits::Result;
using K = PendingEdgeEdits::Op::Kind;

} // namespace

TEST_CASE("PendingEdits_Add_StagesAConfirmedMeasurement",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(2, 3, "loop_closure", "rejected")});
  EdgeRecord m = edge(1, 5);
  m.transform =
      std::array<double, 16>{1, 0, 0, 0.1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  m.source = "manual";
  CHECK(e.add(m) == R::added);
  CHECK(e.count() == 1);
  const auto ops = e.ops();
  REQUIRE(ops.size() == 1);
  CHECK(ops[0].kind == K::upsert);
  CHECK(ops[0].edge.status == "confirmed"); // a user edge is confirmed (D4)
  CHECK(ops[0].edge.transform.has_value());
  CHECK_FALSE(ops[0].overwrite_rejected);
  // Staging the same key again replaces the staged measurement.
  CHECK(e.add(edge(1, 5)) == R::replaced);
  CHECK(e.count() == 1);
  // Over a stored active edge without a measurement: the store only
  // changes its status, so it is a confirmation, not a replacement.
  CHECK(e.add(edge(1, 2, "odometry")) == R::confirmed);
  CHECK(e.count() == 2);
  // With a measurement it replaces the stored one.
  CHECK(e.add(m) == R::replaced); // (1, 5) is staged: re-staged
  EdgeRecord m12 = edge(1, 2, "odometry");
  m12.transform = m.transform;
  CHECK(e.add(m12) == R::replaced);
  CHECK(e.count() == 2);
}

TEST_CASE("PendingEdits_AddWithoutMeasurement_OverStored_KeepsTheMeasurement",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  EdgeRecord stored = edge(1, 2, "loop_closure", "auto", 3.5, 9.0);
  stored.transform =
      std::array<double, 16>{1, 0, 0, 0.3, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  stored.source = "orb";
  stored.deviation_m = 0.2;
  e.set_base({stored, edge(3, 4, "loop_closure", "confirmed")});
  REQUIRE(e.add(edge(1, 2, "loop_closure", "auto", 0.0, 400.0)) ==
          R::confirmed);
  // Already confirmed: nothing to stage.
  CHECK(e.add(edge(3, 4)) == R::unchanged);
  CHECK(e.count() == 1);
  // Shown as the stored measurement, now confirmed.
  const auto v = e.effective();
  REQUIRE(v.size() == 2);
  CHECK(v[0].pending == EdgeView::Pending::status);
  CHECK(v[0].edge.status == "confirmed");
  CHECK(v[0].edge.weight == 9.0);
  CHECK(v[0].edge.source == "orb");
  // After the save the base holds what the store holds: status only.
  e.commit_saved(e.ops());
  CHECK(e.empty());
  REQUIRE(e.base().size() == 2);
  CHECK(e.base()[0].status == "confirmed");
  CHECK(e.base()[0].weight == 9.0);
  CHECK(e.base()[0].residual == 3.5);
  CHECK(e.base()[0].source == "orb");
  CHECK(e.base()[0].transform == stored.transform);
  CHECK(e.base()[0].deviation_m == 0.2);
}

TEST_CASE("PendingEdits_CommitSaved_NewKey_StoresTheWrittenSource",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  REQUIRE(e.add(edge(1, 2)) == R::added);
  e.commit_saved(e.ops());
  REQUIRE(e.base().size() == 1);
  CHECK(e.base()[0].source == "manual"); // what edge_row_from_record writes
}

TEST_CASE("PendingEdits_AddOverRejected_Revives",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(2, 3, "loop_closure", "rejected")});
  CHECK(e.add(edge(2, 3)) == R::revived);
  const auto ops = e.ops();
  REQUIRE(ops.size() == 1);
  CHECK(ops[0].overwrite_rejected); // the store refuses a tombstone otherwise
}

TEST_CASE("PendingEdits_RejectsInvalidEdges", "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  CHECK(e.add(edge(3, 3)) == R::invalid);
  CHECK(e.add(edge(1, 2, "bogus")) == R::invalid);
  CHECK(e.add(edge(1, 2, "loop_closure", "auto", 0.0, 0.0)) == R::invalid);
  CHECK(e.add(edge(1, 2, "loop_closure", "auto", 0.0, -1.0)) == R::invalid);
  // No ICP: the weight is unknown (NaN) and that is allowed.
  CHECK(e.add(edge(1, 2, "loop_closure", "auto", 0.0,
                   std::numeric_limits<double>::quiet_NaN())) == R::added);
}

TEST_CASE("PendingEdits_ConfirmRejectUnreject", "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(4, 5, "loop_closure", "rejected"),
              edge(6, 7, "loop_closure", "confirmed")});
  CHECK(e.confirm({1, 2, "odometry"}) == R::confirmed);
  CHECK(e.confirm({6, 7, "loop_closure"}) == R::unchanged);
  CHECK(e.confirm({8, 9, "loop_closure"}) == R::not_found);
  // Rejecting a staged confirmation replaces it.
  CHECK(e.reject({1, 2, "odometry"}) == R::rejected);
  CHECK(e.count() == 1);
  CHECK(e.ops()[0].edge.status == "rejected");
  // Un-rejecting a staged rejection of an active edge just drops it.
  CHECK(e.unreject({1, 2, "odometry"}) == R::unstaged);
  CHECK(e.empty());
  // A stored tombstone comes back as auto.
  CHECK(e.unreject({4, 5, "loop_closure"}) == R::unrejected);
  CHECK(e.ops()[0].edge.status == "auto");
  CHECK(e.reject({4, 5, "loop_closure"}) == R::unstaged); // back to the file
  CHECK(e.empty());
  // A staged new edge is simply dropped on reject.
  REQUIRE(e.add(edge(10, 11)) == R::added);
  CHECK(e.reject({10, 11, "loop_closure"}) == R::unstaged);
  CHECK(e.empty());
  CHECK(e.reject({10, 11, "loop_closure"}) == R::not_found);
}

TEST_CASE("PendingEdits_Revert_DropsWhateverIsStaged",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry")});
  e.reject({1, 2, "odometry"});
  CHECK(e.revert({1, 2, "odometry"}) == R::unstaged);
  CHECK(e.revert({1, 2, "odometry"}) == R::unchanged);
  CHECK(e.empty());
}

TEST_CASE("PendingEdits_Ops_StatusChangesFirstThenMeasurements",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(4, 5, "panorama")});
  REQUIRE(e.add(edge(7, 8)) == R::added);
  REQUIRE(e.reject({1, 2, "odometry"}) == R::rejected);
  REQUIRE(e.add(edge(2, 9)) == R::added);
  REQUIRE(e.confirm({4, 5, "panorama"}) == R::confirmed);
  const auto ops = e.ops();
  REQUIRE(ops.size() == 4);
  CHECK(ops[0].kind == K::set_status);
  CHECK(ops[0].edge.key == EdgeKey{1, 2, "odometry"});
  CHECK(ops[1].kind == K::set_status);
  CHECK(ops[1].edge.key == EdgeKey{4, 5, "panorama"});
  CHECK(ops[2].edge.key == EdgeKey{7, 8, "loop_closure"});
  CHECK(ops[3].edge.key == EdgeKey{2, 9, "loop_closure"});
}

TEST_CASE("PendingEdits_Effective_AppliesPendingOnTheBase",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  auto stored = edge(1, 2, "odometry", "auto", 3.5, 9.0);
  stored.deviation_m = 0.4;
  e.set_base({stored, edge(2, 1, "panorama"), edge(2, 3, "odometry")});
  REQUIRE(e.reject({2, 1, "panorama"}) == R::rejected);
  REQUIRE(e.add(edge(1, 2, "loop_closure")) == R::added);
  const auto all = e.effective();
  REQUIRE(all.size() == 4);
  CHECK(all[0].pending == EdgeView::Pending::none);
  CHECK(all[0].edge.deviation_m == 0.4);
  CHECK(all[1].pending == EdgeView::Pending::status);
  CHECK(all[1].edge.status == "rejected");
  CHECK(all[1].stored_status == "auto");
  CHECK(all[3].pending == EdgeView::Pending::added);
  CHECK(all[3].stored_status.empty());

  const auto v = e.between(1, 2);
  REQUIRE(v.size() == 3);
  CHECK_FALSE(v[0].reversed);
  CHECK(v[1].reversed);
  CHECK(e.between(2, 1)[0].reversed);
  CHECK(e.between(1, 3).empty());
  // Rejected edges do not count towards a frame's degree.
  CHECK(e.degree(1) == 2); // odometry + staged loop closure
  CHECK(e.degree(2) == 3);
  CHECK(e.degree(9) == 0);
}

TEST_CASE("PendingEdits_Degrees_MatchDegreePerFrame", "[rux_qt][database]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(2, 3, "odometry"),
              edge(1, 2, "odometry"), // duplicate row, counted once
              edge(3, 1, "loop_closure")});
  e.reject({2, 3, "odometry"});
  e.add(edge(4, 1));
  const auto all = e.degrees();
  for (int f : {1, 2, 3, 4}) {
    INFO(f);
    const auto it = all.find(f);
    CHECK((it == all.end() ? 0 : it->second) == e.degree(f));
  }
}

TEST_CASE("PendingEdits_CommitSaved_KeepsEditsStagedDuringTheSave",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(4, 5, "panorama")});
  REQUIRE(e.add(edge(7, 8)) == R::added);
  REQUIRE(e.reject({1, 2, "odometry"}) == R::rejected);
  const auto snapshot = e.ops(); // the save starts with these two
  // While it runs: one more edit, and one that changes a saved one again.
  REQUIRE(e.confirm({4, 5, "panorama"}) == R::confirmed);
  REQUIRE(e.add(edge(7, 8, "loop_closure", "auto", 0.0, 4.0)) == R::replaced);
  e.commit_saved(snapshot);
  // The base now holds the saved state.
  REQUIRE(e.base().size() == 3);
  CHECK(e.base()[0].status == "rejected");
  CHECK(e.base()[2].key == EdgeKey{7, 8, "loop_closure"});
  // What was staged during the save stays pending.
  const auto left = e.ops();
  REQUIRE(left.size() == 2);
  CHECK(left[0].kind == K::set_status);
  CHECK(left[0].edge.key == EdgeKey{4, 5, "panorama"});
  CHECK(left[1].kind == K::upsert);
  CHECK(left[1].edge.weight == 4.0);
}

TEST_CASE("PendingEdits_CommitSaved_ManualEdgeWithoutWeight_LeavesNothing",
          "[rux_qt][database][pending]") {
  // A manual edge has a NaN weight; NaN != NaN must not keep it pending.
  PendingEdgeEdits e;
  REQUIRE(e.add(edge(1, 2, "loop_closure", "auto", 0.0,
                     std::numeric_limits<double>::quiet_NaN())) == R::added);
  e.commit_saved(e.ops());
  CHECK(e.empty());
  CHECK(e.base().size() == 1);
}

TEST_CASE("PendingSummary_NamesTheKindsInDanish",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(3, 4, "loop_closure")});
  e.add(edge(5, 6));
  e.reject({1, 2, "odometry"});
  e.confirm({3, 4, "loop_closure"});
  CHECK(pending_summary_da(e.ops()) ==
        "1 ny eller målt kant, 1 afvist, 1 bekræftet");
  e.discard();
  e.add(edge(5, 6));
  e.add(edge(7, 6));
  CHECK(pending_summary_da(e.ops()) == "2 nye eller målte kanter");
}

TEST_CASE("EdgeTypes_HaveDanishNames", "[rux_qt][database]") {
  CHECK(is_edge_type("odometry"));
  CHECK(is_edge_type("loop_closure"));
  CHECK(is_edge_type("panorama"));
  CHECK_FALSE(is_edge_type("gps"));
  CHECK(edge_type_da("odometry") == "Odometri");
  CHECK(edge_type_da("loop_closure") == "Løkkelukning");
  CHECK(edge_type_da("panorama") == "Panorama");
  CHECK(edge_type_da("gps") == "gps");
}

TEST_CASE("WeightFromIcp_IsInverseVarianceWithAFloor", "[rux_qt][database]") {
  CHECK(weight_from_icp_fitness(0.1) == Approx(100.0));
  CHECK(weight_from_icp_fitness(0.02) == Approx(2500.0));
  CHECK(weight_from_icp_fitness(0.0) == Approx(10000.0));
  CHECK(weight_from_icp_fitness(-1.0) == Approx(10000.0));
  CHECK(weight_from_icp_fitness(std::nan("")) == Approx(10000.0));
}

// ---------------------------------------------------------------- paging --

TEST_CASE("Paging_FetchesPagesUntilTheEnd", "[rux_qt][database][paging]") {
  Paging p{450, 0, 200};
  CHECK(p.can_fetch_more());
  CHECK(p.next_count() == 200);
  p.loaded += p.next_count();
  CHECK(p.next_count() == 200);
  p.loaded += p.next_count();
  CHECK(p.next_count() == 50);
  p.loaded += p.next_count();
  CHECK_FALSE(p.can_fetch_more());
  CHECK(p.next_count() == 0);

  Paging empty{0, 0, 200};
  CHECK_FALSE(empty.can_fetch_more());
  CHECK(empty.next_count() == 0);
}

TEST_CASE("Paging_CountToReach_RoundsUpToWholePages",
          "[rux_qt][database][paging]") {
  Paging p{1000, 200, 200};
  CHECK(p.count_to_reach(150) == 0);   // already loaded
  CHECK(p.count_to_reach(200) == 200); // first row of the next page
  CHECK(p.count_to_reach(401) == 400);
  CHECK(p.count_to_reach(999) == 800);
  CHECK(p.count_to_reach(5000) == 800); // clamped to the table
  CHECK(p.count_to_reach(-3) == 0);
}

// ------------------------------------------------------------ formatting --

TEST_CASE("FormatBytes_IsDanishAndScaled", "[rux_qt][database][format]") {
  CHECK(format_bytes_da(0) == "0 B");
  CHECK(format_bytes_da(834) == "834 B");
  CHECK(format_bytes_da(33391) == "33,4 kB");
  CHECK(format_bytes_da(1234567) == "1,2 MB");
  CHECK(format_bytes_da(3400000000ULL) == "3,40 GB");
  CHECK(format_bytes_da(999999) == "1,0 MB");
}

TEST_CASE("SniffBlob_RecognisesCommonFormats", "[rux_qt][database][format]") {
  CHECK(sniff_blob(std::string("\x89PNG\r\n\x1a\n....", 12)) == "PNG");
  CHECK(sniff_blob(std::string("\xff\xd8\xff\xe0", 4)) == "JPEG");
  CHECK(sniff_blob("ply\nformat binary") == "PLY");
  CHECK(sniff_blob("%PDF-1.7") == "PDF");
  CHECK(sniff_blob(std::string("RIFF\x10\0\0\0WEBPVP8 ", 16)) == "WebP");
  CHECK(sniff_blob(std::string("\x1f\x8b\x08", 3)) == "GZIP");
  CHECK(sniff_blob("PK\x03\x04") == "ZIP");
  CHECK(sniff_blob("  {\"a\":1}") == "JSON");
  CHECK(sniff_blob(std::string("{\0\x80?\x12\0\0\0", 8)) == ""); // floats
  CHECK(sniff_blob(std::string("\0\1\2", 3)) == "");
  CHECK(sniff_blob("") == "");
}

TEST_CASE("DescribeBlob_CombinesMeaningFormatAndSize",
          "[rux_qt][database][format]") {
  const std::string jpeg("\xff\xd8\xff\xe0", 4);
  CHECK(describe_blob("sensor_frames", "color", jpeg, 176846) ==
        "Farvebillede · JPEG · 176,8 kB");
  CHECK(describe_blob("sensor_frames", "transform", std::string(16, '\0'),
                      128) == "Pose 4×4 · 128 B");
  CHECK(describe_blob("whatever", "payload", std::string(4, '\1'), 2048) ==
        "2,0 kB");
  CHECK(describe_blob("whatever", "image", "%PDF-1.4", 10) == "PDF · 10 B");
  CHECK(blob_column_meaning("segmentation_images", "label_image") ==
        "Mærkatbillede");
  CHECK(blob_column_meaning("nope", "color").empty());
}

TEST_CASE("SummarizePose_ReadsTranslationAndAngles",
          "[rux_qt][database][pose]") {
  const std::array<double, 16> identity{1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};
  auto s = summarize_pose(identity);
  CHECK(s.t[0] == 0.0);
  CHECK(s.angle_deg == Approx(0.0).margin(1e-9));

  // 90 degrees about z, translated (1, 2, 3): row-major, translation in
  // the last column.
  const std::array<double, 16> yaw90{0, -1, 0, 1, 1, 0, 0, 2,
                                     0, 0,  1, 3, 0, 0, 0, 1};
  s = summarize_pose(yaw90);
  CHECK(s.t[0] == 1.0);
  CHECK(s.t[1] == 2.0);
  CHECK(s.t[2] == 3.0);
  CHECK(s.yaw_deg == Approx(90.0));
  CHECK(s.pitch_deg == Approx(0.0).margin(1e-9));
  CHECK(s.roll_deg == Approx(0.0).margin(1e-9));
  CHECK(s.angle_deg == Approx(90.0));
}

TEST_CASE("PoseDelta_DistanceAndRelativeAngle", "[rux_qt][database][pose]") {
  const std::array<double, 16> a{1, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1};
  const std::array<double, 16> b{0, -1, 0, 3, 1, 0, 0, 4,
                                 0, 0,  1, 0, 0, 0, 0, 1};
  const auto d = pose_delta(a, b);
  CHECK(d.distance_m == Approx(5.0));
  CHECK(d.angle_deg == Approx(90.0));
  CHECK(pose_delta(b, b).angle_deg == Approx(0.0).margin(1e-6));
}

TEST_CASE("FormatDecimal_UsesADanishComma", "[rux_qt][database][format]") {
  CHECK(format_decimal_da(0.4213, 2) == "0,42");
  CHECK(format_decimal_da(-1.5, 1) == "-1,5");
  CHECK(format_decimal_da(12.0, 0) == "12");
  CHECK(format_decimal_da(-0.0001, 2) == "0,00");
}

TEST_CASE("RowKeyTitle_JoinsEveryKeyColumn", "[rux_qt][database]") {
  CHECK(row_key_title({}).empty());
  CHECK(row_key_title({{"node_id", "1177"}}) == "node_id 1177");
  // A composite key (cloud_chunks' cloud_id+chunk_index): every column, not
  // just the first — otherwise every chunk of the same cloud would show the
  // same title (final review finding 10).
  CHECK(row_key_title({{"cloud_id", "42"}, {"chunk_index", "3"}}) ==
        "cloud_id 42 · chunk_index 3");
  CHECK(row_key_title({{"material_guid", "abc"}, {"key", "u-value"}}) ==
        "material_guid abc · key u-value");
}

// ----------------------------------------------------------- write errors --

TEST_CASE("WriteErrors_AreClassifiedWithDanishMessages",
          "[rux_qt][database][errors]") {
  CHECK(classify_write_error("SQL exec failed: database is locked") ==
        WriteErrorKind::locked);
  CHECK(classify_write_error("pose_graph_edges insert failed: database is "
                             "busy") == WriteErrorKind::locked);
  CHECK(classify_write_error("attempt to write a readonly database") ==
        WriteErrorKind::read_only);
  CHECK(classify_write_error("Database is opened read-only") ==
        WriteErrorKind::read_only);
  CHECK(classify_write_error("disk full") == WriteErrorKind::other);
  CHECK(write_error_da(WriteErrorKind::locked).find("låst") !=
        std::string::npos);
  CHECK(write_error_da(WriteErrorKind::read_only).find("skrivebeskyttet") !=
        std::string::npos);
  CHECK_FALSE(write_error_da(WriteErrorKind::other).empty());
}

TEST_CASE("PendingEdits_KeysMatchInEitherDirection",
          "[rux_qt][database][pending]") {
  // The store keeps from < to; a key given the other way round is the same
  // edge (the library's upsert flips the measurement).
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "loop_closure", "rejected"), edge(3, 4, "odometry")});
  CHECK(e.add(edge(2, 1)) == R::revived);
  CHECK(e.ops()[0].overwrite_rejected);
  CHECK(e.reject({4, 3, "odometry"}) == R::rejected);
  CHECK(e.count() == 2);
  CHECK(e.revert({1, 2, "loop_closure"}) == R::unstaged);
  CHECK(e.effective().size() == 2); // no duplicate reversed edge
  CHECK(e.degree(1) == 0);          // still rejected
  CHECK(e.degree(3) == 0);
}

TEST_CASE("PendingEdits_Add_ClearsTheResidual", "[rux_qt][database][pending]") {
  // The residual is written by optimize; a staged measurement has none yet.
  EdgeRecord unknown;
  CHECK(std::isnan(unknown.residual));
  CHECK(std::isnan(unknown.weight));
  PendingEdgeEdits e;
  REQUIRE(e.add(edge(1, 2, "loop_closure", "auto", 2.5, 4.0)) == R::added);
  CHECK(std::isnan(e.ops()[0].edge.residual));
  CHECK(e.ops()[0].edge.weight == 4.0);
}
