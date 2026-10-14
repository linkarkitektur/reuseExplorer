// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The Database workspace's save path, without Qt: the pending ops are written
// in one ProjectDB::Transaction. Another process holding the write lock must
// make the save fail with a message classify_write_error() calls "locked",
// leave the file untouched, and leave the edits pending; a read-only open
// must classify as read-only.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <rux_qt/database_logic.hpp>
#include <rux_qt/edge_store.hpp>
#include <rux_qt/pair_alignment.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <array>
#include <stdexcept>
#include <string>

using reusex::ProjectDB;
using namespace rux::qt;

namespace {

EdgeRecord record(int from, int to, double weight) {
  EdgeRecord r;
  r.key = {from, to, "loop_closure"};
  r.weight = weight;
  return r;
}

} // namespace

TEST_CASE("EdgeSave_LockedProject_FailsAsLockedAndKeepsEditsPending",
          "[rux_qt][database][save]") {
  reusex::test_support::TempPath tmp("edge_save_locked");
  {
    ProjectDB init(tmp.path);
  }
  ProjectDB db(tmp.path);

  PendingEdgeEdits edits;
  edits.set_base({});
  REQUIRE(edits.add(record(1, 2, 4.0)) == PendingEdgeEdits::Result::added);

  sqlite3 *other = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &other) == SQLITE_OK);
  REQUIRE(sqlite3_exec(other, "BEGIN IMMEDIATE;", nullptr, nullptr, nullptr) ==
          SQLITE_OK);

  std::string what;
  try {
    apply_edge_ops(db, edits.ops()); // waits out the 5 s busy timeout
  } catch (const std::exception &e) {
    what = e.what();
  }
  sqlite3_exec(other, "ROLLBACK;", nullptr, nullptr, nullptr);
  sqlite3_close(other);

  REQUIRE_FALSE(what.empty());
  CHECK(classify_write_error(what) == WriteErrorKind::locked);
  CHECK(edits.count() == 1);                 // nothing was dropped
  CHECK(db.list_pose_graph_edges().empty()); // nothing was written

  apply_edge_ops(db, edits.ops()); // with the lock gone it goes through
  edits.commit_saved(edits.ops());
  CHECK(edits.empty());
  CHECK(db.list_pose_graph_edges().size() == 1);
}

TEST_CASE("EdgeSave_ReadOnlyProject_FailsAsReadOnly",
          "[rux_qt][database][save]") {
  reusex::test_support::TempPath tmp("edge_save_ro");
  {
    ProjectDB init(tmp.path);
  }
  ProjectDB db(tmp.path, /*readOnly=*/true);
  std::string what;
  try {
    apply_edge_ops(
        db,
        {{PendingEdgeEdits::Op::Kind::upsert, record(1, 2, 1.0), false, {}}});
  } catch (const std::exception &e) {
    what = e.what();
  }
  CHECK(classify_write_error(what) == WriteErrorKind::read_only);
}

TEST_CASE("EdgeSave_StatusAndMeasurement_RoundTrip",
          "[rux_qt][database][save]") {
  reusex::test_support::TempPath tmp("edge_save_roundtrip");
  ProjectDB db(tmp.path);
  PendingEdgeEdits edits;
  edits.set_base({});
  EdgeRecord m = record(1, 2, 400.0);
  m.transform =
      std::array<double, 16>{1, 0, 0, 0.25, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  m.source = "icp";
  REQUIRE(edits.add(m) == PendingEdgeEdits::Result::added);
  apply_edge_ops(db, edits.ops());
  edits.commit_saved(edits.ops());

  std::vector<EdgeRecord> back;
  for (const auto &row : db.list_pose_graph_edges())
    back.push_back(edge_record_from_row(row));
  REQUIRE(back.size() == 1);
  CHECK(back[0].status == "confirmed");
  CHECK(back[0].source == "icp");
  REQUIRE(back[0].transform.has_value());
  CHECK((*back[0].transform)[3] == 0.25);

  edits.set_base(back);
  REQUIRE(edits.reject({1, 2, "loop_closure"}) ==
          PendingEdgeEdits::Result::rejected);
  apply_edge_ops(db, edits.ops());
  CHECK(db.list_pose_graph_edges()[0].status == "rejected");
}

TEST_CASE("EdgeSave_UpsertOverTombstone_RefusesWithoutRevive",
          "[rux_qt][database][save]") {
  reusex::test_support::TempPath tmp("edge_save_tombstone");
  ProjectDB db(tmp.path);
  ProjectDB::PoseGraphEdge row;
  row.from_node_id = 1;
  row.to_node_id = 2;
  row.edge_type = "loop_closure";
  row.status = "rejected";
  row.source = "orb";
  REQUIRE(db.upsert_pose_graph_edge(row));
  // A stale base that never saw the rejection: the save must fail, unwritten.
  PendingEdgeEdits stale;
  stale.set_base({});
  REQUIRE(stale.add(record(1, 2, 9.0)) == PendingEdgeEdits::Result::added);
  CHECK_THROWS_AS(apply_edge_ops(db, stale.ops()), std::runtime_error);
  CHECK(db.list_pose_graph_edges()[0].status == "rejected");
  // Loaded properly, the same add revives it.
  PendingEdgeEdits fresh;
  fresh.set_base({edge_record_from_row(db.list_pose_graph_edges()[0])});
  REQUIRE(fresh.add(record(1, 2, 9.0)) == PendingEdgeEdits::Result::revived);
  apply_edge_ops(db, fresh.ops());
  CHECK(db.list_pose_graph_edges()[0].status == "confirmed");
}

TEST_CASE("EdgeSave_ReversedKey_UpdatesTheStoredEdge",
          "[rux_qt][database][save]") {
  // A key staged the other way round lands on the canonical row; no
  // duplicate reversed edge, and a manual edge is written with source
  // "manual".
  reusex::test_support::TempPath tmp("edge_save_reversed");
  ProjectDB db(tmp.path);
  ProjectDB::PoseGraphEdge row;
  row.from_node_id = 1;
  row.to_node_id = 2;
  row.edge_type = "loop_closure";
  row.source = "orb";
  REQUIRE(db.upsert_pose_graph_edge(row));
  PendingEdgeEdits edits;
  edits.set_base({edge_record_from_row(db.list_pose_graph_edges()[0])});
  EdgeRecord m = record(2, 1, 25.0);
  m.transform =
      std::array<double, 16>{1, 0, 0, 0.5, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  REQUIRE(edits.add(m) == PendingEdgeEdits::Result::replaced);
  apply_edge_ops(db, edits.ops());
  const auto rows = db.list_pose_graph_edges();
  REQUIRE(rows.size() == 1);
  CHECK(rows[0].from_node_id == 1);
  CHECK(rows[0].status == "confirmed");
  CHECK(rows[0].source == "manual");
  REQUIRE(rows[0].transform.has_value());
  CHECK((*rows[0].transform)[3] == -0.5); // inverted by the store
}

namespace {

void store_edge(ProjectDB &db, int from, int to, const char *status) {
  ProjectDB::PoseGraphEdge row;
  row.from_node_id = from;
  row.to_node_id = to;
  row.edge_type = "loop_closure";
  row.source = "orb";
  row.status = status;
  REQUIRE(db.upsert_pose_graph_edge(row));
}

std::vector<EdgeRecord> load(const ProjectDB &db) {
  std::vector<EdgeRecord> out;
  for (const auto &row : db.list_pose_graph_edges())
    out.push_back(edge_record_from_row(row));
  return out;
}

} // namespace

TEST_CASE("EdgeSave_StatusOnAVanishedRow_FailsAndKeepsTheEditPending",
          "[rux_qt][database][save]") {
  reusex::test_support::TempPath tmp("edge_save_vanished");
  ProjectDB db(tmp.path);
  store_edge(db, 1, 2, "auto");
  store_edge(db, 3, 4, "auto");
  PendingEdgeEdits edits;
  edits.set_base(load(db));
  REQUIRE(edits.confirm({3, 4, "loop_closure"}) ==
          PendingEdgeEdits::Result::confirmed);
  REQUIRE(edits.reject({1, 2, "loop_closure"}) ==
          PendingEdgeEdits::Result::rejected);
  // Another writer drops (1, 2) after the base was loaded.
  REQUIRE(db.delete_pose_graph_edges_from_source("orb") == 2);
  store_edge(db, 3, 4, "auto");
  CHECK_THROWS_AS(apply_edge_ops(db, edits.ops()), std::runtime_error);
  CHECK(edits.count() == 2); // the caller commits nothing on failure
  // Rolled back: (3, 4) was not confirmed either.
  REQUIRE(db.list_pose_graph_edges().size() == 1);
  CHECK(db.list_pose_graph_edges()[0].status == "auto");
}

TEST_CASE("EdgeSave_ConfirmAndUnreject_RoundTrip", "[rux_qt][database][save]") {
  reusex::test_support::TempPath tmp("edge_save_status");
  ProjectDB db(tmp.path);
  store_edge(db, 1, 2, "auto");
  store_edge(db, 3, 4, "rejected");
  PendingEdgeEdits edits;
  edits.set_base(load(db));
  REQUIRE(edits.confirm({1, 2, "loop_closure"}) ==
          PendingEdgeEdits::Result::confirmed);
  REQUIRE(edits.unreject({3, 4, "loop_closure"}) ==
          PendingEdgeEdits::Result::unrejected);
  const auto ops = edits.ops();
  apply_edge_ops(db, ops);
  edits.commit_saved(ops);
  CHECK(edits.empty());
  const auto back = load(db);
  REQUIRE(back.size() == 2);
  CHECK(back[0].status == "confirmed");
  CHECK(back[1].status == "auto"); // the tombstone is lifted
  CHECK(back[1].source == "orb");
  for (std::size_t i = 0; i < back.size(); ++i)
    CHECK(same_record(back[i], edits.base()[i]));
}

TEST_CASE("EdgeSave_AddWithoutMeasurement_OverStored_MatchesTheStore",
          "[rux_qt][database][save]") {
  reusex::test_support::TempPath tmp("edge_save_confirm_add");
  ProjectDB db(tmp.path);
  ProjectDB::PoseGraphEdge row;
  row.from_node_id = 1;
  row.to_node_id = 2;
  row.edge_type = "loop_closure";
  row.source = "orb";
  row.transform =
      std::array<double, 16>{1, 0, 0, 0.3, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  REQUIRE(db.upsert_pose_graph_edge(row));
  PendingEdgeEdits edits;
  edits.set_base(load(db));
  REQUIRE(edits.add(record(2, 1, 400.0)) ==
          PendingEdgeEdits::Result::confirmed);
  const auto ops = edits.ops();
  apply_edge_ops(db, ops);
  edits.commit_saved(ops);
  const auto back = load(db);
  REQUIRE(back.size() == 1);
  CHECK(back[0].source == "orb");
  CHECK(back[0].status == "confirmed");
  CHECK(same_record(back[0], edits.base()[0]));
}

namespace {

/// Stage @p e on an empty base, save it to a fresh project and read the row
/// back; @p staged gets the record the editor shows after the save.
EdgeRecord save_and_load(const EdgeRecord &e, const char *name,
                         EdgeRecord &staged) {
  reusex::test_support::TempPath tmp(name);
  ProjectDB db(tmp.path);
  PendingEdgeEdits edits;
  edits.set_base({});
  REQUIRE(edits.add(e) == PendingEdgeEdits::Result::added);
  const auto ops = edits.ops();
  apply_edge_ops(db, ops);
  edits.commit_saved(ops);
  REQUIRE(edits.base().size() == 1);
  staged = edits.base()[0];
  const auto back = load(db);
  REQUIRE(back.size() == 1);
  return back[0];
}

void check_close(const std::array<double, 16> &a,
                 const std::array<double, 16> &b) {
  for (std::size_t i = 0; i < 16; ++i) {
    INFO(i);
    CHECK(a[i] == Catch::Approx(b[i]).margin(1e-9));
  }
}

} // namespace

TEST_CASE("EdgeSave_IcpBToA_AndManualAToB_StoreTheSameCanonicalRow",
          "[rux_qt][database][save]") {
  // One ΔT, measured two ways: ICP's B -> A (edge_transform, information)
  // and the manual A -> B (T_A^-1 · ΔT · T_B). Whichever of A, B has the
  // lower id, both land on the same canonical row, and what the editor
  // holds after the save is what the file holds.
  const Mat4 pose_lo = offset_matrix({1.0, 0.5, 0.0, 20, 0, 0}, {0, 0, 0});
  const Mat4 pose_hi = offset_matrix({3.0, -1.0, 0.1, -35, 2, 1}, {0, 0, 0});
  const Mat4 delta = offset_matrix({0.12, -0.05, 0.01, 4, 0, 0}, {3, -1, 0});
  std::array<double, 36> info{};
  for (std::size_t i = 0; i < 3; ++i)
    info[i * 7] = 400.0;
  for (std::size_t i = 3; i < 6; ++i)
    info[i * 7] = 2500.0;

  for (const bool a_is_low : {true, false}) {
    INFO("a_is_low " << a_is_low);
    const int a = a_is_low ? 3 : 7, b = a_is_low ? 7 : 3;
    const Mat4 &pa = a_is_low ? pose_lo : pose_hi;
    const Mat4 &pb = a_is_low ? pose_hi : pose_lo;
    IcpEdgeMeasurement icp;
    icp.edge_transform = rigid_inverse(edge_measurement(pa, pb, delta));
    icp.information = info;
    const EdgeRecord by_icp =
        pair_edge_to_add(a, b, "loop_closure", pa, pb, delta, icp);
    const EdgeRecord by_hand =
        pair_edge_to_add(a, b, "loop_closure", pa, pb, delta, std::nullopt);

    EdgeRecord staged_icp, staged_hand;
    const EdgeRecord row_icp = save_and_load(by_icp, "edge_eq_icp", staged_icp);
    const EdgeRecord row_hand =
        save_and_load(by_hand, "edge_eq_hand", staged_hand);

    const std::array<const EdgeRecord *, 4> all{&row_icp, &row_hand,
                                                &staged_icp, &staged_hand};
    for (const EdgeRecord *r : all) {
      CHECK(r->key == EdgeKey{3, 7, "loop_closure"});
      CHECK(r->status == "confirmed");
      REQUIRE(r->transform.has_value());
    }
    check_close(*row_icp.transform, *row_hand.transform);
    check_close(*staged_icp.transform, *row_icp.transform);
    check_close(*staged_hand.transform, *row_hand.transform);
    // Canonical lo -> hi: T_lo^-1 · T_hi with B moved by ΔT.
    check_close(*row_hand.transform,
                a_is_low ? edge_measurement(pa, pb, delta)
                         : rigid_inverse(edge_measurement(pa, pb, delta)));
    CHECK(row_icp.source == "icp");
    CHECK(row_hand.source == "manual");
    CHECK_FALSE(row_hand.information.has_value());

    // The information is the library's reversal of ICP's B -> A one.
    REQUIRE(row_icp.information.has_value());
    REQUIRE(staged_icp.information.has_value());
    namespace pg = reusex::core::pose_graph;
    std::array<double, 36> expected = info;
    if (b > a) // ICP's B -> A was the reversed direction
      expected = pg::row_major_from_information(pg::reverse_edge_information(
          pg::information_from_row_major(info),
          pg::matrix_from_row_major(icp.edge_transform)));
    for (std::size_t i = 0; i < 36; ++i) {
      INFO(i);
      CHECK((*row_icp.information)[i] ==
            Catch::Approx(expected[i]).margin(1e-6));
      CHECK((*staged_icp.information)[i] ==
            Catch::Approx((*row_icp.information)[i]).margin(1e-6));
    }
    // The store derives the weight: the mean translational precision.
    const auto &m = *row_icp.information;
    CHECK(row_icp.weight == Catch::Approx((m[21] + m[28] + m[35]) / 3.0));
  }
}
