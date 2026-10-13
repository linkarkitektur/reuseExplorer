// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The Database workspace's save path, without Qt: the pending ops are written
// in one ProjectDB::Transaction. Another process holding the write lock must
// make the save fail with a message classify_write_error() calls "locked",
// leave the file untouched, and leave the edits pending; a read-only open
// must classify as read-only.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>
#include <rux_qt/database_logic.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <stdexcept>
#include <string>

using reusex::ProjectDB;
using namespace rux::qt;

namespace {

/// What EdgeEditor's save thread does with a snapshot of ops().
void apply_ops(ProjectDB &db, const std::vector<PendingEdgeEdits::Op> &ops) {
  ProjectDB::Transaction tx(db);
  for (const auto &op : ops) {
    if (op.kind == PendingEdgeEdits::Op::Kind::remove) {
      db.set_pose_graph_edge_status(op.edge.key.from, op.edge.key.to,
                                    op.edge.key.type,
                                    reusex::core::pose_graph::kStatusRejected);
    } else {
      ProjectDB::PoseGraphEdge row;
      row.from_node_id = op.edge.key.from;
      row.to_node_id = op.edge.key.to;
      row.edge_type = op.edge.key.type;
      row.residual = op.edge.residual;
      row.weight = op.edge.weight;
      row.source = std::string(reusex::core::pose_graph::kSourceManual);
      row.status = std::string(reusex::core::pose_graph::kStatusConfirmed);
      db.upsert_pose_graph_edge(row, /*overwrite_rejected=*/true);
    }
  }
  tx.commit();
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
  REQUIRE(edits.add({{1, 2, "loop_closure"}, 0.0, 4.0}) ==
          PendingEdgeEdits::Result::added);

  sqlite3 *other = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &other) == SQLITE_OK);
  REQUIRE(sqlite3_exec(other, "BEGIN IMMEDIATE;", nullptr, nullptr, nullptr) ==
          SQLITE_OK);

  std::string what;
  try {
    apply_ops(db, edits.ops()); // waits out the 5 s busy timeout
  } catch (const std::exception &e) {
    what = e.what();
  }
  sqlite3_exec(other, "ROLLBACK;", nullptr, nullptr, nullptr);
  sqlite3_close(other);

  REQUIRE_FALSE(what.empty());
  CHECK(classify_write_error(what) == WriteErrorKind::locked);
  CHECK(edits.count() == 1);                 // nothing was dropped
  CHECK(db.list_pose_graph_edges().empty()); // nothing was written

  apply_ops(db, edits.ops()); // with the lock gone it goes through
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
    apply_ops(db, {{PendingEdgeEdits::Op::Kind::add,
                    {{1, 2, "loop_closure"}, 0.0, 1.0}}});
  } catch (const std::exception &e) {
    what = e.what();
  }
  CHECK(classify_write_error(what) == WriteErrorKind::read_only);
}
