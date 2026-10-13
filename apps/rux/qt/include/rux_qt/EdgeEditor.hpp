// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pose-graph edits of the Database workspace: staged in memory
// (PendingEdgeEdits, rux_qt_core) until "Gem ændringer", then written in ONE
// ProjectDB transaction — status changes first, then measurements
// (apply_edge_ops, edge_store.hpp). A "delete" is a rejection: the edge stays
// in the file as a tombstone that discover and optimize respect.
//
// The save runs OFF the GUI thread: a locked project waits out sqlite's 5 s
// busy timeout, which must not freeze the window. save() snapshots ops() and
// a detached, counted thread (BackgroundWork) applies the snapshot on its OWN
// read-write connection (the session's is the GUI thread's alone). Staging
// stays possible meanwhile; on success exactly the snapshot leaves the
// pending set (PendingEdgeEdits::commit_saved), on failure every edit stays
// pending and last_error() says why in Danish (read-only, locked, other).

#include <rux_qt/database_logic.hpp>

#include <reusex/pipeline/JobRunner.hpp>

#include <QObject>
#include <QString>

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

class QWidget;

namespace rux::qt {

class ProjectSession;

class EdgeEditor : public QObject {
  Q_OBJECT
    public:
  explicit EdgeEditor(ProjectSession &session, QObject *parent = nullptr);

  const PendingEdgeEdits &edits() const { return edits_; }
  int pending() const { return edits_.count(); }

  /// A project is open with write access. Edits can be staged anyway; only
  /// saving needs it.
  bool can_save() const;
  /// Why the project cannot be written (Danish), or empty.
  QString read_only_reason() const;

  PendingEdgeEdits::Result add(const EdgeRecord &edge);
  PendingEdgeEdits::Result confirm(const EdgeKey &key);
  PendingEdgeEdits::Result reject(const EdgeKey &key);
  PendingEdgeEdits::Result unreject(const EdgeKey &key);
  PendingEdgeEdits::Result revert(const EdgeKey &key);
  void discard();

  /// Start writing every pending edit (see the header). False when it cannot
  /// start: nothing open, read-only (last_error() says so), or a save is
  /// already running. save_finished() reports the outcome.
  bool save();
  bool is_saving() const { return saving_; }
  /// Run save() and wait for its outcome with a small modal "Gemmer …"
  /// (quit, close, reload). True when nothing is pending afterwards.
  bool save_and_wait(QWidget *parent);
  QString last_error() const { return error_; }
  QString last_error_detail() const { return error_detail_; }

  /// Re-read the stored edges (after an open or a reload). Drops pending
  /// edits — callers ask first.
  void reload();

  /// The in-process pipeline runner whose writer lease a save must hold
  /// (JobRunner::try_acquire_writer): a save never writes while a stage
  /// does. Asked on the GUI thread at save(); the lease is taken on the
  /// save's own thread.
  using RunnerSource =
      std::function<std::weak_ptr<reusex::pipeline::JobRunner>()>;
  void set_runner_source(RunnerSource source) { runner_source_ = source; }
  /// A pipeline stage is running (Danish stage name), or empty: saving is
  /// held back until it ends.
  void set_pipeline_busy(const QString &stage);
  QString pipeline_busy() const { return pipeline_busy_; }
  /// How long a save waits for a running stage to release the project.
  static constexpr std::chrono::milliseconds kLeaseTimeout{1500};

    signals:
  /// The stored or pending edges changed.
  void changed();
  /// A save wrote @p count edits.
  void saved(int count);
  /// A save ended (after saved() on success).
  void save_finished(bool ok);

    private:
  void save_done(unsigned id, bool ok, bool busy, const QString &what,
                 const std::vector<PendingEdgeEdits::Op> &snapshot);
  QString busy_message() const;
  RunnerSource runner_source_;
  QString pipeline_busy_;
  ProjectSession &session_;
  bool saving_ = false;
  unsigned save_id_ = 0;
  PendingEdgeEdits edits_;
  QString error_;
  QString error_detail_;
};

} // namespace rux::qt
