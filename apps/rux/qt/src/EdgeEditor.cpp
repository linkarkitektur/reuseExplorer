// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/edge_store.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <rux_qt/background.hpp>

#include <QApplication>
#include <QDialog>
#include <QLabel>
#include <QPointer>
#include <QVBoxLayout>

#include <memory>
#include <thread>

namespace rux::qt {

EdgeEditor::EdgeEditor(ProjectSession &session, QObject *parent)
    : QObject(parent), session_(session) {
  connect(&session_, &ProjectSession::opened, this, &EdgeEditor::reload);
  connect(&session_, &ProjectSession::closed, this, &EdgeEditor::reload);
}

bool EdgeEditor::can_save() const {
  return session_.is_open() && !session_.is_read_only();
}

QString EdgeEditor::read_only_reason() const {
  if (!session_.is_open() || !session_.is_read_only())
    return {};
  return QString::fromStdString(write_error_da(WriteErrorKind::read_only));
}

PendingEdgeEdits::Result EdgeEditor::add(const EdgeRecord &edge) {
  const auto r = edits_.add(edge);
  error_.clear();
  emit changed();
  return r;
}

PendingEdgeEdits::Result EdgeEditor::confirm(const EdgeKey &key) {
  const auto r = edits_.confirm(key);
  error_.clear();
  emit changed();
  return r;
}

PendingEdgeEdits::Result EdgeEditor::reject(const EdgeKey &key) {
  const auto r = edits_.reject(key);
  error_.clear();
  emit changed();
  return r;
}

PendingEdgeEdits::Result EdgeEditor::unreject(const EdgeKey &key) {
  const auto r = edits_.unreject(key);
  error_.clear();
  emit changed();
  return r;
}

PendingEdgeEdits::Result EdgeEditor::unconfirm(const EdgeKey &key) {
  const auto r = edits_.unconfirm(key);
  error_.clear();
  emit changed();
  return r;
}

PendingEdgeEdits::Result EdgeEditor::revert(const EdgeKey &key) {
  const auto r = edits_.revert(key);
  error_.clear();
  emit changed();
  return r;
}

void EdgeEditor::discard() {
  edits_.discard();
  error_.clear();
  error_detail_.clear();
  emit changed();
}

bool EdgeEditor::save() {
  if (!session_.is_open() || edits_.empty() || saving_)
    return false;
  error_.clear();
  error_detail_.clear();
  if (session_.is_read_only()) {
    error_ = QString::fromStdString(write_error_da(WriteErrorKind::read_only));
    error_detail_ = session_.read_only_reason();
    emit changed();
    emit save_finished(false);
    return false;
  }

  if (!pipeline_busy_.isEmpty()) {
    error_ = busy_message();
    emit changed();
    emit save_finished(false);
    return false;
  }

  const auto snapshot = edits_.ops();
  const unsigned id = ++save_id_;
  // The pipeline's writer lease, taken on the save's thread (a timed_mutex
  // must be unlocked by the thread that locked it).
  std::weak_ptr<reusex::pipeline::JobRunner> runner;
  if (runner_source_)
    runner = runner_source_();
  saving_ = true;
  emit changed();

  const std::string path = session_.path().toStdString();
  auto work = std::make_shared<BackgroundWork>();
  QPointer<EdgeEditor> guard(this);
  std::thread([path, snapshot, id, guard, work, runner]() mutable {
    bool ok = false;
    bool busy = false;
    QString what;
    // Holds the runner (and its lease) until the write is done.
    std::shared_ptr<reusex::pipeline::JobRunner> holder = runner.lock();
    reusex::pipeline::WriterLease lease;
    if (holder) {
      lease = holder->try_acquire_writer(kLeaseTimeout);
      busy = !lease.owns_lock();
    }
    if (!busy)
      try {
        reusex::ProjectDB db(path, /*readOnly=*/false);
        apply_edge_ops(db, snapshot);
        ok = true;
      } catch (const std::exception &ex) {
        what = QString::fromUtf8(ex.what());
      }
    QMetaObject::invokeMethod(
        qApp,
        [guard, id, ok, busy, what, snapshot] {
          if (guard)
            guard->save_done(id, ok, busy, what, snapshot);
        },
        Qt::QueuedConnection);
    if (lease.owns_lock())
      lease.unlock();
    holder.reset(); // may be the last reference: joins an idle runner here
    work.reset();   // last: counted until the thread has let go of everything
  }).detach();
  return true;
}

void EdgeEditor::set_pipeline_busy(const QString &stage) {
  pipeline_busy_ = stage;
  if (stage.isEmpty() && error_ == busy_message())
    error_.clear();
  emit changed();
}

QString EdgeEditor::busy_message() const {
  return QString("Et pipeline-trin%1 skriver til projektet — gem, når det er "
                 "færdigt.")
      .arg(pipeline_busy_.isEmpty() ? QString()
                                    : QString(" (%1)").arg(pipeline_busy_));
}

void EdgeEditor::save_done(unsigned id, bool ok, bool busy, const QString &what,
                           const std::vector<PendingEdgeEdits::Op> &snapshot) {
  if (id != save_id_ || !saving_)
    return; // a reload or a new project dropped this save's state
  saving_ = false;
  if (busy) {
    error_ = busy_message();
    error_detail_.clear();
    emit changed();
    emit save_finished(false);
    return;
  }
  if (!ok) {
    error_ = QString::fromStdString(
        write_error_da(classify_write_error(what.toStdString())));
    error_detail_ = what;
    emit changed();
    emit save_finished(false);
    return;
  }
  edits_.commit_saved(snapshot);
  emit changed();
  emit saved(static_cast<int>(snapshot.size()));
  emit save_finished(true);
}

bool EdgeEditor::save_and_wait(QWidget *parent) {
  if (!saving_ && !save())
    return edits_.empty();
  QDialog wait(parent);
  wait.setObjectName("savingDialog");
  wait.setWindowTitle("Gemmer");
  wait.setModal(true);
  auto *l = new QVBoxLayout(&wait);
  auto *text = new QLabel("Gemmer ændringer i posegrafen …");
  text->setObjectName("emptyText");
  l->addWidget(text);
  bool ok = false;
  connect(this, &EdgeEditor::save_finished, &wait, [&](bool result) {
    ok = result;
    wait.accept();
  });
  // A save can finish before exec() starts its loop: the queued delivery
  // only runs inside it, so this cannot miss the signal.
  wait.exec();
  return ok && edits_.empty();
}

void EdgeEditor::reload() {
  saving_ = false; // a running save's result is for the old base: dropped
  ++save_id_;
  error_.clear();
  error_detail_.clear();
  std::vector<EdgeRecord> base;
  if (reusex::ProjectDB *db = session_.db()) {
    try {
      // Every status: rejected edges are part of the base (tombstones the
      // views hide or show, and un-reject).
      for (const auto &e : db->list_pose_graph_edges())
        base.push_back(edge_record_from_row(e));
    } catch (const std::exception &ex) {
      error_ = "Posegrafens kanter kunne ikke læses.";
      error_detail_ = QString::fromUtf8(ex.what());
    }
  }
  edits_.set_base(std::move(base));
  emit changed();
}

} // namespace rux::qt
