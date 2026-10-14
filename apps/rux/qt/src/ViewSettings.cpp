// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/ViewSettings.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/graph_view.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <QApplication>
#include <QPointer>
#include <QTimer>

#include <mutex>
#include <optional>
#include <thread>

namespace rux::qt {

/// The single write queue: shared with the worker, so it outlives the
/// ViewSettings when the window closes mid-write.
struct ViewSettings::Writer {
  struct Job {
    std::string path, text;
    unsigned seq = 0;
  };
  std::mutex m;
  bool running = false;
  std::optional<Job> queued; ///< only the latest is kept
};

ViewSettings::ViewSettings(ProjectSession &session, QObject *parent)
    : QObject(parent), session_(session), writer_(std::make_shared<Writer>()) {
  debounce_ = new QTimer(this);
  debounce_->setSingleShot(true);
  debounce_->setInterval(500);
  connect(debounce_, &QTimer::timeout, this, &ViewSettings::flush);
  // The pending yaw carries its own path: write it before the project goes.
  connect(&session_, &ProjectSession::opened, this, [this] {
    flush();
    reload();
  });
  connect(&session_, &ProjectSession::closed, this, [this] {
    flush();
    reload();
  });
  // Before run_app's wait for detached work, unlike the destructor.
  if (QCoreApplication *app = QCoreApplication::instance())
    connect(app, &QCoreApplication::aboutToQuit, this, &ViewSettings::flush);
  reload(); // a project already open
}

ViewSettings::~ViewSettings() { flush(); }

bool ViewSettings::persists() const {
  return session_.is_open() && !session_.is_read_only();
}

void ViewSettings::reload() {
  double yaw = 0.0;
  // A yaw set on this project that the file does not hold yet wins: the
  // worker (or its retry behind the writer lease) writes it shortly.
  if (const auto unsaved =
          session_.is_open()
              ? unsaved_.for_project(session_.path().toStdString())
              : std::nullopt) {
    yaw_ = *unsaved;
    emit yaw_changed(yaw_);
    return;
  }
  if (const reusex::ProjectDB *db = session_.db()) {
    try {
      if (const auto text = db->setting(kViewYawSetting))
        yaw = parse_yaw_setting(*text).value_or(0.0);
    } catch (const std::exception &e) {
      reusex::warn("view yaw not read: {}", e.what());
    }
  }
  yaw_ = yaw;
  emit yaw_changed(yaw_);
}

void ViewSettings::set_yaw_deg(double deg) {
  const double y = normalize_yaw_deg(deg);
  if (format_yaw_setting(y) == format_yaw_setting(yaw_))
    return;
  yaw_ = y;
  ++seq_;
  emit yaw_changed(yaw_);
  if (persists()) {
    pending_path_ = session_.path().toStdString();
    pending_text_ = format_yaw_setting(yaw_);
    unsaved_.changed(pending_path_, seq_, yaw_);
    debounce_->start();
  }
}

void ViewSettings::flush() {
  debounce_->stop();
  if (pending_path_.empty())
    return;
  write(std::move(pending_path_), std::move(pending_text_), seq_);
  pending_path_.clear();
  pending_text_.clear();
}

void ViewSettings::write(std::string path, std::string text, unsigned seq) {
  {
    const std::lock_guard<std::mutex> l(writer_->m);
    writer_->queued = Writer::Job{std::move(path), std::move(text), seq};
    if (writer_->running)
      return; // the running worker takes it next
    writer_->running = true;
  }
  std::weak_ptr<reusex::pipeline::JobRunner> runner;
  if (runner_source_)
    runner = runner_source_();
  auto work = std::make_shared<BackgroundWork>();
  QPointer<ViewSettings> guard(this);
  std::thread([writer = writer_, runner, guard, work]() mutable {
    for (;;) {
      Writer::Job job;
      {
        const std::lock_guard<std::mutex> l(writer->m);
        if (!writer->queued) {
          writer->running = false;
          break;
        }
        job = std::move(*writer->queued);
        writer->queued.reset();
      }
      bool busy = false;
      std::shared_ptr<reusex::pipeline::JobRunner> holder = runner.lock();
      reusex::pipeline::WriterLease lease;
      if (holder) {
        lease = holder->try_acquire_writer(EdgeEditor::kLeaseTimeout);
        busy = !lease.owns_lock();
      }
      if (!busy) {
        try {
          reusex::ProjectDB db(job.path, /*readOnly=*/false);
          db.set_setting(kViewYawSetting, job.text);
        } catch (const std::exception &e) {
          reusex::warn("view yaw not saved: {}", e.what());
        }
        // Landed (or given up): a re-open reads the file again.
        QMetaObject::invokeMethod(
            qApp,
            [guard, seq = job.seq] {
              if (guard)
                guard->unsaved_.written(seq);
            },
            Qt::QueuedConnection);
      }
      if (lease.owns_lock())
        lease.unlock();
      holder.reset();
      // A stage holds the project: try again shortly — unless the yaw has
      // changed since, when the newer value is (or will be) written instead
      // and retrying this one could land it after that.
      if (busy)
        QMetaObject::invokeMethod(
            qApp,
            [guard, job] {
              if (!guard || guard->seq_ != job.seq)
                return;
              guard->pending_path_ = job.path;
              guard->pending_text_ = job.text;
              guard->debounce_->start();
            },
            Qt::QueuedConnection);
    }
    work.reset();
  }).detach();
}

} // namespace rux::qt
