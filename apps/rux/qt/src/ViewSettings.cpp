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

#include <thread>

namespace rux::qt {

ViewSettings::ViewSettings(ProjectSession &session, QObject *parent)
    : QObject(parent), session_(session) {
  debounce_ = new QTimer(this);
  debounce_->setSingleShot(true);
  debounce_->setInterval(500);
  connect(debounce_, &QTimer::timeout, this, &ViewSettings::write_now);
  connect(&session_, &ProjectSession::opened, this, &ViewSettings::reload);
  connect(&session_, &ProjectSession::closed, this, &ViewSettings::reload);
  reload(); // a project already open
}

bool ViewSettings::persists() const {
  return session_.is_open() && !session_.is_read_only();
}

void ViewSettings::reload() {
  debounce_->stop();
  double yaw = 0.0;
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
  emit yaw_changed(yaw_);
  if (persists())
    debounce_->start();
}

void ViewSettings::write_now() {
  if (!persists())
    return;
  const std::string path = session_.path().toStdString();
  const std::string text = format_yaw_setting(yaw_);
  std::weak_ptr<reusex::pipeline::JobRunner> runner;
  if (runner_source_)
    runner = runner_source_();
  auto work = std::make_shared<BackgroundWork>();
  QPointer<ViewSettings> guard(this);
  std::thread([path, text, runner, guard, work]() mutable {
    bool busy = false;
    std::shared_ptr<reusex::pipeline::JobRunner> holder = runner.lock();
    reusex::pipeline::WriterLease lease;
    if (holder) {
      lease = holder->try_acquire_writer(EdgeEditor::kLeaseTimeout);
      busy = !lease.owns_lock();
    }
    if (!busy)
      try {
        reusex::ProjectDB db(path, /*readOnly=*/false);
        db.set_setting(kViewYawSetting, text);
      } catch (const std::exception &e) {
        reusex::warn("view yaw not saved: {}", e.what());
      }
    if (busy) // a stage holds the project: try again shortly
      QMetaObject::invokeMethod(
          qApp,
          [guard] {
            if (guard)
              guard->debounce_->start();
          },
          Qt::QueuedConnection);
    if (lease.owns_lock())
      lease.unlock();
    holder.reset();
    work.reset();
  }).detach();
}

} // namespace rux::qt
