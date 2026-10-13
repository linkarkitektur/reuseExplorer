// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The project's display settings (spec D7): today the view yaw, read from
// project_settings when a project opens and written back half a second
// after the last change — off the GUI thread, on its own connection, under
// the pipeline writer lease (like EdgeEditor::save). Display only: nothing
// in the model rotates. On a read-only project the yaw applies but is not
// written (persists() is false).

#include <rux_qt/EdgeEditor.hpp>

#include <QObject>

#include <utility>

class QTimer;

namespace rux::qt {

class ProjectSession;

class ViewSettings : public QObject {
  Q_OBJECT
    public:
  explicit ViewSettings(ProjectSession &session, QObject *parent = nullptr);
  double yaw_deg() const { return yaw_; }
  /// Apply now (yaw_changed), write after a pause.
  void set_yaw_deg(double deg);
  bool persists() const;
  void set_runner_source(EdgeEditor::RunnerSource source) {
    runner_source_ = std::move(source);
  }

    signals:
  void yaw_changed(double deg);

    private:
  void reload();
  void write_now();
  ProjectSession &session_;
  EdgeEditor::RunnerSource runner_source_;
  QTimer *debounce_ = nullptr;
  double yaw_ = 0.0;
};

} // namespace rux::qt
