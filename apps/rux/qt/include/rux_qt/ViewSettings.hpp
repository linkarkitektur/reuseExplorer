// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The project's display settings (spec D7): today the view yaw, read from
// project_settings when a project opens and written back half a second
// after the last change — off the GUI thread, on its own connection, under
// the pipeline writer lease (like EdgeEditor::save). Display only: nothing
// in the model rotates. On a read-only project the yaw applies but is not
// written (persists() is false).
//
// A pending yaw belongs to the project it was set on: closing that project,
// opening another or quitting writes it at once rather than dropping it.
// Writes go through one worker at a time, so they land in the order they
// were asked for; one asked while another runs is queued (only the latest
// is kept) and written when it ends.

#include <rux_qt/EdgeEditor.hpp>

#include <QObject>

#include <memory>
#include <string>
#include <utility>

class QTimer;

namespace rux::qt {

class ProjectSession;

class ViewSettings : public QObject {
  Q_OBJECT
    public:
  explicit ViewSettings(ProjectSession &session, QObject *parent = nullptr);
  ~ViewSettings() override;
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
  struct Writer;
  void reload();
  /// Write the pending yaw now, if there is one.
  void flush();
  void write(std::string path, std::string text, unsigned seq);
  ProjectSession &session_;
  EdgeEditor::RunnerSource runner_source_;
  QTimer *debounce_ = nullptr;
  double yaw_ = 0.0;
  /// Project and text of a yaw not yet written (path empty: none).
  std::string pending_path_;
  std::string pending_text_;
  /// Bumped by every yaw change: a busy write is retried only while it is
  /// still the latest.
  unsigned seq_ = 0;
  std::shared_ptr<Writer> writer_;
};

} // namespace rux::qt
