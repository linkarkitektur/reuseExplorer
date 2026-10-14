// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The shell's pages, in nav-rail order, and the placeholder every workspace
// shows until its phase lands. Rail order: Start, Oversigt, 3D, Rediger,
// Pipeline, Log (Oversigt is Workspace::database). A placeholder is a designed
// empty state: what the workspace will do, and the rux commands that do it
// today.

#include <QString>
#include <QStringList>
#include <QVector>
#include <QWidget>

class QLabel;
class QPushButton;

namespace rux::qt {

/// Nav-rail index of each page.
enum class Workspace : int {
  start = 0,
  database,
  viewer3d,
  edit,
  pipeline,
  log,
};
inline constexpr int kWorkspaceCount = 6;

struct WorkspaceInfo {
  Workspace id;
  QString name;       ///< nav label, palette title
  QString keywords;   ///< palette aliases
  QString subtitle;   ///< one line under the view title
  QString phase;      ///< "Q2" / "Q3"; empty for the start page
  QStringList coming; ///< what lands, one bullet each
  QStringList cli;    ///< today's equivalents in the terminal
};

const QVector<WorkspaceInfo> &workspace_infos();
const WorkspaceInfo &workspace_info(Workspace w);

/// The empty state of a workspace that is not built yet.
class WorkspacePlaceholder : public QWidget {
  Q_OBJECT
    public:
  explicit WorkspacePlaceholder(const WorkspaceInfo &info,
                                QWidget *parent = nullptr);
  /// Whether a project is open: without one the page offers "Åbn projekt…".
  void set_project_open(bool open, const QString &status = {});

    signals:
  void browse_requested();

    private:
  QPushButton *open_ = nullptr;
  QLabel *status_ = nullptr;
};

} // namespace rux::qt
