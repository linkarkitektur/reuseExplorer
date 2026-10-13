// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Qt client's window content: a title bar (project, schema status), the
// left nav rail (Start, Oversigt, 3D, Rediger, Pipeline, Log), a stack of
// workspaces, a collapsible inspector on the right, and the Ctrl+K command
// palette over all of it. A plain QWidget, so the gallery can screenshot it
// without a QMainWindow; MainWindow wraps it for the real app.
//
// The shell owns no project: it is handed the ProjectSession and the recent
// list, and reacts to the session's signals.

#include <rux_qt/CommandPalette.hpp>
#include <rux_qt/selection.hpp>
#include <rux_qt/workspaces.hpp>

#include <reusex/pipeline/stages.hpp>

#include <QFrame>
#include <QWidget>
#include <functional>

class QAction;
class QLabel;
class QPushButton;
class QStackedWidget;

namespace rux::qt {

class DatabaseWorkspace;
class EdgeEditor;
class EditWorkspace;
class Inspector;
class PipelineLogView;
class PipelineWorkspace;
class Viewer3DWorkspace;
class NavRail;
class ProjectSession;
class RecentProjects;
class StartPage;
class Pill;

/// What differs between the real app and a gallery shot.
struct ShellOptions {
  /// The 3D workspace uses a real QVTKOpenGLNativeWidget (needs a display);
  /// otherwise it renders offscreen into an image (screenshots).
  bool interactive_3d = false;
  /// Runs the Pipeline workspace's stages; empty = the library default.
  reusex::pipeline::StageExecutor stage_executor;
  /// Splitter sizes and pane visibility persist in QSettings (false in a
  /// gallery shot).
  bool persist_layout = true;
};

class AppShell : public QWidget {
  Q_OBJECT
    public:
  AppShell(ProjectSession &session, RecentProjects &recent,
           ShellOptions options = {}, QWidget *parent = nullptr);

  void show_page(Workspace w);
  Workspace current_page() const;

  void set_inspector_visible(bool on);
  bool inspector_visible() const;

  CommandPalette *palette() const { return palette_; }
  void open_palette(const QString &query = {});
  /// Every page, action and recent project, as the palette lists them.
  QVector<Command> commands();

  /// Open @p path through the session and remember it in the recent list.
  void open_project(const QString &path, bool read_only = false);

  DatabaseWorkspace *database() const { return database_; }
  Viewer3DWorkspace *viewer() const { return viewer_; }
  EditWorkspace *edit() const { return edit_; }
  /// The pending pose-graph edits, shared by Oversigt and Rediger.
  EdgeEditor &edges() const { return *edges_; }
  PipelineWorkspace *pipeline() const { return pipeline_; }
  PipelineLogView *log() const { return log_; }
  /// Unsaved pose-graph edits: ask whether to save, discard or cancel
  /// before @p action (Danish infinitive: "lukke projektet"). True when it is
  /// fine to go on — nothing pending, saved, or discarded.
  bool resolve_pending_edits(const QString &action);
  /// A pipeline run in progress before @p action ("lukke projektet"): true
  /// when none runs. Otherwise asks; "Stop kørslen" cancels the job and runs
  /// @p retry once it has ended — the GUI thread never waits on a stage (a
  /// MIP solve may not stop for minutes). False means "not now".
  bool resolve_running_job(const QString &action, std::function<void()> retry);
  /// The QFileDialog for .rux files.
  void browse();

    signals:
  void quit_requested();
  void toggle_theme_requested();
  /// The inspector was shown or hidden (MainWindow stores it).
  void inspector_toggled(bool visible);

    protected:
  void dragEnterEvent(QDragEnterEvent *e) override;
  void dragLeaveEvent(QDragLeaveEvent *e) override;
  void dropEvent(QDropEvent *e) override;

    private:
  QWidget *make_title_bar();
  void build_actions();
  void sync_project();
  void refresh_product_label();
  /// The selection the inspector shows for page @p w.
  Selection page_selection(Workspace w) const;

  ProjectSession &session_;
  RecentProjects &recent_;

  QLabel *product_ = nullptr;
  QLabel *title_project_ = nullptr;
  QLabel *title_path_ = nullptr;
  Pill *schema_pill_ = nullptr;
  Pill *access_pill_ = nullptr;
  QPushButton *inspector_button_ = nullptr;

  NavRail *rail_ = nullptr;
  QStackedWidget *stack_ = nullptr;
  StartPage *start_ = nullptr;
  DatabaseWorkspace *database_ = nullptr;
  Viewer3DWorkspace *viewer_ = nullptr;
  EditWorkspace *edit_ = nullptr;
  EdgeEditor *edges_ = nullptr;
  PipelineWorkspace *pipeline_ = nullptr;
  PipelineLogView *log_ = nullptr;
  Selection log_selection_;
  /// What to do once the running job has ended (resolve_running_job).
  std::function<void()> after_job_;
  QString log_command_;
  Inspector *inspector_ = nullptr;
  CommandPalette *palette_ = nullptr;

  QAction *open_action_ = nullptr;
  QAction *close_action_ = nullptr;
  QAction *reload_action_ = nullptr;
  QAction *inspector_action_ = nullptr;
  QAction *palette_action_ = nullptr;
  QAction *quit_action_ = nullptr;
  QAction *theme_action_ = nullptr;
};

/// The right-hand context panel: the current selection of the workspace on
/// screen (a frame, an edge, a table row, a log entry), or the open
/// project's properties when nothing is selected.
class Inspector : public QFrame {
  Q_OBJECT
    public:
  explicit Inspector(ProjectSession &session, QWidget *parent = nullptr);
  void refresh();
  /// Show @p selection; an empty one shows the project again.
  void set_selection(const Selection &selection);

    signals:
  void hide_requested();

    private:
  void show_project();
  void show_selection();
  ProjectSession &session_;
  QWidget *body_ = nullptr;
  Selection selection_;
};

} // namespace rux::qt
