// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Database workspace (Stream Q, Q2) — RTABMap's DatabaseViewer
// reimagined: a project tree on the left (frames, panoramas, clouds, meshes,
// the pose graph, survey data, the pipeline log and every sqlite table, with
// counts); in the centre the frame browser, a table or the log, by what the
// tree selects. It only reads: pose-graph edits happen in Rediger.
// Whatever is selected is described to the inspector (selection_changed).

#include <rux_qt/selection.hpp>

#include <QFrame>
#include <QTreeWidget>
#include <QWidget>

class QLabel;
class QPushButton;
class QStackedWidget;

namespace rux::qt {

class EdgeEditor;
class FrameBrowser;
class FrameImageLoader;
class PipelineLogView;
class ProjectSession;
class TableBrowser;

/// The project as a tree of what it holds, with counts.
class ProjectTree : public QTreeWidget {
  Q_OBJECT
    public:
  /// What an item opens.
  enum class Kind { none, frames, scan, table, cloud, mesh, log };
  enum Role { KindRole = Qt::UserRole + 1, KeyRole, CountRole, TableRole };

  explicit ProjectTree(QWidget *parent = nullptr);
  /// Rebuild from @p session (empty when nothing is open).
  void rebuild(const ProjectSession &session);
  /// Select the item of @p kind (and key), without emitting activation.
  void select(Kind kind, const QString &key = {});

    signals:
  void activated_item(rux::qt::ProjectTree::Kind kind, const QString &key,
                      const QString &table, qint64 count);
};

class DatabaseWorkspace : public QWidget {
  Q_OBJECT
    public:
  /// @p editor is the shell's (Rediger edits through the same one).
  DatabaseWorkspace(ProjectSession &session, EdgeEditor &editor,
                    QWidget *parent = nullptr);
  ~DatabaseWorkspace() override;

  EdgeEditor &editor() const { return *editor_; }
  FrameBrowser *frames() const { return frames_; }
  ProjectTree *tree() const { return tree_; }
  /// The selection the inspector should show (empty: none).
  const Selection &selection() const { return selection_; }

  /// Open a tree item as if clicked (the gallery, the palette).
  void open_item(ProjectTree::Kind kind, const QString &key = {});

    signals:
  void selection_changed(const rux::qt::Selection &selection);
  /// "Åbn projekt…" in the empty state.
  void browse_requested();

    private:
  void rebuild();
  void show_item(ProjectTree::Kind kind, const QString &key,
                 const QString &table, qint64 count);
  void set_selection(const Selection &s);
  Selection cloud_selection(const QString &name) const;
  Selection mesh_selection(const QString &name) const;

  ProjectSession &session_;
  EdgeEditor *editor_ = nullptr;
  FrameImageLoader *loader_ = nullptr;
  ProjectTree *tree_ = nullptr;
  QStackedWidget *stack_ = nullptr;
  FrameBrowser *frames_ = nullptr;
  TableBrowser *tables_ = nullptr;
  PipelineLogView *log_ = nullptr;
  QWidget *empty_ = nullptr;
  QLabel *empty_text_ = nullptr;
  QPushButton *empty_open_ = nullptr;
  QWidget *tree_pane_ = nullptr;
  Selection selection_;
};

} // namespace rux::qt
