// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Rediger workspace (spec §5.2): the pose graph and its edge list on the
// left, the selected pair (images over a 3D overlay, then its edges) on the
// right, in nested QSplitters. Each pane can be hidden from its header and
// shown again from the toolbar's "Paneler" menu or the command palette;
// splitter sizes and visibility persist in QSettings under rediger/… (not
// in a gallery shot).

#include <rux_qt/palette_table.hpp>
#include <rux_qt/selection.hpp>

#include <QWidget>

#include <array>

class QAction;
class QLabel;
class QSplitter;

namespace rux::qt {

class EdgeEditor;
class PaneFrame;
class PoseGraphWorkspace;
class ProjectSession;

class EditWorkspace : public QWidget {
  Q_OBJECT
    public:
  EditWorkspace(ProjectSession &session, EdgeEditor &editor,
                bool persist_layout, QWidget *parent = nullptr);
  ~EditWorkspace() override;

  /// The page is on screen: load what is not loaded yet.
  void activate();
  PoseGraphWorkspace *graph() const { return graph_; }
  EdgeEditor &editor() const { return editor_; }
  PaneFrame *pane(EditPane p) const { return panes_[static_cast<int>(p)]; }
  /// The user's choice for @p p (the overlay also needs a complete pair).
  bool pane_visible(EditPane p) const { return wanted_[static_cast<int>(p)]; }
  void set_pane_visible(EditPane p, bool on);
  void toggle_pane(EditPane p) { set_pane_visible(p, !pane_visible(p)); }
  const Selection &selection() const { return selection_; }

    signals:
  void selection_changed(const rux::qt::Selection &selection);

    protected:
  /// Show what is wanted, and the overlay only with a complete pair.
  void sync_panes();
  bool pair_complete_ = false;

    private:
  QWidget *make_toolbar();
  void restore_layout();
  void save_layout() const;

  ProjectSession &session_;
  EdgeEditor &editor_;
  bool persist_ = true;
  QSplitter *main_ = nullptr;
  QSplitter *left_ = nullptr;
  QSplitter *right_ = nullptr;
  QWidget *right_column_ = nullptr;
  std::array<PaneFrame *, kEditPaneCount> panes_{};
  std::array<bool, kEditPaneCount> wanted_{true, true, true, true};
  std::array<QAction *, kEditPaneCount> pane_actions_{};
  PoseGraphWorkspace *graph_ = nullptr;
  QLabel *meta_ = nullptr;
  Selection selection_;
};

} // namespace rux::qt
