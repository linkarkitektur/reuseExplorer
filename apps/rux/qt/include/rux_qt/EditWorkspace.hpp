// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Rediger workspace (spec §5.2): the pose graph and its edge list on the
// left, the selected pair (images over a 3D overlay, then its edges) on the
// right, in nested QSplitters, under the pending-edits banner. A node click
// fills the next free slot of the pair (Shift: B), an edge click both; the
// right column's header names A and B and its foot is the pair strip. Each pane
// can be hidden from its header and shown again from the toolbar's "Paneler"
// menu or the command palette; splitter sizes and visibility persist in
// QSettings under rediger/… (not in a gallery shot).

#include <rux_qt/graph_view.hpp>
#include <rux_qt/palette_table.hpp>
#include <rux_qt/selection.hpp>

#include <QWidget>

#include <array>

class QAction;
class QLabel;
class QSplitter;

namespace rux::qt {

class EdgeEditor;
class PairStrip;
class PaneFrame;
class PendingBanner;
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

  /// A and B at once (an edge, a list row, the gallery).
  void set_pair(int a, int b);
  int a() const { return slots_.a(); }
  int b() const { return slots_.b(); }
  Slot next_slot() const { return slots_.next(); }
  PairStrip *strip() const { return strip_; }
  /// Pending pose-graph edits not yet saved.
  int pending_edits() const;
  /// Start saving them (off the GUI thread); false when it cannot start.
  /// The banner shows the progress and, on failure, why.
  bool save_edits();
  /// Save and wait for the outcome behind a small modal (quit, close).
  bool save_edits_and_wait();
  bool is_saving() const;
  void discard_edits();

    signals:
  void selection_changed(const rux::qt::Selection &selection);
  void pair_changed(int a, int b);

    protected:
  /// Show what is wanted, and the overlay only with a complete pair.
  void sync_panes();
  bool pair_complete_ = false;

    private:
  QWidget *make_toolbar();
  void node_clicked(int id, bool force_b);
  void apply_pair();
  void sync_pair_header();
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
  SlotPicker slots_;
  PendingBanner *banner_ = nullptr;
  PairStrip *strip_ = nullptr;
  QLabel *pair_a_ = nullptr;
  QLabel *pair_b_ = nullptr;
  QLabel *next_hint_ = nullptr;
};

} // namespace rux::qt
