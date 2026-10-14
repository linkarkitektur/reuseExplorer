// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's pair images as the match editor (reusex-xlg.2.5): A | B on one
// MatchCanvas (colour or depth, decoded off the GUI thread by its own
// FrameImageLoader), and a toolbar — Find match (ORB + ratio test,
// slam::match_frame_pair), the Vælg / Tilføj / Område modes, Ryd område,
// Placér B fra match (slam::rigid_from_matches on the enabled matches with
// depth at both ends) — and one status line. Find, a click's lift and the
// fit run on detached threads with their own read-only ProjectDB; each
// first checks the frames it needs (colour, depth, pose) so the status can
// name the missing piece in Danish ("billede 2241 har ingen dybde"); the
// library's English error is only the status line's tooltip. A result for
// a pair no longer shown is dropped (generation). A different pair clears
// the matches (C4). A fit is handed out as align_ready(world_delta);
// EditWorkspace places B exactly as it places ICP's result and answers
// align_applied().

#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/graph_view.hpp>
#include <rux_qt/match_editor.hpp>
#include <rux_qt/pair_alignment.hpp>

#include <QWidget>

#include <memory>

class QButtonGroup;
class QPushButton;
class QShortcut;

namespace rux::qt {

class ElidedLabel;
class FrameImageLoader;
class MatchCanvas;
class ProjectSession;

class PairImages : public QWidget {
  Q_OBJECT
    public:
  explicit PairImages(ProjectSession &session, QWidget *parent = nullptr);
  /// A and B (-1 = empty) and the slot the next plain click fills, which
  /// an empty side's hint names. A different pair clears the matches.
  void set_pair(int a, int b, Slot next);
  void set_layer(FrameLayer layer);
  FrameLayer layer() const { return layer_; }
  /// The Farve / Dybde switch (floating in the canvas).
  QWidget *layer_switch() const { return layer_switch_; }
  /// The Vælg / Tilføj / Område switch, for the pane header.
  QWidget *mode_switch() const { return mode_switch_; }
  /// Where Esc cancels a half-made match: focus anywhere inside @p scope
  /// (EditWorkspace passes itself; default this widget).
  void set_shortcut_scope(QWidget *scope);

  void find_matches();
  void align_from_matches();
  void set_mode(MatchMode mode);
  MatchMode mode() const { return mode_; }
  /// A Tilføj click at colour pixel (u, v) of @p side: lifted off-thread,
  /// then the first or second end of a manual match.
  void pick(MatchSide side, float u, float v);
  /// Esc: drops a half-made manual match.
  void cancel_pick();
  void toggle_line(int line);
  void set_region(MatchSide side, PixelBox box);
  void clear_regions();
  /// Whether EditWorkspace placed B on the last align_ready.
  void align_applied(bool placed);
  const MatchSet &matches() const { return set_; }
  MatchCanvas *canvas() const { return canvas_; }
  QString status_text() const;

    signals:
  /// Placér B fra match fitted the current pair: @p world_delta is B's new
  /// world-space offset (RigidFit::world_delta, the overlay's ΔT).
  void align_ready(int a, int b, const rux::qt::Mat4 &world_delta);

    private:
  enum class Busy { none, finding, lifting, aligning };
  struct Found;
  struct Lifted;
  struct Fitted;
  void reload();
  /// Forget the pair's working state (matches, pick, note, views); work
  /// still running describes the old state and is dropped (generation).
  void restart();
  void sync_images();
  void frame_ready(int id);
  void found(std::shared_ptr<Found> r);
  void lifted(std::shared_ptr<Lifted> r);
  void fitted(std::shared_ptr<Fitted> r);
  /// The status line's event: @p text (Danish) in @p tone; @p detail (the
  /// library's English message, if any) only in its tooltip.
  void note(const QString &text, const QString &tone,
            const QString &detail = {});
  void refresh();
  bool pair_ok() const;
  /// A click while find, lift or fit runs: says to wait; true if busy.
  bool refuse_busy();

  ProjectSession &session_;
  FrameImageLoader *loader_ = nullptr;
  MatchCanvas *canvas_ = nullptr;
  QButtonGroup *layers_ = nullptr;
  QWidget *layer_switch_ = nullptr;
  QWidget *mode_switch_ = nullptr;
  QButtonGroup *modes_ = nullptr;
  QPushButton *find_ = nullptr;
  QPushButton *align_ = nullptr;
  QPushButton *clear_region_ = nullptr;
  ElidedLabel *status_ = nullptr;
  /// Esc for a half-made match with focus in the shortcut scope (the canvas
  /// handles its own Esc too); enabled only while a pick waits.
  QShortcut *escape_ = nullptr;
  FrameLayer layer_ = FrameLayer::color;
  MatchMode mode_ = MatchMode::select;
  MatchSet set_;
  ManualPick pick_;
  int a_ = -1, b_ = -1;
  Slot next_ = Slot::a;
  unsigned generation_ = 0; ///< bumped on every pair change / reload
  Busy busy_ = Busy::none;
  QString note_, tone_, detail_; ///< the last event; empty = the read-out
  int last_inliers_ = 0;
  double last_rms_ = 0.0;
};

} // namespace rux::qt
