// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's 3D overlay (spec §5.2): A's and B's depth clouds back-projected
// with their current poses (slam::frame_world_points, the clouds ICP
// aligns), A in the accent, B in the chrome ink. B carries a manual offset
// ΔT (rux_qt/pair_alignment.hpp): a VTK box widget that translates and
// rotates but never scales (interactive mode only), six numeric fields, and
// keys — ←/→ x, ↑/↓ y, PgUp/PgDn z, [ / ] yaw, Shift ×10 (VTK's own keys,
// e/q/r/…, are left alone). "Nulstil" clears ΔT. Clouds load off the GUI
// thread on their own read-only connection, thinned to kOverlayMaxPoints
// per frame; a result for a pair no longer shown is dropped.

#include <rux_qt/pair_alignment.hpp>

#include <QWidget>

#include <array>
#include <cstddef>
#include <memory>

class QDoubleSpinBox;
class QFrame;
class QLabel;

namespace rux::qt {

class ProjectSession;
class SceneView;

class PairOverlay : public QWidget {
  Q_OBJECT
    public:
  PairOverlay(ProjectSession &session, bool interactive,
              QWidget *parent = nullptr);
  ~PairOverlay() override;

  void set_pair(int a, int b);
  const ManualOffset &offset() const { return offset_; }
  void set_offset(const ManualOffset &o);
  void reset_offset() { set_offset({}); }
  /// B's cloud centroid (world): the offset turns about it.
  Vec3 pivot() const { return pivot_; }
  Mat4 delta() const { return offset_matrix(offset_, pivot_); }
  /// Both clouds of the current pair are drawn.
  bool ready() const { return ready_; }
  SceneView *view() const { return view_; }
  /// The box widget moved B (called by its VTK observer).
  void widget_moved(const Mat4 &m);

    signals:
  void offset_changed();

    protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

    private:
  struct Clouds;
  struct Vtk;
  void load();
  void loaded(std::shared_ptr<Clouds> c);
  void frame_camera();
  void build_legend(std::size_t a_points, std::size_t b_points);
  void place_legend();
  void apply_offset(bool from_widget);
  void sync_fields();
  bool handle_key(int key, bool shift);
  ProjectSession &session_;
  SceneView *view_ = nullptr;
  std::array<QDoubleSpinBox *, 6> fields_{};
  QLabel *status_ = nullptr;
  QFrame *legend_ = nullptr; ///< A / B swatches over the canvas
  std::unique_ptr<Vtk> vtk_;
  ManualOffset offset_;
  Vec3 pivot_{};
  int a_ = -1, b_ = -1;
  unsigned generation_ = 0;
  bool ready_ = false;
};

} // namespace rux::qt
