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
// per frame; a result for a pair no longer shown is dropped. "Farve: Tint |
// RGB" (colour_switch(), over the canvas' top-right corner) draws both clouds
// in their frames' own colours instead (slam::frame_world_colored_points); a
// frame without a colour image keeps its tint, and the legend says so.

#include <rux_qt/pair_alignment.hpp>

#include <QPointF>
#include <QWidget>

#include <array>
#include <cstddef>
#include <memory>
#include <optional>

class QButtonGroup;
class QDoubleSpinBox;
class QFrame;
class QLabel;

namespace rux::qt {

class ElidedLabel;
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
  /// Where B stands, one line ("B flyttet 1,8 cm · drejet 3,6°"), for the
  /// pane header: a status row under the fields took the canvas' height.
  QWidget *status_label() const;
  /// "Farve: Tint | RGB", floating over the canvas' top-right corner.
  QWidget *colour_switch() const { return colour_switch_; }
  OverlayColouring colouring() const { return colouring_; }
  /// Recolours the drawn clouds in place (no reload).
  void set_colouring(OverlayColouring c);
  /// The clouds' colours drawn right now: true for A / B when that cloud
  /// shows its RGB (RGB mode and the frame has a colour image).
  bool shows_rgb_a() const;
  bool shows_rgb_b() const;
  /// The box widget moved B (called by its VTK observer).
  void widget_moved(const Mat4 &m);
  /// Test probes for the gallery's Qt-level drag (rediger-drag-qt): the
  /// box centre in the GL widget's logical coordinates (nullopt without
  /// a box), the centre handle's projected radius in logical px (0 without
  /// a box), and the box representation's interaction state (-1 without).
  std::optional<QPointF> box_centre_px() const;
  double handle_radius_px() const;
  int box_state() const;

    signals:
  void offset_changed();
  void colouring_changed(rux::qt::OverlayColouring c);

    protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

    private:
  struct Clouds;
  struct Vtk;
  void load();
  void loaded(std::shared_ptr<Clouds> c);
  void frame_camera();
  void build_legend();
  void apply_colouring();
  void place_legend();
  void apply_offset(bool from_widget);
  void sync_fields();
  bool handle_key(int key, bool shift);
  ProjectSession &session_;
  SceneView *view_ = nullptr;
  std::array<QDoubleSpinBox *, 6> fields_{};
  ElidedLabel *status_ = nullptr;
  QFrame *legend_ = nullptr; ///< A / B swatches over the canvas
  QWidget *colour_switch_ = nullptr;
  QButtonGroup *colour_group_ = nullptr;
  OverlayColouring colouring_ = OverlayColouring::tint;
  std::size_t a_points_ = 0, b_points_ = 0; ///< drawn, for the legend
  bool a_has_rgb_ = false, b_has_rgb_ = false;
  std::unique_ptr<Vtk> vtk_;
  ManualOffset offset_;
  Vec3 pivot_{};
  int a_ = -1, b_ = -1;
  unsigned generation_ = 0;
  bool ready_ = false;
};

} // namespace rux::qt
