// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's match canvas (reusex-xlg.2.4): A and B side by side on the
// near-black canvas, each with its own zoom (wheel, about the cursor;
// double-click resets, and leaves a line it lands on as it was) and pan (drag
// in Vælg, middle-drag in any mode), the keypoints (hollow = no depth) and the
// match lines across the gap — inlier --color-status-succeeded, outlier
// --color-status-failed, excluded --color-text-faint, before any fit
// --color-accent; manual lines dashed; the hovered line thicker with both ends
// ringed. Vælg: click a line; Tilføj: click a point; Område: drag a rectangle.
// Esc cancels a half-made match. All state lives in the MatchSet it is given.

#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/match_editor.hpp>

#include <QImage>
#include <QPointF>
#include <QRectF>
#include <QVector>
#include <QWidget>

#include <array>
#include <memory>
#include <optional>
#include <vector>

class QPainter;

namespace rux::qt {

struct DecodedFrame;

class MatchCanvas : public QWidget {
  Q_OBJECT
    public:
  explicit MatchCanvas(QWidget *parent = nullptr);
  void set_frame(MatchSide side, std::shared_ptr<const DecodedFrame> frame);
  /// A text instead of that side's image (an empty slot, an error); clears
  /// that side's frame.
  void set_message(MatchSide side, const QString &text);
  void set_loading(MatchSide side, bool loading);
  /// "A · billede 1998", painted in that side's top-left corner (no row
  /// of its own over the canvas: the column's height goes to the images).
  void set_caption(MatchSide side, const QString &text);
  /// A control floating in the canvas' bottom-right corner (Farve / Dybde),
  /// like the 3D overlay's Tint | RGB switch; reparented, not owned.
  void set_corner_widget(QWidget *w);
  /// A label floating in the bottom-left corner (the match status), as
  /// wide as the corner widget leaves; reparented, not owned.
  void set_status_widget(QWidget *w);
  void set_layer(FrameLayer layer);
  /// The match set and half-made pick to draw; not owned, may be null.
  void set_data(const MatchSet *set, const ManualPick *pick);
  void set_mode(MatchMode mode);
  /// Both images back to fitted (a new pair).
  void reset_views();
  /// The image of @p side as drawn now (fitted, zoomed, panned).
  ViewRect drawn_rect(MatchSide side) const;
  /// The colour image's size (keypoints are colour pixels), else the depth
  /// plane's; empty without a frame.
  QSize image_size(MatchSide side) const;
  QSize sizeHint() const override;

    signals:
  void line_clicked(int line);
  void pixel_clicked(rux::qt::MatchSide side, float u, float v);
  void region_dragged(rux::qt::MatchSide side, rux::qt::PixelBox box);
  void cancel_requested();

    protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *e) override;
  void mouseMoveEvent(QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *e) override;
  void mouseDoubleClickEvent(QMouseEvent *e) override;
  void wheelEvent(QWheelEvent *e) override;
  void keyPressEvent(QKeyEvent *e) override;
  void leaveEvent(QEvent *e) override;
  void resizeEvent(QResizeEvent *e) override;

    private:
  /// A's and B's slots for the current size (R8: not named after the
  /// Qt `slots` keyword macro).
  std::array<ViewRect, 2> slot_rects() const;
  /// One side's geometry for the current size and view, computed once per
  /// paint or mouse event rather than per keypoint.
  struct SideGeometry {
    ViewRect slot, drawn;
    int w = 0, h = 0; ///< image_size(); 0 without a frame
  };
  std::array<SideGeometry, 2> geometry() const;
  std::optional<MatchSide> side_at(QPointF p) const;
  /// Canvas position of keypoint @p k, or nullopt when it is outside that
  /// side's slot (zoomed or panned away). A keypoint need not lie inside
  /// the search region or even the image.
  static std::optional<QPointF> at(const SideGeometry &g, const MatchPoint &k);
  /// The visible lines as segments, and the MatchSet line each one is.
  std::vector<Segment> segments(const std::array<SideGeometry, 2> &g,
                                std::vector<int> &line_of) const;
  int line_at(QPointF p) const;
  /// The depth plane with the theme's colour table, rebuilt per frame and
  /// theme change instead of copied on every paint.
  void rebuild_depth(int side);
  void paint_side(QPainter &p, MatchSide side, const SideGeometry &g) const;
  void paint_caption(QPainter &p, MatchSide side, const QRectF &slot) const;

  std::array<std::shared_ptr<const DecodedFrame>, 2> frames_;
  std::array<QString, 2> messages_;
  std::array<QString, 2> captions_;
  QWidget *corner_ = nullptr;
  QWidget *status_ = nullptr;
  void place_floating();
  std::array<bool, 2> loading_{};
  std::array<ImageView, 2> views_{};
  FrameLayer layer_ = FrameLayer::color;
  const MatchSet *set_ = nullptr;
  const ManualPick *pick_ = nullptr;
  MatchMode mode_ = MatchMode::select;
  QVector<QRgb> depth_table_;
  std::array<QImage, 2> depth_shown_;
  int hover_ = -1;
  /// The line the last Vælg click toggled (a double-click undoes it).
  int clicked_line_ = -1;
  // One press: which button, where, on which side, and whether it moved
  // past the click slop (then it pans or draws a region, never clicks).
  Qt::MouseButton button_ = Qt::NoButton;
  QPointF press_, last_;
  std::optional<MatchSide> press_side_;
  bool dragged_ = false;
  std::optional<QRectF> rubber_; ///< Område drag, canvas coordinates
};

} // namespace rux::qt
