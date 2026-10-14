// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/MatchCanvas.hpp>
#include <rux_qt/Theme.hpp>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>
#include <QWheelEvent>

#include <cmath>

namespace rux::qt {
namespace {

constexpr double kWheelStep = 1.25; ///< zoom factor per wheel notch
constexpr double kLineWidth = 1.5;  ///< a match line, canvas px

int idx(MatchSide s) { return s == MatchSide::a ? 0 : 1; }

const char *tone_token(LineTone t) {
  switch (t) {
  case LineTone::inlier:
    return "--color-status-succeeded";
  case LineTone::outlier:
    return "--color-status-failed";
  case LineTone::excluded:
    return "--color-text-faint";
  case LineTone::neutral:
    break;
  }
  return "--color-accent";
}

} // namespace

MatchCanvas::MatchCanvas(QWidget *parent) : QWidget(parent) {
  setObjectName("matchCanvas");
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMinimumSize(theme().px("--space-7") * 2, theme().px("--space-7"));
  setMouseTracking(true);
  setFocusPolicy(Qt::ClickFocus);
  setCursor(Qt::OpenHandCursor);
  depth_table_ = depth_colour_table();
  connect(&theme(), &Theme::changed, this, [this] {
    depth_table_ = depth_colour_table();
    rebuild_depth(0);
    rebuild_depth(1);
    update();
  });
}

QSize MatchCanvas::sizeHint() const {
  return {theme().px("--layout-panel-width"),
          theme().px("--layout-panel-width") / 2};
}

void MatchCanvas::set_frame(MatchSide side,
                            std::shared_ptr<const DecodedFrame> frame) {
  frames_[idx(side)] = std::move(frame);
  messages_[idx(side)].clear();
  loading_[idx(side)] = false;
  rebuild_depth(idx(side));
  update();
}

void MatchCanvas::set_message(MatchSide side, const QString &text) {
  frames_[idx(side)].reset();
  depth_shown_[idx(side)] = QImage();
  messages_[idx(side)] = text;
  update();
}

void MatchCanvas::set_loading(MatchSide side, bool loading) {
  loading_[idx(side)] = loading;
  update();
}

void MatchCanvas::set_caption(MatchSide side, const QString &text) {
  captions_[idx(side)] = text;
  update();
}

void MatchCanvas::set_corner_widget(QWidget *w) {
  corner_ = w;
  if (w) {
    w->setParent(this);
    w->show();
  }
  place_floating();
}

void MatchCanvas::set_status_widget(QWidget *w) {
  status_ = w;
  if (w) {
    w->setParent(this);
    w->show();
  }
  place_floating();
}

void MatchCanvas::resizeEvent(QResizeEvent *e) {
  QWidget::resizeEvent(e);
  place_floating();
}

void MatchCanvas::place_floating() {
  const int m = theme().px("--space-2");
  int right = width() - m; // the status line ends where the switch starts
  if (corner_) {
    corner_->adjustSize();
    corner_->move(width() - corner_->width() - m,
                  height() - corner_->height() - m);
    corner_->raise();
    right = corner_->x() - m;
  }
  if (status_) {
    const int h = status_->sizeHint().height();
    const int bottom =
        corner_ ? corner_->y() + corner_->height() : height() - m;
    status_->setGeometry(m, bottom - h, std::max(0, right - m), h);
    status_->raise();
  }
}

void MatchCanvas::set_layer(FrameLayer layer) {
  layer_ = layer;
  update();
}

void MatchCanvas::set_data(const MatchSet *set, const ManualPick *pick) {
  set_ = set;
  pick_ = pick;
  hover_ = -1;
  update();
}

void MatchCanvas::set_mode(MatchMode mode) {
  mode_ = mode;
  rubber_.reset();
  hover_ = -1; // only Vælg highlights a line
  setCursor(mode == MatchMode::select ? Qt::OpenHandCursor : Qt::CrossCursor);
  update();
}

void MatchCanvas::reset_views() {
  views_ = {};
  hover_ = -1;
  update();
}

std::array<ViewRect, 2> MatchCanvas::slot_rects() const {
  return pair_slots(width(), height(), theme().px("--space-4"));
}

QSize MatchCanvas::image_size(MatchSide side) const {
  const auto &f = frames_[idx(side)];
  if (!f)
    return {};
  // The colour frame decides the geometry (keypoints are colour pixels);
  // the depth plane is drawn stretched into the same rect, as ImagePane does.
  return !f->color.isNull() ? f->color.size() : f->depth.size();
}

ViewRect MatchCanvas::drawn_rect(MatchSide side) const {
  return geometry()[idx(side)].drawn;
}

std::array<MatchCanvas::SideGeometry, 2> MatchCanvas::geometry() const {
  const auto slots_now = slot_rects();
  std::array<SideGeometry, 2> g;
  for (int i = 0; i < 2; ++i) {
    const QSize s = image_size(i == 0 ? MatchSide::a : MatchSide::b);
    g[i].slot = slots_now[i];
    g[i].w = s.width();
    g[i].h = s.height();
    g[i].drawn = image_rect(slots_now[i], g[i].w, g[i].h, views_[i]);
  }
  return g;
}

std::optional<MatchSide> MatchCanvas::side_at(QPointF p) const {
  const auto s = slot_rects();
  if (s[0].contains(p.x(), p.y()))
    return MatchSide::a;
  if (s[1].contains(p.x(), p.y()))
    return MatchSide::b;
  return std::nullopt;
}

std::optional<QPointF> MatchCanvas::at(const SideGeometry &g,
                                       const MatchPoint &k) {
  if (g.w <= 0 || g.h <= 0)
    return std::nullopt;
  const auto c = image_to_canvas(g.drawn, g.w, g.h, k.u, k.v);
  if (!g.slot.contains(c[0], c[1]))
    return std::nullopt;
  return QPointF(c[0], c[1]);
}

std::vector<Segment> MatchCanvas::segments(const std::array<SideGeometry, 2> &g,
                                           std::vector<int> &line_of) const {
  std::vector<Segment> out;
  line_of.clear();
  if (!set_)
    return out;
  const auto &ka = set_->keypoints(MatchSide::a);
  const auto &kb = set_->keypoints(MatchSide::b);
  const int na = static_cast<int>(ka.size());
  const int nb = static_cast<int>(kb.size());
  for (int i = 0; i < static_cast<int>(set_->lines().size()); ++i) {
    const MatchLine &l = set_->lines()[static_cast<std::size_t>(i)];
    if (l.a < 0 || l.a >= na || l.b < 0 || l.b >= nb)
      continue;
    const auto pa = at(g[0], ka[static_cast<std::size_t>(l.a)]);
    const auto pb = at(g[1], kb[static_cast<std::size_t>(l.b)]);
    if (!pa || !pb)
      continue; // an end zoomed or panned out of view: not drawn, not hit
    out.push_back({pa->x(), pa->y(), pb->x(), pb->y()});
    line_of.push_back(i);
  }
  return out;
}

int MatchCanvas::line_at(QPointF p) const {
  std::vector<int> line_of;
  const int s = nearest_segment(segments(geometry(), line_of), p.x(), p.y(),
                                theme().px("--space-2"));
  return s < 0 ? -1 : line_of[static_cast<std::size_t>(s)];
}

void MatchCanvas::rebuild_depth(int side) {
  const auto &f = frames_[side];
  QImage d = f ? f->depth : QImage();
  if (!d.isNull())
    d.setColorTable(depth_table_); // detaches once here, not per paint
  depth_shown_[side] = d;
}

void MatchCanvas::paint_side(QPainter &p, MatchSide side,
                             const SideGeometry &g) const {
  const Theme &t = theme();
  const ViewRect slot = g.slot;
  const QRectF sr(slot.x, slot.y, slot.w, slot.h);
  p.save();
  QPainterPath clip;
  clip.addRoundedRect(sr, t.px("--radius-md"), t.px("--radius-md"));
  p.setClipPath(clip);
  p.fillRect(sr, t.color("--color-canvas"));
  const auto &f = frames_[idx(side)];
  const auto centred = [&](const QString &text) {
    p.setPen(t.color("--color-on-chrome-muted"));
    p.setFont(t.font(FontRole::sans, "--font-size-sm"));
    p.drawText(sr.adjusted(t.px("--space-3"), 0, -t.px("--space-3"), 0),
               Qt::AlignCenter | Qt::TextWordWrap, text);
  };
  if (!f) {
    centred(!messages_[idx(side)].isEmpty() ? messages_[idx(side)]
            : loading_[idx(side)]           ? QString("Henter billedet …")
                                            : QString());
    paint_caption(p, side, sr);
    p.restore();
    return;
  }
  if (!f->error.isEmpty()) {
    centred("Billedet kunne ikke læses.\n" + f->error);
    paint_caption(p, side, sr);
    p.restore();
    return;
  }
  const QImage &base =
      layer_ == FrameLayer::depth ? depth_shown_[idx(side)] : f->color;
  const ViewRect d = g.drawn;
  if (!base.isNull()) {
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
    p.drawImage(QRectF(d.x, d.y, d.w, d.h), base);
  } else {
    centred(layer_ == FrameLayer::depth ? "Intet dybdebillede"
                                        : "Intet farvebillede");
  }
  const QSize s(g.w, g.h);
  // The search region, dashed accent.
  if (set_ && set_->region(side) && !s.isEmpty()) {
    const PixelBox &b = *set_->region(side);
    const auto tl =
        image_to_canvas(d, s.width(), s.height(), b.x - 0.5, b.y - 0.5);
    const auto br = image_to_canvas(d, s.width(), s.height(), b.x + b.w - 0.5,
                                    b.y + b.h - 0.5);
    p.setPen(QPen(t.color("--color-accent"), kLineWidth, Qt::DashLine));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(QPointF(tl[0], tl[1]), QPointF(br[0], br[1])));
  }
  // Every keypoint, inside the region or not: a small dot, hollow without
  // depth.
  if (set_) {
    const double r = t.px("--space-1") * 0.5;
    const QColor ink = t.color("--color-on-chrome-muted");
    for (const MatchPoint &k : set_->keypoints(side)) {
      const auto c = at(g, k);
      if (!c)
        continue;
      p.setPen(k.xyz ? Qt::NoPen : QPen(ink, 1.0));
      p.setBrush(k.xyz ? QBrush(ink) : QBrush(Qt::NoBrush));
      p.drawEllipse(*c, r, r);
    }
  }
  if (loading_[idx(side)]) {
    QColor veil = t.color("--color-canvas");
    veil.setAlphaF(0.45);
    p.fillRect(sr, veil);
  }
  paint_caption(p, side, sr);
  p.restore();
}

void MatchCanvas::paint_caption(QPainter &p, MatchSide side,
                                const QRectF &slot) const {
  const QString &text = captions_[idx(side)];
  if (text.isEmpty())
    return;
  const Theme &t = theme();
  const QFont font = t.font(FontRole::mono, "--font-size-xs");
  const QFontMetrics fm(font);
  const int pad = t.px("--space-1");
  const QString shown = fm.elidedText(text, Qt::ElideRight,
                                      static_cast<int>(slot.width()) - 4 * pad);
  const QRectF box(slot.x() + pad, slot.y() + pad,
                   fm.horizontalAdvance(shown) + 2 * pad, fm.height() + pad);
  QColor scrim = t.color("--color-scrim");
  scrim.setAlphaF(0.6);
  p.setPen(Qt::NoPen);
  p.setBrush(scrim);
  p.drawRoundedRect(box, t.px("--radius-sm"), t.px("--radius-sm"));
  p.setFont(font);
  p.setPen(t.color("--color-on-chrome"));
  p.drawText(box, Qt::AlignCenter, shown);
}

void MatchCanvas::paintEvent(QPaintEvent *) {
  const Theme &t = theme();
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  const auto g = geometry();
  paint_side(p, MatchSide::a, g[0]);
  paint_side(p, MatchSide::b, g[1]);
  if (set_) {
    // The lines, each over a scrim halo so it reads on any photo.
    std::vector<int> line_of;
    const auto segs = segments(g, line_of);
    QColor halo = t.color("--color-scrim");
    halo.setAlphaF(0.6);
    const double ring = t.px("--space-2") * 0.75;
    // The hovered line last, so it sits on top of its neighbours.
    std::vector<std::size_t> order;
    order.reserve(segs.size());
    std::size_t hot_k = segs.size();
    for (std::size_t k = 0; k < segs.size(); ++k) {
      if (line_of[k] == hover_)
        hot_k = k;
      else
        order.push_back(k);
    }
    if (hot_k < segs.size())
      order.push_back(hot_k);
    for (const std::size_t k : order) {
      const MatchLine &l = set_->lines()[static_cast<std::size_t>(line_of[k])];
      const bool hot = k == hot_k;
      const double w = hot ? 2 * kLineWidth : kLineWidth;
      const QLineF line(segs[k].x0, segs[k].y0, segs[k].x1, segs[k].y1);
      p.setPen(QPen(halo, w + 2.0, Qt::SolidLine, Qt::RoundCap));
      p.drawLine(line);
      p.setPen(QPen(t.color(tone_token(line_tone(l))), w,
                    l.manual ? Qt::DashLine : Qt::SolidLine, Qt::RoundCap));
      p.drawLine(line);
      if (hot) {
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(t.color(tone_token(line_tone(l))), kLineWidth));
        p.drawEllipse(line.p1(), ring, ring);
        p.drawEllipse(line.p2(), ring, ring);
      }
    }
  }
  // The half-made manual match: a ring where the first click landed.
  if (pick_ && pick_->pending() && pick_->side()) {
    if (const auto c = at(g[idx(*pick_->side())], *pick_->point())) {
      p.setPen(QPen(t.color("--color-accent"), 2.0));
      p.setBrush(Qt::NoBrush);
      const double ring = t.px("--space-2");
      p.drawEllipse(*c, ring, ring);
    }
  }
  if (rubber_) {
    p.setPen(QPen(t.color("--color-accent"), 1.0, Qt::DashLine));
    QColor fill = t.color("--color-accent");
    fill.setAlphaF(0.12);
    p.setBrush(fill);
    p.drawRect(rubber_->normalized());
  }
}

void MatchCanvas::mousePressEvent(QMouseEvent *e) {
  clicked_line_ = -1;
  button_ = e->button();
  press_ = last_ = e->position();
  press_side_ = side_at(e->position());
  dragged_ = false;
  if (button_ == Qt::LeftButton && mode_ == MatchMode::select)
    setCursor(Qt::ClosedHandCursor);
}

void MatchCanvas::mouseMoveEvent(QMouseEvent *e) {
  const QPointF pos = e->position();
  if (button_ == Qt::NoButton) {
    const int h = mode_ == MatchMode::select ? line_at(pos) : -1;
    if (h != hover_) {
      hover_ = h;
      update();
    }
    return;
  }
  if ((pos - press_).manhattanLength() > theme().px("--space-1"))
    dragged_ = true;
  if (!dragged_ || !press_side_) {
    last_ = pos;
    return;
  }
  const bool pan = button_ == Qt::MiddleButton ||
                   (button_ == Qt::LeftButton && mode_ == MatchMode::select);
  if (pan) {
    ImageView &v = views_[idx(*press_side_)];
    if (v.zoom > 1.0) { // a fitted image does not pan
      v.pan_x += pos.x() - last_.x();
      v.pan_y += pos.y() - last_.y();
      hover_ = -1;
      update();
    }
  } else if (button_ == Qt::LeftButton && mode_ == MatchMode::region) {
    rubber_ = QRectF(press_, pos);
    update();
  }
  last_ = pos;
}

void MatchCanvas::mouseReleaseEvent(QMouseEvent *e) {
  const QPointF pos = e->position();
  const Qt::MouseButton b = button_;
  button_ = Qt::NoButton;
  if (mode_ == MatchMode::select)
    setCursor(Qt::OpenHandCursor);
  if (b != Qt::LeftButton)
    return;
  if (mode_ == MatchMode::select) {
    // Lines cross the gap between the slots, so a click there counts too.
    if (!dragged_)
      if (const int l = line_at(pos); l >= 0) {
        clicked_line_ = l;
        emit line_clicked(l);
      }
    return;
  }
  if (!press_side_) {
    rubber_.reset();
    update();
    return;
  }
  const MatchSide side = *press_side_;
  const SideGeometry g = geometry()[idx(side)];
  const bool has_image = g.w > 0 && g.h > 0;
  if (mode_ == MatchMode::region && rubber_) {
    rubber_.reset();
    if (has_image) {
      // The drag may run outside the image or the slot: box_from_drag
      // clamps it, and a click-sized drag comes back empty.
      const auto p0 =
          canvas_to_image(g.drawn, g.w, g.h, press_.x(), press_.y());
      const auto p1 = canvas_to_image(g.drawn, g.w, g.h, pos.x(), pos.y());
      const PixelBox box = box_from_drag(p0[0], p0[1], p1[0], p1[1], g.w, g.h);
      if (!box.empty())
        emit region_dragged(side, box);
    }
    update();
    return;
  }
  if (dragged_)
    return; // a drag, not a click
  if (mode_ == MatchMode::add && has_image) {
    const auto px = canvas_to_image(g.drawn, g.w, g.h, pos.x(), pos.y());
    if (in_image(px[0], px[1], g.w, g.h))
      emit pixel_clicked(side, static_cast<float>(px[0]),
                         static_cast<float>(px[1]));
  }
}

void MatchCanvas::mouseDoubleClickEvent(QMouseEvent *e) {
  // In Vælg the double-click's first click already toggled a line: toggle
  // that same line back (not whatever lies under the second click), so a
  // double-click only resets the view.
  if (e->button() == Qt::LeftButton && clicked_line_ >= 0)
    emit line_clicked(clicked_line_);
  clicked_line_ = -1;
  if (const auto side = side_at(e->position())) {
    views_[idx(*side)] = {};
    hover_ = -1;
    update();
  }
}

void MatchCanvas::wheelEvent(QWheelEvent *e) {
  const auto side = side_at(e->position());
  const QSize s = side ? image_size(*side) : QSize();
  if (!side || s.isEmpty() || e->angleDelta().y() == 0) {
    e->ignore();
    return;
  }
  const double factor = std::pow(kWheelStep, e->angleDelta().y() / 120.0);
  ImageView &v = views_[idx(*side)];
  v = zoom_about(slot_rects()[idx(*side)], s.width(), s.height(), v,
                 e->position().x(), e->position().y(), factor);
  hover_ = -1;
  update();
  e->accept();
}

void MatchCanvas::keyPressEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_Escape) {
    rubber_.reset();
    button_ = Qt::NoButton;
    emit cancel_requested();
    update();
    return;
  }
  QWidget::keyPressEvent(e);
}

void MatchCanvas::leaveEvent(QEvent *e) {
  if (hover_ != -1) {
    hover_ = -1;
    update();
  }
  QWidget::leaveEvent(e);
}

} // namespace rux::qt
