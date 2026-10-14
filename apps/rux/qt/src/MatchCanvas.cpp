// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/MatchCanvas.hpp>
#include <rux_qt/Theme.hpp>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
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
  update();
}

void MatchCanvas::set_message(MatchSide side, const QString &text) {
  frames_[idx(side)].reset();
  messages_[idx(side)] = text;
  update();
}

void MatchCanvas::set_loading(MatchSide side, bool loading) {
  loading_[idx(side)] = loading;
  update();
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
  const QSize s = image_size(side);
  return image_rect(slot_rects()[idx(side)], s.width(), s.height(),
                    views_[idx(side)]);
}

std::optional<MatchSide> MatchCanvas::side_at(QPointF p) const {
  const auto s = slot_rects();
  if (s[0].contains(p.x(), p.y()))
    return MatchSide::a;
  if (s[1].contains(p.x(), p.y()))
    return MatchSide::b;
  return std::nullopt;
}

std::optional<QPointF> MatchCanvas::at(MatchSide side,
                                       const MatchPoint &k) const {
  const QSize s = image_size(side);
  if (s.isEmpty())
    return std::nullopt;
  const auto c =
      image_to_canvas(drawn_rect(side), s.width(), s.height(), k.u, k.v);
  if (!slot_rects()[idx(side)].contains(c[0], c[1]))
    return std::nullopt;
  return QPointF(c[0], c[1]);
}

std::vector<Segment> MatchCanvas::segments(std::vector<int> &line_of) const {
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
    const auto pa = at(MatchSide::a, ka[static_cast<std::size_t>(l.a)]);
    const auto pb = at(MatchSide::b, kb[static_cast<std::size_t>(l.b)]);
    if (!pa || !pb)
      continue; // an end zoomed or panned out of view: not drawn, not hit
    out.push_back({pa->x(), pa->y(), pb->x(), pb->y()});
    line_of.push_back(i);
  }
  return out;
}

int MatchCanvas::line_at(QPointF p) const {
  std::vector<int> line_of;
  const int s =
      nearest_segment(segments(line_of), p.x(), p.y(), theme().px("--space-2"));
  return s < 0 ? -1 : line_of[static_cast<std::size_t>(s)];
}

void MatchCanvas::paint_side(QPainter &p, MatchSide side) const {
  const Theme &t = theme();
  const ViewRect slot = slot_rects()[idx(side)];
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
    p.restore();
    return;
  }
  if (!f->error.isEmpty()) {
    centred("Billedet kunne ikke læses.\n" + f->error);
    p.restore();
    return;
  }
  QImage base = layer_ == FrameLayer::depth ? f->depth : f->color;
  if (layer_ == FrameLayer::depth && !base.isNull())
    base.setColorTable(depth_table_);
  const ViewRect d = drawn_rect(side);
  if (!base.isNull()) {
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
    p.drawImage(QRectF(d.x, d.y, d.w, d.h), base);
  } else {
    centred(layer_ == FrameLayer::depth ? "Intet dybdebillede"
                                        : "Intet farvebillede");
  }
  const QSize s = image_size(side);
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
      const auto c = at(side, k);
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
  p.restore();
}

void MatchCanvas::paintEvent(QPaintEvent *) {
  const Theme &t = theme();
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  paint_side(p, MatchSide::a);
  paint_side(p, MatchSide::b);
  if (set_) {
    // The lines, each over a scrim halo so it reads on any photo.
    std::vector<int> line_of;
    const auto segs = segments(line_of);
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
    if (const auto c = at(*pick_->side(), *pick_->point())) {
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
    const int h = line_at(pos);
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
  if (b != Qt::LeftButton || !press_side_)
    return;
  const MatchSide side = *press_side_;
  const QSize s = image_size(side);
  if (mode_ == MatchMode::region && rubber_) {
    rubber_.reset();
    if (!s.isEmpty()) {
      // The drag may run outside the image or the slot: box_from_drag
      // clamps it, and a click-sized drag comes back empty.
      const ViewRect d = drawn_rect(side);
      const auto p0 =
          canvas_to_image(d, s.width(), s.height(), press_.x(), press_.y());
      const auto p1 =
          canvas_to_image(d, s.width(), s.height(), pos.x(), pos.y());
      const PixelBox box =
          box_from_drag(p0[0], p0[1], p1[0], p1[1], s.width(), s.height());
      if (!box.empty())
        emit region_dragged(side, box);
    }
    update();
    return;
  }
  if (dragged_)
    return; // a pan or a drag, not a click
  if (mode_ == MatchMode::select) {
    if (const int l = line_at(pos); l >= 0)
      emit line_clicked(l);
  } else if (mode_ == MatchMode::add && !s.isEmpty()) {
    const auto px = canvas_to_image(drawn_rect(side), s.width(), s.height(),
                                    pos.x(), pos.y());
    if (px[0] >= 0 && px[1] >= 0 && px[0] <= s.width() - 1 &&
        px[1] <= s.height() - 1)
      emit pixel_clicked(side, static_cast<float>(px[0]),
                         static_cast<float>(px[1]));
  }
}

void MatchCanvas::mouseDoubleClickEvent(QMouseEvent *e) {
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
