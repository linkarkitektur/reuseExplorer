// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/GraphPanel.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/ViewSettings.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/edge_colouring.hpp>
#include <rux_qt/edge_table.hpp>
#include <rux_qt/graph_view.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>

#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLocale>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QPointer>
#include <QPushButton>
#include <QScrollBar>
#include <QSettings>
#include <QSignalBlocker>
#include <QSlider>
#include <QStackedLayout>
#include <QStyleOptionGraphicsItem>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <thread>
#include <tuple>

namespace rux::qt {

namespace {

const QLocale &da() {
  static const QLocale l(QLocale::Danish, QLocale::Denmark);
  return l;
}

/// Screen pixels of a node's radius, its heading tick and the click
/// tolerance.
constexpr double kNodePx = 1.6;
constexpr double kTickPx = 7.0;
/// Heading ticks are blitted from sprites, one per 5° of direction.
constexpr int kTickBins = 72;
constexpr double kHitPx = 7.0;
/// Edges drawn antialiased up to this many (see EdgesItem::paint).
constexpr std::size_t kAntialiasedEdges = 2000;

QColor with_alpha(QColor c, double a) {
  c.setAlphaF(static_cast<float>(a));
  return c;
}

int label_slots() {
  bool ok = false;
  const int n = theme().value("--label-count").toInt(&ok);
  return ok && n > 0 ? n : 8;
}

// "Nodes disappear when zoomed in" (posegraf-zoom): bounds_ was grown with
// QRectF::united(zero-size rect), which ignores null rects, so the item's
// bounding rect was the LAST node +-1 m and the scene culled it elsewhere.
/// Every node in one item: thousands of QGraphicsEllipseItems would make
/// the scene slow to build and to hit-test, and the dots must keep their
/// size on screen at every zoom.
class NodesItem : public QGraphicsItem {
    public:
  /// @p step: median scene distance of consecutive nodes (median_step);
  /// the ticks show only once that is a tick long on screen.
  NodesItem(const std::vector<GraphNode> &nodes, std::vector<int> scans,
            const std::vector<Heading> &ticks, double step)
      : scans_(std::move(scans)), step_(step) {
    // The direction bin of each tick (-1: hidden), fixed for the item.
    tick_bin_.reserve(ticks.size());
    for (const Heading &h : ticks) {
      if (!h.visible) {
        tick_bin_.push_back(-1);
        continue;
      }
      // Screen y points down: the canvas negates the view's up.
      const double a = std::atan2(-h.dir.y, h.dir.x);
      int bin = static_cast<int>(
                    std::lround(a / (2.0 * std::numbers::pi) * kTickBins)) %
                kTickBins;
      tick_bin_.push_back(bin < 0 ? bin + kTickBins : bin);
    }
    points_.reserve(nodes.size());
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    for (const auto &n : nodes) {
      const QPointF p(n.x, -n.y);
      if (points_.empty()) {
        x0 = x1 = p.x();
        y0 = y1 = p.y();
      }
      x0 = std::min(x0, p.x());
      x1 = std::max(x1, p.x());
      y0 = std::min(y0, p.y());
      y1 = std::max(y1, p.y());
      points_.push_back(p);
    }
    bounds_ = QRectF(QPointF(x0, y0), QPointF(x1, y1));
    // The dots are screen-sized: at a far zoom a dot is wider than any fixed
    // pad in metres. A tenth of the graph covers every zoom the wheel allows
    // (zoom_limits: out to four graphs across the viewport).
    pad_ = 0.1 * std::max(bounds_.width(), bounds_.height()) + 1.0;
    setFlag(ItemUsesExtendedStyleOption);
    setZValue(2);
  }
  QRectF boundingRect() const override {
    return bounds_.adjusted(-pad_, -pad_, pad_, pad_);
  }
  void paint(QPainter *p, const QStyleOptionGraphicsItem *opt,
             QWidget *) override {
    // Device-space dots of a fixed pixel size: each exposed node is mapped
    // to pixels and gets a blit of one pre-rendered antialiased disc per
    // colour. Neither a scene-space drawPoints (its point is a 1/63-unit
    // stroke, so the dot's shape follows the zoom) nor a device-space one
    // (the raster engine draws a wide cosmetic point as a sliver, round cap
    // ignored) is a dot, and thousands of drawEllipse calls cost ~50 % more.
    // The heading ticks are device-space too (a fixed length on screen),
    // blitted the same way from a sprite per colour and 5° of direction,
    // under the dots: antialiased wide drawLines cost ~35 ms a frame for
    // NewOffice's 3876 nodes, the sprites next to nothing.
    const QTransform to_device = p->worldTransform();
    const double r = kNodePx;
    const double m = std::max(2 * r, kTickPx + 2);
    const QRectF visible =
        to_device.mapRect(opt->exposedRect).adjusted(-m, -m, m, m);
    const double dpr = p->device() ? p->device()->devicePixelRatioF() : 1.0;
    const int nslots = label_slots();
    // Uniform scale (the canvas only zooms and pans), whatever the yaw.
    const double scale = std::sqrt(std::abs(to_device.determinant()));
    const bool ticks =
        heading_ticks_visible(step_, scale > 0 ? 1.0 / scale : 0.0, kTickPx);
    p->save();
    p->resetTransform();
    std::size_t begin = 0;
    while (begin < points_.size()) {
      const int scan = scans_.empty() ? 0 : scans_[begin];
      std::size_t end = begin + 1;
      while (end < points_.size() && (scans_.empty() || scans_[end] == scan))
        ++end;
      // The first scan in the chrome's muted ink, so the coloured edges read
      // on top; further scans take the --label-* scale in turn.
      const QColor colour =
          scan <= 1 ? theme().color("--color-on-chrome-muted")
                    : theme().color(QString("--label-%1").arg(scan % nslots));
      const QPixmap &dot = disc(colour, dpr);
      const QPointF half(dot.width() / dpr / 2.0, dot.height() / dpr / 2.0);
      device_.clear();
      for (std::size_t i = begin; i < end; ++i) {
        const QPointF d = to_device.map(points_[i]);
        if (!visible.contains(d))
          continue;
        device_.push_back(d);
        const int bin = ticks && i < tick_bin_.size() ? tick_bin_[i] : -1;
        if (bin >= 0) {
          const QPixmap &t = tick(colour, dpr, bin);
          p->drawPixmap(
              d - QPointF(t.width() / dpr / 2.0, t.height() / dpr / 2.0), t);
        }
      }
      for (const QPointF &d : device_)
        p->drawPixmap(d - half, dot);
      begin = end;
    }
    p->restore();
  }

    private:
  std::vector<QPointF> points_;
  std::vector<int> scans_;
  std::vector<int> tick_bin_; ///< per point; -1 = no tick
  double step_ = 0.0;
  /// Per-paint scratch, kept to avoid reallocating every frame.
  std::vector<QPointF> device_;
  /// The heading tick of direction @p bin in @p colour, centred on the
  /// node, rendered once and kept like the discs.
  const QPixmap &tick(const QColor &colour, double dpr, int bin) {
    const auto key = std::make_tuple(colour.rgba(), dpr, bin);
    auto it = ticks_.find(key);
    if (it == ticks_.end()) {
      const int side = static_cast<int>(std::ceil((2.0 * kTickPx + 4.0) * dpr));
      QPixmap pm(side, side);
      pm.setDevicePixelRatio(dpr);
      pm.fill(Qt::transparent);
      QPainter tp(&pm);
      tp.setRenderHint(QPainter::Antialiasing);
      // Toned down: in a dense overview the ticks are a texture, and the
      // dots, path and edges must still read through them.
      QPen pen(with_alpha(colour, 0.7), 1.5);
      pen.setCapStyle(Qt::RoundCap);
      tp.setPen(pen);
      const double c = side / dpr / 2.0;
      const double a = 2.0 * std::numbers::pi * bin / kTickBins;
      tp.drawLine(QPointF(c, c), QPointF(c + std::cos(a) * kTickPx,
                                         c + std::sin(a) * kTickPx));
      tp.end();
      it = ticks_.emplace(key, std::move(pm)).first;
    }
    return it->second;
  }
  std::map<std::tuple<QRgb, double, int>, QPixmap> ticks_;
  /// One node disc of @p colour at @p dpr, rendered once and kept: the
  /// cache key follows a theme switch.
  const QPixmap &disc(const QColor &colour, double dpr) {
    const auto key = std::make_pair(colour.rgba(), dpr);
    auto it = discs_.find(key);
    if (it == discs_.end()) {
      const int side = static_cast<int>(std::ceil((2.0 * kNodePx + 2.0) * dpr));
      QPixmap pm(side, side);
      pm.setDevicePixelRatio(dpr);
      pm.fill(Qt::transparent);
      QPainter dp(&pm);
      dp.setRenderHint(QPainter::Antialiasing);
      dp.setPen(Qt::NoPen);
      dp.setBrush(colour);
      const double c = side / dpr / 2.0;
      dp.drawEllipse(QPointF(c, c), kNodePx, kNodePx);
      dp.end();
      it = discs_.emplace(key, std::move(pm)).first;
    }
    return it->second;
  }

  std::map<std::pair<QRgb, double>, QPixmap> discs_;
  QRectF bounds_;
  double pad_ = 1.0;
};

/// Every edge in one item, batched by stroke: a project with a hundred
/// thousand loop edges must not be a hundred thousand scene items, rebuilt
/// on every staged edit.
class EdgesItem : public QGraphicsItem {
    public:
  struct Batch {
    QColor colour;
    double width = 1.0;
    bool dashed = false;
    std::vector<QLineF> lines;
  };
  explicit EdgesItem(std::vector<Batch> batches)
      : batches_(std::move(batches)) {
    for (const auto &b : batches_)
      for (const auto &l : b.lines)
        bounds_ = bounds_.united(QRectF(l.p1(), l.p2()).normalized());
    count_ = count();
    setZValue(1);
  }
  QRectF boundingRect() const override {
    return bounds_.adjusted(-1, -1, 1, 1);
  }
  void paint(QPainter *p, const QStyleOptionGraphicsItem *,
             QWidget *) override {
    // Antialiased lines cost ~20x aliased ones in the raster engine; past a
    // few thousand edges the difference is not visible, the frame rate is.
    p->setRenderHint(QPainter::Antialiasing, count_ < kAntialiasedEdges);
    for (const auto &b : batches_) {
      QPen pen(b.colour, b.width);
      pen.setCosmetic(true);
      if (b.dashed)
        pen.setStyle(Qt::DashLine);
      p->setPen(pen);
      p->drawLines(b.lines.data(), static_cast<int>(b.lines.size()));
    }
  }
  std::size_t count() const {
    std::size_t n = 0;
    for (const auto &b : batches_)
      n += b.lines.size();
    return n;
  }

    private:
  std::vector<Batch> batches_;
  QRectF bounds_;
  std::size_t count_ = 0;
};

/// The A or B ring, constant size on screen.
class MarkerItem : public QGraphicsItem {
    public:
  MarkerItem(QString letter, QString token, QString text_token)
      : letter_(std::move(letter)), token_(std::move(token)),
        text_token_(std::move(text_token)) {
    setFlag(ItemIgnoresTransformations);
    setZValue(5);
  }
  QRectF boundingRect() const override {
    const double r = theme().px("--space-4");
    return {-r, -r * 2.6, r * 2.6, r * 3.6};
  }
  void paint(QPainter *p, const QStyleOptionGraphicsItem *,
             QWidget *) override {
    const Theme &t = theme();
    const double r = t.px("--space-2");
    const QColor c = t.color(token_);
    p->setRenderHint(QPainter::Antialiasing);
    p->setPen(QPen(c, 2.0));
    p->setBrush(Qt::NoBrush);
    p->drawEllipse(QPointF(0, 0), r, r);
    // The letter in a chip above the ring.
    QFont f = t.font(FontRole::mono, "--font-size-xs", "--font-weight-bold");
    p->setFont(f);
    const QRectF chip(-r, -r * 2.0 - t.px("--space-4"), t.px("--space-4"),
                      t.px("--space-4"));
    p->setPen(Qt::NoPen);
    p->setBrush(c);
    p->drawRoundedRect(chip, t.px("--radius-sm"), t.px("--radius-sm"));
    p->setPen(t.color(text_token_));
    p->drawText(chip, Qt::AlignCenter, letter_);
  }

    private:
  QString letter_;
  QString token_;
  QString text_token_;
};

/// Continuous modes: mix the three ramp tokens (good -> middle -> bad).
QColor ramp_colour(int bucket) {
  const double t = (bucket + 0.5) / kRampBuckets;
  auto token = [](std::size_t i) {
    return theme().color(QString::fromLatin1(
        kRampTokens[i].data(), static_cast<int>(kRampTokens[i].size())));
  };
  const QColor a = token(0), m = token(1), b = token(2);
  auto mix = [](const QColor &x, const QColor &y, double u) {
    return QColor::fromRgbF(
        static_cast<float>(x.redF() + (y.redF() - x.redF()) * u),
        static_cast<float>(x.greenF() + (y.greenF() - x.greenF()) * u),
        static_cast<float>(x.blueF() + (y.blueF() - x.blueF()) * u));
  };
  return t < 0.5 ? mix(a, m, t * 2.0) : mix(m, b, t * 2.0 - 1.0);
}

/// The legend's colour bar: the ramp's buckets side by side.
class RampBar : public QWidget {
    public:
  RampBar() {
    setFixedSize(theme().px("--space-7") * 2, theme().px("--space-2"));
  }

    protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    const double w = static_cast<double>(width()) / kRampBuckets;
    for (int i = 0; i < kRampBuckets; ++i)
      p.fillRect(QRectF(i * w, 0, w + 0.5, height()), ramp_colour(i));
  }
};

QString da_text(const std::string &s) { return QString::fromStdString(s); }

/// A legend key for a dashed stroke (staged, rejected): the pattern the
/// canvas draws, not a filled chip — a chip reads as a colour of its own.
class DashSwatch : public QWidget {
    public:
  explicit DashSwatch(QString token) : token_(std::move(token)) {
    const int s = theme().px("--space-3");
    setFixedSize(s, s);
  }

    protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    QPen pen(theme().color(token_), 2.0);
    pen.setDashPattern({2.0, 1.5});
    p.setPen(pen);
    const double y = height() / 2.0;
    p.drawLine(QPointF(0, y), QPointF(width(), y));
  }

    private:
  QString token_;
};

} // namespace

// ----------------------------------------------------------- PoseGraphView --

PoseGraphView::PoseGraphView(QWidget *parent) : QGraphicsView(parent) {
  setObjectName("poseGraphView");
  setRenderHint(QPainter::Antialiasing);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  setFrameShape(QFrame::NoFrame);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setFocusPolicy(Qt::StrongFocus);
}

namespace {
/// The graph's rect with a margin: what fit() frames.
QRectF padded(QRectF r) {
  const double pad = std::max(r.width(), r.height()) * 0.06 + 0.5;
  return r.adjusted(-pad, -pad, pad, pad);
}
} // namespace

void PoseGraphView::set_content_rect(const QRectF &r) {
  content_ = r;
  if (!scene() || content_.isNull())
    return;
  // Let the view pan past the graph's edge a little. Set here, not only in
  // fit(): a yaw change that keeps the zoom moves the graph, and centerOn
  // clamps to the scene rect.
  const QRectF p = padded(content_);
  scene()->setSceneRect(
      p.adjusted(-p.width(), -p.height(), p.width(), p.height()));
}

void PoseGraphView::fit() {
  if (!scene() || content_.isNull())
    return;
  const QRectF r = padded(content_);
  const QRect vp = viewport()->rect();
  const FitFrame f = fit_avoiding_corner(
      {r.center().x(), r.center().y(), r.width(), r.height()}, vp.width(),
      vp.height(), reserve_.width(), reserve_.height());
  setTransform(QTransform::fromScale(f.scale, f.scale));
  centerOn(f.cx, f.cy);
  user_view_ = false;
}

void PoseGraphView::set_corner_reserve(QSize px) {
  if (px == reserve_)
    return;
  reserve_ = px;
  if (!user_view_)
    fit();
}

void PoseGraphView::resizeEvent(QResizeEvent *e) {
  QGraphicsView::resizeEvent(e);
  if (!user_view_)
    fit();
}

void PoseGraphView::zoom_on(const QPointF &scene_pos, double factor) {
  scale(factor, factor);
  centerOn(scene_pos);
  user_view_ = true;
}

double PoseGraphView::units_per_pixel() const {
  const double m = transform().m11();
  return m > 0 ? 1.0 / m : 1.0;
}

void PoseGraphView::wheelEvent(QWheelEvent *e) {
  const double want = std::pow(1.0015, e->angleDelta().y());
  const double extent = std::max(content_.width(), content_.height());
  // The short side: fit() frames by it, so the zoom-out limit (four graphs
  // across) must too, or a wide viewport starts out past the limit.
  const int px = std::min(viewport()->width(), viewport()->height());
  const double f =
      clamp_zoom_factor(units_per_pixel(), want, zoom_limits(extent, px));
  if (f != 1.0)
    scale(f, f);
  user_view_ = true;
  e->accept();
}

void PoseGraphView::mousePressEvent(QMouseEvent *e) {
  press_ = e->position().toPoint();
  QGraphicsView::mousePressEvent(e);
}

void PoseGraphView::mouseReleaseEvent(QMouseEvent *e) {
  QGraphicsView::mouseReleaseEvent(e);
  const bool click = (e->position().toPoint() - press_).manhattanLength() <=
                     theme().px("--space-1");
  if (!click)
    user_view_ = true; // a drag panned the view
  else if (e->button() == Qt::LeftButton)
    emit clicked_at(mapToScene(e->position().toPoint()), e->modifiers());
}

void PoseGraphView::keyPressEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_F && e->modifiers() == Qt::NoModifier) {
    fit();
    return;
  }
  QGraphicsView::keyPressEvent(e);
}

// ------------------------------------------------------ GraphPanel --

struct GraphPanel::Loaded {
  unsigned generation = 0;
  std::vector<NodeData> nodes;
  int unposed = 0;
  QString error;
};

GraphPanel::GraphPanel(ProjectSession &session, EdgeEditor &editor,
                       ViewSettings &view_settings, bool persist,
                       QWidget *parent)
    : QWidget(parent), session_(session), editor_(editor),
      view_settings_(view_settings), persist_(persist) {
  setObjectName("graphPanel");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);

  auto *bar = new QFrame;
  bar->setObjectName("dbToolbar");
  bar->setProperty("row", "first"); // the second row carries the hairline
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-2"),
                         t.px("--space-4"), t.px("--space-2"));
  bl->setSpacing(t.px("--space-3"));
  // The pane header already says "Graf": the toolbar is the view's
  // controls only, so it fits the left column of a 1440 px window.
  auto *seg = new QFrame;
  seg->setObjectName("segmented");
  auto *segl = new QHBoxLayout(seg);
  segl->setContentsMargins(0, 0, 0, 0);
  segl->setSpacing(0);
  planes_ = new QButtonGroup(this);
  const std::array<std::pair<const char *, const char *>, 3> planes = {
      {{"XY", "Plan, set ovenfra"},
       {"XZ", "Opstalt, set fra −y"},
       {"YZ", "Opstalt, set fra +x"}}};
  for (int i = 0; i < 3; ++i) {
    auto *b = new QPushButton(planes[i].first);
    b->setObjectName("segment");
    b->setCheckable(true);
    b->setCursor(Qt::PointingHandCursor);
    b->setProperty("position", i == 0 ? "first" : i == 2 ? "last" : "middle");
    b->setToolTip(planes[i].second);
    planes_->addButton(b, i);
    segl->addWidget(b);
  }
  bl->addWidget(seg);

  auto *drej = new CapsLabel("Drej", "fieldLabel");
  drej->ensurePolished();
  drej->setMinimumWidth(drej->sizeHint().width()); // a word, never elided
  bl->addWidget(drej);
  yaw_slider_ = new QSlider(Qt::Horizontal);
  yaw_slider_->setObjectName("yawSlider");
  yaw_slider_->setRange(-179, 180); // (-180, 180]: -180 would snap to 180
  yaw_slider_->setMinimumWidth(t.px("--layout-nav-width") / 2);
  bl->addWidget(yaw_slider_);
  yaw_spin_ = new QDoubleSpinBox;
  yaw_spin_->setObjectName("yawSpin");
  yaw_spin_->setRange(-180.0, 180.0);
  yaw_spin_->setDecimals(1);
  yaw_spin_->setSuffix(" °");
  yaw_spin_->setButtonSymbols(QAbstractSpinBox::NoButtons);
  yaw_spin_->setLocale(QLocale(QLocale::Danish, QLocale::Denmark));
  yaw_spin_->setToolTip(
      "Visningens drejning om lodret akse. Gemmes i projektet "
      "og bruges også af 3D og rux render; modellen ændres "
      "ikke.");
  // As wide as its widest value, not a default line edit.
  yaw_spin_->ensurePolished();
  yaw_spin_->setMaximumWidth(
      yaw_spin_->fontMetrics().horizontalAdvance("−180,0 °") +
      2 * t.px("--space-2") + 2 * t.px("--space-1"));
  bl->addWidget(yaw_spin_);
  auto *autob = new QPushButton(NavItem::escape_mnemonic("Auto"));
  autob->setProperty("kind", "ghost");
  autob->setCursor(Qt::PointingHandCursor);
  autob->setToolTip("Drej, så bygningens hovedretning ligger vandret — ud fra "
                    "kameraernes retninger");
  bl->addWidget(autob);
  auto *fit = new QPushButton(NavItem::escape_mnemonic("Tilpas"));
  fit->setProperty("kind", "ghost");
  fit->setCursor(Qt::PointingHandCursor);
  fit->setToolTip("Vis hele grafen (F)");
  bl->addStretch(1);
  bl->addWidget(fit);
  v->addWidget(bar);

  // A second, compact row: the first is full at 1440 px.
  auto *bar2 = new QFrame;
  bar2->setObjectName("dbToolbar");
  auto *b2 = new QHBoxLayout(bar2);
  b2->setContentsMargins(t.px("--space-4"), 0, t.px("--space-4"),
                         t.px("--space-2"));
  b2->setSpacing(t.px("--space-3"));
  auto *farv = new CapsLabel("Farv efter", "fieldLabel");
  farv->ensurePolished();
  farv->setMinimumWidth(farv->sizeHint().width());
  b2->addWidget(farv);
  colour_ = new QComboBox;
  colour_->setObjectName("edgeColour");
  for (auto m : colour_modes())
    colour_->addItem(
        da_text(colour_mode_da(m)),
        QString::fromLatin1(colour_mode_key(m).data(),
                            static_cast<int>(colour_mode_key(m).size())));
  colour_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  b2->addWidget(colour_);
  rejected_ = new QCheckBox(NavItem::escape_mnemonic("Vis afviste"));
  rejected_->setCursor(Qt::PointingHandCursor);
  rejected_->setToolTip("Afviste kanter bruges ikke af optimeringen; vist "
                        "stiplet");
  b2->addWidget(rejected_);
  b2->addStretch(1);
  v->addWidget(bar2);

  auto *stack_host = new QWidget;
  auto *stack = new QStackedLayout(stack_host);
  view_ = new PoseGraphView;
  view_->setScene(new QGraphicsScene(view_));
  // A handful of large items: a BSP index costs more than it saves.
  view_->scene()->setItemIndexMethod(QGraphicsScene::NoIndex);
  view_->setBackgroundBrush(t.color("--color-canvas"));
  stack->addWidget(view_);
  legend_ = new QFrame(view_);
  legend_->setObjectName("graphLegend");
  legend_layout_ = new QVBoxLayout(legend_);
  legend_layout_->setContentsMargins(t.px("--space-3"), t.px("--space-2"),
                                     t.px("--space-3"), t.px("--space-2"));
  legend_layout_->setSizeConstraint(QLayout::SetFixedSize);
  legend_->hide();
  // Clicks go through to the canvas: the card is a key, not a control.
  legend_->setAttribute(Qt::WA_TransparentForMouseEvents);
  view_->installEventFilter(this);
  legend_->installEventFilter(this); // its size settles after a rebuild
  empty_ = new QLabel;
  empty_->setObjectName("viewportEmpty");
  empty_->setAlignment(Qt::AlignCenter);
  empty_->setWordWrap(true);
  stack->addWidget(empty_);
  v->addWidget(stack_host, 1);

  auto *status = new QFrame;
  status->setObjectName("statusBar");
  auto *sl = new QHBoxLayout(status);
  sl->setContentsMargins(t.px("--space-4"), t.px("--space-1"),
                         t.px("--space-4"), t.px("--space-1"));
  auto *hint = new QLabel("Klik: næste af A/B · Shift+klik: B · klik på en "
                          "kant: A og B · hjul: zoom · træk: panorér · F: "
                          "tilpas");
  hint->setObjectName("statusText");
  // Wraps rather than widening Rediger's left column past a 1440 px window.
  hint->setWordWrap(true);
  sl->setSpacing(t.px("--space-4"));
  // Elides rather than widening the column (a long "ikke gemt" count).
  meta_ = new ElidedLabel(QString(), Qt::ElideRight);
  meta_->setObjectName("statusText");
  // Its natural width when there is room; it can still shrink (and elide).
  meta_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  sl->addWidget(meta_, 0, Qt::AlignTop);
  hint->setAlignment(Qt::AlignRight | Qt::AlignTop);
  sl->addWidget(hint, 1);
  v->addWidget(status);

  connect(fit, &QPushButton::clicked, view_, &PoseGraphView::fit);
  connect(view_, &PoseGraphView::clicked_at, this, &GraphPanel::on_click);
  connect(&editor_, &EdgeEditor::changed, this, [this] {
    rebuild_edges();
    update_meta();
  });
  connect(&session_, &ProjectSession::state_changed, this, &GraphPanel::reset);
  connect(&theme(), &Theme::changed, this, [this] {
    view_->setBackgroundBrush(theme().color("--color-canvas"));
    rebuild_scene();
  });

  connect(colour_, &QComboBox::currentIndexChanged, this, [this](int i) {
    if (i >= 0 && i < static_cast<int>(colour_modes().size()))
      set_colour_mode(colour_modes()[static_cast<std::size_t>(i)]);
  });
  connect(rejected_, &QCheckBox::toggled, this, &GraphPanel::set_show_rejected);
  connect(planes_, &QButtonGroup::idClicked, this,
          [this](int id) { set_plane(static_cast<ViewPlane>(id)); });
  connect(yaw_slider_, &QSlider::valueChanged, this,
          [this](int v) { view_settings_.set_yaw_deg(v); });
  connect(yaw_spin_, &QDoubleSpinBox::valueChanged, this,
          [this](double v) { view_settings_.set_yaw_deg(v); });
  connect(autob, &QPushButton::clicked, this, [this] {
    std::vector<std::array<double, 3>> f;
    f.reserve(data_.size());
    for (const auto &n : data_)
      f.push_back(n.forward);
    view_settings_.set_yaw_deg(auto_yaw_deg(f));
  });
  connect(&view_settings_, &ViewSettings::yaw_changed, this,
          &GraphPanel::set_yaw);
  ViewPlane plane = ViewPlane::xy;
  if (persist_)
    plane =
        parse_view_plane(
            QSettings().value("rediger/plane", "xy").toString().toStdString())
            .value_or(ViewPlane::xy);
  projection_.plane = plane;
  planes_->button(static_cast<int>(plane))->setChecked(true);
  if (persist_) {
    QSettings st;
    mode_ = parse_colour_mode(
                st.value("rediger/colour", "type").toString().toStdString())
                .value_or(EdgeColourMode::type);
    show_rejected_ = st.value("rediger/show_rejected", false).toBool();
  }
  {
    const QSignalBlocker b1(colour_), b2(rejected_);
    colour_->setCurrentIndex(static_cast<int>(
        std::find(colour_modes().begin(), colour_modes().end(), mode_) -
        colour_modes().begin()));
    rejected_->setChecked(show_rejected_);
  }
  reset();
  set_yaw(view_settings_.yaw_deg());
}

GraphPanel::~GraphPanel() = default;

void GraphPanel::reset() {
  ++generation_;
  loading_ = false;
  data_.clear();
  nodes_.clear();
  ticks_.clear();
  selection_ = {};
  rebuild_scene();
  update_meta();
  if (active_)
    activate();
}

void GraphPanel::activate() {
  active_ = true;
  if (!session_.is_open() || loading_ || !nodes_.empty())
    return;
  loading_ = true;
  update_meta();
  const std::string path = session_.path().toStdString();
  const unsigned gen = generation_;
  auto work = std::make_shared<BackgroundWork>();
  QPointer<GraphPanel> guard(this);
  std::thread([path, gen, guard, work]() mutable {
    auto d = std::make_shared<Loaded>();
    d->generation = gen;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      auto ids = db.sensor_frame_ids_with_scan();
      std::sort(ids.begin(), ids.end());
      for (const auto &[id, scan] : ids) {
        if (!db.has_sensor_frame_pose(id)) {
          ++d->unposed;
          continue;
        }
        const auto pose = db.sensor_frame_pose(id);
        std::array<double, 16> local{1, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 1};
        try {
          const auto &l = db.sensor_frame_intrinsics(id).local_transform;
          for (std::size_t i = 0; i < 16; ++i)
            local[i] = l[i];
        } catch (const std::exception &) {
          // no intrinsics: the pose's own z is the optical axis
        }
        d->nodes.push_back({id,
                            scan,
                            {pose[3], pose[7], pose[11]},
                            camera_forward(pose, local)});
      }
    } catch (const std::exception &e) {
      d->error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, d] {
          if (guard)
            guard->loaded(d);
        },
        Qt::QueuedConnection);
    work.reset();
  }).detach();
}

void GraphPanel::loaded(std::shared_ptr<Loaded> d) {
  if (d->generation != generation_)
    return;
  loading_ = false;
  data_ = std::move(d->nodes);
  project();
  rebuild_scene();
  if (!d->error.isEmpty())
    empty_->setText(QString("Poserne kunne ikke læses.\n%1").arg(d->error));
  update_meta();
  view_->fit();
}

void GraphPanel::project() {
  nodes_.clear();
  ticks_.clear();
  nodes_.reserve(data_.size());
  ticks_.reserve(data_.size());
  for (const auto &n : data_) {
    const Vec2 p = project_point(projection_, n.t[0], n.t[1], n.t[2]);
    nodes_.push_back({n.id, p.x, p.y});
    ticks_.push_back(heading_in_view(projection_, n.forward));
  }
  std::vector<Vec2> pts;
  std::vector<int> scans;
  pts.reserve(nodes_.size());
  scans.reserve(nodes_.size());
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    pts.push_back({nodes_[i].x, nodes_[i].y});
    scans.push_back(data_[i].scan);
  }
  step_ = median_step(pts, scans);
}

void GraphPanel::set_plane(ViewPlane plane) {
  projection_.plane = plane;
  if (planes_)
    if (auto *b = planes_->button(static_cast<int>(plane)))
      b->setChecked(true);
  if (persist_)
    QSettings().setValue("rediger/plane",
                         QString::fromLatin1(view_plane_key(plane)));
  project();
  rebuild_scene();
  view_->fit();
}

void GraphPanel::set_yaw(double deg) {
  const ViewProjection old = projection_;
  projection_.yaw_deg = deg;
  {
    const QSignalBlocker a(yaw_slider_), b(yaw_spin_);
    yaw_slider_->setValue(static_cast<int>(std::lround(deg)));
    yaw_spin_->setValue(deg);
  }
  // Zoomed or panned: the yaw is for lining the plan up close, so keep the
  // scale and the world point at the centre. Otherwise frame the graph.
  const bool keep = view_->user_view() && !data_.empty();
  Vec2 centre;
  if (keep) {
    const QPointF c = view_->mapToScene(view_->viewport()->rect().center());
    // Lifted with the graph's mean depth on the axis the plane drops.
    double depth = 0.0;
    for (const auto &n : data_)
      depth += view_depth(old, n.t[0], n.t[1], n.t[2]);
    depth /= static_cast<double>(data_.size());
    centre = reproject_after_yaw(old, deg, {c.x(), -c.y()}, depth);
  }
  project();
  rebuild_scene();
  if (keep)
    view_->centerOn(centre.x, -centre.y);
  else
    view_->fit();
}

void GraphPanel::rebuild_scene() {
  QGraphicsScene *sc = view_->scene();
  sc->clear();
  edge_items_.clear();
  nodes_item_ = nullptr;
  marker_a_ = marker_b_ = nullptr;
  auto *stack = static_cast<QStackedLayout *>(view_->parentWidget()->layout());
  if (nodes_.empty()) {
    empty_->setText(!session_.is_open() ? QString("Intet projekt åbent")
                    : loading_          ? QString("Læser poserne …")
                                        : QString("Ingen billeder med pose — "
                                                  "importér en scanning"));
    stack->setCurrentWidget(empty_);
    return;
  }
  stack->setCurrentWidget(view_);
  const Theme &t = theme();

  // The capture path, one polyline per scan, under everything.
  QPainterPath path;
  int scan = -1;
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const QPointF p(nodes_[i].x, -nodes_[i].y);
    if (data_[i].scan != scan) {
      path.moveTo(p);
      scan = data_[i].scan;
    } else {
      path.lineTo(p);
    }
  }
  QPen trail(with_alpha(t.color("--color-on-chrome-muted"), 0.45), 1.0);
  trail.setCosmetic(true);
  auto *trail_item = sc->addPath(path, trail);
  trail_item->setZValue(0);

  std::vector<int> scans;
  scans.reserve(data_.size());
  for (const auto &n : data_)
    scans.push_back(n.scan);
  nodes_item_ = new NodesItem(nodes_, std::move(scans), ticks_, step_);
  sc->addItem(nodes_item_);
  view_->set_content_rect(path.boundingRect());
  // Chrome colours, not class colours: a marker must never read as a label.
  marker_a_ = new MarkerItem("A", "--color-accent", "--color-on-accent");
  marker_b_ = new MarkerItem("B", "--color-on-chrome", "--color-canvas");
  sc->addItem(marker_a_);
  sc->addItem(marker_b_);
  rebuild_edges();
  place_markers();
}

void GraphPanel::rebuild_edges() {
  QGraphicsScene *sc = view_->scene();
  for (QGraphicsItem *it : edge_items_) {
    sc->removeItem(it);
    delete it;
  }
  edge_items_.clear();
  drawn_.clear();
  const auto all = editor_.edits().effective();
  for (const auto &v : all)
    if (edge_drawn(v, show_rejected_))
      drawn_.push_back(v);
  rebuild_legend(all);
  if (nodes_.empty())
    return;
  // nodes_ is in id order (as loaded): a binary search, not a map per edge.
  const auto at = [this](int id) -> const GraphNode * {
    const auto it =
        std::lower_bound(nodes_.begin(), nodes_.end(), id,
                         [](const GraphNode &n, int i) { return n.id < i; });
    return it != nodes_.end() && it->id == id ? &*it : nullptr;
  };
  const ColourScale scale = colour_scale(mode_, drawn_);
  const auto unknown = unknown_edge_types(drawn_);
  const int nslots = label_slots(); // `slots` is a Qt macro
  // Batched in paint order (edge_draw_order): hairlines under wide strokes,
  // dashed (staged / rejected) on top, the ramp good to bad so the worst
  // edge reads, and the highlighted edge last of all.
  std::map<EdgeDrawOrder, EdgesItem::Batch> batches;
  EdgesItem::Batch highlight{
      theme().color("--color-on-chrome"), 4.0, false, {}};
  for (const auto &v : drawn_) {
    const GraphNode *a = at(v.edge.key.from), *b = at(v.edge.key.to);
    if (!a || !b)
      continue;
    const QLineF line(a->x, -a->y, b->x, -b->y);
    const EdgeStyle st = edge_style(v, mode_, scale, unknown, nslots);
    auto &batch = batches[edge_draw_order(st)];
    if (batch.lines.empty()) {
      batch.colour = st.ramp >= 0 ? ramp_colour(st.ramp)
                                  : theme().color(da_text(st.token));
      // The reddest bucket (at or past the scale's bad end) a stroke wider:
      // one bogus loop closure among hundreds must catch the eye, and the
      // ramp's top and middle tokens are close at 2 px.
      batch.width = !st.wide ? 1.0 : st.ramp == kRampBuckets - 1 ? 3.0 : 2.0;
      batch.dashed = st.dashed;
    }
    batch.lines.push_back(line);
    if (highlight_ && v.edge.key == *highlight_)
      highlight.lines.push_back(line);
  }
  std::vector<EdgesItem::Batch> ordered;
  ordered.reserve(batches.size() + 1);
  for (auto &[k, b] : batches)
    ordered.push_back(std::move(b));
  if (!highlight.lines.empty())
    ordered.push_back(std::move(highlight));
  auto *item = new EdgesItem(std::move(ordered));
  sc->addItem(item);
  edge_items_.push_back(item);
}

void GraphPanel::rebuild_legend(const std::vector<EdgeView> &all) {
  // One body widget per build: hide the old one first, or it paints through
  // the new one until its deleteLater runs.
  while (QLayoutItem *it = legend_layout_->takeAt(0)) {
    if (QWidget *w = it->widget()) {
      w->hide();
      w->deleteLater();
    }
    delete it;
  }
  const Theme &t = theme();
  const EdgeLegend l = build_legend(mode_, all, show_rejected_, label_slots());
  auto *body = new QWidget;
  body->setObjectName("legendHost");
  auto *g = new QGridLayout(body);
  g->setContentsMargins(0, 0, 0, 0);
  g->setHorizontalSpacing(t.px("--space-2"));
  g->setVerticalSpacing(t.px("--space-1"));
  auto text = [](const QString &s, const char *name = "legendText") {
    auto *q = new QLabel(s);
    q->setObjectName(name);
    return q;
  };
  int row = 0;
  if (l.continuous && l.scale.valid) {
    const auto num = [&](double v) {
      return da_text(format_decimal_da(v, 2)) +
             (l.unit.empty() ? QString() : " " + da_text(l.unit));
    };
    auto *bar = new QHBoxLayout;
    bar->setSpacing(t.px("--space-2"));
    bar->addWidget(text(num(l.scale.lo), "legendCount"));
    bar->addWidget(new RampBar, 0, Qt::AlignVCenter);
    bar->addWidget(text(num(l.scale.hi), "legendCount"));
    bar->addStretch(1);
    g->addLayout(bar, row++, 0, 1, 3);
    g->addWidget(text(l.scale.fixed ? QString("χ²₆ 95 % → 5×, grøn til rød")
                                    : QString("5.–95. percentil af de viste")),
                 row++, 0, 1, 3);
  }
  const auto entry = [&](const QString &token, const QString &label, int n,
                         bool dashed = false) {
    g->addWidget(dashed ? static_cast<QWidget *>(new DashSwatch(token))
                        : new Swatch(token),
                 row, 0, Qt::AlignVCenter);
    g->addWidget(text(label), row, 1);
    auto *c = text(format_count(n), "legendCount");
    c->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    g->addWidget(c, row, 2);
    ++row;
  };
  if (l.continuous && l.missing > 0)
    entry("--color-text-faint", "Ingen måling", l.missing);
  for (const auto &e : l.entries)
    entry(da_text(e.token),
          da_text(e.label) + (e.dashed ? QString(" (stiplet)") : QString()),
          e.count, e.dashed);
  if (l.hidden_rejected > 0)
    g->addWidget(
        text(QString("%1 afviste skjult").arg(format_count(l.hidden_rejected)),
             "legendNote"),
        row++, 0, 1, 3);
  legend_layout_->addWidget(body);
  legend_->setVisible(row > 0 && !nodes_.empty());
  legend_->adjustSize();
  place_legend();
}

void GraphPanel::place_legend() {
  if (!legend_)
    return;
  const int m = theme().px("--space-3");
  const QRect vp = view_->viewport()->geometry();
  legend_->move(vp.left() + m, vp.bottom() - legend_->height() - m + 1);
  legend_->raise();
  // The fit zoom keeps the graph clear of the card (it used to cover the
  // lower-left nodes and take their clicks).
  view_->set_corner_reserve(
      legend_->isHidden() ? QSize() : legend_->size() + QSize(2 * m, 2 * m));
}

bool GraphPanel::eventFilter(QObject *watched, QEvent *event) {
  if ((watched == view_ || watched == legend_) &&
      event->type() == QEvent::Resize)
    place_legend();
  return QWidget::eventFilter(watched, event);
}

void GraphPanel::set_colour_mode(EdgeColourMode m) {
  mode_ = m;
  {
    const QSignalBlocker b(colour_);
    colour_->setCurrentIndex(static_cast<int>(
        std::find(colour_modes().begin(), colour_modes().end(), m) -
        colour_modes().begin()));
  }
  if (persist_)
    QSettings().setValue(
        "rediger/colour",
        QString::fromLatin1(colour_mode_key(m).data(),
                            static_cast<int>(colour_mode_key(m).size())));
  rebuild_edges();
  emit colour_mode_changed(m);
}

void GraphPanel::set_show_rejected(bool on) {
  show_rejected_ = on;
  {
    const QSignalBlocker b(rejected_);
    rejected_->setChecked(on);
  }
  if (persist_)
    QSettings().setValue("rediger/show_rejected", on);
  rebuild_edges();
  update_meta();
}

void GraphPanel::set_highlight(std::optional<EdgeKey> key) {
  highlight_ = std::move(key);
  rebuild_edges();
}

void GraphPanel::place_markers() {
  auto place = [this](QGraphicsItem *m, int id) {
    if (!m)
      return;
    const auto it =
        std::find_if(nodes_.begin(), nodes_.end(),
                     [id](const GraphNode &n) { return n.id == id; });
    m->setVisible(it != nodes_.end());
    if (it != nodes_.end())
      m->setPos(it->x, -it->y);
  };
  place(marker_a_, a_);
  place(marker_b_, b_);
}

void GraphPanel::set_pair(int a_id, int b_id) {
  a_ = a_id;
  b_ = b_id;
  place_markers();
}

void GraphPanel::on_click(const QPointF &pos, Qt::KeyboardModifiers mods) {
  const double tol = kHitPx * view_->units_per_pixel();
  const int n = nearest_node(nodes_, pos.x(), -pos.y(), tol);
  if (n >= 0) {
    click_node(nodes_[static_cast<std::size_t>(n)].id,
               mods.testFlag(Qt::ShiftModifier));
    return;
  }
  std::vector<GraphEdge> plain;
  plain.reserve(drawn_.size());
  for (const auto &v : drawn_)
    plain.push_back({v.edge.key.from, v.edge.key.to});
  if (const int e = nearest_edge(nodes_, plain, pos.x(), -pos.y(), tol);
      e >= 0) {
    const EdgeKey key = drawn_[static_cast<std::size_t>(e)].edge.key;
    select_edge(key);
    emit edge_clicked(key);
  }
}

void GraphPanel::click_node(int id, bool as_b) {
  const auto it = std::find_if(nodes_.begin(), nodes_.end(),
                               [id](const GraphNode &n) { return n.id == id; });
  if (it == nodes_.end())
    return;
  if (highlight_)
    set_highlight(std::nullopt); // the inspector now shows the node
  Selection s;
  s.kind = "Node";
  s.title = QString("Billede %1").arg(id);
  s.subtitle = QString("Billede i posegrafen");
  // World coordinates, not the projected ones: the yaw is display only.
  const NodeData &n = data_[static_cast<std::size_t>(it - nodes_.begin())];
  SelectionSection p;
  p.title = "Pose";
  const auto row = [&p](const char *key, const QString &value) {
    SelectionRow r;
    r.key = key;
    r.value = value;
    p.rows.push_back(r);
  };
  row("X", da().toString(n.t[0], 'f', 2) + " m");
  row("Y", da().toString(n.t[1], 'f', 2) + " m");
  row("Z", da().toString(n.t[2], 'f', 2) + " m");
  row("Scanning", QString::number(n.scan));
  row("Kanter", QString::number(editor_.edits().degree(id)));
  s.sections.push_back(p);
  selection_ = s;
  emit selection_changed(selection_);
  emit frame_requested(id, as_b);
}

bool GraphPanel::click_edge(const EdgeKey &key) { return select_edge(key); }

bool GraphPanel::select_edge(const EdgeKey &key) {
  const auto all = editor_.edits().effective();
  const auto it =
      std::find_if(all.begin(), all.end(),
                   [&key](const EdgeView &v) { return v.edge.key == key; });
  if (it == all.end())
    return false;
  const EdgeView v = *it;
  Selection s;
  s.kind = "Kant";
  s.title = QString("%1 → %2").arg(key.from).arg(key.to);
  s.subtitle = da_text(edge_type_da(key.type));
  s.pill = da_text(edge_status_da(v.edge.status));
  s.pill_tone = v.edge.status == "confirmed"  ? "good"
                : v.edge.status == "rejected" ? "crit"
                                              : "outline";
  if (v.pending != EdgeView::Pending::none) {
    s.pill += " · ikke gemt";
    s.pill_tone = "warn";
  }
  SelectionSection p;
  p.title = "Kant";
  const auto cell = [&v](EdgeColumn c) {
    return da_text(edge_cell_text(v, c));
  };
  // "0,12 m / 3,4 °", or one dash when the edge has no such metric.
  const auto pair = [&cell](EdgeColumn m, EdgeColumn deg) {
    const QString a = cell(m), b = cell(deg);
    return a == "—" && b == "—" ? QString("—") : a + " m / " + b + " °";
  };
  const auto row = [&p](const char *key, const QString &value,
                        SelectionRow::Style style = SelectionRow::Style::mono) {
    SelectionRow r;
    r.key = key;
    r.value = value;
    r.style = style;
    p.rows.push_back(r);
  };
  row("Type", cell(EdgeColumn::type), SelectionRow::Style::value);
  row("Kilde", cell(EdgeColumn::source), SelectionRow::Style::value);
  row("Residual", cell(EdgeColumn::residual));
  row("Afvigelse", pair(EdgeColumn::deviation_m, EdgeColumn::deviation_deg));
  row("Frøafvigelse", pair(EdgeColumn::seed_m, EdgeColumn::seed_deg));
  row("Vægt", std::isnan(v.edge.weight) ? QString("—")
                                        : da().toString(v.edge.weight, 'g', 4));
  s.sections.push_back(p);
  selection_ = s;
  set_highlight(key);
  emit selection_changed(selection_);
  emit pair_requested(key.from, key.to);
  return true;
}

void GraphPanel::update_meta() {
  if (!session_.is_open()) {
    meta_->set_full_text(QString());
    return;
  }
  if (loading_) {
    meta_->set_full_text("Læser poserne …");
    return;
  }
  QString m = QString("%1 billeder · %2 kanter")
                  .arg(format_count(nodes_.size()))
                  .arg(format_count(static_cast<qulonglong>(drawn_.size())));
  if (const int n = editor_.pending())
    m += QString(" · %1 ikke gemt").arg(n);
  meta_->set_full_text(m);
}

} // namespace rux::qt
