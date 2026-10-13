// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/GraphPanel.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/ViewSettings.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/graph_view.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>

#include <QApplication>
#include <QButtonGroup>
#include <QDoubleSpinBox>
#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
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
    QString token;
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
      QPen pen(theme().color(b.token), b.width);
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

QString edge_type_da(const std::string &type) {
  if (type == "odometry")
    return "Odometri";
  if (type == "loop_closure")
    return "Løkkelukning";
  if (type == "panorama")
    return "Panorama";
  return QString::fromStdString(type);
}

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
  fitInView(padded(content_), Qt::KeepAspectRatio);
  user_view_ = false;
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

  auto *stack_host = new QWidget;
  auto *stack = new QStackedLayout(stack_host);
  view_ = new PoseGraphView;
  view_->setScene(new QGraphicsScene(view_));
  // A handful of large items: a BSP index costs more than it saves.
  view_->scene()->setItemIndexMethod(QGraphicsScene::NoIndex);
  view_->setBackgroundBrush(t.color("--color-canvas"));
  stack->addWidget(view_);
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
  edges_.clear();
  const auto &edits = editor_.edits();
  for (const auto &v : edits.effective()) {
    if (v.edge.status == "rejected")
      continue; // Task 10 adds the "Vis afviste" toggle
    DrawnEdge d;
    d.edge = {v.edge.key.from, v.edge.key.to};
    d.type = v.edge.key.type;
    d.pending_add = v.pending != EdgeView::Pending::none;
    d.residual = v.edge.residual;
    d.weight = v.edge.weight;
    edges_.push_back(d);
  }
  if (nodes_.empty())
    return;
  std::map<int, QPointF> at;
  for (const auto &n : nodes_)
    at[n.id] = QPointF(n.x, -n.y);
  // Batches in draw order: odometry under the rest, staged edits on top.
  std::vector<EdgesItem::Batch> batches = {
      {"--color-text-faint", 1.0, false, {}}, // odometry
      {"--color-accent", 2.0, false, {}},     // loop closure
      {"--color-star", 2.0, false, {}},       // panorama
      {"--color-accent", 2.0, true, {}}};     // staged (not saved)
  for (const auto &d : edges_) {
    const auto a = at.find(d.edge.from), b = at.find(d.edge.to);
    if (a == at.end() || b == at.end())
      continue;
    const std::size_t k = d.pending_add              ? 3
                          : d.type == "loop_closure" ? 1
                          : d.type == "panorama"     ? 2
                                                     : 0;
    batches[k].lines.emplace_back(a->second, b->second);
  }
  auto *item = new EdgesItem(std::move(batches));
  view_->scene()->addItem(item);
  edge_items_.push_back(item);
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
  for (const auto &d : edges_)
    plain.push_back(d.edge);
  const int e = nearest_edge(nodes_, plain, pos.x(), -pos.y(), tol);
  if (e >= 0) {
    const auto &d = edges_[static_cast<std::size_t>(e)];
    click_edge(d.edge.from, d.edge.to);
  }
}

void GraphPanel::click_node(int id, bool as_b) {
  const auto it = std::find_if(nodes_.begin(), nodes_.end(),
                               [id](const GraphNode &n) { return n.id == id; });
  if (it == nodes_.end())
    return;
  Selection s;
  s.kind = "Node";
  s.title = QString("Billede %1").arg(id);
  s.subtitle = QString("Billede i posegrafen");
  // World coordinates, not the projected ones: the yaw is display only.
  const NodeData &n = data_[static_cast<std::size_t>(it - nodes_.begin())];
  SelectionSection p{"Pose", {}, {}};
  p.rows.push_back({"X", da().toString(n.t[0], 'f', 2) + " m"});
  p.rows.push_back({"Y", da().toString(n.t[1], 'f', 2) + " m"});
  p.rows.push_back({"Z", da().toString(n.t[2], 'f', 2) + " m"});
  p.rows.push_back({"Scanning", QString::number(n.scan)});
  p.rows.push_back({"Kanter", QString::number(editor_.edits().degree(id))});
  s.sections.push_back(p);
  selection_ = s;
  emit selection_changed(selection_);
  emit frame_requested(id, as_b);
}

void GraphPanel::click_edge(int from, int to) {
  for (const auto &d : edges_) {
    if (d.edge.from != from || d.edge.to != to)
      continue;
    Selection s;
    s.kind = "Kant";
    s.title = QString("%1 → %2").arg(from).arg(to);
    s.subtitle = edge_type_da(d.type);
    if (d.pending_add) {
      s.pill = "Ikke gemt";
      s.pill_tone = "warn";
    }
    SelectionSection p{"Kant", {}, {}};
    p.rows.push_back(
        {"Type", edge_type_da(d.type), SelectionRow::Style::value});
    p.rows.push_back({"Residual", std::isnan(d.residual)
                                      ? QString("—")
                                      : da().toString(d.residual, 'g', 4)});
    p.rows.push_back({"Vægt", std::isnan(d.weight)
                                  ? QString("—")
                                  : da().toString(d.weight, 'g', 4)});
    s.sections.push_back(p);
    selection_ = s;
    emit selection_changed(selection_);
    emit pair_requested(from, to);
    return;
  }
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
                  .arg(format_count(static_cast<qulonglong>(edges_.size())));
  if (const int n = editor_.pending())
    m += QString(" · %1 ikke gemt").arg(n);
  meta_->set_full_text(m);
}

} // namespace rux::qt
