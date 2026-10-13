// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/PoseGraphWorkspace.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/graph_view.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QApplication>
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
#include <QStackedLayout>
#include <QStyleOptionGraphicsItem>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <map>
#include <thread>

namespace rux::qt {

namespace {

const QLocale &da() {
  static const QLocale l(QLocale::Danish, QLocale::Denmark);
  return l;
}

/// Screen pixels of a node's radius and of the click tolerance.
constexpr double kNodePx = 1.6;
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
  NodesItem(const std::vector<GraphNode> &nodes, const std::vector<int> &scans)
      : scans_(scans) {
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
    const QTransform to_device = p->worldTransform();
    const double r = kNodePx;
    const QRectF visible = to_device.mapRect(opt->exposedRect)
                               .adjusted(-2 * r, -2 * r, 2 * r, 2 * r);
    const double dpr = p->device() ? p->device()->devicePixelRatioF() : 1.0;
    const int nslots = label_slots();
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
      const QPixmap &dot = disc(
          scan <= 1 ? theme().color("--color-on-chrome-muted")
                    : theme().color(QString("--label-%1").arg(scan % nslots)),
          dpr);
      const QPointF half(dot.width() / dpr / 2.0, dot.height() / dpr / 2.0);
      for (std::size_t i = begin; i < end; ++i) {
        const QPointF d = to_device.map(points_[i]);
        if (visible.contains(d))
          p->drawPixmap(d - half, dot);
      }
      begin = end;
    }
    p->restore();
  }

    private:
  std::vector<QPointF> points_;
  std::vector<int> scans_;
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

void PoseGraphView::fit() {
  if (!scene() || content_.isNull())
    return;
  QRectF r = content_;
  const double pad = std::max(r.width(), r.height()) * 0.06 + 0.5;
  r.adjust(-pad, -pad, pad, pad);
  // Let the view pan past the graph's edge a little.
  scene()->setSceneRect(
      r.adjusted(-r.width(), -r.height(), r.width(), r.height()));
  fitInView(r, Qt::KeepAspectRatio);
  user_view_ = false;
}

void PoseGraphView::resizeEvent(QResizeEvent *e) {
  QGraphicsView::resizeEvent(e);
  if (!user_view_)
    fit();
}

double PoseGraphView::units_per_pixel() const {
  const double m = transform().m11();
  return m > 0 ? 1.0 / m : 1.0;
}

void PoseGraphView::wheelEvent(QWheelEvent *e) {
  const double want = std::pow(1.0015, e->angleDelta().y());
  const double extent = std::max(content_.width(), content_.height());
  const int px = std::max(viewport()->width(), viewport()->height());
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

// ------------------------------------------------------ PoseGraphWorkspace --

struct PoseGraphWorkspace::Loaded {
  unsigned generation = 0;
  std::vector<GraphNode> nodes;
  std::vector<int> scans;
  int unposed = 0;
  QString error;
};

PoseGraphWorkspace::PoseGraphWorkspace(ProjectSession &session,
                                       EdgeEditor &editor, QWidget *parent)
    : QWidget(parent), session_(session), editor_(editor) {
  setObjectName("poseGraphWorkspace");
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
  bl->addWidget(new CapsLabel("Posegraf", "panelTitle"));
  meta_ = new QLabel;
  meta_->setObjectName("toolbarMeta");
  bl->addWidget(meta_, 1);
  // Edge legend: the stroke colours of the canvas.
  legend_ = new QWidget;
  auto *lg = new QHBoxLayout(legend_);
  lg->setContentsMargins(0, 0, 0, 0);
  lg->setSpacing(t.px("--space-3"));
  for (const auto &[token, name] : std::vector<std::pair<QString, QString>>{
           {"--color-text-faint", "Odometri"},
           {"--color-accent", "Løkkelukning"},
           {"--color-star", "Panorama"},
           {"--color-status-failed", "Slettes"}}) {
    auto *item = new QWidget;
    auto *il = new QHBoxLayout(item);
    il->setContentsMargins(0, 0, 0, 0);
    il->setSpacing(t.px("--space-1"));
    il->addWidget(new Swatch(token));
    auto *l = new QLabel(name);
    l->setObjectName("legendText");
    il->addWidget(l);
    lg->addWidget(item);
  }
  bl->addWidget(legend_);
  auto *fit = new QPushButton(NavItem::escape_mnemonic("Tilpas"));
  fit->setProperty("kind", "ghost");
  fit->setCursor(Qt::PointingHandCursor);
  fit->setToolTip("Vis hele grafen (F)");
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
  auto *hint = new QLabel("Klik på en node: billede A · Shift+klik: billede B "
                          "· klik på en kant: A og B · hjul: zoom · træk: "
                          "panorér");
  hint->setObjectName("statusText");
  sl->addWidget(hint, 1);
  v->addWidget(status);

  connect(fit, &QPushButton::clicked, view_, &PoseGraphView::fit);
  connect(view_, &PoseGraphView::clicked_at, this,
          &PoseGraphWorkspace::on_click);
  connect(&editor_, &EdgeEditor::changed, this, [this] {
    rebuild_edges();
    update_meta();
  });
  connect(&session_, &ProjectSession::state_changed, this,
          &PoseGraphWorkspace::reset);
  connect(&theme(), &Theme::changed, this, [this] {
    view_->setBackgroundBrush(theme().color("--color-canvas"));
    rebuild_scene();
  });
  reset();
}

PoseGraphWorkspace::~PoseGraphWorkspace() = default;

void PoseGraphWorkspace::reset() {
  ++generation_;
  loading_ = false;
  nodes_.clear();
  scan_of_.clear();
  selection_ = {};
  rebuild_scene();
  update_meta();
  if (active_)
    activate();
}

void PoseGraphWorkspace::activate() {
  active_ = true;
  if (!session_.is_open() || loading_ || !nodes_.empty())
    return;
  loading_ = true;
  update_meta();
  const std::string path = session_.path().toStdString();
  const unsigned gen = generation_;
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PoseGraphWorkspace> guard(this);
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
        d->nodes.push_back({id, pose[3], pose[7]});
        d->scans.push_back(scan);
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

void PoseGraphWorkspace::loaded(std::shared_ptr<Loaded> d) {
  if (d->generation != generation_)
    return;
  loading_ = false;
  nodes_ = std::move(d->nodes);
  scan_of_ = std::move(d->scans);
  rebuild_scene();
  if (!d->error.isEmpty())
    empty_->setText(QString("Poserne kunne ikke læses.\n%1").arg(d->error));
  update_meta();
  view_->fit();
}

void PoseGraphWorkspace::rebuild_scene() {
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
    if (scan_of_[i] != scan) {
      path.moveTo(p);
      scan = scan_of_[i];
    } else {
      path.lineTo(p);
    }
  }
  QPen trail(with_alpha(t.color("--color-on-chrome-muted"), 0.45), 1.0);
  trail.setCosmetic(true);
  auto *trail_item = sc->addPath(path, trail);
  trail_item->setZValue(0);

  nodes_item_ = new NodesItem(nodes_, scan_of_);
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

void PoseGraphWorkspace::rebuild_edges() {
  QGraphicsScene *sc = view_->scene();
  for (QGraphicsItem *it : edge_items_) {
    sc->removeItem(it);
    delete it;
  }
  edge_items_.clear();
  edges_.clear();
  const auto &edits = editor_.edits();
  for (const auto &e : edits.base()) {
    DrawnEdge d;
    d.edge = {e.key.from, e.key.to};
    d.type = e.key.type;
    d.residual = e.residual;
    d.weight = e.weight;
    edges_.push_back(d);
  }
  for (const auto &op : edits.ops()) {
    if (op.kind == PendingEdgeEdits::Op::Kind::add) {
      DrawnEdge d;
      d.edge = {op.edge.key.from, op.edge.key.to};
      d.type = op.edge.key.type;
      d.pending_add = true;
      d.weight = op.edge.weight;
      edges_.push_back(d);
    } else {
      for (auto &d : edges_)
        if (!d.pending_add && d.edge.from == op.edge.key.from &&
            d.edge.to == op.edge.key.to && d.type == op.edge.key.type)
          d.pending_delete = true;
    }
  }
  if (nodes_.empty())
    return;
  std::map<int, QPointF> at;
  for (const auto &n : nodes_)
    at[n.id] = QPointF(n.x, -n.y);
  // Batches in draw order: odometry under the rest, staged edits on top.
  std::vector<EdgesItem::Batch> batches = {
      {"--color-text-faint", 1.0, false, {}},    // odometry
      {"--color-accent", 2.0, false, {}},        // loop closure
      {"--color-star", 2.0, false, {}},          // panorama
      {"--color-accent", 2.0, true, {}},         // staged addition
      {"--color-status-failed", 2.0, true, {}}}; // staged deletion
  for (const auto &d : edges_) {
    const auto a = at.find(d.edge.from), b = at.find(d.edge.to);
    if (a == at.end() || b == at.end())
      continue;
    const std::size_t k = d.pending_delete           ? 4
                          : d.pending_add            ? 3
                          : d.type == "loop_closure" ? 1
                          : d.type == "panorama"     ? 2
                                                     : 0;
    batches[k].lines.emplace_back(a->second, b->second);
  }
  auto *item = new EdgesItem(std::move(batches));
  view_->scene()->addItem(item);
  edge_items_.push_back(item);
}

void PoseGraphWorkspace::place_markers() {
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

void PoseGraphWorkspace::set_pair(int a_id, int b_id) {
  a_ = a_id;
  b_ = b_id;
  place_markers();
}

void PoseGraphWorkspace::on_click(const QPointF &pos,
                                  Qt::KeyboardModifiers mods) {
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

void PoseGraphWorkspace::click_node(int id, bool as_b) {
  const auto it = std::find_if(nodes_.begin(), nodes_.end(),
                               [id](const GraphNode &n) { return n.id == id; });
  if (it == nodes_.end())
    return;
  Selection s;
  s.kind = "Node";
  s.title = QString("Billede %1").arg(id);
  s.subtitle = as_b ? QString("Valgt som B i Database")
                    : QString("Valgt som A i Database");
  SelectionSection p{"Pose", {}, {}};
  p.rows.push_back({"X", da().toString(it->x, 'f', 2) + " m"});
  p.rows.push_back({"Y", da().toString(it->y, 'f', 2) + " m"});
  p.rows.push_back(
      {"Scanning",
       QString::number(
           scan_of_[static_cast<std::size_t>(it - nodes_.begin())])});
  p.rows.push_back({"Kanter", QString::number(editor_.edits().degree(id))});
  s.sections.push_back(p);
  selection_ = s;
  emit selection_changed(selection_);
  emit frame_requested(id, as_b);
}

void PoseGraphWorkspace::click_edge(int from, int to) {
  for (const auto &d : edges_) {
    if (d.edge.from != from || d.edge.to != to)
      continue;
    Selection s;
    s.kind = "Kant";
    s.title = QString("%1 → %2").arg(from).arg(to);
    s.subtitle = edge_type_da(d.type);
    if (d.pending_add) {
      s.pill = "Ny · ikke gemt";
      s.pill_tone = "warn";
    } else if (d.pending_delete) {
      s.pill = "Slettes";
      s.pill_tone = "crit";
    }
    SelectionSection p{"Kant", {}, {}};
    p.rows.push_back(
        {"Type", edge_type_da(d.type), SelectionRow::Style::value});
    p.rows.push_back({"Residual", da().toString(d.residual, 'g', 4)});
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

void PoseGraphWorkspace::update_meta() {
  if (!session_.is_open()) {
    meta_->setText(QString());
    return;
  }
  if (loading_) {
    meta_->setText("Læser poserne …");
    return;
  }
  int stored = 0, adds = 0, dels = 0;
  for (const auto &d : edges_) {
    stored += !d.pending_add;
    adds += d.pending_add;
    dels += d.pending_delete;
  }
  QString m = QString("%1 billeder · %2 kanter")
                  .arg(format_count(nodes_.size()))
                  .arg(format_count(static_cast<qulonglong>(stored)));
  if (adds || dels)
    m += QString(" · %1 ny, %2 slettes (ikke gemt)").arg(adds).arg(dels);
  meta_->setText(m);
}

} // namespace rux::qt
