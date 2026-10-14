// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/SceneView.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/ViewSettings.hpp>
#include <rux_qt/Viewer3DWorkspace.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/label_semantics.hpp>
#include <reusex/visualize/scene.hpp>

#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QPointer>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkCellArray.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <algorithm>
#include <cstring>
#include <map>
#include <set>
#include <thread>

namespace rux::qt {

namespace vis = reusex::visualize;
using vis::Layer;

namespace {

QString qs(const std::string &s) { return QString::fromStdString(s); }

const QLocale &da() {
  static const QLocale l(QLocale::Danish, QLocale::Denmark);
  return l;
}

/// Points drawn per point layer, and how many while the camera moves.
constexpr std::size_t kMaxPoints = 4'000'000;
constexpr std::size_t kCoarsePoints = 300'000;

bool is_point_layer(Layer l) {
  return l == Layer::cloud || l == Layer::labels || l == Layer::planes ||
         l == Layer::rooms || l == Layer::instances;
}

/// The point layers "Farv efter" offers, in that order, with Danish names.
struct ColourChoice {
  Layer layer;
  const char *cloud; ///< the label cloud ("" for stored RGB)
  const char *name;
  const char *item; ///< legend name of label N: "Plan %1"
};
constexpr ColourChoice kColours[] = {
    {Layer::cloud, "", "Lagret farve", ""},
    {Layer::labels, "labels", "Mærkater (semantik)", "Klasse %1"},
    {Layer::planes, "planes", "Planer", "Plan %1"},
    {Layer::rooms, "rooms", "Rum", "Rum %1"},
    {Layer::instances, "instances", "Instanser", "Instans %1"},
};

const ColourChoice *colour_choice(Layer l) {
  for (const auto &c : kColours)
    if (c.layer == l)
      return &c;
  return nullptr;
}

std::array<std::uint8_t, 3> rgb(const QColor &c) {
  return {static_cast<std::uint8_t>(c.red()),
          static_cast<std::uint8_t>(c.green()),
          static_cast<std::uint8_t>(c.blue())};
}

/// The theme's --label-* scale, as the scene builder takes it.
vis::LabelPalette theme_palette() {
  const Theme &t = theme();
  bool ok = false;
  int n = t.value("--label-count").toInt(&ok);
  if (!ok || n <= 0)
    n = 8;
  vis::LabelPalette p;
  for (int i = 0; i < n; ++i)
    p.colors.push_back(rgb(t.color(QString("--label-%1").arg(i))));
  p.unlabeled = rgb(t.color("--label-unlabeled"));
  // NOT t.color("--label-invalid"): that token legitimately differs between
  // the Qt client's light and dark theme (picked for a light legend panel),
  // but the 3D canvas this palette paints onto is near-black in both
  // (tokens.css). Reuse the headless renderer's always-dark constant instead,
  // so an invalid point recedes here exactly as it does in `rux render`
  // (integration review finding 3, same fix as the web viewport).
  p.invalid = vis::default_label_palette().invalid;
  return p;
}

QString metres(double v) { return da().toString(v, 'f', 2) + " m"; }

} // namespace

QString label_swatch_token(std::uint32_t label, int nslots) {
  const int slot = vis::label_palette_slot(
      label, static_cast<std::size_t>(std::max(nslots, 1)));
  if (slot == vis::kInvalidSlot)
    return "--label-invalid";
  if (slot < 0)
    return "--label-unlabeled";
  return QString("--label-%1").arg(slot);
}

struct Viewer3DWorkspace::LayerState {
  bool loading = false;
  QString error;
  std::vector<vtkSmartPointer<vtkActor>> actors;
  vtkSmartPointer<vtkActor> coarse;
  vis::SceneInfo info;
  std::vector<std::pair<std::uint32_t, std::size_t>> counts; ///< by count
  std::map<int, std::string> names;
  std::size_t items = 0;
};

struct Viewer3DWorkspace::BuildResult {
  unsigned generation = 0;
  Layer layer = Layer::cloud;
  vtkSmartPointer<vtkRenderer> scratch;
  vis::SceneInfo info;
  std::vector<std::pair<std::uint32_t, std::size_t>> counts;
  std::map<int, std::string> names;
  QString error;
};

Viewer3DWorkspace::Viewer3DWorkspace(ProjectSession &session,
                                     ViewSettings &view_settings,
                                     bool interactive, QWidget *parent)
    : QWidget(parent), session_(session), view_settings_(view_settings),
      interactive_(interactive) {
  setObjectName("viewerWorkspace");
  const Theme &t = theme();
  auto *body = new QHBoxLayout(this);
  body->setContentsMargins(0, 0, 0, 0);
  body->setSpacing(0);

  // ---- layer panel
  auto *left = new QFrame;
  left->setObjectName("treePane");
  left->setFixedWidth(t.px("--layout-panel-width"));
  auto *ll = new QVBoxLayout(left);
  ll->setContentsMargins(0, 0, 0, 0);
  auto *scroll = new QScrollArea;
  scroll->setObjectName("layerScroll");
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  panel_ = new QWidget;
  panel_->setObjectName("layerPanel");
  panel_layout_ = new QVBoxLayout(panel_);
  scroll->setWidget(panel_);
  ll->addWidget(scroll);
  body->addWidget(left);

  // ---- canvas with its toolbar and status line
  auto *centre = new QVBoxLayout;
  centre->setSpacing(0);
  auto *bar = new QFrame;
  bar->setObjectName("dbToolbar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-2"),
                         t.px("--space-4"), t.px("--space-2"));
  bl->setSpacing(t.px("--space-3"));
  bl->addWidget(new CapsLabel("Visning", "panelTitle"));
  presets_ = new QFrame;
  presets_->setObjectName("segmented");
  auto *pl = new QHBoxLayout(presets_);
  pl->setContentsMargins(0, 0, 0, 0);
  pl->setSpacing(0);
  auto *group = new QButtonGroup(presets_);
  struct Preset {
    vis::ViewPreset view;
    const char *name;
    const char *tip;
  };
  const Preset presets[] = {
      {vis::ViewPreset::orbit, "Perspektiv",
       "Skråt ovenfra (rux render "
       "--view orbit)"},
      {vis::ViewPreset::plan, "Plan", "Ovenfra med snit — en plantegning"},
      {vis::ViewPreset::top, "Top", "Lige ovenfra, uden snit"},
      {vis::ViewPreset::front, "Facade", "Opstalt langs +Y"},
  };
  for (int i = 0; i < 4; ++i) {
    auto *b = new QPushButton(NavItem::escape_mnemonic(presets[i].name));
    b->setObjectName("segment");
    b->setCheckable(true);
    b->setCursor(Qt::PointingHandCursor);
    b->setToolTip(presets[i].tip);
    b->setProperty("position", i == 0 ? "first" : i == 3 ? "last" : "middle");
    b->setChecked(presets[i].view == preset_);
    group->addButton(b, i);
    pl->addWidget(b);
    const vis::ViewPreset v = presets[i].view;
    connect(b, &QPushButton::clicked, this, [this, v] { set_view(v); });
  }
  bl->addWidget(presets_);
  bl->addStretch(1);
  auto *reset_cam = new QPushButton(NavItem::escape_mnemonic("Nulstil kamera"));
  reset_cam->setProperty("kind", "ghost");
  reset_cam->setCursor(Qt::PointingHandCursor);
  reset_cam->setToolTip("Tilpas visningen til scenen igen");
  connect(reset_cam, &QPushButton::clicked, this, [this] { frame_camera(); });
  bl->addWidget(reset_cam);
  centre->addWidget(bar);

  view_ = new SceneView(interactive);
  centre->addWidget(view_, 1);

  auto *status = new QFrame;
  status->setObjectName("statusBar");
  auto *sl = new QHBoxLayout(status);
  sl->setContentsMargins(t.px("--space-4"), t.px("--space-1"),
                         t.px("--space-4"), t.px("--space-1"));
  sl->setSpacing(t.px("--space-3"));
  status_ = new QLabel;
  status_->setObjectName("statusText");
  sl->addWidget(status_, 1);
  mode_ = new QLabel(interactive ? "Træk: drej · højre: zoom · midt: panorér · "
                                   "klik: vælg punkt"
                                 : "VTK · EGL offscreen");
  mode_->setObjectName("statusMono");
  sl->addWidget(mode_);
  centre->addWidget(status);
  body->addLayout(centre, 1);

  connect(view_, &SceneView::clicked, this,
          [this](const QPoint &p) { pick(p); });
  connect(view_, &SceneView::rendered, this, &Viewer3DWorkspace::update_status);
  connect(view_, &SceneView::interacted, this,
          [this] { camera_moved_ = true; });
  connect(view_, &SceneView::resized, this, [this] {
    if (camera_placed_ && !camera_moved_)
      frame_camera();
  });
  connect(&session_, &ProjectSession::state_changed, this,
          &Viewer3DWorkspace::reset);
  // A yaw change turns the default camera, not one the user has moved.
  connect(&view_settings_, &ViewSettings::yaw_changed, this, [this] {
    if (camera_placed_ && !camera_moved_)
      frame_camera();
  });
  // Colours read from tokens at build time follow a theme switch.
  connect(&theme(), &Theme::changed, this, [this] {
    auto paint = [](vtkActor *a, const QColor &c) {
      if (a)
        a->GetProperty()->SetColor(c.redF(), c.greenF(), c.blueF());
    };
    const std::pair<Layer, const char *> tokens[] = {
        {Layer::frustums, "--color-on-chrome-muted"},
        {Layer::panoramas, "--color-star"}};
    for (const auto &[layer, token] : tokens)
      if (auto it = layers_.find(static_cast<int>(layer)); it != layers_.end())
        for (const auto &a : it->second->actors)
          paint(a, theme().color(token));
    paint(static_cast<vtkActor *>(marker_), theme().color("--color-accent"));
    view_->request_render();
  });
  reset();
}

Viewer3DWorkspace::~Viewer3DWorkspace() = default;

bool Viewer3DWorkspace::has_data(Layer l) const {
  const reusex::ProjectDB *db = session_.db();
  if (!db)
    return false;
  const auto &s = session_.summary();
  switch (l) {
  case Layer::cloud:
    return db->has_point_cloud("cloud");
  case Layer::labels:
  case Layer::planes:
  case Layer::rooms:
  case Layer::instances:
    return db->has_point_cloud("cloud") &&
           db->has_point_cloud(colour_choice(l)->cloud);
  case Layer::mesh:
    return db->has_mesh("mesh");
  case Layer::components:
    return s.components.total_count > 0;
  case Layer::frustums:
    return s.sensor_frames.total_count > 0;
  case Layer::panoramas:
    return s.panoramic_images.total_count > 0;
  }
  return false;
}

void Viewer3DWorkspace::reset() {
  ++generation_;
  view_->clear_scene();
  layers_.clear();
  marker_ = nullptr;
  camera_placed_ = false;
  camera_moved_ = false;
  cut_height_m_.reset();
  selection_ = {};
  visible_.clear();
  visible_[static_cast<int>(Layer::cloud)] = true;
  visible_[static_cast<int>(Layer::panoramas)] = true;
  rebuild_panel();
  if (!session_.is_open()) {
    view_->set_message(session_.state() == ProjectSession::State::loading
                           ? QString("Åbner projektet …")
                           : QString("Intet projekt åbent"));
  } else if (!has_data(Layer::cloud) && !has_data(Layer::mesh)) {
    view_->set_message("Ingen punktsky endnu — kør rux create clouds");
  } else if (!view_->unavailable_reason().isEmpty()) {
    view_->set_message(view_->unavailable_reason());
  } else {
    view_->set_message(active_ ? QString("Henter punktskyen …") : QString());
  }
  update_status();
  emit selection_changed(selection_);
  if (active_)
    activate();
}

void Viewer3DWorkspace::activate() {
  active_ = true;
  if (!session_.is_open() || !view_->unavailable_reason().isEmpty())
    return;
  const Layer colour = colour_->currentData().isValid()
                           ? static_cast<Layer>(colour_->currentData().toInt())
                           : Layer::cloud;
  if (visible_.value(static_cast<int>(Layer::cloud)) && has_data(colour))
    ensure_layer(colour);
  for (Layer l :
       {Layer::mesh, Layer::components, Layer::frustums, Layer::panoramas})
    if (visible_.value(static_cast<int>(l)) && has_data(l))
      ensure_layer(l);
  view_->request_render();
}

QString Viewer3DWorkspace::colour_cloud() const {
  if (!colour_)
    return {};
  const auto *c =
      colour_choice(static_cast<Layer>(colour_->currentData().toInt()));
  return c ? QString(c->cloud) : QString();
}

void Viewer3DWorkspace::rebuild_panel() {
  const Theme &t = theme();
  while (QLayoutItem *it = panel_layout_->takeAt(0)) {
    if (QWidget *w = it->widget()) {
      w->hide();
      w->deleteLater();
    }
    delete it->layout();
    delete it;
  }
  layer_boxes_.clear();
  legend_ = nullptr;
  panel_layout_->setContentsMargins(t.px("--space-4"), t.px("--space-3"),
                                    t.px("--space-4"), t.px("--space-4"));
  panel_layout_->setSpacing(t.px("--space-2"));
  panel_layout_->addWidget(
      new CapsLabel("Lag", "eyebrowSurface", "--tracking-wide"));

  const auto &s = session_.summary();
  const bool open = session_.is_open();
  qulonglong cloud_points = 0;
  for (const auto &c : s.clouds)
    if (c.name == "cloud")
      cloud_points = c.point_count;

  auto row = [&](Layer l, const QString &name, const QString &count) {
    auto *w = new QWidget;
    w->setObjectName("layerRow");
    auto *h = new QHBoxLayout(w);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(t.px("--space-2"));
    auto *box = new QCheckBox(NavItem::escape_mnemonic(name));
    const bool avail = open && has_data(l);
    box->setEnabled(avail);
    box->setChecked(avail && visible_.value(static_cast<int>(l)));
    h->addWidget(box);
    h->addStretch(1);
    auto *hint = new QLabel(avail ? count : QString("ikke kørt endnu"));
    hint->setObjectName(avail ? "layerCount" : "layerHint");
    h->addWidget(hint);
    connect(box, &QCheckBox::toggled, this,
            [this, l](bool on) { set_layer_visible(l, on); });
    layer_boxes_[static_cast<int>(l)] = box;
    panel_layout_->addWidget(w);
  };

  row(Layer::cloud, "Punkter", format_count(cloud_points));

  // Colour by: stored RGB or any label cloud the project has.
  auto *cl = new QWidget;
  auto *cv = new QVBoxLayout(cl);
  cv->setContentsMargins(t.px("--space-5"), 0, 0, t.px("--space-2"));
  cv->setSpacing(t.px("--space-1"));
  cv->addWidget(new CapsLabel("Farv efter", "fieldLabel"));
  const QString previous = colour_ ? colour_cloud() : QString();
  colour_ = new QComboBox;
  colour_->setObjectName("colourBy");
  for (const auto &c : kColours)
    if (open && has_data(c.layer))
      colour_->addItem(c.name, static_cast<int>(c.layer));
  if (colour_->count() == 0)
    colour_->addItem(kColours[0].name, static_cast<int>(Layer::cloud));
  for (int i = 0; i < colour_->count(); ++i)
    if (colour_choice(static_cast<Layer>(colour_->itemData(i).toInt()))
            ->cloud == previous)
      colour_->setCurrentIndex(i);
  colour_->setEnabled(open && colour_->count() > 1);
  connect(colour_, &QComboBox::currentIndexChanged, this, [this](int) {
    apply_visibility();
    activate();
    update_legend();
  });
  cv->addWidget(colour_);
  legend_ = new QWidget;
  legend_->setObjectName("legendHost");
  new QVBoxLayout(legend_);
  legend_->layout()->setContentsMargins(0, t.px("--space-1"), 0, 0);
  cv->addWidget(legend_);
  panel_layout_->addWidget(cl);

  auto *rule = new QFrame;
  rule->setObjectName("rule");
  rule->setFixedHeight(1);
  panel_layout_->addWidget(rule);

  std::size_t meshes = s.meshes.size();
  row(Layer::mesh, "Mesh", meshes ? QString::number(meshes) : QString());
  row(Layer::components, "Bygningsdele",
      format_count(static_cast<qulonglong>(s.components.total_count)));
  row(Layer::frustums, "Kamerafrustummer",
      format_count(static_cast<qulonglong>(s.sensor_frames.total_count)));
  row(Layer::panoramas, "Panoramaer",
      format_count(static_cast<qulonglong>(s.panoramic_images.total_count)));

  auto *rule2 = new QFrame;
  rule2->setObjectName("rule");
  rule2->setFixedHeight(1);
  panel_layout_->addWidget(rule2);

  panel_layout_->addWidget(new CapsLabel("Snitplan", "fieldLabel"));
  auto *cutw = new QWidget;
  auto *ch = new QHBoxLayout(cutw);
  ch->setContentsMargins(0, 0, 0, 0);
  ch->setSpacing(t.px("--space-2"));
  cut_on_ = new QCheckBox(NavItem::escape_mnemonic("Skjul over"));
  cut_on_->setChecked(cut_);
  cut_on_->setEnabled(open);
  ch->addWidget(cut_on_);
  cut_height_ = new QDoubleSpinBox;
  cut_height_->setObjectName("cutHeight");
  cut_height_->setLocale(da());
  cut_height_->setDecimals(2);
  cut_height_->setSingleStep(0.1);
  cut_height_->setRange(0.1, 50.0);
  cut_height_->setSuffix(" m");
  cut_height_->setButtonSymbols(QAbstractSpinBox::NoButtons);
  cut_height_->setValue(cut_height_m_.value_or(vis::kDefaultCutHeightM));
  cut_height_->setEnabled(open && cut_);
  ch->addWidget(cut_height_, 1);
  panel_layout_->addWidget(cutw);
  auto *cut_hint = new QLabel(
      "Over gulvet fra planerne, ellers fra scenens nederste punkt — som rux "
      "render --view plan.");
  cut_hint->setObjectName("layerHint");
  cut_hint->setWordWrap(true);
  panel_layout_->addWidget(cut_hint);
  connect(cut_on_, &QCheckBox::toggled, this, [this](bool on) {
    cut_height_->setEnabled(on);
    set_cut(on, cut_height_->value());
  });
  connect(cut_height_, &QDoubleSpinBox::valueChanged, this,
          [this](double v) { set_cut(cut_on_->isChecked(), v); });

  panel_layout_->addStretch(1);
  update_legend();
}

void Viewer3DWorkspace::ensure_layer(Layer layer) {
  const int key = static_cast<int>(layer);
  if (auto it = layers_.find(key); it != layers_.end()) {
    // A layer that failed is tried again (the user may have fixed the data,
    // or a lock may have gone); a built or loading one is left alone.
    if (it->second->loading || it->second->error.isEmpty())
      return;
    layers_.erase(it);
  }
  auto state = std::make_unique<LayerState>();
  state->loading = true;
  layers_[key] = std::move(state);
  if (!camera_placed_)
    view_->set_message(is_point_layer(layer) ? QString("Henter punktskyen …")
                                             : QString("Henter scenen …"));

  // Everything the worker needs, read on the GUI thread.
  const std::string path = session_.path().toStdString();
  vis::SceneOptions o;
  o.layers = {layer};
  o.palette = theme_palette();
  o.point_size = 1.5 * devicePixelRatioF();
  o.max_points = kMaxPoints;
  o.coarse_points = interactive_ ? kCoarsePoints : 0;
  o.frustum_rgb = rgb(theme().color("--color-on-chrome-muted"));
  o.panorama_rgb = rgb(theme().color("--color-star"));
  const unsigned gen = generation_;
  const ColourChoice *choice = colour_choice(layer);
  const std::string label_cloud =
      choice && std::strlen(choice->cloud) > 0 ? choice->cloud : "";

  auto work = std::make_shared<BackgroundWork>();
  QPointer<Viewer3DWorkspace> guard(this);
  std::thread([path, o, gen, layer, label_cloud, guard, work]() mutable {
    auto r = std::make_shared<BuildResult>();
    r->generation = gen;
    r->layer = layer;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      r->scratch = vtkSmartPointer<vtkRenderer>::New();
      r->info = vis::populate_scene(r->scratch, db, o);
      if (!label_cloud.empty()) {
        std::map<std::uint32_t, std::size_t> counts;
        for (const auto &p : *db.point_cloud_label(label_cloud))
          ++counts[p.label];
        r->counts.assign(counts.begin(), counts.end());
        std::stable_sort(
            r->counts.begin(), r->counts.end(),
            [](const auto &a, const auto &b) { return a.second > b.second; });
        r->names = db.label_definitions(label_cloud);
      }
    } catch (const std::exception &e) {
      r->error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, r] {
          if (guard)
            guard->layer_built(r);
        },
        Qt::QueuedConnection);
    work.reset(); // last: counted until the thread has let go of everything
  }).detach();
}

void Viewer3DWorkspace::layer_built(std::shared_ptr<BuildResult> r) {
  if (r->generation != generation_)
    return; // a project that is no longer open
  auto it = layers_.find(static_cast<int>(r->layer));
  if (it == layers_.end())
    return;
  LayerState &st = *it->second;
  st.loading = false;
  st.error = r->error;
  if (r->error.isEmpty()) {
    vtkRenderer *target = view_->renderer();
    st.info = r->info;
    for (const auto &sl : r->info.layers) {
      for (vtkActor *a : sl.actors) {
        st.actors.emplace_back(a);
        target->AddActor(a);
      }
      if (sl.coarse) {
        st.coarse = sl.coarse;
        target->AddActor(sl.coarse);
      }
      st.items += sl.items;
    }
    st.counts = std::move(r->counts);
    st.names = std::move(r->names);
    r->scratch->RemoveAllViewProps();
  }
  // LOD twins of every point layer.
  std::vector<std::pair<vtkActor *, vtkActor *>> pairs;
  for (const auto &[k, s] : layers_)
    if (is_point_layer(static_cast<Layer>(k)) && s->coarse &&
        !s->actors.empty())
      pairs.emplace_back(s->actors.front().GetPointer(),
                         s->coarse.GetPointer());
  view_->set_lod_pairs(std::move(pairs));

  apply_visibility();
  // Re-frame as layers arrive (the cloud after the frustums, say) until the
  // user has moved the camera.
  if (r->error.isEmpty() && (!camera_placed_ || !camera_moved_)) {
    frame_camera();
  } else {
    apply_cut();
  }
  if (camera_placed_)
    view_->set_message({});
  else if (!r->error.isEmpty())
    view_->set_message(QString("Kunne ikke tegne: %1").arg(r->error));
  update_legend();
  view_->request_render();
  update_status();
}

void Viewer3DWorkspace::apply_visibility() {
  const int colour =
      colour_ ? colour_->currentData().toInt() : static_cast<int>(Layer::cloud);
  for (const auto &[k, s] : layers_) {
    bool on;
    if (is_point_layer(static_cast<Layer>(k)))
      on = visible_.value(static_cast<int>(Layer::cloud)) && k == colour;
    else
      on = visible_.value(k);
    for (const auto &a : s->actors)
      a->SetVisibility(on);
    if (s->coarse)
      s->coarse->SetVisibility(false);
  }
  view_->request_render();
}

void Viewer3DWorkspace::set_colour_by(const QString &layer) {
  for (int i = 0; i < colour_->count(); ++i) {
    const auto *c =
        colour_choice(static_cast<Layer>(colour_->itemData(i).toInt()));
    if (QString(c->cloud) == layer || (layer == "cloud" && c->cloud[0] == 0))
      colour_->setCurrentIndex(i);
  }
}

void Viewer3DWorkspace::set_layer_visible(Layer layer, bool on) {
  const int key =
      static_cast<int>(is_point_layer(layer) ? Layer::cloud : layer);
  visible_[key] = on;
  if (auto *box = layer_boxes_.value(key)) {
    const QSignalBlocker b(box);
    box->setChecked(on);
  }
  apply_visibility();
  if (on)
    activate();
}

void Viewer3DWorkspace::frame_camera() {
  vis::SceneBounds b;
  for (const auto &[k, s] : layers_)
    if (is_point_layer(static_cast<Layer>(k)) && s->info.bounds.valid) {
      b = s->info.bounds;
      break;
    }
  if (!b.valid)
    for (const auto &[k, s] : layers_)
      if (s->info.bounds.valid) {
        b.add(s->info.bounds.min[0], s->info.bounds.min[1],
              s->info.bounds.min[2]);
        b.add(s->info.bounds.max[0], s->info.bounds.max[1],
              s->info.bounds.max[2]);
      }
  if (!b.valid)
    return;
  apply_cut();
  // Frame what is visible: below the cut, not the ceiling it removed (the
  // Q0 pane put a cut scene low in the frame).
  vis::SceneBounds framed = b;
  const bool cutting = cut_ || preset_ == vis::ViewPreset::plan;
  if (cutting && cut_z_ > b.min[2] && cut_z_ < b.max[2])
    framed.max[2] = cut_z_;
  vis::CameraFraming f;
  f.view = preset_;
  f.orbit_index = 1; // the corner view, as rux render --view orbit:8 #1
  f.orbit_elevation_deg = 35.0;
  f.margin = preset_ == vis::ViewPreset::orbit ? 0.95 : 1.04;
  f.aspect = view_->aspect();
  f.yaw_deg = view_settings_.yaw_deg();
  vis::place_preset_camera(view_->renderer(), f, framed);
  camera_placed_ = true;
  camera_moved_ = false;
  view_->request_render();
}

void Viewer3DWorkspace::set_view(vis::ViewPreset view) {
  preset_ = view;
  if (presets_) {
    int i = 0;
    for (auto *b : presets_->findChildren<QPushButton *>("segment")) {
      const vis::ViewPreset order[] = {
          vis::ViewPreset::orbit, vis::ViewPreset::plan, vis::ViewPreset::top,
          vis::ViewPreset::front};
      b->setChecked(order[i++] == view);
    }
  }
  frame_camera();
}

void Viewer3DWorkspace::set_cut(bool on, std::optional<double> height) {
  cut_ = on;
  if (height)
    cut_height_m_ = height;
  if (cut_on_ && cut_on_->isChecked() != on) {
    const QSignalBlocker b(cut_on_);
    cut_on_->setChecked(on);
    cut_height_->setEnabled(on);
  }
  if (height && cut_height_ && cut_height_->value() != *height) {
    const QSignalBlocker b(cut_height_);
    cut_height_->setValue(*height);
  }
  apply_cut();
  if (camera_placed_)
    frame_camera();
}

void Viewer3DWorkspace::apply_cut() {
  vtkRenderer *r = view_->renderer();
  vis::clear_cut_planes(r);
  const bool cutting = cut_ || preset_ == vis::ViewPreset::plan;
  if (!cutting || !session_.db())
    return;
  vis::SceneBounds b;
  for (const auto &[k, s] : layers_)
    if (s->info.bounds.valid) {
      b.add(s->info.bounds.min[0], s->info.bounds.min[1],
            s->info.bounds.min[2]);
      b.add(s->info.bounds.max[0], s->info.bounds.max[1],
            s->info.bounds.max[2]);
    }
  if (!b.valid)
    return;
  try {
    const vis::CutPlane c = vis::resolve_cut_plane(
        *session_.db(), b, cut_height_m_.value_or(vis::kDefaultCutHeightM));
    cut_z_ = c.z;
    cut_from_planes_ = c.from_planes;
    vis::apply_cut_plane(r, c.z);
    // Cameras and panoramas sit at eye height: a plan must still show them.
    for (Layer l : {Layer::frustums, Layer::panoramas})
      if (auto it = layers_.find(static_cast<int>(l)); it != layers_.end())
        for (const auto &a : it->second->actors)
          if (a->GetMapper())
            a->GetMapper()->RemoveAllClippingPlanes();
  } catch (const std::exception &) {
  }
  view_->request_render();
}

void Viewer3DWorkspace::update_legend() {
  if (!legend_)
    return;
  auto *l = static_cast<QVBoxLayout *>(legend_->layout());
  while (QLayoutItem *it = l->takeAt(0)) {
    if (QWidget *w = it->widget()) {
      w->hide();
      w->deleteLater();
    }
    delete it;
  }
  const Layer colour = static_cast<Layer>(colour_->currentData().toInt());
  const ColourChoice *choice = colour_choice(colour);
  auto it = layers_.find(static_cast<int>(colour));
  if (!choice || colour == Layer::cloud || it == layers_.end() ||
      it->second->loading) {
    legend_->setVisible(colour != Layer::cloud && session_.is_open());
    if (colour != Layer::cloud) {
      auto *w = new QLabel("Tæller mærkater …");
      w->setObjectName("layerHint");
      l->addWidget(w);
    }
    return;
  }
  legend_->setVisible(true);
  const LayerState &st = *it->second;
  bool ok = false;
  int nslots = theme().value("--label-count").toInt(&ok);
  if (!ok || nslots <= 0)
    nslots = 8;
  std::size_t total = 0;
  for (const auto &[label, n] : st.counts)
    total += n;
  QVector<LegendEntry> entries;
  constexpr int kShown = 10;
  int shown = 0;
  std::size_t rest = 0, rest_points = 0;
  // Unlabeled and out-of-contract points first: they are not classes, so
  // they never take a class's place in the list (or its colour).
  std::size_t unlabeled = 0, invalid = 0;
  for (const auto &[label, n] : st.counts) {
    if (label == reusex::core::kUnlabeled)
      unlabeled += n;
    else if (reusex::core::is_out_of_contract_label(label))
      invalid += n;
  }
  auto pct = [&](std::size_t n) {
    return total ? da().toString(100.0 * n / total, 'f', 1) + " %" : QString();
  };
  if (unlabeled > 0)
    entries.push_back({"Uden mærkat", "--label-unlabeled", pct(unlabeled)});
  if (invalid > 0)
    entries.push_back({"Ugyldig mærkat (−1)", "--label-invalid", pct(invalid)});
  // Only `nslots` colours exist (label_palette_slot() wraps at
  // (label - 1) % nslots, on the raw semantic label value — not on how many
  // distinct classes are present), so two shown classes can land on the same
  // slot even with few classes, if their label values happen to collide mod
  // nslots. Detected empirically on what is actually shown, rather than
  // guessed from a class count (finding 9: document it, the cheap option,
  // rather than invent a pattern/index system for what is already a
  // design-owed token).
  std::set<int> seen_slots;
  bool slot_collision = false;
  for (const auto &[label, n] : st.counts) {
    if (label == reusex::core::kUnlabeled ||
        reusex::core::is_out_of_contract_label(label))
      continue;
    if (shown++ >= kShown) {
      ++rest;
      rest_points += n;
      continue;
    }
    const auto nm = st.names.find(static_cast<int>(label));
    const QString name = nm != st.names.end()
                             ? qs(nm->second)
                             : QString(choice->item).arg(label);
    const int slot =
        vis::label_palette_slot(label, static_cast<std::size_t>(nslots));
    if (!seen_slots.insert(slot).second)
      slot_collision = true;
    entries.push_back({name, label_swatch_token(label, nslots), pct(n)});
  }
  l->addWidget(new LabelLegend(entries));
  if (rest > 0) {
    auto *more = new QLabel(
        QString("+ %1 flere · %2 punkter")
            .arg(rest)
            .arg(format_count(static_cast<qulonglong>(rest_points))));
    more->setObjectName("layerHint");
    l->addWidget(more);
  }
  if (slot_collision) {
    auto *note =
        new QLabel(QString("Kun %1 farver — to viste klasser deler en farve.")
                       .arg(nslots));
    note->setObjectName("layerHint");
    note->setWordWrap(true);
    l->addWidget(note);
  }
}

void Viewer3DWorkspace::update_status() {
  if (!session_.is_open()) {
    status_->setText(QString());
    return;
  }
  QStringList parts;
  for (const auto &[k, s] : layers_) {
    if (!is_point_layer(static_cast<Layer>(k)) || s->actors.empty() ||
        !s->actors.front()->GetVisibility())
      continue;
    const auto &info = s->info;
    QString p = QString("%1 punkter")
                    .arg(format_count(static_cast<qulonglong>(
                        info.indices.empty() ? info.source_points
                                             : info.indices.size())));
    if (info.lod != vis::LodMethod::all)
      p += QString(" af %1").arg(
          format_count(static_cast<qulonglong>(info.source_points)));
    if (s->coarse)
      p += QString(" · %1 under bevægelse")
               .arg(format_count(static_cast<qulonglong>(kCoarsePoints)));
    parts << p;
  }
  bool loading = false;
  for (const auto &[k, s] : layers_)
    loading = loading || s->loading;
  if (loading)
    parts << "henter …";
  if (cut_ || preset_ == vis::ViewPreset::plan)
    parts << QString("snit %1 over gulvet")
                 .arg(metres(cut_height_m_.value_or(vis::kDefaultCutHeightM)));
  if (view_->last_render_ms() > 0 && !view_->interactive())
    parts << QString("tegnet på %1 ms").arg(view_->last_render_ms());
  status_->setText(parts.join("  ·  "));
}

void Viewer3DWorkspace::pick(const QPoint &pos) { pick_at(pos); }

bool Viewer3DWorkspace::pick_at(const QPoint &pos) {
  // Only the visible points can be picked.
  const int colour = colour_->currentData().toInt();
  auto it = layers_.find(colour);
  if (it == layers_.end() || it->second->actors.empty() ||
      !it->second->actors.front()->GetVisibility())
    return false;
  const LayerState &st = *it->second;
  const SceneView::Pick hit =
      view_->pick(pos, {st.actors.front().GetPointer()});
  vtkRenderer *r = view_->renderer();
  if (marker_) {
    view_->remove_actor(static_cast<vtkActor *>(marker_));
    marker_ = nullptr;
  }
  if (!hit.hit) {
    selection_ = {};
    emit selection_changed(selection_);
    view_->request_render();
    return false;
  }
  const std::size_t index =
      st.info.indices.empty()
          ? static_cast<std::size_t>(hit.point_id)
          : st.info.indices[static_cast<std::size_t>(hit.point_id)];

  // A marker on the point, in the accent.
  vtkNew<vtkPoints> pts;
  pts->InsertNextPoint(hit.xyz);
  vtkNew<vtkCellArray> verts;
  const vtkIdType id0 = 0;
  verts->InsertNextCell(1, &id0);
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(pts);
  poly->SetVerts(verts);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  vtkNew<vtkActor> marker;
  marker->SetMapper(mapper);
  const QColor accent = theme().color("--color-accent");
  marker->GetProperty()->SetColor(accent.redF(), accent.greenF(),
                                  accent.blueF());
  marker->GetProperty()->SetPointSize(7.0 * devicePixelRatioF());
  marker->GetProperty()->SetRenderPointsAsSpheres(true);
  marker->GetProperty()->SetLighting(false);
  r->AddActor(marker);
  marker_ = marker.GetPointer();

  Selection s;
  s.kind = "Punkt";
  s.title = QString("Punkt %1").arg(format_count(index));
  s.subtitle = QString("Indeks i punktskyen cloud");
  SelectionSection where{"Position", {}, {}};
  where.rows.push_back({"X", metres(hit.xyz[0])});
  where.rows.push_back({"Y", metres(hit.xyz[1])});
  where.rows.push_back({"Z", metres(hit.xyz[2])});
  if (cut_ || preset_ == vis::ViewPreset::plan)
    where.rows.push_back(
        {"Over gulvet",
         metres(hit.xyz[2] -
                (cut_z_ - cut_height_m_.value_or(vis::kDefaultCutHeightM)))});
  s.sections.push_back(where);

  // The point's value in each label cloud (one stored record each).
  SelectionSection labels{"Mærkater", {}, {}};
  bool ok = false;
  int nslots = theme().value("--label-count").toInt(&ok);
  if (!ok || nslots <= 0)
    nslots = 8;
  if (const reusex::ProjectDB *db = session_.db()) {
    for (const auto &c : kColours) {
      if (c.cloud[0] == 0 || !db->has_point_cloud(c.cloud))
        continue;
      try {
        const auto page = db->point_cloud_page(c.cloud, index, 1);
        if (page.count != 1 || page.data.size() < sizeof(std::uint32_t))
          continue;
        std::uint32_t label = 0;
        std::memcpy(&label, page.data.data(), sizeof label);
        // 0 is unlabeled (STANDARDS §3); an all-ones value is a -1 that
        // reached a point cloud, which the contract does not allow.
        QString value =
            label == reusex::core::kUnlabeled ? QString("uden")
            : reusex::core::is_out_of_contract_label(label)
                ? QString("%1 (ugyldig)").arg(static_cast<std::int32_t>(label))
                : QString::number(label);
        if (label != 0 && !reusex::core::is_out_of_contract_label(label)) {
          auto it2 = layers_.find(static_cast<int>(c.layer));
          if (it2 != layers_.end()) {
            const auto nm = it2->second->names.find(static_cast<int>(label));
            if (nm != it2->second->names.end())
              value = QString("%1 · %2").arg(label).arg(qs(nm->second));
          }
        }
        labels.rows.push_back({c.name, value, SelectionRow::Style::mono,
                               label_swatch_token(label, nslots)});
      } catch (const std::exception &) {
      }
    }
  }
  if (!labels.rows.isEmpty())
    s.sections.push_back(labels);
  selection_ = s;
  emit selection_changed(selection_);
  view_->request_render();
  return true;
}

} // namespace rux::qt
