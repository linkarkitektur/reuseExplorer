// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/PairOverlay.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/SceneView.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/database_logic.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/frame_pair_icp.hpp>
#include <reusex/visualize/scene.hpp>

#include <QApplication>
#include <QButtonGroup>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPointer>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <vtkActor.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget2.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkUnsignedCharArray.h>

#include <cmath>
#include <thread>

namespace rux::qt {
namespace {

constexpr double kPointPx = 2.0; ///< logical pixels
const char *const kTokenA = "--label-1";
const char *const kTokenB = "--label-0";

/// @p colours (index-aligned RGB, may be empty) ride along as the points'
/// scalars, shown only while the mapper's scalar visibility is on (RGB
/// mode); the actor colour @p c is the tint otherwise.
vtkSmartPointer<vtkActor> points_actor(const CloudPoints &pts,
                                       const CloudColours &colours,
                                       const QColor &c, double point_px) {
  vtkNew<vtkPoints> p;
  p->SetNumberOfPoints(static_cast<vtkIdType>(pts.size()));
  vtkNew<vtkCellArray> verts;
  for (vtkIdType i = 0; i < static_cast<vtkIdType>(pts.size()); ++i) {
    const auto &q = pts[static_cast<std::size_t>(i)];
    p->SetPoint(i, q[0], q[1], q[2]);
    verts->InsertNextCell(1, &i);
  }
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(p);
  poly->SetVerts(verts);
  if (!colours.empty() && colours.size() == pts.size()) {
    vtkNew<vtkUnsignedCharArray> rgb;
    rgb->SetName("rgb");
    rgb->SetNumberOfComponents(3);
    rgb->SetNumberOfTuples(static_cast<vtkIdType>(colours.size()));
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(colours.size()); ++i) {
      const auto &q = colours[static_cast<std::size_t>(i)];
      rgb->SetTypedTuple(i, q.data());
    }
    poly->GetPointData()->SetScalars(rgb);
  }
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  mapper->SetColorModeToDirectScalars();
  mapper->ScalarVisibilityOff(); // the tint until apply_colouring()
  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(c.redF(), c.greenF(), c.blueF());
  actor->GetProperty()->SetPointSize(static_cast<float>(point_px));
  return actor;
}

void to_vtk(const Mat4 &m, vtkMatrix4x4 *out) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      out->SetElement(i, j, m[static_cast<std::size_t>(i * 4 + j)]);
}

/// A box representation without its six face handles: with face moves
/// disabled they would only invite a drag that does nothing (a rigid ΔT
/// never scales). The centre handle (translate) and the faces themselves
/// (rotate) stay.
class RigidBoxRepresentation : public vtkBoxRepresentation {
    public:
  static RigidBoxRepresentation *New() {
    auto *r = new RigidBoxRepresentation;
    r->InitializeObjectBase();
    return r;
  }
  vtkTypeMacro(RigidBoxRepresentation, vtkBoxRepresentation);
  void HandlesOn() override {
    Superclass::HandlesOn();
    hide_face_handles();
  }
  void hide_face_handles() {
    for (int i = 0; i < 6; ++i) { // 0..5 faces, 6 the centre
      this->Handle[i]->VisibilityOff();
      this->Handle[i]->PickableOff();
    }
  }
  /// World radius of the centre (translate) handle.
  double centre_handle_radius() const {
    return this->HandleGeometry[6]->GetRadius();
  }
  const double *centre() const { return this->Points->GetPoint(14); }
  /// Handle radius in LOGICAL pixels of @p canvas, converted to display
  /// pixels with its device pixel ratio each time the handles are sized
  /// (the window may move to another screen); 0 keeps VTK's own sizing.
  void set_handle_logical_px(double px, QWidget *canvas) {
    handle_px_ = px;
    canvas_ = canvas;
  }

    protected:
  RigidBoxRepresentation() = default;
  void SizeHandles() override {
    Superclass::SizeHandles();
    vtkRenderer *r = this->Renderer;
    if (handle_px_ <= 0.0 || !canvas_ || !r || !r->GetActiveCamera())
      return;
    const double display_px = handle_px_ * canvas_->devicePixelRatioF();
    // The world length of display_px display pixels at the box centre.
    double c[3];
    this->Points->GetPoint(14, c);
    r->SetWorldPoint(c[0], c[1], c[2], 1.0);
    r->WorldToDisplay();
    double d[3];
    r->GetDisplayPoint(d);
    r->SetDisplayPoint(d[0] + display_px, d[1], d[2]);
    r->DisplayToWorld();
    double w[4];
    r->GetWorldPoint(w);
    if (w[3] != 0.0)
      for (int i = 0; i < 3; ++i)
        w[i] /= w[3];
    const double radius = std::sqrt((w[0] - c[0]) * (w[0] - c[0]) +
                                    (w[1] - c[1]) * (w[1] - c[1]) +
                                    (w[2] - c[2]) * (w[2] - c[2]));
    for (int i = 0; i < 7; ++i)
      this->HandleGeometry[i]->SetRadius(radius);
  }
  double handle_px_ = 0.0;
  QPointer<QWidget> canvas_;
};

void on_box(vtkObject *caller, unsigned long, void *client, void *) {
  auto *w = vtkBoxWidget2::SafeDownCast(caller);
  auto *rep = vtkBoxRepresentation::SafeDownCast(w->GetRepresentation());
  vtkNew<vtkTransform> t;
  rep->GetTransform(t);
  Mat4 m{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      m[static_cast<std::size_t>(i * 4 + j)] = t->GetMatrix()->GetElement(i, j);
  static_cast<PairOverlay *>(client)->widget_moved(m);
}

} // namespace

struct PairOverlay::Clouds {
  unsigned generation = 0;
  CloudPoints a, b;
  CloudColours a_rgb, b_rgb; ///< empty: that frame has no colour image
  Vec3 pivot{};
  QString error;
};

struct PairOverlay::Vtk {
  vtkSmartPointer<vtkActor> a, b;
  vtkSmartPointer<vtkBoxWidget2> box;
  vtkSmartPointer<vtkCallbackCommand> observer;
  /// Layer 1 above the clouds, sharing their camera: the box widget.
  vtkSmartPointer<vtkRenderer> top;
  reusex::visualize::SceneBounds bounds; ///< both clouds, unmoved
};

PairOverlay::PairOverlay(ProjectSession &session, bool interactive,
                         QWidget *parent)
    : QWidget(parent), session_(session), vtk_(std::make_unique<Vtk>()) {
  setObjectName("pairOverlay");
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  view_ = new SceneView(interactive);
  v->addWidget(view_, 1);

  // Shown in the pane header (status_label()); the overlay's whole height
  // is canvas.
  status_ = new ElidedLabel(QString(), Qt::ElideRight);
  status_->setObjectName("statusText");
  status_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  // "Farve: Tint | RGB", floating in the canvas' top-right corner like the
  // legend in its bottom-left: at 1440 px the pane header has no room for
  // it beside the title, the status line and Nulstil (the status line was
  // elided to nothing), so the tooltips carry the "Farve" caption.
  auto *seg = new QFrame(view_);
  seg->setObjectName("segmented");
  auto *sl = new QHBoxLayout(seg);
  sl->setContentsMargins(0, 0, 0, 0);
  sl->setSpacing(0);
  colour_group_ = new QButtonGroup(this);
  const struct {
    const char *text, *tip;
  } modes[2] = {
      {"Tint", "Farve: A og B i hver sin farve, så de er lette at skelne"},
      {"RGB", "Farve: skyerne i billedernes egne farver (A og B ses i "
              "forklaringen; et billede uden farvebillede beholder sin "
              "farve)"}};
  for (int i = 0; i < 2; ++i) {
    auto *b = new QPushButton(NavItem::escape_mnemonic(modes[i].text));
    b->setObjectName("segment");
    b->setCheckable(true);
    b->setCursor(Qt::PointingHandCursor);
    b->setToolTip(modes[i].tip);
    b->setProperty("position", i == 0 ? "first" : "last");
    colour_group_->addButton(b, i);
    sl->addWidget(b);
  }
  colour_group_->button(0)->setChecked(true);
  colour_switch_ = seg;
  place_legend();
  connect(colour_group_, &QButtonGroup::idClicked, this, [this](int id) {
    set_colouring(id == 1 ? OverlayColouring::rgb : OverlayColouring::tint);
  });

  setFocusPolicy(Qt::StrongFocus);
  installEventFilter(this);
  view_->installEventFilter(this);
  for (QWidget *w : view_->findChildren<QWidget *>())
    w->installEventFilter(this);
  // Re-frame to the canvas' new shape (the pane is laid out after the
  // clouds may already have arrived).
  connect(view_, &SceneView::resized, this, [this] {
    if (ready_)
      frame_camera();
  });
  connect(&session_, &ProjectSession::state_changed, this, [this] {
    a_ = b_ = -1;
    offset_ = {};
    load();
  });
  load();
}

PairOverlay::~PairOverlay() {
  if (vtk_->box)
    vtk_->box->Off();
}

void PairOverlay::set_pair(int a, int b) {
  if (a == a_ && b == b_)
    return;
  a_ = a;
  b_ = b;
  offset_ = {}; // a new pair starts unmoved
  load();
}

void PairOverlay::load() {
  ++generation_;
  ready_ = false;
  if (vtk_->box)
    vtk_->box->Off();
  vtk_->box = nullptr;
  if (vtk_->top) {
    if (vtkRenderWindow *rw = vtk_->top->GetRenderWindow())
      rw->RemoveRenderer(vtk_->top);
    vtk_->top = nullptr;
  }
  vtk_->a = vtk_->b = nullptr;
  delete legend_;
  legend_ = nullptr;
  a_points_ = b_points_ = 0;
  a_has_rgb_ = b_has_rgb_ = false;
  view_->clear_scene();
  apply_offset(false);
  if (!session_.is_open() || a_ < 0 || b_ < 0 || a_ == b_) {
    view_->set_message("Vælg A og B i grafen for at se deres skyer.");
    return;
  }
  view_->set_message("Indlæser skyerne …");
  const std::string path = session_.path().toStdString();
  const int a = a_, b = b_;
  const unsigned gen = generation_;
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PairOverlay> guard(this);
  std::thread([path, a, b, gen, guard, work]() mutable {
    auto c = std::make_shared<Clouds>();
    c->generation = gen;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      // Colours always come along, so the Tint | RGB switch never reloads;
      // thinned with the same stride as their points.
      auto ca = reusex::slam::frame_world_colored_points(db, a);
      auto cb = reusex::slam::frame_world_colored_points(db, b);
      c->a = thin_points(ca.points, kOverlayMaxPoints);
      c->a_rgb = thin_colours(ca.colors, kOverlayMaxPoints);
      c->b = thin_points(cb.points, kOverlayMaxPoints);
      c->b_rgb = thin_colours(cb.colors, kOverlayMaxPoints);
      c->pivot = centroid(c->b);
    } catch (const std::exception &e) {
      c->error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, c] {
          if (guard)
            guard->loaded(c);
        },
        Qt::QueuedConnection);
    work.reset();
  }).detach();
}

void PairOverlay::loaded(std::shared_ptr<Clouds> c) {
  if (c->generation != generation_)
    return; // the pair changed while this one loaded
  if (!c->error.isEmpty()) {
    view_->set_message(QString("Skyerne kunne ikke læses — 3D-overlayet "
                               "kræver dybde og pose i begge billeder.\n%1")
                           .arg(c->error));
    return;
  }
  const Theme &t = theme();
  pivot_ = c->pivot;
  // Okabe-Ito sky blue and orange: apart in hue and luminance, colour-blind
  // safe, and drawn for the near-black canvas in both themes.
  const double px = kPointPx * view_->devicePixelRatioF();
  vtk_->a = points_actor(c->a, c->a_rgb, t.color(kTokenA), px);
  vtk_->b = points_actor(c->b, c->b_rgb, t.color(kTokenB), px);
  a_points_ = c->a.size();
  b_points_ = c->b.size();
  a_has_rgb_ = !c->a_rgb.empty();
  b_has_rgb_ = !c->b_rgb.empty();
  apply_colouring(); // also builds the legend
  vtkRenderer *r = view_->renderer();
  r->AddActor(vtk_->a);
  r->AddActor(vtk_->b);
  vtk_->bounds = {};
  for (const CloudPoints *pts : {&c->a, &c->b})
    for (const auto &p : *pts)
      vtk_->bounds.add(p[0], p[1], p[2]);
  frame_camera();
  if (vtkRenderWindowInteractor *iren = view_->interactor()) {
    vtk_->box = vtkSmartPointer<vtkBoxWidget2>::New();
    vtk_->box->SetInteractor(iren);
    vtk_->box->SetScalingEnabled(false);
    vtk_->box->SetMoveFacesEnabled(false);
    vtk_->box->SetRotationEnabled(true);
    vtk_->box->SetTranslationEnabled(true);
    vtkNew<RigidBoxRepresentation> rep;
    rep->SetPlaceFactor(1.0);
    rep->OutlineCursorWiresOff();
    rep->PlaceWidget(vtk_->b->GetBounds()); // untransformed B
    rep->hide_face_handles();
    // The centre sphere is the only translate grip: --space-2 logical px
    // radius on any screen, so a HiDPI display does not shrink it to a
    // speck while the faces (rotate) stay easy to hit.
    rep->set_handle_logical_px(t.px("--space-2"), view_);
    vtk_->box->SetRepresentation(rep);
    vtk_->observer = vtkSmartPointer<vtkCallbackCommand>::New();
    vtk_->observer->SetCallback(on_box);
    vtk_->observer->SetClientData(this);
    vtk_->box->AddObserver(vtkCommand::InteractionEvent, vtk_->observer);
    // The box and its handles in a layer above the clouds, sharing the
    // camera, so B's points never hide the grip (the centre sphere sits
    // inside B's cloud, behind whatever surface faces the camera).
    vtkRenderWindow *rw = r->GetRenderWindow();
    vtk_->top = vtkSmartPointer<vtkRenderer>::New();
    vtk_->top->SetLayer(1);
    vtk_->top->InteractiveOff();
    vtk_->top->SetActiveCamera(r->GetActiveCamera());
    if (rw->GetNumberOfLayers() < 2)
      rw->SetNumberOfLayers(2);
    rw->AddRenderer(vtk_->top);
    vtk_->box->SetCurrentRenderer(vtk_->top);
    vtk_->box->On();
  }
  view_->set_message({});
  ready_ = true;
  apply_offset(false);
}

void PairOverlay::set_colouring(OverlayColouring c) {
  if (QAbstractButton *b =
          colour_group_->button(c == OverlayColouring::rgb ? 1 : 0)) {
    const QSignalBlocker block(colour_group_);
    b->setChecked(true);
  }
  if (c == colouring_)
    return;
  colouring_ = c;
  apply_colouring();
  emit colouring_changed(c);
}

bool PairOverlay::shows_rgb_a() const {
  return ready_ && colouring_ == OverlayColouring::rgb && a_has_rgb_;
}
bool PairOverlay::shows_rgb_b() const {
  return ready_ && colouring_ == OverlayColouring::rgb && b_has_rgb_;
}

void PairOverlay::apply_colouring() {
  const bool rgb = colouring_ == OverlayColouring::rgb;
  if (vtk_->a)
    vtk_->a->GetMapper()->SetScalarVisibility(rgb && a_has_rgb_);
  if (vtk_->b)
    vtk_->b->GetMapper()->SetScalarVisibility(rgb && b_has_rgb_);
  if (vtk_->a || vtk_->b) {
    build_legend();
    view_->request_render();
  }
}

void PairOverlay::frame_camera() {
  reusex::visualize::CameraFraming f;
  f.view = reusex::visualize::ViewPreset::orbit;
  f.orbit_index = 1;
  f.orbit_elevation_deg = 35.0;
  f.aspect = view_->aspect();
  reusex::visualize::place_preset_camera(view_->renderer(), f, vtk_->bounds);
  view_->request_render();
}

void PairOverlay::set_offset(const ManualOffset &o) {
  offset_ = normalized(o);
  apply_offset(false);
}

void PairOverlay::widget_moved(const Mat4 &m) {
  const ManualOffset raw = offset_from_matrix(m, pivot_);
  offset_ = normalized(raw);
  // A drag past ΔT's range is clamped: put the box back where the clamped
  // ΔT has it, so the box and the cloud agree.
  const bool clamped =
      raw.x != offset_.x || raw.y != offset_.y || raw.z != offset_.z;
  apply_offset(/*from_widget=*/!clamped);
}

void PairOverlay::apply_offset(bool from_widget) {
  if (vtk_->b) {
    vtkNew<vtkMatrix4x4> m;
    to_vtk(delta(), m);
    vtk_->b->SetUserMatrix(m);
    if (vtk_->box && !from_widget) {
      vtkNew<vtkTransform> tr;
      tr->SetMatrix(m);
      vtkBoxRepresentation::SafeDownCast(vtk_->box->GetRepresentation())
          ->SetTransform(tr);
    }
    view_->request_render();
  }
  const double shift = std::sqrt(offset_.x * offset_.x + offset_.y * offset_.y +
                                 offset_.z * offset_.z);
  const QString cm =
      QString::fromStdString(format_decimal_da(shift * 100.0, 1));
  const QString deg =
      QString::fromStdString(format_decimal_da(rotation_angle_deg(delta()), 1));
  const QString grips =
      "Træk kuglen i midten for at flytte B; træk en side for at dreje.\n";
  const QString keys = "Klik på lærredet, og flyt B med tasterne: ←/→ x · "
                       "↑/↓ y · PgUp/PgDn z · [ / ] drej · Shift: ×10";
  status_->set_full_text(!ready_ ? QString() // the canvas says why
                         : offset_.is_zero()
                             ? QString("B på gemt pose")
                             : QString("Flyttet %1 cm · %2°").arg(cm, deg));
  status_->setToolTip(
      offset_.is_zero()
          ? grips + QString("B står på sin gemte pose.\n") + keys
          : grips +
                QString("B er flyttet %1 cm og drejet %2° i forhold til den "
                        "gemte pose.\n")
                    .arg(cm, deg) +
                keys);
  emit offset_changed();
}

QWidget *PairOverlay::status_label() const { return status_; }

bool PairOverlay::handle_key(int key, bool shift) {
  NudgeKey k;
  switch (key) {
  case Qt::Key_Left:
    k = NudgeKey::x_minus;
    break;
  case Qt::Key_Right:
    k = NudgeKey::x_plus;
    break;
  case Qt::Key_Up:
    k = NudgeKey::y_plus;
    break;
  case Qt::Key_Down:
    k = NudgeKey::y_minus;
    break;
  case Qt::Key_PageUp:
    k = NudgeKey::z_plus;
    break;
  case Qt::Key_PageDown:
    k = NudgeKey::z_minus;
    break;
  case Qt::Key_BracketLeft:
    k = NudgeKey::yaw_plus;
    break;
  case Qt::Key_BracketRight:
    k = NudgeKey::yaw_minus;
    break;
  default:
    return false;
  }
  if (!ready_)
    return true;
  set_offset(nudge(offset_, k, shift));
  return true;
}

void PairOverlay::build_legend() {
  delete legend_;
  const Theme &t = theme();
  const bool rgb = colouring_ == OverlayColouring::rgb;
  legend_ = new QFrame(view_);
  legend_->setObjectName("overlayLegend");
  legend_->setAttribute(Qt::WA_TransparentForMouseEvents);
  auto *g = new QGridLayout(legend_);
  g->setContentsMargins(t.px("--space-2"), t.px("--space-1"), t.px("--space-2"),
                        t.px("--space-1"));
  g->setHorizontalSpacing(t.px("--space-2"));
  g->setVerticalSpacing(t.px("--space-1"));
  g->setSizeConstraint(QLayout::SetFixedSize);
  const struct {
    const char *token;
    QString name;
    std::size_t points;
    bool has_rgb;
  } rows[2] = {{kTokenA, QString("A · %1").arg(a_), a_points_, a_has_rgb_},
               {kTokenB, QString("B · %1").arg(b_), b_points_, b_has_rgb_}};
  for (int r = 0; r < 2; ++r) {
    // In RGB mode the swatch still says which cloud is which; a frame
    // without a colour image keeps its tint, and the row says so.
    g->addWidget(new Swatch(rows[r].token), r, 0, Qt::AlignVCenter);
    QString name = rows[r].name;
    if (rgb)
      name += rows[r].has_rgb ? QString(" · RGB")
                              : QString(" · intet farvebillede");
    auto *label = new QLabel(name);
    label->setObjectName("legendText");
    g->addWidget(label, r, 1);
    auto *count =
        new QLabel(format_count(static_cast<qulonglong>(rows[r].points)));
    count->setObjectName("legendCount");
    count->setToolTip("Punkter tegnet");
    count->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    g->addWidget(count, r, 2);
  }
  legend_->show();
  place_legend();
}

void PairOverlay::place_legend() {
  const int m = theme().px("--space-2");
  if (colour_switch_) {
    colour_switch_->adjustSize();
    colour_switch_->move(view_->width() - colour_switch_->width() - m, m);
    colour_switch_->raise();
  }
  if (!legend_)
    return;
  legend_->adjustSize();
  legend_->move(m, view_->height() - legend_->height() - m);
  legend_->raise();
}

bool PairOverlay::eventFilter(QObject *watched, QEvent *event) {
  if (watched == view_ && event->type() == QEvent::Resize)
    place_legend();
  if (event->type() == QEvent::KeyPress) {
    auto *k = static_cast<QKeyEvent *>(event);
    if (handle_key(k->key(), k->modifiers().testFlag(Qt::ShiftModifier)))
      return true;
  }
  return QWidget::eventFilter(watched, event);
}

namespace {
/// World point -> logical widget px of the overlay canvas (VTK's display
/// origin is bottom-left, in device pixels).
std::optional<QPointF> to_logical(SceneView *view, const double w[3]) {
  vtkRenderer *r = view->renderer();
  if (!r || !r->GetRenderWindow())
    return std::nullopt;
  r->SetWorldPoint(w[0], w[1], w[2], 1.0);
  r->WorldToDisplay();
  double d[3];
  r->GetDisplayPoint(d);
  const double dpr = view->devicePixelRatioF();
  const int h = r->GetRenderWindow()->GetSize()[1];
  return QPointF(d[0] / dpr, (h - d[1]) / dpr);
}
} // namespace

std::optional<QPointF> PairOverlay::box_centre_px() const {
  if (!vtk_->box)
    return std::nullopt;
  auto *rep =
      static_cast<RigidBoxRepresentation *>(vtk_->box->GetRepresentation());
  const double *c = rep->centre();
  const double w[3] = {c[0], c[1], c[2]};
  return to_logical(view_, w);
}

double PairOverlay::handle_radius_px() const {
  if (!vtk_->box)
    return 0.0;
  auto *rep =
      static_cast<RigidBoxRepresentation *>(vtk_->box->GetRepresentation());
  const double *c = rep->centre();
  double up[3];
  view_->renderer()->GetActiveCamera()->GetViewUp(up);
  const double r = rep->centre_handle_radius();
  const double a[3] = {c[0], c[1], c[2]};
  const double b[3] = {c[0] + up[0] * r, c[1] + up[1] * r, c[2] + up[2] * r};
  const auto pa = to_logical(view_, a), pb = to_logical(view_, b);
  if (!pa || !pb)
    return 0.0;
  return std::hypot(pb->x() - pa->x(), pb->y() - pa->y());
}

int PairOverlay::box_state() const {
  if (!vtk_->box)
    return -1;
  return vtk_->box->GetRepresentation()->GetInteractionState();
}

} // namespace rux::qt
