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
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLocale>
#include <QPointer>
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

#include <cmath>
#include <thread>

namespace rux::qt {
namespace {

constexpr double kPointPx = 2.0; ///< logical pixels
const char *const kTokenA = "--label-1";
const char *const kTokenB = "--label-0";

vtkSmartPointer<vtkActor> points_actor(const CloudPoints &pts, const QColor &c,
                                       double point_px) {
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
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
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
  /// Handle radius in DISPLAY pixels (logical px x the canvas' dpr); 0
  /// keeps VTK's own sizing.
  void set_handle_display_px(double px) { handle_px_ = px; }

    protected:
  RigidBoxRepresentation() = default;
  void SizeHandles() override {
    Superclass::SizeHandles();
    vtkRenderer *r = this->Renderer;
    if (handle_px_ <= 0.0 || !r || !r->GetActiveCamera())
      return;
    // The world length of handle_px_ display pixels at the box centre.
    double c[3];
    this->Points->GetPoint(14, c);
    r->SetWorldPoint(c[0], c[1], c[2], 1.0);
    r->WorldToDisplay();
    double d[3];
    r->GetDisplayPoint(d);
    r->SetDisplayPoint(d[0] + handle_px_, d[1], d[2]);
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
  Vec3 pivot{};
  QString error;
};

struct PairOverlay::Vtk {
  vtkSmartPointer<vtkActor> a, b;
  vtkSmartPointer<vtkBoxWidget2> box;
  vtkSmartPointer<vtkCallbackCommand> observer;
  reusex::visualize::SceneBounds bounds; ///< both clouds, unmoved
};

PairOverlay::PairOverlay(ProjectSession &session, bool interactive,
                         QWidget *parent)
    : QWidget(parent), session_(session), vtk_(std::make_unique<Vtk>()) {
  setObjectName("pairOverlay");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  view_ = new SceneView(interactive);
  v->addWidget(view_, 1);

  auto *fields = new QFrame;
  fields->setObjectName("offsetFields");
  auto *g = new QGridLayout(fields);
  g->setContentsMargins(t.px("--space-3"), t.px("--space-2"), t.px("--space-3"),
                        t.px("--space-2"));
  g->setHorizontalSpacing(t.px("--space-2"));
  g->setVerticalSpacing(t.px("--space-1"));
  // Two rows of three — shift over turn — each label beside its field and
  // the unit in the field, so the fields take two lines of the overlay's
  // height, not four, and the pane never widens the window.
  const char *labels[6] = {"X", "Y", "Z", "Drej", "Hæld", "Rul"};
  const char *tips[6] = {"Flyt B langs verdens x-akse",
                         "Flyt B langs verdens y-akse",
                         "Flyt B langs verdens z-akse",
                         "Drej B om lodret (z) gennem dens midtpunkt",
                         "Hæld B om y-aksen gennem dens midtpunkt",
                         "Rul B om x-aksen gennem dens midtpunkt"};
  for (int i = 0; i < 6; ++i) {
    auto *s = new QDoubleSpinBox;
    s->setObjectName("offsetField");
    s->setButtonSymbols(QAbstractSpinBox::NoButtons);
    s->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    // Commit on Enter / focus out, not per keystroke: a per-keystroke
    // valueChanged would reformat the very text being typed.
    s->setKeyboardTracking(false);
    s->setLocale(QLocale(QLocale::Danish, QLocale::Denmark));
    if (i < 3) {
      s->setRange(-kOffsetMaxM, kOffsetMaxM);
      s->setDecimals(3);
      s->setSingleStep(kNudgeM);
      s->setSuffix(" m");
    } else {
      s->setRange(-180.0, 180.0);
      s->setDecimals(2);
      s->setSingleStep(kNudgeDeg);
      s->setSuffix(" °");
    }
    s->setToolTip(tips[i]);
    s->setMinimumWidth(t.px("--space-7") + t.px("--space-2"));
    s->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    const int row = i / 3, col = (i % 3) * 2;
    auto *label = new CapsLabel(labels[i], "fieldLabel");
    label->setToolTip(tips[i]);
    g->addWidget(label, row, col, Qt::AlignVCenter);
    g->addWidget(s, row, col + 1);
    g->setColumnStretch(col + 1, 1);
    fields_[static_cast<std::size_t>(i)] = s;
    connect(s, &QDoubleSpinBox::valueChanged, this, [this, i](double val) {
      double *slot[6] = {&offset_.x,         &offset_.y,
                         &offset_.z,         &offset_.yaw_deg,
                         &offset_.pitch_deg, &offset_.roll_deg};
      *slot[i] = val;
      offset_ = normalized(offset_);
      apply_offset(false);
    });
  }
  // Shown in the pane header (status_label()), not under the fields.
  status_ = new ElidedLabel(QString(), Qt::ElideRight);
  status_->setObjectName("statusText");
  status_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  v->addWidget(fields);

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
  vtk_->a = vtk_->b = nullptr;
  delete legend_;
  legend_ = nullptr;
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
      c->a = thin_points(reusex::slam::frame_world_points(db, a),
                         kOverlayMaxPoints);
      c->b = thin_points(reusex::slam::frame_world_points(db, b),
                         kOverlayMaxPoints);
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
  vtk_->a = points_actor(c->a, t.color(kTokenA), px);
  vtk_->b = points_actor(c->b, t.color(kTokenB), px);
  build_legend(c->a.size(), c->b.size());
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
    rep->set_handle_display_px(t.px("--space-2") * view_->devicePixelRatioF());
    vtk_->box->SetRepresentation(rep);
    vtk_->observer = vtkSmartPointer<vtkCallbackCommand>::New();
    vtk_->observer->SetCallback(on_box);
    vtk_->observer->SetClientData(this);
    vtk_->box->AddObserver(vtkCommand::InteractionEvent, vtk_->observer);
    vtk_->box->On();
  }
  view_->set_message({});
  ready_ = true;
  apply_offset(false);
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
  // A drag past the fields' range is clamped: put the box back where the
  // clamped ΔT has it, so box, cloud and fields agree.
  const bool clamped =
      raw.x != offset_.x || raw.y != offset_.y || raw.z != offset_.z;
  apply_offset(/*from_widget=*/!clamped);
}

void PairOverlay::apply_offset(bool from_widget) {
  sync_fields();
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

void PairOverlay::sync_fields() {
  const double values[6] = {offset_.x,         offset_.y,
                            offset_.z,         offset_.yaw_deg,
                            offset_.pitch_deg, offset_.roll_deg};
  for (int i = 0; i < 6; ++i) {
    QDoubleSpinBox *s = fields_[static_cast<std::size_t>(i)];
    const QSignalBlocker b(s);
    s->setValue(values[i]);
    s->setEnabled(ready_);
  }
}

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

void PairOverlay::build_legend(std::size_t a_points, std::size_t b_points) {
  delete legend_;
  const Theme &t = theme();
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
  } rows[2] = {{kTokenA, QString("A · %1").arg(a_), a_points},
               {kTokenB, QString("B · %1").arg(b_), b_points}};
  for (int r = 0; r < 2; ++r) {
    g->addWidget(new Swatch(rows[r].token), r, 0, Qt::AlignVCenter);
    auto *name = new QLabel(rows[r].name);
    name->setObjectName("legendText");
    g->addWidget(name, r, 1);
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
  if (!legend_)
    return;
  legend_->adjustSize();
  const int m = theme().px("--space-2");
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
