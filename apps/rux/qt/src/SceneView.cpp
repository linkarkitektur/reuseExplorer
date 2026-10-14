// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/SceneView.hpp>
#include <rux_qt/Theme.hpp>

#include <reusex/visualize/offscreen_gl.hpp>
#include <reusex/visualize/scene.hpp>

#include <QElapsedTimer>
#include <QLabel>
#include <QMouseEvent>
#include <QPixmap>
#include <QResizeEvent>
#include <QStackedLayout>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkDataObject.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkHardwareSelector.h>
#include <vtkIdTypeArray.h>
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace rux::qt {

namespace vis = reusex::visualize;

struct SceneView::Impl {
  bool interactive = false;
  QString unavailable;
  vtkSmartPointer<vtkRenderer> renderer;
  // interactive
  QVTKOpenGLNativeWidget *gl = nullptr;
  vtkSmartPointer<vtkGenericOpenGLRenderWindow> gl_window;
  vtkSmartPointer<vtkCallbackCommand> start_cb, end_cb;
  // snapshot
  vtkSmartPointer<vtkRenderWindow> offscreen;
  QLabel *image = nullptr;

  QStackedLayout *stack = nullptr;
  QLabel *message = nullptr;
  QTimer *debounce = nullptr;
  qint64 render_ms = 0;

  std::vector<std::pair<vtkActor *, vtkActor *>> lod;
  std::vector<std::pair<vtkActor *, vtkActor *>> swapped;
  bool lod_active = false;

  QPoint press;
  bool pressed = false;
};

namespace {

/// Is there a GL implementation to render offscreen with? Asked once.
QString offscreen_unavailable() {
  static const QString reason = [] {
    const vis::OffscreenGlProbe probe = vis::probe_offscreen_gl();
    if (probe.status == vis::OffscreenGlStatus::unusable &&
        !vis::display_configured())
      return QString("3D kræver OpenGL, og her er ingen offscreen-kontekst "
                     "(%1)")
          .arg(QString::fromStdString(probe.detail));
    return QString();
  }();
  return reason;
}

} // namespace

SceneView::SceneView(bool interactive, QWidget *parent)
    : QWidget(parent), impl_(std::make_unique<Impl>()) {
  setObjectName("viewport");
  setAttribute(Qt::WA_StyledBackground);
  vis::install_vtk_log_bridge();
  impl_->interactive = interactive;
  impl_->renderer = vtkSmartPointer<vtkRenderer>::New();
  const QColor bg = theme().color("--color-canvas");
  impl_->renderer->SetBackground(bg.redF(), bg.greenF(), bg.blueF());
  connect(&theme(), &Theme::changed, this, [this] {
    const QColor c = theme().color("--color-canvas");
    impl_->renderer->SetBackground(c.redF(), c.greenF(), c.blueF());
    request_render();
  });

  impl_->stack = new QStackedLayout(this);
  impl_->stack->setContentsMargins(0, 0, 0, 0);
  impl_->message = new QLabel;
  impl_->message->setObjectName("viewportEmpty");
  impl_->message->setAlignment(Qt::AlignCenter);
  impl_->message->setWordWrap(true);
  impl_->stack->addWidget(impl_->message);

  impl_->debounce = new QTimer(this);
  impl_->debounce->setSingleShot(true);
  impl_->debounce->setInterval(0);

  if (interactive) {
    impl_->gl = new QVTKOpenGLNativeWidget;
    impl_->gl_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    impl_->gl->setRenderWindow(impl_->gl_window);
    impl_->gl_window->AddRenderer(impl_->renderer);
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    impl_->gl->interactor()->SetInteractorStyle(style);
    // LOD: coarse twins while the camera moves.
    impl_->start_cb = vtkSmartPointer<vtkCallbackCommand>::New();
    impl_->start_cb->SetClientData(this);
    impl_->start_cb->SetCallback(
        [](vtkObject *, unsigned long, void *client, void *) {
          static_cast<SceneView *>(client)->lod_begin();
        });
    impl_->end_cb = vtkSmartPointer<vtkCallbackCommand>::New();
    impl_->end_cb->SetClientData(this);
    impl_->end_cb->SetCallback(
        [](vtkObject *, unsigned long, void *client, void *) {
          static_cast<SceneView *>(client)->lod_end();
        });
    style->AddObserver(vtkCommand::StartInteractionEvent, impl_->start_cb);
    style->AddObserver(vtkCommand::EndInteractionEvent, impl_->end_cb);
    impl_->gl->installEventFilter(this);
    impl_->stack->addWidget(impl_->gl);
    impl_->stack->setCurrentWidget(impl_->gl);
    connect(impl_->debounce, &QTimer::timeout, this, [this] {
      QElapsedTimer t;
      t.start();
      impl_->gl->renderWindow()->Render();
      impl_->render_ms = t.elapsed();
      emit rendered();
    });
  } else {
    impl_->unavailable = offscreen_unavailable();
    impl_->image = new QLabel;
    impl_->image->setObjectName("viewportImage");
    impl_->image->setAlignment(Qt::AlignCenter);
    impl_->image->setMinimumSize(1, 1);
    impl_->image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    impl_->image->installEventFilter(this);
    impl_->stack->addWidget(impl_->image);
    impl_->stack->setCurrentWidget(impl_->image);
    if (impl_->unavailable.isEmpty()) {
      impl_->offscreen = vtkSmartPointer<vtkRenderWindow>::New();
      impl_->offscreen->SetOffScreenRendering(1);
      impl_->offscreen->SetMultiSamples(0);
      impl_->offscreen->AddRenderer(impl_->renderer);
    } else {
      set_message(impl_->unavailable);
    }
    connect(impl_->debounce, &QTimer::timeout, this,
            &SceneView::render_snapshot);
  }
}

SceneView::~SceneView() {
  if (impl_->offscreen)
    impl_->offscreen->Finalize();
}

vtkRenderer *SceneView::renderer() const { return impl_->renderer; }
vtkRenderWindowInteractor *SceneView::interactor() const {
  return impl_->gl ? impl_->gl->interactor() : nullptr;
}
bool SceneView::interactive() const { return impl_->interactive; }
QString SceneView::unavailable_reason() const { return impl_->unavailable; }
qint64 SceneView::last_render_ms() const { return impl_->render_ms; }
bool SceneView::lod_active() const { return impl_->lod_active; }

double SceneView::aspect() const {
  return height() > 0 ? static_cast<double>(width()) / height() : 4.0 / 3.0;
}

void SceneView::set_message(const QString &message) {
  impl_->message->setText(message);
  if (!message.isEmpty())
    impl_->stack->setCurrentWidget(impl_->message);
  else if (impl_->gl)
    impl_->stack->setCurrentWidget(impl_->gl);
  else
    impl_->stack->setCurrentWidget(impl_->image);
}

void SceneView::clear_scene() {
  set_lod_pairs({});
  if (impl_->gl)
    impl_->gl->makeCurrent();
  impl_->renderer->RemoveAllViewProps();
  if (impl_->gl)
    impl_->gl->doneCurrent();
  request_render();
}

void SceneView::remove_actor(vtkActor *actor) {
  if (!actor)
    return;
  if (impl_->gl)
    impl_->gl->makeCurrent();
  impl_->renderer->RemoveActor(actor);
  if (impl_->gl)
    impl_->gl->doneCurrent();
}

void SceneView::request_render() {
  if (impl_->unavailable.isEmpty())
    impl_->debounce->start();
}

void SceneView::render_snapshot() {
  if (!impl_->offscreen || !isVisible() || width() <= 1 || height() <= 1)
    return;
  const qreal dpr = devicePixelRatioF();
  const int w = static_cast<int>(width() * dpr);
  const int h = static_cast<int>(height() * dpr);
  QElapsedTimer t;
  t.start();
  impl_->offscreen->SetSize(w, h);
  impl_->offscreen->Render();
  vtkNew<vtkWindowToImageFilter> capture;
  capture->SetInput(impl_->offscreen);
  capture->SetInputBufferTypeToRGB();
  capture->ReadFrontBufferOff();
  capture->ShouldRerenderOff();
  capture->Update();
  vtkImageData *img = capture->GetOutput();
  int dims[3] = {0, 0, 0};
  img->GetDimensions(dims);
  if (dims[0] > 0 && dims[1] > 0) {
    QImage out(dims[0], dims[1], QImage::Format_RGB888);
    const auto *src =
        static_cast<const unsigned char *>(img->GetScalarPointer());
    const std::size_t stride = static_cast<std::size_t>(dims[0]) * 3;
    for (int y = 0; y < dims[1]; ++y) // VTK's origin is bottom-left
      std::memcpy(out.scanLine(dims[1] - 1 - y), src + y * stride, stride);
    QPixmap pm = QPixmap::fromImage(out);
    pm.setDevicePixelRatio(dpr);
    impl_->image->setPixmap(pm);
  }
  impl_->render_ms = t.elapsed();
  emit rendered();
}

void SceneView::resizeEvent(QResizeEvent *e) {
  QWidget::resizeEvent(e);
  emit resized();
  if (!impl_->interactive)
    request_render();
}

void SceneView::set_lod_pairs(
    std::vector<std::pair<vtkActor *, vtkActor *>> pairs) {
  lod_end();
  impl_->lod = std::move(pairs);
}

void SceneView::lod_begin() {
  if (impl_->lod_active)
    return;
  emit interacted();
  impl_->lod_active = true;
  impl_->swapped.clear();
  for (auto [full, coarse] : impl_->lod) {
    if (full && coarse && full->GetVisibility()) {
      full->SetVisibility(false);
      coarse->SetVisibility(true);
      impl_->swapped.emplace_back(full, coarse);
    }
  }
}

void SceneView::lod_end() {
  if (!impl_->lod_active)
    return;
  impl_->lod_active = false;
  for (auto [full, coarse] : impl_->swapped) {
    full->SetVisibility(true);
    coarse->SetVisibility(false);
  }
  impl_->swapped.clear();
  // The interactor renders after EndInteraction anyway; ask once more so a
  // swap made outside an interaction lands too.
  request_render();
}

SceneView::Pick SceneView::pick(const QPoint &pos,
                                const std::vector<vtkActor *> &actors) {
  Pick out;
  if (actors.empty())
    return out;
  // A hardware selection: it renders point ids, so it sees what is drawn —
  // a geometric picker would happily return a ceiling point the cut plane
  // has clipped away.
  vtkRenderWindow *win = impl_->renderer->GetRenderWindow();
  if (!win)
    return out;
  const qreal dpr = devicePixelRatioF();
  const int x = static_cast<int>(pos.x() * dpr);
  const int y = static_cast<int>((height() - pos.y()) * dpr); // VTK y is up
  const int r = static_cast<int>(theme().px("--space-1") * dpr);
  vtkNew<vtkHardwareSelector> selector;
  selector->SetRenderer(impl_->renderer);
  selector->SetFieldAssociation(vtkDataObject::FIELD_ASSOCIATION_POINTS);
  selector->SetArea(static_cast<unsigned int>(std::max(0, x - r)),
                    static_cast<unsigned int>(std::max(0, y - r)),
                    static_cast<unsigned int>(x + r),
                    static_cast<unsigned int>(y + r));
  vtkSmartPointer<vtkSelection> sel;
  sel.TakeReference(selector->Select());
  if (!sel)
    return out;
  for (unsigned int i = 0; i < sel->GetNumberOfNodes(); ++i) {
    vtkSelectionNode *node = sel->GetNode(i);
    auto *actor = vtkActor::SafeDownCast(
        node->GetProperties()->Get(vtkSelectionNode::PROP()));
    if (!actor ||
        std::find(actors.begin(), actors.end(), actor) == actors.end())
      continue;
    auto *ids = vtkIdTypeArray::SafeDownCast(node->GetSelectionList());
    auto *poly = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
    if (!ids || ids->GetNumberOfTuples() == 0 || !poly)
      continue;
    out.hit = true;
    out.actor = actor;
    out.point_id = ids->GetValue(0);
    poly->GetPoint(out.point_id, out.xyz);
    break;
  }
  // The selection passes left the buffers with ids in them: draw the scene.
  request_render();
  return out;
}

bool SceneView::eventFilter(QObject *watched, QEvent *e) {
  if (watched == impl_->gl || watched == impl_->image) {
    if (e->type() == QEvent::MouseButtonPress) {
      auto *m = static_cast<QMouseEvent *>(e);
      if (m->button() == Qt::LeftButton) {
        impl_->press = m->position().toPoint();
        impl_->pressed = true;
      }
    } else if (e->type() == QEvent::MouseButtonRelease) {
      auto *m = static_cast<QMouseEvent *>(e);
      const QPoint p = m->position().toPoint();
      if (impl_->pressed && m->button() == Qt::LeftButton &&
          (p - impl_->press).manhattanLength() <= theme().px("--space-1"))
        emit clicked(p);
      impl_->pressed = false;
    }
  }
  return QWidget::eventFilter(watched, e);
}

} // namespace rux::qt
