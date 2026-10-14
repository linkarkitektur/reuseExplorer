// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The 3D canvas of the Qt client: one vtkRenderer, shown in one of two ways.
//
//  - interactive: a QVTKOpenGLNativeWidget (the real app, and the gallery's
//    `--gl` under xvfb). Trackball camera; while the camera moves, every
//    registered full point actor is swapped for its coarse LOD twin
//    (visualize::SceneOptions::coarse_points), so a million-point cloud stays
//    responsive, and swapped back when it stops.
//  - snapshot: an offscreen vtkRenderWindow (EGL with no display) rendered
//    into an image in the pane — QVTKOpenGLNativeWidget is blank under
//    QT_QPA_PLATFORM=offscreen, so this is what every gallery shot uses.
//
// Both draw the same renderer, filled by visualize::populate_scene(), the
// scene builder `rux render` uses. A click (a press and release without a
// drag) is reported as clicked(); pick() turns it into a point.

#include <QImage>
#include <QPoint>
#include <QWidget>

#include <memory>
#include <utility>
#include <vector>

class vtkActor;
class vtkRenderer;
class vtkRenderWindowInteractor;

namespace rux::qt {

class SceneView : public QWidget {
  Q_OBJECT
    public:
  explicit SceneView(bool interactive, QWidget *parent = nullptr);
  ~SceneView() override;

  vtkRenderer *renderer() const;
  /// The interactive widget's interactor, or nullptr in snapshot mode.
  vtkRenderWindowInteractor *interactor() const;
  bool interactive() const;
  /// Why nothing can be drawn here (no OpenGL); empty when it can.
  QString unavailable_reason() const;

  /// Show @p message over the canvas instead of the scene (empty = scene).
  void set_message(const QString &message);

  /// Remove every prop / one actor from the renderer with the widget's GL
  /// context current, so VTK can actually free their buffers (outside
  /// paintGL the QOpenGLWidget's context is not current, and the deletes
  /// would go nowhere — or to another context).
  void clear_scene();
  void remove_actor(vtkActor *actor);

  /// Coalesced: renders on the next event-loop turn.
  void request_render();
  /// Milliseconds the last render took.
  qint64 last_render_ms() const;
  /// Width / height of the canvas (for camera framing).
  double aspect() const;

  /// Full actor -> its coarse twin, swapped while the camera moves.
  void set_lod_pairs(std::vector<std::pair<vtkActor *, vtkActor *>> pairs);
  /// True while the camera moves and the coarse twins are showing.
  bool lod_active() const;

  struct Pick {
    bool hit = false;
    vtkActor *actor = nullptr;
    long long point_id = -1; ///< vtk point id within @ref actor
    double xyz[3] = {0, 0, 0};
  };
  /// The point under @p pos (widget coordinates) among @p actors.
  Pick pick(const QPoint &pos, const std::vector<vtkActor *> &actors);

    signals:
  /// A press and release without a drag, at widget coordinates.
  void clicked(const QPoint &pos);
  void rendered();
  /// The canvas changed size (a preset camera re-frames to the new aspect).
  void resized();
  /// The user started moving the camera.
  void interacted();

    protected:
  void resizeEvent(QResizeEvent *e) override;
  bool eventFilter(QObject *watched, QEvent *e) override;

    private:
  void render_snapshot();
  void lod_begin();
  void lod_end();
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace rux::qt
