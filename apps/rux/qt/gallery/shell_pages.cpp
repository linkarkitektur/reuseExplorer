// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Gallery pages of the real app shell (Q1): the same AppShell the `rux`
// binary shows, with the project opened synchronously and an in-memory
// recent list, so a screenshot never touches the user's settings.

#include "demo_pages.hpp"

#include <rux_qt/AppShell.hpp>
#include <rux_qt/DatabaseWorkspace.hpp>
#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/EdgeListPanel.hpp>
#include <rux_qt/EditWorkspace.hpp>
#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/GraphPanel.hpp>
#include <rux_qt/PairImages.hpp>
#include <rux_qt/PairOverlay.hpp>
#include <rux_qt/PairStrip.hpp>
#include <rux_qt/PipelineWorkspace.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/RecentProjects.hpp>
#include <rux_qt/SceneView.hpp>
#include <rux_qt/TableBrowser.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/ViewSettings.hpp>
#include <rux_qt/Viewer3DWorkspace.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/graph_view.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/optimize_stage.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include <QAbstractButton>
#include <QApplication>
#include <QButtonGroup>
#include <QComboBox>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QGraphicsView>
#include <QItemSelectionModel>
#include <QProgressBar>
#include <QPushButton>
#include <QTableView>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rux::qt::gallery {
namespace {

/// A recent list with the shot's project first and two that no longer
/// exist, so the "Mangler" state is always on screen.
QStringList demo_recent(const PageContext &ctx) {
  QStringList l;
  if (!ctx.project_path.isEmpty())
    l << ctx.project_path;
  l << QDir::homePath() + "/sager/Kontorhus Valby/project.rux"
    << QDir::homePath() + "/sager/Skolen 2025/scan-02.rux";
  return l;
}

enum class Open { none, project, read_only, junk };

AppShell *make_shell(const PageContext &ctx, Open open) {
  auto *session = new ProjectSession;
  auto *recent = new RecentProjects(demo_recent(ctx));
  ShellOptions options;
  options.interactive_3d = ctx.interactive_3d;
  options.persist_layout = false; // a shot never touches QSettings
  // What `rux` hands the client (rux::GuiLaunch): optimize and
  // graph_discover run in-process.
  options.stage_executor = reusex::pipeline::stage_executor_with_optimize();
  auto *shell = new AppShell(*session, *recent, options);
  session->setParent(shell);
  recent->setParent(shell);
  if (open == Open::project && !ctx.project_path.isEmpty()) {
    session->open_blocking(ctx.project_path);
  } else if (open == Open::read_only && !ctx.project_path.isEmpty()) {
    session->open_blocking(ctx.project_path, /*read_only=*/true);
  } else if (open == Open::junk) {
    // A file that is not a ReUseX project, to show the failure card.
    static QTemporaryDir dir;
    const QString junk = dir.filePath("noter.rux");
    QFile f(junk);
    if (f.open(QIODevice::WriteOnly))
      f.write("Ikke en database, bare noter.\n");
    f.close();
    session->open_blocking(junk);
  }
  return shell;
}

QWidget *make_start(const PageContext &ctx) {
  return make_shell(ctx, Open::project);
}

QWidget *make_start_readonly(const PageContext &ctx) {
  return make_shell(ctx, Open::read_only);
}

QWidget *make_start_empty(const PageContext &ctx) {
  return make_shell(ctx, Open::none);
}

QWidget *make_start_error(const PageContext &ctx) {
  return make_shell(ctx, Open::junk);
}

QWidget *make_shell_empty(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::none);
  shell->show_page(Workspace::database);
  return shell;
}

QWidget *make_shell_project(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->show_page(Workspace::viewer3d);
  return shell;
}

} // namespace

QWidget *make_shell_3d(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->show_page(Workspace::viewer3d);
  return shell;
}

namespace {

// ---- Database workspace (Q2)

/// A loop closure the way the pair strip stages one (no measurement).
EdgeRecord loop_closure(int from, int to, double weight) {
  EdgeRecord r;
  r.key = {from, to, "loop_closure"};
  r.weight = weight;
  return r;
}

/// The shell on the Database page with A and B a few frames apart in the
/// middle of the capture (frame ids differ per project: use positions).
AppShell *make_db(const PageContext &ctx, Open open = Open::project) {
  auto *shell = make_shell(ctx, open);
  shell->show_page(Workspace::database);
  if (FrameBrowser *f = shell->database()->frames(); !f->pair().empty()) {
    const auto &ids = f->pair().ids();
    const std::size_t a = ids.size() * 3 / 10;
    f->set_a(ids[a]);
    f->set_b(ids[std::min(ids.size() - 1, a + 12)]);
  }
  return shell;
}

/// Click a toolbar layer button of the frame browser ("Dybde", …).
void pick_layer(AppShell *shell, const QString &name) {
  for (auto *b :
       shell->database()->frames()->findChildren<QPushButton *>("segment"))
    if (b->text() == name)
      b->click();
}

QWidget *make_db_frames(const PageContext &ctx) { return make_db(ctx); }

QWidget *make_db_depth(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  pick_layer(shell, "Dybde");
  return shell;
}

QWidget *make_db_confidence(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  pick_layer(shell, "Konfidens");
  return shell;
}

QWidget *make_db_table(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  shell->database()->open_item(ProjectTree::Kind::table, "sensor_frames");
  if (auto *t = shell->database()->findChild<TableBrowser *>())
    t->select_where(
        "node_id", QString::number(shell->database()->frames()->pair().a_id()));
  return shell;
}

QWidget *make_db_log(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  shell->database()->open_item(ProjectTree::Kind::log);
  return shell;
}

QWidget *make_db_cloud(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  // The semantic label cloud if there is one (its legend), else any cloud.
  shell->database()->open_item(ProjectTree::Kind::cloud, "labels");
  return shell;
}

/// A panorama in Oversigt. The fixture has none: a synthetic 2:1 grid is
/// written to the COPY (so the shot shows the pan/zoom view, not real data).
/// A project that has panoramas shows its first one, untouched.
QWidget *make_db_panorama(const PageContext &ctx) {
  if (!ctx.project_path.isEmpty()) {
    reusex::ProjectDB db(ctx.project_path.toStdString());
    if (db.panoramic_image_count() == 0) {
      cv::Mat img(1024, 2048, CV_8UC3);
      for (int y = 0; y < img.rows; ++y)
        for (int x = 0; x < img.cols; ++x)
          img.at<cv::Vec3b>(y, x) = {
              static_cast<uchar>(x / 8), static_cast<uchar>(y / 4),
              static_cast<uchar>((x / 64 + y / 64) % 2 ? 200 : 60)};
      std::vector<uchar> jpg;
      cv::imencode(".jpg", img, jpg);
      db.save_panoramic_image("syntetisk_360.jpg", jpg, 0.0, -1);
    }
  }
  auto *shell = make_db(ctx); // after the write, so the tree lists it
  int id = -1;
  if (!ctx.project_path.isEmpty()) {
    reusex::ProjectDB db(ctx.project_path.toStdString(), /*readOnly=*/true);
    const auto list = db.list_panoramic_images();
    if (!list.empty())
      id = list.front().id;
  }
  shell->database()->open_item(ProjectTree::Kind::panorama,
                               QString::number(id));
  return shell;
}

// ---- 3D, Rediger, Pipeline, Log

/// Run the event loop until the off-thread work a page started has landed
/// (bounded), so a page can act on its result — pick a point once the cloud
/// is in, wait for a pipeline run.
void settle(int timeout_ms = 120000) {
  QElapsedTimer t;
  t.start();
  int idle = 0;
  while (idle < 3 && t.elapsed() < timeout_ms) {
    QApplication::processEvents(QEventLoop::AllEvents);
    QThread::msleep(10);
    idle = background_work_in_flight() == 0 ? idle + 1 : 0;
  }
  QApplication::processEvents(QEventLoop::AllEvents);
}

/// The shell sized like the shot, so a pick lands where it will be drawn.
AppShell *sized(AppShell *shell) {
  shell->resize(1440, 900);
  return shell;
}

AppShell *make_3d(const PageContext &ctx, const QString &colour = "cloud") {
  auto *shell = sized(make_shell(ctx, Open::project));
  shell->viewer()->set_colour_by(colour);
  shell->show_page(Workspace::viewer3d);
  return shell;
}

QWidget *make_3d_page(const PageContext &ctx) { return make_3d(ctx); }

/// Rooms with their legend, and a picked point in the inspector.
QWidget *make_3d_pick(const PageContext &ctx) {
  auto *shell = make_3d(ctx, "rooms");
  // Pick once the cloud is drawn at its final size: the first render that
  // has something under the centre of the canvas.
  SceneView *v = shell->viewer()->view();
  auto conn = std::make_shared<QMetaObject::Connection>();
  *conn = QObject::connect(v, &SceneView::rendered, shell, [shell, v, conn] {
    if (shell->viewer()->pick_at(QPoint(v->width() / 2, v->height() / 2)))
      QObject::disconnect(*conn);
  });
  return shell;
}

/// A floor plan coloured by plane, with camera frustums and panoramas.
AppShell *make_3d_plan_shell(const PageContext &ctx) {
  auto *shell = make_3d(ctx, "planes");
  shell->viewer()->set_layer_visible(reusex::visualize::Layer::frustums, true);
  shell->viewer()->set_view(reusex::visualize::ViewPreset::plan);
  return shell;
}

QWidget *make_3d_plan(const PageContext &ctx) {
  return make_3d_plan_shell(ctx);
}

/// The same plan with the project's view yaw at 30°: the default camera
/// turns with it. WRITES view.yaw_deg to the project copy.
QWidget *make_3d_yaw(const PageContext &ctx) {
  auto *shell = make_3d_plan_shell(ctx);
  shell->view_settings().set_yaw_deg(30.0);
  return shell;
}

QWidget *make_3d_labels(const PageContext &ctx) {
  auto *shell = make_3d(ctx, "labels");
  shell->viewer()->set_view(reusex::visualize::ViewPreset::top);
  shell->viewer()->set_cut(false);
  return shell;
}

/// The graph with A and B marked and a few staged loop closures.
AppShell *make_pg(const PageContext &ctx) {
  auto *shell = sized(make_shell(ctx, Open::project));
  FrameBrowser *f = shell->database()->frames();
  if (!f->pair().empty()) {
    const auto &ids = f->pair().ids();
    const auto at = [&](double t) {
      return ids[std::min(ids.size() - 1,
                          static_cast<std::size_t>(t * ids.size()))];
    };
    auto &ed = shell->edges();
    ed.add(loop_closure(at(0.05), at(0.62), 400.0));
    ed.add(loop_closure(at(0.30), at(0.81), 400.0));
    ed.add(loop_closure(at(0.45), at(0.97), 400.0));
    shell->edit()->set_pair(at(0.30), at(0.81));
  }
  shell->show_page(Workspace::edit);
  return shell;
}

QWidget *make_rediger(const PageContext &ctx) { return make_pg(ctx); }

/// The pair images: A | B in colour (make_rediger sets the pair; the
/// gallery waits for the off-thread decode before it shoots).
QWidget *make_rediger_pair(const PageContext &ctx) { return make_rediger(ctx); }
/// The same pair in depth.
QWidget *make_rediger_pair_depth(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  shell->edit()->images()->set_layer(FrameLayer::depth);
  return shell;
}
/// Only A chosen: B's side says which click fills it.
QWidget *make_rediger_pair_half(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  shell->edit()->set_pair(shell->edit()->a(), -1);
  return shell;
}

/// Click the pair strip's first button labelled one of @p texts.
void click_strip_button(AppShell *shell, std::initializer_list<QString> texts) {
  for (auto *b : shell->edit()->strip()->findChildren<QPushButton *>())
    for (const QString &t : texts)
      if (b->text() == t) {
        b->click();
        return;
      }
}

/// B moved by hand: 20 cm in x, 5° yaw. The interactive box widget needs
/// --gl (xvfb); the offscreen shot shows the fields and the moved cloud.
QWidget *make_rediger_offset(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  settle(); // clouds loaded
  shell->edit()->overlay()->set_offset({0.20, 0.0, 0.0, 5.0, 0.0, 0.0});
  return shell;
}

/// Move B by hand, then ICP-forfin from there: B snaps onto A. Logs the
/// manual and the ICP offset so a run shows B was pulled back.
AppShell *make_rediger_icp_shell(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  settle(); // clouds loaded
  PairOverlay *ov = shell->edit()->overlay();
  ov->set_offset({0.05, -0.03, 0.0, 2.0, 0.0, 0.0});
  for (auto *b : shell->edit()->strip()->findChildren<QPushButton *>())
    if (b->text() == "ICP-forfin")
      b->click();
  settle(); // ICP landed, B moved
  const ManualOffset &o = ov->offset();
  std::fprintf(stderr,
               "rediger-icp: manual x=0.0500 y=-0.0300 yaw=2.000 -> ICP "
               "x=%.4f y=%.4f z=%.4f yaw=%.3f pitch=%.3f roll=%.3f\n",
               o.x, o.y, o.z, o.yaw_deg, o.pitch_deg, o.roll_deg);
  return shell;
}
QWidget *make_rediger_icp(const PageContext &ctx) {
  return make_rediger_icp_shell(ctx);
}

/// After ICP, B nudged 1 cm by hand: the fit no longer describes B, so the
/// line says so and a new edge falls back to weight 1.
QWidget *make_rediger_icp_moved(const PageContext &ctx) {
  auto *shell = make_rediger_icp_shell(ctx);
  PairOverlay *ov = shell->edit()->overlay();
  ov->set_offset(nudge(ov->offset(), NudgeKey::x_plus, false));
  return shell;
}

namespace {
/// One left-button drag on the overlay's interactor, in render-window
/// pixels (VTK's y is up), the way a mouse drives the box widget.
void vtk_drag(vtkRenderWindowInteractor *iren, int x0, int y0, int x1, int y1) {
  iren->SetEventInformation(x0, y0, 0, 0);
  iren->InvokeEvent(vtkCommand::MouseMoveEvent);
  iren->InvokeEvent(vtkCommand::LeftButtonPressEvent);
  constexpr int kSteps = 8;
  for (int i = 1; i <= kSteps; ++i) {
    iren->SetEventInformation(x0 + (x1 - x0) * i / kSteps,
                              y0 + (y1 - y0) * i / kSteps, 0, 0);
    iren->InvokeEvent(vtkCommand::MouseMoveEvent);
  }
  iren->InvokeEvent(vtkCommand::LeftButtonReleaseEvent);
}

void log_offset(const char *what, const ManualOffset &o) {
  std::fprintf(stderr,
               "rediger-drag: %s -> x=%.4f y=%.4f z=%.4f yaw=%.3f "
               "pitch=%.3f roll=%.3f\n",
               what, o.x, o.y, o.z, o.yaw_deg, o.pitch_deg, o.roll_deg);
}
} // namespace

/// The box widget driven by synthesized mouse events (needs --gl): a drag
/// of the centre handle (translate), then a drag on a face (rotate). Logs
/// the resulting ΔT; the shot shows the fields that followed the widget.
QWidget *make_rediger_drag(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  shell->show();
  settle(); // clouds loaded, GL initialised
  PairOverlay *ov = shell->edit()->overlay();
  vtkRenderWindowInteractor *iren = ov->view()->interactor();
  if (!iren || !ov->ready()) {
    std::fprintf(stderr, "rediger-drag: needs --gl and both clouds "
                         "(no interactor or not ready)\n");
    return shell;
  }
  vtkRenderer *r = ov->view()->renderer();
  vtkActorCollection *actors = r->GetActors();
  actors->InitTraversal();
  actors->GetNextActor();               // A
  vtkActor *b = actors->GetNextActor(); // B, still unmoved
  double bb[6];
  b->GetBounds(bb);
  const auto to_display = [&](double x, double y, double z) {
    r->SetWorldPoint(x, y, z, 1.0);
    r->WorldToDisplay();
    double d[3];
    r->GetDisplayPoint(d);
    return std::array<int, 2>{static_cast<int>(std::lround(d[0])),
                              static_cast<int>(std::lround(d[1]))};
  };
  const double cx = (bb[0] + bb[1]) / 2, cy = (bb[2] + bb[3]) / 2,
               cz = (bb[4] + bb[5]) / 2;
  const auto c = to_display(cx, cy, cz);
  const int step = 2 * theme().px("--space-7"); // a short, clear drag
  vtk_drag(iren, c[0], c[1], c[0] + step, c[1]);
  QApplication::processEvents();
  log_offset("centre handle dragged right", ov->offset());
  // A face: a third of the way from the (moved) centre to a top corner,
  // inside the box outline and clear of the centre handle.
  const Mat4 d = ov->delta();
  const Vec3 corner = transform_point(d, {bb[1], bb[3], bb[5]});
  const Vec3 centre = transform_point(d, {cx, cy, cz});
  const auto f = to_display(centre[0] + (corner[0] - centre[0]) / 3,
                            centre[1] + (corner[1] - centre[1]) / 3,
                            centre[2] + (corner[2] - centre[2]) / 3);
  vtk_drag(iren, f[0], f[1], f[0], f[1] + step);
  QApplication::processEvents();
  log_offset("face dragged up", ov->offset());
  ov->view()->request_render();
  return shell;
}

QWidget *make_rediger_plane(const PageContext &ctx, ViewPlane plane) {
  auto *shell = make_pg(ctx);
  settle();
  shell->edit()->graph()->set_plane(plane);
  return shell;
}
QWidget *make_rediger_xz(const PageContext &ctx) {
  return make_rediger_plane(ctx, ViewPlane::xz);
}
QWidget *make_rediger_yz(const PageContext &ctx) {
  return make_rediger_plane(ctx, ViewPlane::yz);
}

/// Rediger with the edge list and both pair panes hidden, from the palette.
QWidget *make_rediger_panes(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  shell->edit()->set_pane_visible(EditPane::edges, false);
  shell->edit()->set_pane_visible(EditPane::images, false);
  return shell;
}

/// Rediger with A and B a few frames apart (ids differ per project).
AppShell *rediger_shell(const PageContext &ctx, Open open = Open::project) {
  auto *shell = sized(make_shell(ctx, open));
  shell->show_page(Workspace::edit);
  const auto &ids = shell->database()->frames()->pair().ids();
  if (ids.size() > 1) {
    const std::size_t a = ids.size() * 3 / 10;
    shell->edit()->set_pair(ids[a], ids[std::min(ids.size() - 1, a + 12)]);
  }
  return shell;
}

/// Pending edits: an ICP run on the pair and a staged loop closure, plus a
/// staged rejection when the project has stored edges.
QWidget *make_rediger_pending(const PageContext &ctx) {
  auto *shell = rediger_shell(ctx);
  if (shell->edit()->a() < 0)
    return shell;
  EdgeEditor &ed = shell->edges();
  for (const auto &e : ed.edits().base())
    if (e.status != "rejected") {
      ed.reject(e.key);
      break;
    }
  for (auto *b : shell->edit()->strip()->findChildren<QPushButton *>())
    if (b->text() == "ICP-forfin")
      b->click();
  ed.add(loop_closure(shell->edit()->a(), shell->edit()->b(), 400.0));
  return shell;
}

/// Set the view yaw to 30°, wait out the 500 ms debounce and the write,
/// and print what the project file holds afterwards under @p tag.
void yaw_30_and_report(AppShell *shell, const PageContext &ctx,
                       const char *tag) {
  shell->view_settings().set_yaw_deg(30.0);
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < 2000) { // the 500 ms debounce, then the write
    QApplication::processEvents();
    QThread::msleep(10);
  }
  settle();
  if (!ctx.project_path.isEmpty()) {
    reusex::ProjectDB db(ctx.project_path.toStdString(), /*readOnly=*/true);
    std::fprintf(stderr, "%s: stored view.yaw_deg=%s\n", tag,
                 db.setting(kViewYawSetting).value_or("<none>").c_str());
  }
}

/// A save that cannot happen: the project is open read-only. The 30° yaw
/// applies on screen but is not written.
QWidget *make_rediger_readonly(const PageContext &ctx) {
  auto *shell = rediger_shell(ctx, Open::read_only);
  if (shell->edit()->a() < 0)
    return shell;
  shell->edges().add(loop_closure(shell->edit()->a(), shell->edit()->b(), 1.0));
  shell->edit()->save_edits();
  yaw_30_and_report(shell, ctx, "rediger-readonly");
  return shell;
}

/// A saved edit: ICP-forfin on the pair, "Tilføj kant" (ICP's measurement,
/// or the stored relative pose when ICP fails), a stored loop closure off
/// the pair rejected, then "Gem ændringer" — the edges come back from the
/// file. Prints what the file holds for both. WRITES to the project copy.
QWidget *make_rediger_saved(const PageContext &ctx) {
  auto *shell = rediger_shell(ctx);
  if (shell->edit()->a() < 0)
    return shell;
  settle(); // clouds loaded
  click_strip_button(shell, {"ICP-forfin"});
  settle(); // ICP landed
  click_strip_button(shell, {"Tilføj kant", "Erstat måling"});
  // And a stored loop closure off the pair rejected: saved as a tombstone.
  std::optional<EdgeKey> rejected;
  for (const auto &e : shell->edges().edits().base())
    if (e.key.type == "loop_closure" && e.status != "rejected" &&
        std::minmax(e.key.from, e.key.to) !=
            std::minmax(shell->edit()->a(), shell->edit()->b())) {
      shell->edges().reject(e.key);
      rejected = e.key;
      break;
    }
  shell->edit()->save_edits();
  settle(); // written
  if (!ctx.project_path.isEmpty()) {
    reusex::ProjectDB db(ctx.project_path.toStdString(), /*readOnly=*/true);
    const int a = shell->edit()->a(), b = shell->edit()->b();
    for (const auto &e : db.list_pose_graph_edges())
      if (std::minmax(e.from_node_id, e.to_node_id) == std::minmax(a, b) ||
          (rejected &&
           std::minmax(e.from_node_id, e.to_node_id) ==
               std::minmax(rejected->from, rejected->to) &&
           e.edge_type == rejected->type))
        std::fprintf(stderr,
                     "rediger-saved: %d -> %d %s source=%s status=%s "
                     "transform=%s information=%s weight=%g\n",
                     e.from_node_id, e.to_node_id, e.edge_type.c_str(),
                     e.source.c_str(), e.status.c_str(),
                     e.transform ? "yes" : "no", e.information ? "yes" : "no",
                     e.weight);
  }
  return shell;
}

/// Stress: 10 000 edges written (to the project copy) — odometry along the
/// capture, then deterministic pseudo-random loop closures and panorama
/// edges — and the paint and edit cost measured, on stderr. Edges are
/// canonical (from < to) and keyed by pair and type, so a capture with few
/// frames stores far fewer distinct rows: the line reports what is stored.
QWidget *make_rediger_stress(const PageContext &ctx) {
  constexpr std::size_t kEdges = 10000;
  std::size_t stored = 0;
  if (!ctx.project_path.isEmpty()) {
    reusex::ProjectDB db(ctx.project_path.toStdString());
    const auto ids = db.sensor_frame_ids();
    std::vector<reusex::ProjectDB::PoseGraphEdge> edges;
    const auto edge = [](int from, int to, const char *type, double residual,
                         double weight) {
      reusex::ProjectDB::PoseGraphEdge e;
      e.from_node_id = from;
      e.to_node_id = to;
      e.edge_type = type;
      e.residual = residual;
      e.weight = weight;
      return e;
    };
    for (std::size_t i = 1; i < ids.size() && edges.size() < kEdges / 2; ++i)
      edges.push_back(edge(ids[i - 1], ids[i], "odometry", 0.1, 100.0));
    std::uint64_t x = 0x9E3779B97F4A7C15ULL; // fixed seed: the same shot
    while (edges.size() < kEdges && ids.size() > 1) {
      x ^= x << 13;
      x ^= x >> 7;
      x ^= x << 17;
      const int a = ids[x % ids.size()];
      const int b = ids[(x >> 32) % ids.size()];
      if (a != b)
        edges.push_back(
            edge(a, b, (x & 15) == 0 ? "panorama" : "loop_closure", 0.5, 25.0));
    }
    reusex::ProjectDB::Transaction tx(db);
    for (auto &e : edges) {
      e.source = "manual";
      db.upsert_pose_graph_edge(e, /*overwrite_rejected=*/true);
    }
    tx.commit();
    stored = db.list_pose_graph_edges().size();
  }
  auto *shell = make_pg(ctx);
  // Measure once the graph is on screen at its shot size: poll until the
  // poses have loaded and the shell is shown by the gallery.
  auto *probe = new QTimer(shell);
  probe->setInterval(50);
  QObject::connect(probe, &QTimer::timeout, shell, [shell, probe, stored] {
    QGraphicsView *v = shell->edit()->graph()->view();
    if (!shell->isVisible() || shell->edit()->graph()->node_count() == 0)
      return;
    probe->stop();
    QElapsedTimer t;
    constexpr int kFrames = 30;
    t.start();
    for (int i = 0; i < kFrames; ++i)
      v->viewport()->repaint();
    const double paint_ms =
        static_cast<double>(t.nsecsElapsed()) / 1e6 / kFrames;
    // One staged edit rebuilds the edge batches.
    const auto &ids = shell->database()->frames()->pair().ids();
    t.restart();
    if (ids.size() > 2)
      shell->database()->editor().add(
          loop_closure(ids.front(), ids[ids.size() / 2], 9.0));
    QApplication::processEvents();
    const double edit_ms = static_cast<double>(t.nsecsElapsed()) / 1e6;
    std::fprintf(
        stderr, "%s\n",
        qPrintable(QString("rediger-stress: %1 nodes, %2 stored edges, "
                           "%3x%4 px: %5 ms/frame (%6 fps); a staged edit "
                           "%7 ms")
                       .arg(shell->edit()->graph()->node_count())
                       .arg(stored)
                       .arg(v->viewport()->width())
                       .arg(v->viewport()->height())
                       .arg(paint_ms, 0, 'f', 2)
                       .arg(1000.0 / std::max(paint_ms, 1e-3), 0, 'f', 0)
                       .arg(edit_ms, 0, 'f', 2)));
  });
  probe->start();
  return shell;
}

/// Writes a deterministic pose graph to the project COPY: odometry along
/// the capture plus @p loops loop closures / panorama edges with residuals
/// spanning the χ² scale, metrics, every status, and legacy rows without
/// metrics — so every colour mode has something to show. One loop closure
/// is bogus (the motivating case): it jumps across the capture with a
/// residual far past the scale, and must pop out of the normal ones.
void seed_graph(const QString &path, std::size_t loops) {
  if (path.isEmpty())
    return;
  reusex::ProjectDB db(path.toStdString());
  const auto ids = db.sensor_frame_ids();
  std::uint64_t x = 0x9E3779B97F4A7C15ULL; // fixed seed: the same shot
  auto next = [&x] {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    return x;
  };
  const auto unit = [&next](double hi) {
    return static_cast<double>(next() % 1000) / 1000.0 * hi;
  };
  reusex::ProjectDB::Transaction tx(db);
  for (std::size_t i = 1; i < ids.size(); ++i) {
    reusex::ProjectDB::PoseGraphEdge e;
    e.from_node_id = ids[i - 1];
    e.to_node_id = ids[i];
    e.edge_type = "odometry";
    e.source = "odometry_init";
    e.status = "auto";
    e.weight = 100.0;
    e.residual = 0.2 + unit(2.0);
    e.deviation_m = unit(0.05);
    e.deviation_deg = unit(2.0);
    e.seed_disagreement_m = unit(0.1);
    e.seed_disagreement_deg = unit(2.5);
    db.upsert_pose_graph_edge(e, /*overwrite_rejected=*/true);
  }
  for (std::size_t k = 0; k < loops && ids.size() > 2; ++k) {
    const int a = ids[next() % ids.size()];
    const int b = ids[next() % ids.size()];
    if (a == b)
      continue;
    reusex::ProjectDB::PoseGraphEdge e;
    e.from_node_id = a;
    e.to_node_id = b;
    e.edge_type = k % 9 == 0 ? "panorama" : "loop_closure";
    e.status = k % 7 == 0 ? "rejected" : k % 5 == 0 ? "confirmed" : "auto";
    e.weight = 25.0;
    if (k % 11 == 0) {
      e.source = "legacy"; // no measurement, no metrics
    } else {
      e.source = "orb";
      // Mostly inside the χ² gate, some across the ramp: a sound graph.
      e.residual = k % 6 == 0 ? unit(30.0) : unit(7.0);
      e.deviation_m = unit(0.4);
      e.deviation_deg = unit(6.0);
      e.seed_disagreement_m = unit(0.8);
      e.seed_disagreement_deg = unit(12.0);
    }
    db.upsert_pose_graph_edge(e, /*overwrite_rejected=*/true);
  }
  if (ids.size() > 4) {
    // Across the building: the two of 16 frames along the capture that lie
    // farthest apart.
    std::vector<std::pair<int, std::array<double, 3>>> at;
    for (std::size_t i = 0; i < 16; ++i) {
      const int id = ids[i * ids.size() / 16];
      if (db.has_sensor_frame_pose(id)) {
        const auto p = db.sensor_frame_pose(id);
        at.push_back({id, {p[3], p[7], p[11]}});
      }
    }
    int from = ids[ids.size() / 8], to = ids[ids.size() * 3 / 4];
    double best = -1.0;
    for (std::size_t i = 0; i < at.size(); ++i)
      for (std::size_t j = i + 1; j < at.size(); ++j) {
        const auto &p = at[i].second, &q = at[j].second;
        const double d = std::hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
        if (d > best) {
          best = d;
          from = at[i].first;
          to = at[j].first;
        }
      }
    reusex::ProjectDB::PoseGraphEdge e;
    e.from_node_id = from;
    e.to_node_id = to;
    e.edge_type = "loop_closure";
    e.source = "orb";
    e.status = "auto";
    e.weight = 25.0;
    e.residual = 412.0; // 0.5·χ² — sixty-five times the gate
    e.deviation_m = 2.4;
    e.deviation_deg = 38.0;
    e.seed_disagreement_m = 3.1;
    e.seed_disagreement_deg = 41.0;
    db.upsert_pose_graph_edge(e, /*overwrite_rejected=*/true);
    std::fprintf(stderr,
                 "seed-graph: bogus loop closure %d -> %d, residual %g\n",
                 e.from_node_id, e.to_node_id, e.residual);
  }
  tx.commit();
}

/// The staged states: a confirmation, a rejection and a new ICP-measured
/// edge on the rejected edge's pair, with the banner counting them.
QWidget *make_rediger_staged(const PageContext &ctx) {
  seed_graph(ctx.project_path, 40);
  auto *shell = rediger_shell(ctx);
  settle();
  EdgeEditor &ed = shell->edges();
  for (const auto &v : ed.edits().effective())
    if (v.edge.status == "auto" && v.edge.key.type == "loop_closure") {
      ed.confirm(v.edge.key);
      break;
    }
  for (const auto &v : ed.edits().effective())
    if (v.edge.status == "auto" && v.edge.key.type == "odometry") {
      ed.reject(v.edge.key);
      shell->edit()->set_pair(v.edge.key.from, v.edge.key.to);
      break;
    }
  settle(); // the pair's clouds loaded
  click_strip_button(shell, {"ICP-forfin"});
  settle(); // ICP landed, B placed
  click_strip_button(shell, {"Tilføj kant", "Erstat måling"});
  return shell;
}

QWidget *make_rediger_colour(const PageContext &ctx, EdgeColourMode mode,
                             bool rejected = false) {
  seed_graph(ctx.project_path, 40);
  auto *shell = make_pg(ctx);
  settle();
  shell->edit()->graph()->set_show_rejected(rejected);
  shell->edit()->graph()->set_colour_mode(mode);
  return shell;
}
QWidget *make_rediger_type(const PageContext &ctx) {
  return make_rediger_colour(ctx, EdgeColourMode::type);
}
QWidget *make_rediger_status(const PageContext &ctx) {
  return make_rediger_colour(ctx, EdgeColourMode::status, true);
}
QWidget *make_rediger_residual(const PageContext &ctx) {
  return make_rediger_colour(ctx, EdgeColourMode::residual);
}
QWidget *make_rediger_deviation(const PageContext &ctx) {
  return make_rediger_colour(ctx, EdgeColourMode::deviation);
}
QWidget *make_rediger_seed(const PageContext &ctx) {
  return make_rediger_colour(ctx, EdgeColourMode::seed);
}

/// Click a Rediger toolbar button (on the copy: the run WRITES to it) and
/// wait for the job and the reload after it. The staged demo edits are
/// dropped first, or the unsaved-edits prompt would block the shot.
AppShell *run_from_rediger(const PageContext &ctx, const QString &button) {
  auto *shell = make_pg(ctx);
  settle();
  shell->edit()->discard_edits();
  for (auto *b : shell->edit()->findChildren<QPushButton *>())
    if (b->text() == button)
      b->click();
  settle(600000);
  return shell;
}

/// "Optimer" from the Rediger toolbar on the copy (WRITES poses and edge
/// metrics), shown coloured by residual once it has finished; A and B
/// survive the reload after it.
QWidget *make_rediger_optimize(const PageContext &ctx) {
  auto *shell = run_from_rediger(ctx, "Optimer");
  shell->edit()->graph()->set_colour_mode(EdgeColourMode::residual);
  return shell;
}

/// "Find kanter" from the Rediger toolbar on the copy (WRITES loop edges):
/// the toolbar reports what the run added, skipped and rejected.
QWidget *make_rediger_discover(const PageContext &ctx) {
  auto *shell = run_from_rediger(ctx, "Find kanter");
  shell->edit()->graph()->set_colour_mode(EdgeColourMode::type);
  return shell;
}

/// A click on the first drawn stored loop closure of the seeded graph: it
/// is highlighted, the inspector shows it and A and B move to its ends.
QWidget *make_rediger_click(const PageContext &ctx) {
  seed_graph(ctx.project_path, 40);
  auto *shell = make_pg(ctx);
  settle();
  for (const auto &e : shell->database()->editor().edits().base())
    if (e.key.type == "loop_closure" && e.status != "rejected") {
      shell->edit()->graph()->click_edge(e.key);
      break;
    }
  return shell;
}

/// The edge list sorted by residual with its first row selected: the graph
/// highlights that edge and A/B follow.
QWidget *make_rediger_list(const PageContext &ctx) {
  auto *shell = static_cast<AppShell *>(
      make_rediger_colour(ctx, EdgeColourMode::residual));
  EdgeListPanel *list = shell->edit()->edges();
  if (list->row_count() > 0)
    list->view()->selectRow(0); // -> edge_activated
  return shell;
}

/// Reproduces "the nodes disappear when zoomed in" (spec §5.2): zoom about
/// the middle node by setting the view transform directly (no clamp), and
/// count the pixels of the node's own colour within 6 px of where it should
/// be. stderr gets one line per level; 0 means the node vanished.
QWidget *make_rediger_zoom(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  auto *probe = new QTimer(shell);
  probe->setInterval(50);
  QObject::connect(probe, &QTimer::timeout, shell, [shell, probe] {
    GraphPanel *pg = shell->edit()->graph();
    PoseGraphView *v = pg->view();
    if (!shell->isVisible() || pg->node_count() == 0)
      return;
    probe->stop();
    const QPointF at = pg->node_scene_pos(pg->node_count() / 2);
    const QColor ink = theme().color("--color-on-chrome-muted");
    v->fit();
    const double fit_px_per_unit = 1.0 / v->units_per_pixel();
    for (const double level : {0.0625, 0.25, 1.0, 4.0, 16.0, 64.0, 256.0,
                               1024.0, 4096.0, 16384.0, 65536.0}) {
      v->setTransform(QTransform::fromScale(fit_px_per_unit * level,
                                            fit_px_per_unit * level));
      v->centerOn(at);
      QApplication::processEvents();
      const QPixmap shot = v->viewport()->grab();
      const QImage img = shot.toImage();
      const double dpr = shot.devicePixelRatio();
      const QPoint c = v->mapFromScene(at);
      int lit = 0;
      for (int dy = -6; dy <= 6; ++dy)
        for (int dx = -6; dx <= 6; ++dx) {
          const QPoint p((c.x() + dx) * dpr, (c.y() + dy) * dpr);
          if (!img.rect().contains(p))
            continue;
          const QColor px = img.pixelColor(p);
          lit += std::abs(px.red() - ink.red()) +
                     std::abs(px.green() - ink.green()) +
                     std::abs(px.blue() - ink.blue()) <
                 40;
        }
      std::fprintf(stderr,
                   "rediger-zoom: x%g upp=%.3g centre=(%d,%d) node-pixels=%d\n",
                   level, v->units_per_pixel(), c.x(), c.y(), lit);
    }
  });
  probe->start();
  return shell;
}

/// Yaw 30° (WRITES project_settings to the copy) and reads it back.
QWidget *make_rediger_yaw(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  settle();
  yaw_30_and_report(shell, ctx, "rediger-yaw");
  return shell;
}

/// Zoomed in on a node, then a 30° yaw: the scale and the centre must stay
/// (the yaw is for lining up the plan close up). stderr: how far the node
/// moved from the centre, and the scale before and after.
QWidget *make_rediger_yaw_zoomed(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  settle();
  GraphPanel *g = shell->edit()->graph();
  PoseGraphView *v = g->view();
  if (g->node_count() == 0)
    return shell;
  shell->show();
  QApplication::processEvents();
  const int k = g->node_count() / 2;
  v->zoom_on(g->node_scene_pos(k), 8.0);
  const double upp = v->units_per_pixel();
  const auto from_centre = [g, v, k] {
    const QPointF c = v->mapToScene(v->viewport()->rect().center());
    const QPointF off = (g->node_scene_pos(k) - c) / v->units_per_pixel();
    return std::hypot(off.x(), off.y());
  };
  const double before = from_centre();
  shell->view_settings().set_yaw_deg(30.0);
  std::fprintf(stderr,
               "rediger-yaw-zoomed: node %.2f px from centre before, %.2f "
               "after; upp %.4g -> %.4g\n",
               before, from_centre(), upp, v->units_per_pixel());
  return shell;
}

/// A reload of the same project (what Optimer and F5 do) right after the
/// user zoomed in, clicked an edge and turned the yaw: the zoom, the centre,
/// the highlighted edge, A/B and the yaw all survive it (WRITES the yaw to
/// the copy). stderr: before/after.
QWidget *make_rediger_reload(const PageContext &ctx) {
  seed_graph(ctx.project_path, 40);
  auto *shell = make_pg(ctx);
  settle();
  shell->show();
  QApplication::processEvents();
  GraphPanel *g = shell->edit()->graph();
  PoseGraphView *v = g->view();
  auto *session = shell->findChild<ProjectSession *>();
  if (g->node_count() == 0 || !session)
    return shell;
  shell->edit()->discard_edits(); // no unsaved-edits prompt on reload
  for (const auto &e : shell->edges().edits().base())
    if (e.key.type == "loop_closure" && e.status != "rejected") {
      g->click_edge(e.key);
      break;
    }
  const int k = g->node_count() / 2;
  v->zoom_on(g->node_scene_pos(k), 6.0);
  shell->view_settings().set_yaw_deg(20.0);
  const auto centre = [v] {
    return v->mapToScene(v->viewport()->rect().center());
  };
  const QPointF c0 = centre();
  const double upp0 = v->units_per_pixel();
  const auto h0 = g->highlight();
  const int a0 = shell->edit()->a(), b0 = shell->edit()->b();
  session->reload(); // inside the yaw's 500 ms debounce
  settle();
  QApplication::processEvents();
  const QPointF d = (centre() - c0) / v->units_per_pixel();
  const auto h1 = g->highlight();
  std::fprintf(stderr,
               "rediger-reload: upp %.4g -> %.4g, centre moved %.2f px, "
               "user view %s; highlight %s; A/B %d/%d -> %d/%d; yaw %.1f "
               "(graph %.1f)\n",
               upp0, v->units_per_pixel(), std::hypot(d.x(), d.y()),
               v->user_view() ? "kept" : "LOST",
               h0 && h1 && same_edge_key(*h0, *h1) ? "kept" : "LOST", a0, b0,
               shell->edit()->a(), shell->edit()->b(),
               shell->view_settings().yaw_deg(), g->projection().yaw_deg);
  return shell;
}

/// The Rediger round trip on the copy (WRITES edges, poses and metrics):
/// pick the odometry edge between two consecutive frames in the edge list,
/// move B by hand, ICP-forfin from there, "Erstat måling", Gem, Optimer.
/// ICP stages B -> A with A < B, the reversed direction: the list row, the
/// graph highlight, the inspector card and A/B must hold through staging,
/// the save and Optimer's reload. stderr: each step, then the stored row.
/// $RUX_ROUNDTRIP_A picks A (B is the next frame); $RUX_ROUNDTRIP_UNMOVED
/// runs ICP from B's stored pose instead of a hand-moved one.
QWidget *make_rediger_roundtrip(const PageContext &ctx) {
  auto *shell = sized(make_shell(ctx, Open::project));
  shell->show_page(Workspace::edit);
  settle();
  EditWorkspace *ed = shell->edit();
  EdgeListPanel *list = ed->edges();
  const auto &ids = shell->database()->frames()->pair().ids();
  if (ids.size() < 2 || ctx.project_path.isEmpty())
    return shell;
  // The middle pair, or A from $RUX_ROUNDTRIP_A (a pair ICP can fit on a
  // real capture) and the frame after it as B.
  std::size_t at = ids.size() / 2;
  if (const int want = qEnvironmentVariableIntValue("RUX_ROUNDTRIP_A");
      want > 0) {
    const auto it = std::find(ids.begin(), ids.end(), want);
    if (it != ids.end() && it + 1 != ids.end())
      at = static_cast<std::size_t>(it - ids.begin());
  }
  const int a = ids[at], b = ids[at + 1];
  const EdgeKey key{a, b, "odometry"};
  const auto report = [&](const char *step) {
    const auto *sel = list->view()->selectionModel();
    const QModelIndexList rows = sel ? sel->selectedRows() : QModelIndexList();
    const QString row =
        rows.isEmpty() ? QString("none") : rows.front().data().toString();
    const auto h = ed->graph()->highlight();
    QString strip;
    for (const auto &v : ed->editor().edits().between(ed->a(), ed->b()))
      strip += QString(" %1->%2/%3")
                   .arg(v.edge.key.from)
                   .arg(v.edge.key.to)
                   .arg(QString::fromStdString(v.edge.key.type));
    std::fprintf(
        stderr,
        "rediger-roundtrip: %s: A/B %d/%d, list row '%s', "
        "highlight %s, strip%s, pending %d; inspector '%s' %s\n",
        step, ed->a(), ed->b(), qPrintable(row),
        h ? qPrintable(QString("%1->%2").arg(h->from).arg(h->to)) : "none",
        qPrintable(strip), ed->editor().edits().count(),
        qPrintable(ed->selection().title), qPrintable(ed->selection().pill));
  };
  list->select_edge(key);            // the row ...
  if (!ed->graph()->click_edge(key)) // ... and what activating it does
    ed->set_pair(a, b);              // (no such edge yet: a new one)
  settle();                          // both clouds loaded
  report("picked");
  if (!qEnvironmentVariableIsSet("RUX_ROUNDTRIP_UNMOVED"))
    ed->overlay()->set_offset({0.02, -0.01, 0.0, 1.0, 0.0, 0.0});
  click_strip_button(shell, {"ICP-forfin"});
  settle();
  for (auto *c : ed->strip()->findChildren<QComboBox *>())
    if (c->objectName() == "edgeType")
      c->setCurrentIndex(c->findData(QString("odometry")));
  click_strip_button(shell, {"Tilføj kant", "Erstat måling"});
  report("staged");
  ed->save_edits();
  settle();
  report("saved");
  for (auto *btn : ed->findChildren<QPushButton *>())
    if (btn->text() == "Optimer")
      btn->click();
  settle(600000);
  report("optimized");
  reusex::ProjectDB db(ctx.project_path.toStdString(), /*readOnly=*/true);
  for (const auto &e : db.list_pose_graph_edges())
    if (std::minmax(e.from_node_id, e.to_node_id) == std::minmax(a, b))
      std::fprintf(stderr,
                   "rediger-roundtrip: stored %d -> %d %s source=%s "
                   "status=%s transform=%s information=%s weight=%g "
                   "residual=%g deviation=%s\n",
                   e.from_node_id, e.to_node_id, e.edge_type.c_str(),
                   e.source.c_str(), e.status.c_str(),
                   e.transform ? "yes" : "no", e.information ? "yes" : "no",
                   e.weight, e.residual, e.deviation_m ? "yes" : "no");
  return shell;
}

/// A yaw set and the project closed at once, inside the 500 ms debounce:
/// the close writes it rather than dropping it (WRITES to the copy).
QWidget *make_rediger_yaw_close(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  settle();
  shell->view_settings().set_yaw_deg(45.0);
  if (auto *session = shell->findChild<ProjectSession *>())
    session->close();
  settle();
  if (!ctx.project_path.isEmpty()) {
    reusex::ProjectDB db(ctx.project_path.toStdString(), /*readOnly=*/true);
    std::fprintf(stderr, "rediger-yaw-close: stored view.yaw_deg=%s\n",
                 db.setting(kViewYawSetting).value_or("<none>").c_str());
  }
  return shell;
}

QWidget *make_pipeline(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->show_page(Workspace::pipeline);
  PipelineWorkspace *p = shell->pipeline();
  p->select_stage(reusex::pipeline::JobStage::planes);
  p->set_field("angle_threshold", 20.0);
  p->set_field("plane_dist_threshold", 0.04);
  p->set_field("filter", "rooms in [1, 2]");
  return shell;
}

/// Runs `create planes` for real on the project copy (WRITES to it) and
/// shows the finished run, its log tail and its command.
QWidget *make_pipeline_run(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->show_page(Workspace::pipeline);
  PipelineWorkspace *p = shell->pipeline();
  p->select_stage(reusex::pipeline::JobStage::planes);
  p->set_field("angle_threshold", 20.0);
  if (p->run())
    settle(600000);
  return shell;
}

/// Runs `planes` and cancels it as soon as the worker has started it: the
/// shot shows the cancelled run, and stderr says how the job ended. A check
/// of JobRunner's cancel as the Qt client drives it.
QWidget *make_pipeline_cancel(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->show_page(Workspace::pipeline);
  PipelineWorkspace *p = shell->pipeline();
  p->select_stage(reusex::pipeline::JobStage::planes);
  if (!p->run())
    return shell;
  const auto last_status = [p]() -> std::optional<reusex::pipeline::JobStatus> {
    if (auto runner = p->runner_handle().lock()) {
      const auto jobs = runner->jobs();
      if (!jobs.empty())
        return jobs.back().status;
    }
    return std::nullopt;
  };
  QElapsedTimer t;
  t.start();
  while (last_status() != reusex::pipeline::JobStatus::running &&
         p->is_running() && t.elapsed() < 60000) {
    QApplication::processEvents(QEventLoop::AllEvents);
    QThread::msleep(1);
  }
  p->cancel_running();
  settle();
  const auto status = last_status();
  std::fprintf(
      stderr, "pipeline-cancel: job ended %s\n",
      status ? std::string(reusex::pipeline::to_string(*status)).c_str() : "?");
  return shell;
}

QWidget *make_log(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->show_page(Workspace::log);
  shell->log()->select_row(0);
  return shell;
}

QWidget *make_log_filter(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->show_page(Workspace::log);
  LogFilter f;
  f.text = "planes";
  f.status = LogStatusFilter::success;
  shell->log()->set_filter(f);
  shell->log()->select_row(0);
  return shell;
}

QWidget *make_palette_open(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->open_palette("pro");
  return shell;
}

QWidget *make_palette_all(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->open_palette();
  return shell;
}

} // namespace

void register_shell_pages() {
  register_page({"start", "Startside med åbent projekt og seneste projekter",
                 make_start});
  register_page({"start-readonly", "Startside med et skrivebeskyttet projekt",
                 make_start_readonly});
  register_page({"start-empty", "Startside uden projekt", make_start_empty});
  register_page(
      {"start-error", "Startside efter en fejlet åbning", make_start_error});
  register_page({"shell-empty", "Skallen uden projekt: Database uden projekt",
                 make_shell_empty});
  register_page(
      {"shell-project", "Skallen med projekt på 3D", make_shell_project});
  register_page({"db-frames",
                 "Database: A/B-billeder med mærkater og filmstrimmel",
                 make_db_frames});
  register_page({"db-depth", "Database: dybdelaget", make_db_depth});
  register_page(
      {"db-confidence", "Database: konfidenslaget", make_db_confidence});
  register_page(
      {"db-table", "Database: tabelvisning af sensor_frames", make_db_table});
  register_page({"db-log", "Database: pipeline-loggen", make_db_log});
  register_page({"db-cloud", "Database: en punktsky i træet og inspektøren",
                 make_db_cloud});
  register_page({"db-panorama",
                 "Oversigt: et panorama i pan/zoom-visning (skriver et "
                 "syntetisk panorama til projektkopien)",
                 make_db_panorama});
  register_page({"3d", "3D: punktskyen i perspektiv med snit", make_3d_page});
  register_page({"3d-pick",
                 "3D: farvet efter rum, med forklaring og et valgt punkt",
                 make_3d_pick});
  register_page({"3d-plan",
                 "3D: plantegning farvet efter plan, med kamerafrustummer",
                 make_3d_plan});
  register_page({"3d-yaw",
                 "3D: plantegningen drejet 30° med projektets visning "
                 "(skriver til projektkopien)",
                 make_3d_yaw});
  register_page(
      {"3d-labels", "3D: semantiske mærkater ovenfra", make_3d_labels});
  register_page({"rediger", "Rediger: posegraf, A/B og ventende løkkelukninger",
                 make_rediger});
  register_page({"rediger-pair", "Rediger: billederne af A og B i farve",
                 make_rediger_pair});
  register_page({"rediger-pair-depth", "Rediger: billederne af A og B i dybde",
                 make_rediger_pair_depth});
  register_page({"rediger-pair-half",
                 "Rediger: kun A valgt; B's side siger, hvad der mangler",
                 make_rediger_pair_half});
  register_page({"rediger-drag",
                 "Rediger: boksen trukket med musen (kræver --gl)",
                 make_rediger_drag});
  register_page({"rediger-offset",
                 "Rediger: B flyttet 20 cm i x og drejet 5° i 3D-overlayet",
                 make_rediger_offset});
  register_page({"rediger-icp",
                 "Rediger: B flyttet i hånden, så ICP-forfin derfra",
                 make_rediger_icp});
  register_page({"rediger-icp-moved",
                 "Rediger: B flyttet efter ICP — resultatet gælder ikke mere",
                 make_rediger_icp_moved});
  register_page(
      {"rediger-xz", "Rediger: grafen i opstalt, set fra −y", make_rediger_xz});
  register_page(
      {"rediger-yz", "Rediger: grafen i opstalt, set fra +x", make_rediger_yz});
  register_page({"rediger-yaw",
                 "Rediger: visningen drejet 30° (skriver til projektkopien)",
                 make_rediger_yaw});
  register_page({"rediger-yaw-zoomed",
                 "Rediger: indzoomet, så drejet 30° — skala og midte "
                 "bevares (skriver til projektkopien)",
                 make_rediger_yaw_zoomed});
  register_page({"rediger-yaw-close",
                 "Rediger: drejet og lukket straks — drejningen gemmes "
                 "alligevel (skriver til projektkopien)",
                 make_rediger_yaw_close});
  register_page({"rediger-panes",
                 "Rediger: kantliste og billeder skjult, kun grafen og "
                 "højre kolonne",
                 make_rediger_panes});
  register_page({"rediger-pending",
                 "Rediger: ICP-resultat og ventende kantændringer",
                 make_rediger_pending});
  register_page({"rediger-readonly",
                 "Rediger: gem afvist på et skrivebeskyttet projekt",
                 make_rediger_readonly});
  register_page({"rediger-staged",
                 "Rediger: en bekræftet, en afvist og en ny ICP-målt kant "
                 "(skriver grafen til projektkopien)",
                 make_rediger_staged});
  register_page({"rediger-saved",
                 "Rediger: en gemt kant (skriver til projektkopien)",
                 make_rediger_saved});
  register_page({"rediger-stress",
                 "Rediger: 10.000 kanter (skriver til projektkopien), måler "
                 "tegnetid",
                 make_rediger_stress});
  register_page({"rediger-click",
                 "Rediger: klik på en kant fremhæver den og sætter A og B, "
                 "seedet graf (skriver til projektkopien)",
                 make_rediger_click});
  register_page({"rediger-optimize",
                 "Rediger: Optimer kørt fra værktøjslinjen, kanter farvet "
                 "efter residual (skriver til projektkopien)",
                 make_rediger_optimize});
  register_page({"rediger-discover",
                 "Rediger: Find kanter kørt fra værktøjslinjen, med "
                 "optællingen (skriver til projektkopien)",
                 make_rediger_discover});
  register_page({"rediger-type",
                 "Rediger: kanter farvet efter type, seedet graf (skriver "
                 "til projektkopien)",
                 make_rediger_type});
  register_page({"rediger-status",
                 "Rediger: kanter farvet efter status med afviste vist, "
                 "seedet graf (skriver til projektkopien)",
                 make_rediger_status});
  register_page({"rediger-residual",
                 "Rediger: kanter farvet efter residual, seedet graf (skriver "
                 "til projektkopien)",
                 make_rediger_residual});
  register_page({"rediger-deviation",
                 "Rediger: kanter farvet efter afvigelse, seedet graf "
                 "(skriver til projektkopien)",
                 make_rediger_deviation});
  register_page({"rediger-seed",
                 "Rediger: kanter farvet efter frøafvigelse, seedet graf "
                 "(skriver til projektkopien)",
                 make_rediger_seed});
  register_page({"rediger-list",
                 "Rediger: kantlisten sorteret efter residual med første "
                 "række valgt, seedet graf (skriver til projektkopien)",
                 make_rediger_list});
  register_page({"rediger-roundtrip",
                 "Rediger: kant fra listen, flyt B, ICP-forfin, Erstat måling, "
                 "Gem og Optimer (skriver til projektkopien)",
                 make_rediger_roundtrip});
  register_page({"rediger-reload",
                 "Rediger: zoom, valgt kant, A/B og drejning overlever en "
                 "genindlæsning af projektet (skriver til projektkopien)",
                 make_rediger_reload});
  register_page({"rediger-zoom",
                 "Rediger: zoomer ind på én node og tæller dens pixels "
                 "(reproduktion af forsvindende noder)",
                 make_rediger_zoom});
  register_page({"pipeline",
                 "Pipeline: planer med ændrede parametre og rux-kommandoen",
                 make_pipeline});
  register_page({"pipeline-run",
                 "Pipeline: kører create planes (skriver til projektkopien)",
                 make_pipeline_run});
  register_page({"pipeline-cancel",
                 "Pipeline: create planes afbrudt midt i kørslen (skriver til "
                 "projektkopien)",
                 make_pipeline_cancel});
  register_page({"log", "Log: pipeline-loggen med en valgt kørsel", make_log});
  register_page(
      {"log-filter", "Log: filtreret på tekst og status", make_log_filter});
  register_page({"palette-open", "Kommandopaletten med søgningen \"pro\"",
                 make_palette_open});
  register_page({"palette-all", "Kommandopaletten uden søgning (grupperet)",
                 make_palette_all});
}

} // namespace rux::qt::gallery
