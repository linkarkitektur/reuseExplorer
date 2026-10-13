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

#include <QAbstractButton>
#include <QApplication>
#include <QButtonGroup>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QGraphicsView>
#include <QProgressBar>
#include <QPushButton>
#include <QTableView>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
QWidget *make_3d_plan(const PageContext &ctx) {
  auto *shell = make_3d(ctx, "planes");
  shell->viewer()->set_layer_visible(reusex::visualize::Layer::frustums, true);
  shell->viewer()->set_view(reusex::visualize::ViewPreset::plan);
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

/// A saved edit: stage a loop closure on the pair, then "Gem ændringer" —
/// the stored edge comes back from the file marked "Gemt". WRITES to the
/// project copy.
QWidget *make_rediger_saved(const PageContext &ctx) {
  auto *shell = rediger_shell(ctx);
  if (shell->edit()->a() < 0)
    return shell;
  shell->edges().add(
      loop_closure(shell->edit()->a(), shell->edit()->b(), 250.0));
  shell->edit()->save_edits();
  return shell;
}

/// Stress: 10 000 stored edges (WRITTEN to the project copy) — odometry
/// along the capture, then deterministic pseudo-random loop closures and
/// panorama edges — and the paint and edit cost measured, on stderr.
QWidget *make_rediger_stress(const PageContext &ctx) {
  constexpr std::size_t kEdges = 10000;
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
  }
  auto *shell = make_pg(ctx);
  // Measure once the graph is on screen at its shot size: poll until the
  // poses have loaded and the shell is shown by the gallery.
  auto *probe = new QTimer(shell);
  probe->setInterval(50);
  QObject::connect(probe, &QTimer::timeout, shell, [shell, probe] {
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
                       .arg(kEdges)
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
  register_page({"3d", "3D: punktskyen i perspektiv med snit", make_3d_page});
  register_page({"3d-pick",
                 "3D: farvet efter rum, med forklaring og et valgt punkt",
                 make_3d_pick});
  register_page({"3d-plan",
                 "3D: plantegning farvet efter plan, med kamerafrustummer",
                 make_3d_plan});
  register_page(
      {"3d-labels", "3D: semantiske mærkater ovenfra", make_3d_labels});
  register_page({"rediger", "Rediger: posegraf, A/B og ventende løkkelukninger",
                 make_rediger});
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
