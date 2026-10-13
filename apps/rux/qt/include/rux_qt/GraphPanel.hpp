// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's graph pane (spec §5.2): every posed frame as a node at its world
// translation, turned by the project's view yaw and projected onto the
// chosen view plane (XY plan, XZ / YZ elevations), with a fixed-length tick
// for the camera's heading; the capture path through the nodes; and the
// pose-graph edges — stored ones by type, pending ones (staged edits)
// dashed. A click on a node is handed to Rediger's A/B picker; a click on an
// edge sets both. Wheel zooms about the cursor, a drag pans, "Tilpas" (or F)
// fits. The yaw is display only (ViewSettings); the plane is a per-user
// QSettings choice (rediger/plane).
//
// Positions are read off the GUI thread with a read-only ProjectDB of its
// own, like every other bulk read of the client.

#include <rux_qt/graph_view.hpp>
#include <rux_qt/selection.hpp>
#include <rux_qt/workspace_logic.hpp>

#include <QGraphicsView>
#include <QWidget>

#include <array>
#include <vector>

class QButtonGroup;
class QDoubleSpinBox;
class QLabel;
class QSlider;
class QGraphicsItem;
class QGraphicsPathItem;

namespace rux::qt {

class EdgeEditor;
class ElidedLabel;
class ProjectSession;
class ViewSettings;

/// The canvas: zoom about the cursor, drag to pan, click to pick.
class PoseGraphView : public QGraphicsView {
  Q_OBJECT
    public:
  explicit PoseGraphView(QWidget *parent = nullptr);
  void fit();
  /// What fit() frames (the nodes; screen-sized markers would skew it).
  void set_content_rect(const QRectF &r);
  /// Zoom by @p factor and centre on @p scene_pos, as the user would (the
  /// gallery): a resize or a yaw change then keeps the view.
  void zoom_on(const QPointF &scene_pos, double factor);
  /// Zoomed or panned by the user since the last fit.
  bool user_view() const { return user_view_; }
  /// Scene units per screen pixel at the current zoom.
  double units_per_pixel() const;

    signals:
  void clicked_at(const QPointF &scene_pos, Qt::KeyboardModifiers mods);

    protected:
  void wheelEvent(QWheelEvent *e) override;
  void mousePressEvent(QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *e) override;
  void keyPressEvent(QKeyEvent *e) override;
  void resizeEvent(QResizeEvent *e) override;

    private:
  QPoint press_;
  /// Zoomed or panned by the user: a resize keeps their view.
  bool user_view_ = false;
  QRectF content_;
};

class GraphPanel : public QWidget {
  Q_OBJECT
    public:
  /// @p persist: remember the view plane in QSettings (not in a shot).
  GraphPanel(ProjectSession &session, EdgeEditor &editor,
             ViewSettings &view_settings, bool persist,
             QWidget *parent = nullptr);
  ~GraphPanel() override;

  /// The page is on screen: read the poses if not read yet.
  void activate();
  /// Mark the Database's current A and B.
  void set_pair(int a_id, int b_id);
  const Selection &selection() const { return selection_; }
  PoseGraphView *view() const { return view_; }
  int node_count() const { return static_cast<int>(nodes_.size()); }
  /// Scene position of node @p index (the gallery's zoom harness).
  QPointF node_scene_pos(int index) const {
    const auto &n = nodes_[static_cast<std::size_t>(index)];
    return {n.x, -n.y};
  }
  /// Click programmatically at a node / an edge (the gallery).
  void click_node(int id, bool as_b = false);
  void click_edge(int from, int to);
  void set_plane(ViewPlane plane);
  const ViewProjection &projection() const { return projection_; }

    signals:
  /// A node was clicked (@p as_b: Shift held); Rediger picks the slot.
  void frame_requested(int id, bool as_b);
  /// An edge was clicked: A and B become its ends.
  void pair_requested(int a_id, int b_id);
  void selection_changed(const rux::qt::Selection &selection);

    private:
  struct Loaded;
  /// One posed frame as loaded: id, scan, world translation, camera forward.
  struct NodeData {
    int id = 0;
    int scan = 0;
    std::array<double, 3> t{};
    std::array<double, 3> forward{};
  };
  void project();
  void set_yaw(double deg);
  void reset();
  void loaded(std::shared_ptr<Loaded> data);
  void rebuild_scene();
  void rebuild_edges();
  void place_markers();
  void on_click(const QPointF &pos, Qt::KeyboardModifiers mods);
  void update_meta();

  ProjectSession &session_;
  EdgeEditor &editor_;
  ViewSettings &view_settings_;
  bool persist_ = true;
  ViewProjection projection_;
  std::vector<NodeData> data_; ///< id order, as loaded
  std::vector<Heading> ticks_; ///< per node, after project()
  double step_ = 0.0;          ///< median_step of the projected nodes
  QButtonGroup *planes_ = nullptr;
  QSlider *yaw_slider_ = nullptr;
  QDoubleSpinBox *yaw_spin_ = nullptr;
  unsigned generation_ = 0;
  bool active_ = false;
  bool loading_ = false;
  PoseGraphView *view_ = nullptr;
  ElidedLabel *meta_ = nullptr;
  QLabel *empty_ = nullptr;

  std::vector<GraphNode> nodes_; ///< data_ projected, same order
  struct DrawnEdge {
    GraphEdge edge;
    std::string type;
    bool pending_add = false; ///< anything staged on it (not saved)
    double residual = 0, weight = 0;
  };
  std::vector<DrawnEdge> edges_;
  std::vector<QGraphicsItem *> edge_items_;
  QGraphicsItem *nodes_item_ = nullptr;
  QGraphicsItem *marker_a_ = nullptr;
  QGraphicsItem *marker_b_ = nullptr;
  int a_ = -1, b_ = -1;
  Selection selection_;
};

} // namespace rux::qt
