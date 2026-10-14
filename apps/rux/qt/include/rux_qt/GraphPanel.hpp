// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's graph pane (spec §5.2): every posed frame as a node at its world
// translation, turned by the project's view yaw and projected onto the
// chosen view plane (XY plan, XZ / YZ elevations), with a fixed-length tick
// for the camera's heading; the capture path through the nodes; and the
// pose-graph edges, coloured by type, status, residual, deviation or seed
// disagreement (edge_colouring.hpp) with a legend generated from what is
// drawn; staged and (when "Vis afviste" shows them) rejected edges dashed,
// the clicked edge highlighted. A click on a node is handed to Rediger's A/B
// picker; a click on an edge sets both. Wheel zooms about the cursor, a drag
// pans, "Tilpas" (or F) fits. The yaw is display only (ViewSettings); the plane
// is a per-user QSettings choice (rediger/plane), as are the colour mode and
// the rejected toggle (rediger/colour, rediger/show_rejected).
//
// Positions are read off the GUI thread with a read-only ProjectDB of its
// own, like every other bulk read of the client.

#include <rux_qt/database_logic.hpp>
#include <rux_qt/edge_colouring.hpp>
#include <rux_qt/graph_view.hpp>
#include <rux_qt/selection.hpp>
#include <rux_qt/workspace_logic.hpp>

#include <QGraphicsView>
#include <QWidget>

#include <array>
#include <optional>
#include <vector>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QFrame;
class QVBoxLayout;
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
  /// The lower-left corner an overlay covers (the legend card): fit() keeps
  /// the graph out of it. Refits when the view is not the user's.
  void set_corner_reserve(QSize px);
  /// Zoom by @p factor and centre on @p scene_pos, as the user would (the
  /// gallery): a resize or a yaw change then keeps the view.
  void zoom_on(const QPointF &scene_pos, double factor);
  /// Zoomed or panned by the user since the last fit.
  bool user_view() const { return user_view_; }
  /// Put back a view the user had (a same-project reload): @p t's zoom,
  /// centred on @p centre, still counted as the user's.
  void restore_view(const QTransform &t, const QPointF &centre);
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
  QSize reserve_;
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
  /// Click programmatically at a node (the gallery).
  void click_node(int id, bool as_b = false);
  /// Select edge @p key as a click would (the edge list, the gallery):
  /// inspector, highlight, A and B. No edge_clicked. False when there is no
  /// such edge.
  bool click_edge(const EdgeKey &key);
  void set_plane(ViewPlane plane);
  const ViewProjection &projection() const { return projection_; }
  void set_colour_mode(EdgeColourMode mode);
  EdgeColourMode colour_mode() const { return mode_; }
  /// Draw rejected edges (dashed); hidden by default.
  void set_show_rejected(bool on);
  /// Paint @p key over the rest (the clicked / listed edge); nullopt: none.
  void set_highlight(std::optional<EdgeKey> key);
  const std::optional<EdgeKey> &highlight() const { return highlight_; }

    signals:
  /// A node was clicked (@p as_b: Shift held); Rediger picks the slot.
  void frame_requested(int id, bool as_b);
  /// An edge was clicked: A and B become its ends.
  void pair_requested(int a_id, int b_id);
  void selection_changed(const rux::qt::Selection &selection);
  void colour_mode_changed(rux::qt::EdgeColourMode mode);
  /// An edge was clicked on the canvas.
  void edge_clicked(const rux::qt::EdgeKey &key);

    protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

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
  void rebuild_legend(const std::vector<EdgeView> &all);
  void place_legend();
  void place_markers();
  void on_click(const QPointF &pos, Qt::KeyboardModifiers mods);
  /// Inspect, highlight and pair edge @p key (drawn or not: the list can
  /// show rejected edges the canvas hides).
  bool select_edge(const EdgeKey &key);
  struct EdgeCard {
    Selection selection;
    EdgeKey stored; ///< the key as the store holds it (from < to)
  };
  /// The inspector card of @p key, or nullopt when there is no such edge.
  std::optional<EdgeCard> edge_card(const EdgeKey &key);
  /// Rebuild the highlighted edge's card after an edit, save or reload.
  void refresh_edge_card();
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
  EdgeColourMode mode_ = EdgeColourMode::type;
  bool show_rejected_ = false;
  std::optional<EdgeKey> highlight_;
  /// The project the canvas last showed, and the user's zoom and centre on
  /// it: a reload of the same project (Optimer, F5) keeps both, as well as
  /// the highlighted edge; another project starts fitted and unhighlighted.
  QString shown_path_;
  struct KeptView {
    QTransform transform;
    QPointF centre;
  };
  std::optional<KeptView> kept_view_;
  std::vector<EdgeView> drawn_; ///< the edges on the canvas, effective
  QComboBox *colour_ = nullptr;
  QCheckBox *rejected_ = nullptr;
  /// The legend: an overlay in the canvas's lower-left corner, one row per
  /// entry, so it never widens Rediger's column.
  QFrame *legend_ = nullptr;
  QVBoxLayout *legend_layout_ = nullptr;
  std::vector<QGraphicsItem *> edge_items_;
  QGraphicsItem *nodes_item_ = nullptr;
  QGraphicsItem *marker_a_ = nullptr;
  QGraphicsItem *marker_b_ = nullptr;
  int a_ = -1, b_ = -1;
  Selection selection_;
};

} // namespace rux::qt
