// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Posegraf workspace (Stream Q, Q3): every posed frame as a node at its
// pose's x/y, the capture path through them, and the pose-graph edges —
// stored ones by type, pending ones (Rediger's staged edits) dashed. A click
// on a node is handed to Rediger's A/B picker; a click on an edge sets both.
// Wheel zooms about the cursor, a drag pans, "Tilpas" (or F) fits.
//
// Positions are read off the GUI thread with a read-only ProjectDB of its
// own, like every other bulk read of the client.

#include <rux_qt/selection.hpp>
#include <rux_qt/workspace_logic.hpp>

#include <QGraphicsView>
#include <QWidget>

#include <vector>

class QLabel;
class QGraphicsItem;
class QGraphicsPathItem;

namespace rux::qt {

class EdgeEditor;
class ProjectSession;

/// The canvas: zoom about the cursor, drag to pan, click to pick.
class PoseGraphView : public QGraphicsView {
  Q_OBJECT
    public:
  explicit PoseGraphView(QWidget *parent = nullptr);
  void fit();
  /// What fit() frames (the nodes; screen-sized markers would skew it).
  void set_content_rect(const QRectF &r) { content_ = r; }
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

class PoseGraphWorkspace : public QWidget {
  Q_OBJECT
    public:
  PoseGraphWorkspace(ProjectSession &session, EdgeEditor &editor,
                     QWidget *parent = nullptr);
  ~PoseGraphWorkspace() override;

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

    signals:
  /// A node was clicked (@p as_b: Shift held); Rediger picks the slot.
  void frame_requested(int id, bool as_b);
  /// An edge was clicked: A and B become its ends.
  void pair_requested(int a_id, int b_id);
  void selection_changed(const rux::qt::Selection &selection);

    private:
  struct Loaded;
  void reset();
  void loaded(std::shared_ptr<Loaded> data);
  void rebuild_scene();
  void rebuild_edges();
  void place_markers();
  void on_click(const QPointF &pos, Qt::KeyboardModifiers mods);
  void update_meta();

  ProjectSession &session_;
  EdgeEditor &editor_;
  unsigned generation_ = 0;
  bool active_ = false;
  bool loading_ = false;
  PoseGraphView *view_ = nullptr;
  QLabel *meta_ = nullptr;
  QLabel *empty_ = nullptr;
  QWidget *legend_ = nullptr;

  std::vector<GraphNode> nodes_; ///< id order
  std::vector<int> scan_of_;     ///< per node
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
