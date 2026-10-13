// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The A<->B pair strip of Rediger: the pose-graph edges between the two
// frames (stored and pending), how far apart they are, ICP refine, add and
// delete.

#include <rux_qt/database_logic.hpp>
#include <rux_qt/selection.hpp>

#include <QFrame>

#include <map>

class QComboBox;
class QLabel;
class QPushButton;
class QVBoxLayout;

namespace rux::qt {

class EdgeEditor;
class ProjectSession;

/// The A<->B link: the stored and pending edges between the two frames, how
/// far apart they are, ICP refine, add and delete.
class PairStrip : public QFrame {
  Q_OBJECT
    public:
  PairStrip(ProjectSession &session, EdgeEditor &editor,
            QWidget *parent = nullptr);
  void set_pair(int a_id, int b_id);

    signals:
  /// An edge row was clicked: show it in the inspector.
  void edge_selected(const rux::qt::Selection &selection);

    private:
  struct IcpOutcome {
    bool ok = false;
    QString error;
    double fitness = 0, inliers = 0;
    bool converged = false;
    int source_points = 0, target_points = 0;
    double center_shift_m = 0, rotation_deg = 0;
  };
  void rebuild();
  void run_icp();
  void icp_finished(int a, int b, IcpOutcome outcome);
  void add_edge();
  ProjectSession &session_;
  EdgeEditor &editor_;
  int a_ = -1, b_ = -1;
  QLabel *delta_ = nullptr;
  QWidget *rows_ = nullptr;
  QVBoxLayout *rows_layout_ = nullptr;
  QPushButton *icp_ = nullptr;
  QComboBox *type_ = nullptr;
  QPushButton *add_ = nullptr;
  QLabel *icp_line_ = nullptr;
  QLabel *note_ = nullptr;
  std::map<std::pair<int, int>, IcpOutcome> icp_results_;
  std::pair<int, int> icp_running_{-1, -1};
};

} // namespace rux::qt
