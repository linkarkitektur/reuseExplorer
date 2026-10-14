// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The A<->B pair strip of Rediger: the pose-graph edges between the two
// frames (stored and pending), how far apart they are, ICP refine, adding a
// measured edge, and confirming, rejecting or reverting one.

#include <rux_qt/database_logic.hpp>
#include <rux_qt/pair_alignment.hpp>
#include <rux_qt/selection.hpp>

#include <QFrame>

#include <array>
#include <functional>
#include <map>
#include <optional>

class QComboBox;
class QLabel;
class QPushButton;
class QScrollArea;
class QVBoxLayout;

namespace rux::qt {

class EdgeEditor;
class ProjectSession;

/// The A<->B link: the stored and pending edges between the two frames, how
/// far apart they are, ICP refine, add a measured edge (ICP's while its fit
/// is current, else B's manual placement), confirm, reject and revert.
class PairStrip : public QFrame {
  Q_OBJECT
    public:
  PairStrip(ProjectSession &session, EdgeEditor &editor,
            QWidget *parent = nullptr);
  void set_pair(int a_id, int b_id);
  /// Where ICP-forfin starts from: B's manual world-space offset ΔT, or
  /// nullopt for the stored poses alone. Asked again when ICP finishes; a
  /// different answer then makes the result stale.
  void set_delta_source(std::function<std::optional<Mat4>()> source) {
    delta_source_ = std::move(source);
  }
  /// Re-read B's placement: an ICP fit counts only while B stays where ICP
  /// put it (the overlay calls this on every offset change).
  void refresh_icp();

    signals:
  /// An edge row was clicked: show it in the inspector.
  void edge_selected(const rux::qt::Selection &selection);
  /// ICP of B onto A converged for the current pair from the current ΔT:
  /// @p world_delta is B's new world-space offset (the seed included).
  void icp_aligned(int a, int b, const rux::qt::Mat4 &world_delta);

    private:
  struct IcpOutcome {
    bool ok = false;
    QString error;
    double fitness = 0, inliers = 0;
    bool converged = false;
    int source_points = 0, target_points = 0;
    Mat4 world_delta = mat4_identity(); ///< ICP's offset on B, seed included
    bool seeded = false;                ///< started from a manual ΔT
    /// The ICP edge's information (slam::icp_edge_information).
    std::array<double, 36> information{};
    /// The edge B -> A ICP measured (FramePairIcpResult::edge_transform).
    Mat4 edge_transform = mat4_identity();
    /// B was moved by hand while ICP ran: shown, never applied.
    bool stale = false;
    /// ΔT once the result was handled: B placed on it, or the seed B
    /// stayed at (nullopt = no offset). A different ΔT later outdates it.
    std::optional<Mat4> applied;
  };
  IcpFitState fit_state(const IcpOutcome &o) const;
  std::optional<Mat4> current_delta() const;
  void rebuild();
  void run_icp();
  void icp_finished(int a, int b, const std::optional<Mat4> &guess,
                    IcpOutcome outcome);
  void add_edge();
  void refresh_add_label();
  ProjectSession &session_;
  EdgeEditor &editor_;
  int a_ = -1, b_ = -1;
  QLabel *delta_ = nullptr;
  QWidget *rows_ = nullptr;
  /// The rows scroll past one row, so a pair with several edges does not
  /// take the 3D overlay's height.
  QScrollArea *rows_scroll_ = nullptr;
  QVBoxLayout *rows_layout_ = nullptr;
  QPushButton *icp_ = nullptr;
  QComboBox *type_ = nullptr;
  QPushButton *add_ = nullptr;
  QLabel *icp_line_ = nullptr;
  QLabel *note_ = nullptr;
  std::map<std::pair<int, int>, IcpOutcome> icp_results_;
  std::pair<int, int> icp_running_{-1, -1};
  std::function<std::optional<Mat4>()> delta_source_;
};

} // namespace rux::qt
