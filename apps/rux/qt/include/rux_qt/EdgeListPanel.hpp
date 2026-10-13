// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's edge list (spec §5.2): every effective edge (staged edits
// applied) with type, source, status and the optimizer's metrics; sorted so
// the suspicious ones are on top (the colour mode's metric, descending, by
// default), filtered by type and status. Rows come from rux_qt_core's
// edge_table(); this file only adapts them to a QTableView.

#include <rux_qt/edge_table.hpp>

#include <QAbstractItemView>
#include <QAbstractTableModel>
#include <QWidget>

#include <vector>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QLabel;
class QTableView;

namespace rux::qt {

class EdgeEditor;

class EdgeListModel : public QAbstractTableModel {
  Q_OBJECT
    public:
  using QAbstractTableModel::QAbstractTableModel;
  void set_rows(std::vector<EdgeView> rows);
  const EdgeView *row_at(int row) const;
  int row_of(const EdgeKey &key) const;
  int rowCount(const QModelIndex &parent = {}) const override;
  int columnCount(const QModelIndex &parent = {}) const override;
  QVariant data(const QModelIndex &index, int role) const override;
  QVariant headerData(int section, Qt::Orientation o, int role) const override;

    private:
  std::vector<EdgeView> rows_;
};

class EdgeListPanel : public QWidget {
  Q_OBJECT
    public:
  explicit EdgeListPanel(EdgeEditor &editor, QWidget *parent = nullptr);
  void set_sort(EdgeColumn column, bool descending);
  void set_filter(const EdgeFilter &filter);
  /// Select and scroll to @p key without emitting edge_activated; clears
  /// the selection when the list does not show it.
  void select_edge(const EdgeKey &key);
  int row_count() const;
  QTableView *view() const { return view_; }

    signals:
  void edge_activated(const rux::qt::EdgeKey &key);

    protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

    private:
  void rebuild();
  /// Scroll horizontally so the sorted column shows (nine columns overflow
  /// the pane at 1440 px).
  void reveal_sort_column();
  /// Scroll @p row and the sort column into view.
  void
  reveal(int row,
         QAbstractItemView::ScrollHint hint = QAbstractItemView::EnsureVisible);
  void sync_type_combo(const std::vector<EdgeView> &all);
  EdgeEditor &editor_;
  EdgeListModel *model_ = nullptr;
  QTableView *view_ = nullptr;
  QComboBox *type_ = nullptr;
  QButtonGroup *status_ = nullptr;
  QCheckBox *rejected_ = nullptr;
  QLabel *meta_ = nullptr;
  EdgeFilter filter_;
  EdgeColumn sort_ = EdgeColumn::residual;
  bool descending_ = true;
  bool selecting_ = false;
};

} // namespace rux::qt
