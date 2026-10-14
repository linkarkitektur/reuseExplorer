// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's edge list (spec §5.2): every effective edge (staged edits
// applied) with type, source, status and the optimizer's metrics; sorted so
// the suspicious ones are on top (the colour mode's metric, descending, by
// default), filtered by type and status. Rows come from rux_qt_core's
// edge_table(); this file only adapts them to a QTableView. Columns can be
// hidden from the header's context menu (the angle columns start hidden);
// the choice persists under rediger/edge_list/hidden (not in a shot).

#include <rux_qt/edge_table.hpp>

#include <QAbstractItemView>
#include <QAbstractTableModel>
#include <QFont>
#include <QWidget>

#include <vector>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QLabel;
class QTableView;

namespace rux::qt {

class EdgeEditor;
class ElidedLabel;

class EdgeListModel : public QAbstractTableModel {
  Q_OBJECT
    public:
  explicit EdgeListModel(QObject *parent = nullptr);
  void set_rows(std::vector<EdgeView> rows);
  const EdgeView *row_at(int row) const;
  int row_of(const EdgeKey &key) const;
  int rowCount(const QModelIndex &parent = {}) const override;
  int columnCount(const QModelIndex &parent = {}) const override;
  QVariant data(const QModelIndex &index, int role) const override;
  QVariant headerData(int section, Qt::Orientation o, int role) const override;

    private:
  void refresh_font();
  std::vector<EdgeView> rows_;
  QFont mono_; ///< figures and ids; refreshed on a theme change
};

class EdgeListPanel : public QWidget {
  Q_OBJECT
    public:
  /// @p persist: remember hidden columns in QSettings (not in a shot).
  explicit EdgeListPanel(EdgeEditor &editor, bool persist = false,
                         QWidget *parent = nullptr);
  void set_sort(EdgeColumn column, bool descending);
  void set_filter(const EdgeFilter &filter);
  /// Select and scroll to @p key without emitting edge_activated; clears
  /// the selection when the list does not show it.
  void select_edge(const EdgeKey &key);
  int row_count() const;
  QTableView *view() const { return view_; }
  void set_column_hidden(EdgeColumn column, bool hidden);
  bool is_column_hidden(EdgeColumn column) const;

    signals:
  void edge_activated(const rux::qt::EdgeKey &key);

    private:
  void rebuild();
  /// Rebuild once the current event is done: one staged edit can emit
  /// EdgeEditor::changed more than once.
  void schedule_rebuild();
  void column_menu(const QPoint &pos);
  void restore_hidden();
  void save_hidden() const;
  /// A narrow window's fallback: scroll so the sorted column shows. Only
  /// on a sort change, so a splitter drag keeps the user's scroll.
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
  ElidedLabel *meta_ = nullptr;
  EdgeFilter filter_;
  EdgeColumn sort_ = EdgeColumn::residual;
  bool descending_ = true;
  bool selecting_ = false;
  bool persist_ = false;
  bool rebuild_pending_ = false;
};

} // namespace rux::qt
