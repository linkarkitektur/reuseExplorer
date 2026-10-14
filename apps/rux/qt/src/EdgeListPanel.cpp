// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/EdgeListPanel.hpp>
#include <rux_qt/TableBrowser.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMenu>
#include <QPushButton>
#include <QScrollBar>
#include <QSettings>
#include <QSignalBlocker>
#include <QStyle>
#include <QStyleOptionHeader>
#include <QTableView>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <optional>
#include <set>

namespace rux::qt {

namespace {

/// Qt draws SortUp (▲) for a descending sort; the list shows the arrow the
/// way the rows run: ▼ when the largest is on top. And no section reserves
/// QHeaderView's room for the arrow (a section's height plus a margin,
/// added to EVERY section's width while the indicator is shown, which put
/// the columns ~35 px each past the pane): the QSS right padding of
/// QWidget#edgeList's header holds the arrow instead.
class SortHeader : public QHeaderView {
    public:
  using QHeaderView::QHeaderView;

    protected:
  QSize sectionSizeFromContents(int logical) const override {
    QSize s = QHeaderView::sectionSizeFromContents(logical);
    if (isSortIndicatorShown()) {
      QStyleOptionHeader opt;
      initStyleOption(&opt);
      const int margin =
          style()->pixelMetric(QStyle::PM_HeaderMargin, &opt, this);
      s.rwidth() -= s.height() + margin;
    }
    return s;
  }
  void initStyleOptionForIndex(QStyleOptionHeader *option,
                               int logical) const override {
    QHeaderView::initStyleOptionForIndex(option, logical);
    if (option->sortIndicator == QStyleOptionHeader::SortUp)
      option->sortIndicator = QStyleOptionHeader::SortDown;
    else if (option->sortIndicator == QStyleOptionHeader::SortDown)
      option->sortIndicator = QStyleOptionHeader::SortUp;
  }
};

} // namespace

// ----------------------------------------------------------- EdgeListModel --

EdgeListModel::EdgeListModel(QObject *parent) : QAbstractTableModel(parent) {
  refresh_font();
  connect(&theme(), &Theme::changed, this, [this] {
    refresh_font();
    if (!rows_.empty())
      emit dataChanged(index(0, 0),
                       index(rowCount() - 1, kEdgeColumnCount - 1));
  });
}

void EdgeListModel::refresh_font() {
  mono_ =
      theme().font(FontRole::mono, "--font-size-sm", "--font-weight-regular");
}

void EdgeListModel::set_rows(std::vector<EdgeView> rows) {
  beginResetModel();
  rows_ = std::move(rows);
  endResetModel();
}

const EdgeView *EdgeListModel::row_at(int row) const {
  return row >= 0 && row < static_cast<int>(rows_.size())
             ? &rows_[static_cast<std::size_t>(row)]
             : nullptr;
}

int EdgeListModel::row_of(const EdgeKey &key) const {
  for (std::size_t i = 0; i < rows_.size(); ++i)
    if (same_edge_key(rows_[i].edge.key, key))
      return static_cast<int>(i);
  return -1;
}

int EdgeListModel::rowCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : static_cast<int>(rows_.size());
}

int EdgeListModel::columnCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : kEdgeColumnCount;
}

QVariant EdgeListModel::data(const QModelIndex &index, int role) const {
  const EdgeView *v = row_at(index.row());
  if (!v)
    return {};
  const auto c = static_cast<EdgeColumn>(index.column());
  switch (role) {
  case Qt::DisplayRole:
    return QString::fromStdString(edge_list_cell_text(*v, c));
  case Qt::TextAlignmentRole:
    return QVariant::fromValue(
        Qt::AlignVCenter |
        (edge_column_numeric(c) ? Qt::AlignRight : Qt::AlignLeft));
  case Qt::FontRole:
    // Figures and ids in mono; prose (type, source, status) in the body face.
    if (edge_column_numeric(c) || c == EdgeColumn::pair)
      return mono_;
    return {};
  case Qt::ToolTipRole:
    if (v->pending != EdgeView::Pending::none)
      return QString("Ikke gemt");
    return {};
  case Qt::ForegroundRole:
    if (v->edge.status == "rejected")
      return theme().color("--color-text-faint");
    if (v->pending != EdgeView::Pending::none)
      return theme().color("--color-accent-deep");
    return {};
  default:
    return {};
  }
}

QVariant EdgeListModel::headerData(int section, Qt::Orientation o,
                                   int role) const {
  if (o != Qt::Horizontal)
    return {};
  const auto c = static_cast<EdgeColumn>(section);
  if (role == Qt::TextAlignmentRole) // over the figures they name
    return QVariant::fromValue(
        Qt::AlignVCenter |
        (edge_column_numeric(c) ? Qt::AlignRight : Qt::AlignLeft));
  if (role != Qt::DisplayRole)
    return {};
  return QString::fromStdString(edge_column_title(c));
}

// ----------------------------------------------------------- EdgeListPanel --

EdgeListPanel::EdgeListPanel(EdgeEditor &editor, bool persist, QWidget *parent)
    : QWidget(parent), editor_(editor), persist_(persist) {
  setObjectName("edgeList");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);

  auto *bar = new QFrame;
  bar->setObjectName("filterBar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-3"), t.px("--space-1"),
                         t.px("--space-3"), t.px("--space-1"));
  bl->setSpacing(t.px("--space-2"));
  type_ = new QComboBox;
  type_->setObjectName("edgeTypeFilter");
  type_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  type_->addItem("Alle typer", QString());
  bl->addWidget(type_);
  auto *seg = new QFrame;
  seg->setObjectName("segmented");
  auto *sl = new QHBoxLayout(seg);
  sl->setContentsMargins(0, 0, 0, 0);
  sl->setSpacing(0);
  status_ = new QButtonGroup(this);
  const std::array<std::pair<const char *, const char *>, 4> statuses = {
      {{"Alle", ""},
       {"Auto", "auto"},
       {"Bekræftet", "confirmed"},
       {"Afvist", "rejected"}}};
  for (int i = 0; i < 4; ++i) {
    auto *b = new QPushButton(NavItem::escape_mnemonic(statuses[i].first));
    b->setObjectName("segment");
    b->setCheckable(true);
    b->setCursor(Qt::PointingHandCursor);
    b->setProperty("position", i == 0 ? "first" : i == 3 ? "last" : "middle");
    b->setProperty("status", statuses[i].second);
    status_->addButton(b, i);
    sl->addWidget(b);
  }
  status_->button(0)->setChecked(true);
  bl->addWidget(seg);
  rejected_ = new QCheckBox(NavItem::escape_mnemonic("Med afviste"));
  rejected_->setCursor(Qt::PointingHandCursor);
  rejected_->setToolTip("Vis også afviste kanter under \"Alle\"");
  bl->addWidget(rejected_);
  meta_ = new QLabel;
  meta_->setObjectName("toolbarMeta");
  meta_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  bl->addWidget(meta_, 1);
  v->addWidget(bar);

  model_ = new EdgeListModel(this);
  view_ = new QTableView;
  style_table(view_);
  auto *h = new SortHeader(Qt::Horizontal, view_);
  view_->setHorizontalHeader(h);
  view_->setModel(model_);
  h->setHighlightSections(false);
  h->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  h->setSectionsClickable(true);
  h->setSortIndicatorShown(true);
  h->setSortIndicator(static_cast<int>(sort_), Qt::DescendingOrder);
  // Every column as wide as its content (sampling a few hundred rows, not
  // every one of a 10 000-edge graph), but KILDE: it takes what is left and
  // elides ("Odometri (init)"), so the figures fit Rediger's left pane at
  // 1440 px without a horizontal scroll.
  h->setResizeContentsPrecision(200);
  h->setSectionResizeMode(QHeaderView::ResizeToContents);
  h->setSectionResizeMode(static_cast<int>(EdgeColumn::source),
                          QHeaderView::Stretch);
  h->setStretchLastSection(false);
  h->setContextMenuPolicy(Qt::CustomContextMenu);
  h->setToolTip("Højreklik for at vise eller skjule kolonner");
  connect(h, &QHeaderView::customContextMenuRequested, this,
          &EdgeListPanel::column_menu);
  restore_hidden();
  v->addWidget(view_, 1);

  connect(h, &QHeaderView::sortIndicatorChanged, this,
          [this](int section, Qt::SortOrder order) {
            sort_ = static_cast<EdgeColumn>(section);
            descending_ = order == Qt::DescendingOrder;
            if (is_column_hidden(sort_))
              set_column_hidden(sort_, false); // never sort by the unseen
            rebuild();
            // Nine columns overflow the pane at 1440 px: bring the sorted
            // one into view (and, with nothing selected, the top rows).
            if (!view_->currentIndex().isValid())
              reveal(0);
            reveal_sort_column();
          });
  connect(type_, &QComboBox::currentIndexChanged, this, [this] {
    filter_.type = type_->currentData().toString().toStdString();
    rebuild();
  });
  connect(status_, &QButtonGroup::idClicked, this, [this](int id) {
    filter_.status =
        status_->button(id)->property("status").toString().toStdString();
    rebuild();
  });
  connect(rejected_, &QCheckBox::toggled, this, [this](bool on) {
    filter_.show_rejected = on;
    rebuild();
  });
  connect(view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
          this, [this](const QModelIndex &cur) {
            if (selecting_)
              return;
            if (const EdgeView *e = model_->row_at(cur.row()))
              emit edge_activated(e->edge.key);
          });
  connect(&editor_, &EdgeEditor::changed, this,
          &EdgeListPanel::schedule_rebuild);
  rebuild();
}

void EdgeListPanel::schedule_rebuild() {
  if (rebuild_pending_)
    return;
  rebuild_pending_ = true;
  QTimer::singleShot(0, this, [this] {
    rebuild_pending_ = false;
    rebuild();
  });
}

void EdgeListPanel::set_column_hidden(EdgeColumn column, bool hidden) {
  if (column == EdgeColumn::pair)
    return; // the row's identity
  view_->horizontalHeader()->setSectionHidden(static_cast<int>(column), hidden);
  save_hidden();
}

bool EdgeListPanel::is_column_hidden(EdgeColumn column) const {
  return view_->horizontalHeader()->isSectionHidden(static_cast<int>(column));
}

void EdgeListPanel::column_menu(const QPoint &pos) {
  QMenu menu(this);
  for (int i = 0; i < kEdgeColumnCount; ++i) {
    const auto c = static_cast<EdgeColumn>(i);
    QAction *a = menu.addAction(QString::fromStdString(edge_column_title(c)));
    a->setCheckable(true);
    a->setChecked(!is_column_hidden(c));
    // PAR names the row; the sorted column is what the list is about.
    a->setEnabled(c != EdgeColumn::pair && c != sort_);
    connect(a, &QAction::toggled, this,
            [this, c](bool on) { set_column_hidden(c, !on); });
  }
  menu.exec(view_->horizontalHeader()->mapToGlobal(pos));
}

void EdgeListPanel::restore_hidden() {
  std::vector<int> hidden;
  for (EdgeColumn c : default_hidden_edge_columns())
    hidden.push_back(static_cast<int>(c));
  if (persist_) {
    const QVariant stored = QSettings().value("rediger/edge_list/hidden");
    if (stored.isValid()) {
      hidden.clear();
      for (const QString &part :
           stored.toString().split(',', Qt::SkipEmptyParts)) {
        bool ok = false;
        const int i = part.toInt(&ok);
        if (ok && i > 0 && i < kEdgeColumnCount)
          hidden.push_back(i);
      }
    }
  }
  auto *h = view_->horizontalHeader();
  for (int i = 0; i < kEdgeColumnCount; ++i)
    h->setSectionHidden(i, std::find(hidden.begin(), hidden.end(), i) !=
                               hidden.end());
}

void EdgeListPanel::save_hidden() const {
  if (!persist_)
    return;
  QStringList parts;
  for (int i = 0; i < kEdgeColumnCount; ++i)
    if (view_->horizontalHeader()->isSectionHidden(i))
      parts << QString::number(i);
  QSettings().setValue("rediger/edge_list/hidden", parts.join(','));
}

void EdgeListPanel::reveal_sort_column() {
  auto *h = view_->horizontalHeader();
  h->resizeSections(QHeaderView::ResizeToContents);
  const int c = static_cast<int>(sort_);
  const int x = h->sectionPosition(c), w = h->sectionSize(c);
  const int vw = view_->viewport()->width();
  QScrollBar *sb = view_->horizontalScrollBar();
  if (x + w > sb->value() + vw)
    sb->setValue(x + w - vw);
  if (x < sb->value())
    sb->setValue(x);
}

void EdgeListPanel::set_sort(EdgeColumn column, bool descending) {
  auto *h = view_->horizontalHeader();
  const auto order = descending ? Qt::DescendingOrder : Qt::AscendingOrder;
  if (is_column_hidden(column))
    set_column_hidden(column, false);
  if (h->sortIndicatorSection() == static_cast<int>(column) &&
      h->sortIndicatorOrder() == order)
    return; // no sortIndicatorChanged, and nothing to do
  h->setSortIndicator(static_cast<int>(column), order); // -> rebuild()
}

void EdgeListPanel::set_filter(const EdgeFilter &f) {
  filter_ = f;
  {
    const QSignalBlocker b1(type_), b2(rejected_);
    type_->setCurrentIndex(
        std::max(0, type_->findData(QString::fromStdString(f.type))));
    rejected_->setChecked(f.show_rejected);
  }
  for (QAbstractButton *b : status_->buttons())
    if (b->property("status").toString().toStdString() == f.status)
      b->setChecked(true);
  rebuild();
}

void EdgeListPanel::select_edge(const EdgeKey &key) {
  const int r = model_->row_of(key);
  selecting_ = true;
  if (r < 0) {
    view_->selectionModel()->clearCurrentIndex();
    view_->clearSelection();
  } else {
    view_->selectRow(r);
    // Vertically only: keep the user's horizontal scroll.
    auto *h = view_->horizontalHeader();
    view_->scrollTo(model_->index(r, std::max(0, h->logicalIndexAt(0))),
                    QAbstractItemView::PositionAtCenter);
  }
  selecting_ = false;
}

void EdgeListPanel::reveal(int row, QAbstractItemView::ScrollHint hint) {
  if (row < 0 || row >= model_->rowCount())
    return;
  // Section positions are settled lazily after a reset; scroll by the new
  // widths, not the old ones.
  view_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
  view_->scrollTo(model_->index(row, static_cast<int>(sort_)), hint);
}

int EdgeListPanel::row_count() const { return model_->rowCount(); }

void EdgeListPanel::sync_type_combo(const std::vector<EdgeView> &all) {
  std::set<std::string> types;
  for (const auto &e : all)
    types.insert(e.edge.key.type);
  const QString current = type_->currentData().toString();
  const QSignalBlocker b(type_);
  type_->clear();
  type_->addItem("Alle typer", QString());
  for (const auto &ty : types)
    type_->addItem(QString::fromStdString(edge_type_da(ty)),
                   QString::fromStdString(ty));
  type_->setCurrentIndex(std::max(0, type_->findData(current)));
}

void EdgeListPanel::rebuild() {
  // Keep the selected key across a rebuild (a staged edit re-sorts).
  std::optional<EdgeKey> keep;
  if (const EdgeView *e = model_->row_at(view_->currentIndex().row()))
    keep = e->edge.key;
  const auto all = editor_.edits().effective();
  sync_type_combo(all);
  // A filtered type that no longer exists falls back to "Alle typer".
  filter_.type = type_->currentData().toString().toStdString();
  rejected_->setEnabled(filter_.status.empty());
  model_->set_rows(edge_table(all, filter_, sort_, descending_));
  meta_->setText(
      QString("%1 af %2 kanter")
          .arg(format_count(static_cast<qulonglong>(model_->rowCount())))
          .arg(format_count(static_cast<qulonglong>(all.size()))));
  if (keep)
    select_edge(*keep);
}

} // namespace rux::qt
