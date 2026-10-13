// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/DatabaseWorkspace.hpp>
#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/TableBrowser.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/Viewer3DWorkspace.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <map>

namespace rux::qt {
namespace {

QString qs(const std::string &s) { return QString::fromStdString(s); }

QString count_text(qint64 n) {
  return format_count(static_cast<qulonglong>(std::max<qint64>(0, n)));
}

/// Survey and resource tables, in the order the case work reads them.
struct SurveyTable {
  const char *table;
  const char *name;
};
constexpr SurveyTable kSurvey[] = {
    {"survey_parts", "Ressourcer"},
    {"survey_types", "Typer"},
    {"samples", "Prøver"},
    {"templates", "Skabeloner"},
    {"material_passports", "Materialepas"},
    {"instances", "Instanser"},
    {"building_components", "Bygningsdele"},
    {"report_pdfs", "Rapporter"},
};

QString cloud_type_da(const std::string &type) {
  if (type == "PointXYZRGB")
    return "Farvet sky";
  if (type == "Normal")
    return "Normaler";
  if (type == "Label")
    return "Mærkater";
  if (type == "PointXYZ")
    return "Punkter";
  return qs(type);
}

} // namespace

// -------------------------------------------------------------- ProjectTree --

ProjectTree::ProjectTree(QWidget *parent) : QTreeWidget(parent) {
  setObjectName("projectTree");
  setColumnCount(2);
  setHeaderHidden(true);
  setRootIsDecorated(true);
  setIndentation(theme().px("--space-4"));
  setUniformRowHeights(true);
  setFrameShape(QFrame::NoFrame);
  setSelectionMode(QAbstractItemView::SingleSelection);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  header()->setStretchLastSection(false);
  header()->setSectionResizeMode(0, QHeaderView::Stretch);
  header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  connect(this, &QTreeWidget::itemClicked, this,
          [this](QTreeWidgetItem *it, int) {
            const auto kind = static_cast<Kind>(it->data(0, KindRole).toInt());
            if (kind == Kind::none) {
              it->setExpanded(!it->isExpanded());
              return;
            }
            emit activated_item(kind, it->data(0, KeyRole).toString(),
                                it->data(0, TableRole).toString(),
                                it->data(0, CountRole).toLongLong());
          });
  connect(this, &QTreeWidget::currentItemChanged, this,
          [this](QTreeWidgetItem *it, QTreeWidgetItem *) {
            // Keyboard navigation opens what it lands on, like a click.
            if (!it || signalsBlocked() || !hasFocus())
              return;
            const auto kind = static_cast<Kind>(it->data(0, KindRole).toInt());
            if (kind != Kind::none)
              emit activated_item(kind, it->data(0, KeyRole).toString(),
                                  it->data(0, TableRole).toString(),
                                  it->data(0, CountRole).toLongLong());
          });
}

void ProjectTree::rebuild(const ProjectSession &session) {
  const QSignalBlocker block(this);
  clear();
  const reusex::ProjectDB *db = session.db();
  if (!db)
    return;
  const Theme &t = theme();
  const QFont mono = t.font(FontRole::mono, "--font-size-xs");
  const QFont group_font =
      t.font(FontRole::sans, "--font-size-sm", "--font-weight-medium");

  auto item = [&](QTreeWidgetItem *parent, const QString &name, qint64 count,
                  Kind kind, const QString &key = {}, const QString &table = {},
                  bool show_count = true) {
    auto *it = parent ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(this);
    it->setText(0, name);
    if (show_count && count >= 0)
      it->setText(1, count_text(count));
    it->setTextAlignment(1, Qt::AlignRight | Qt::AlignVCenter);
    it->setFont(1, mono);
    it->setForeground(1, t.color("--color-text-faint"));
    it->setData(0, KindRole, static_cast<int>(kind));
    it->setData(0, KeyRole, key);
    it->setData(0, CountRole, count);
    it->setData(0, TableRole, table);
    if (!parent)
      it->setFont(0, group_font);
    return it;
  };

  std::map<std::string, std::int64_t> rows;
  std::vector<reusex::ProjectDB::TableInfo> tables;
  try {
    tables = db->list_tables();
    for (const auto &ti : tables)
      rows[ti.name] = ti.row_count;
  } catch (const std::exception &) {
  }
  auto rows_of = [&](const char *table) -> qint64 {
    auto it = rows.find(table);
    return it == rows.end() ? -1 : it->second;
  };

  const auto &s = session.summary();

  // Frames, by scan.
  auto *frames = item(nullptr, "Billeder", s.sensor_frames.total_count,
                      Kind::frames, {}, "sensor_frames");
  try {
    std::map<int, std::pair<int, int>> scans; // scan -> (first id, count)
    for (const auto &[id, scan] : db->sensor_frame_ids_with_scan()) {
      auto &e = scans[scan];
      if (e.second++ == 0)
        e.first = id;
    }
    if (scans.size() > 1 || (scans.size() == 1 && scans.begin()->first > 0))
      for (const auto &[scan, e] : scans)
        item(frames,
             scan > 0 ? QString("Scanning %1").arg(scan)
                      : QString("Uden scanning"),
             e.second, Kind::scan, QString::number(e.first));
  } catch (const std::exception &) {
  }
  frames->setExpanded(true);

  item(nullptr, "Panoramaer", s.panoramic_images.total_count, Kind::table, {},
       "panoramic_images");

  auto *clouds =
      item(nullptr, "Punktskyer", static_cast<qint64>(s.clouds.size()),
           Kind::table, {}, "point_clouds");
  for (const auto &c : s.clouds) {
    auto *ci = item(clouds, qs(c.name), static_cast<qint64>(c.point_count),
                    Kind::cloud, qs(c.name), "point_clouds");
    ci->setFont(0, mono);
    ci->setToolTip(0, cloud_type_da(c.type));
  }
  auto *meshes = item(nullptr, "Mesh", static_cast<qint64>(s.meshes.size()),
                      Kind::table, {}, "meshes");
  for (const auto &m : s.meshes) {
    auto *mi = item(meshes, qs(m.name), m.face_count, Kind::mesh, qs(m.name),
                    "meshes");
    mi->setFont(0, mono);
    mi->setToolTip(0, QString("%1 flader").arg(count_text(m.face_count)));
  }
  if (!s.gaussian_splats.empty())
    item(nullptr, "Gaussian splats",
         static_cast<qint64>(s.gaussian_splats.size()), Kind::table, {},
         "gaussian_splats");

  item(nullptr, "Posegrafens kanter", rows_of("pose_graph_edges"), Kind::table,
       {}, "pose_graph_edges");

  auto *survey = item(nullptr, "Kortlægning", -1, Kind::none, {}, {}, false);
  for (const auto &st : kSurvey)
    if (const qint64 n = rows_of(st.table); n >= 0)
      item(survey, st.name, n, Kind::table, {}, st.table);
  survey->setExpanded(true);

  item(nullptr, "Pipeline-log", rows_of("pipeline_log"), Kind::log, {},
       "pipeline_log");

  auto *all = item(nullptr, "Alle tabeller", static_cast<qint64>(tables.size()),
                   Kind::none);
  for (const auto &ti : tables) {
    auto *x =
        item(all, qs(ti.name), ti.row_count, Kind::table, {}, qs(ti.name));
    x->setFont(0, mono);
  }
}

void ProjectTree::select(Kind kind, const QString &key) {
  const QSignalBlocker block(this);
  for (QTreeWidgetItemIterator it(this); *it; ++it) {
    if (static_cast<Kind>((*it)->data(0, KindRole).toInt()) == kind &&
        (key.isEmpty() || (*it)->data(0, KeyRole).toString() == key ||
         (*it)->data(0, TableRole).toString() == key)) {
      setCurrentItem(*it);
      return;
    }
  }
}

// --------------------------------------------------------- DatabaseWorkspace
// --

DatabaseWorkspace::DatabaseWorkspace(ProjectSession &session,
                                     EdgeEditor &editor, QWidget *parent)
    : QWidget(parent), session_(session), editor_(&editor) {
  setObjectName("databaseWorkspace");
  const Theme &t = theme();
  loader_ = new FrameImageLoader(this);

  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);

  // ---- pending edits banner (the web's WriteBanner)
  banner_ = new QFrame;
  banner_->setObjectName("pendingBanner");
  auto *bl = new QHBoxLayout(banner_);
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-2"),
                         t.px("--space-4"), t.px("--space-2"));
  bl->setSpacing(t.px("--space-3"));
  auto *dot = new QLabel;
  dot->setObjectName("bannerDot");
  bl->addWidget(dot, 0, Qt::AlignVCenter);
  auto *texts = new QVBoxLayout;
  texts->setSpacing(0);
  banner_text_ = new QLabel;
  banner_text_->setObjectName("bannerText");
  texts->addWidget(banner_text_);
  banner_error_ = new QLabel;
  banner_error_->setObjectName("bannerError");
  banner_error_->setWordWrap(true);
  texts->addWidget(banner_error_);
  bl->addLayout(texts, 1);
  discard_ = new QPushButton(NavItem::escape_mnemonic("Kassér"));
  discard_->setProperty("kind", "ghost");
  discard_->setCursor(Qt::PointingHandCursor);
  bl->addWidget(discard_);
  save_ = new QPushButton(NavItem::escape_mnemonic("Gem ændringer"));
  save_->setProperty("kind", "primary");
  save_->setCursor(Qt::PointingHandCursor);
  save_->setShortcut(QKeySequence::Save);
  bl->addWidget(save_);
  // Buttons in a layout do not size their row by themselves (qt-client.md).
  for (QPushButton *b : {save_, discard_})
    b->setMinimumHeight(b->sizeHint().height());
  banner_->setMinimumHeight(save_->sizeHint().height() + 2 * t.px("--space-2"));
  v->addWidget(banner_);

  auto *body = new QHBoxLayout;
  body->setSpacing(0);
  v->addLayout(body, 1);

  auto *left = new QFrame;
  left->setObjectName("treePane");
  left->setFixedWidth(t.px("--layout-nav-width") + t.px("--space-5"));
  auto *ll = new QVBoxLayout(left);
  ll->setContentsMargins(0, t.px("--space-3"), 0, 0);
  ll->setSpacing(t.px("--space-2"));
  auto *eyebrow =
      new CapsLabel("Projektet", "eyebrowSurface", "--tracking-wide");
  eyebrow->setContentsMargins(t.px("--space-4"), 0, 0, 0);
  ll->addWidget(eyebrow);
  tree_ = new ProjectTree;
  ll->addWidget(tree_, 1);
  body->addWidget(left);
  tree_pane_ = left;

  stack_ = new QStackedWidget;
  stack_->setObjectName("dbStack");
  frames_ = new FrameBrowser(session_, *editor_, *loader_);
  tables_ = new TableBrowser(session_);
  log_ = new PipelineLogView(session_);
  empty_ = new QWidget;
  empty_->setObjectName("dbEmpty");
  {
    auto *ev = new QVBoxLayout(empty_);
    ev->setSpacing(t.px("--space-3"));
    ev->addStretch(1);
    empty_text_ = new QLabel;
    empty_text_->setObjectName("emptyText");
    empty_text_->setAlignment(Qt::AlignCenter);
    ev->addWidget(empty_text_);
    empty_open_ = new QPushButton(NavItem::escape_mnemonic("Åbn projekt…"));
    empty_open_->setProperty("kind", "primary");
    empty_open_->setCursor(Qt::PointingHandCursor);
    ev->addWidget(empty_open_, 0, Qt::AlignHCenter);
    ev->addStretch(2);
    connect(empty_open_, &QPushButton::clicked, this,
            &DatabaseWorkspace::browse_requested);
  }
  stack_->addWidget(frames_);
  stack_->addWidget(tables_);
  stack_->addWidget(log_);
  stack_->addWidget(empty_);
  body->addWidget(stack_, 1);

  connect(tree_, &ProjectTree::activated_item, this,
          &DatabaseWorkspace::show_item);
  // Only the view on screen drives the inspector: a frame that finishes
  // decoding behind the table must not replace the selected row.
  auto from = [this](QWidget *view) {
    return [this, view](const Selection &s) {
      if (stack_->currentWidget() == view)
        set_selection(s);
    };
  };
  connect(frames_, &FrameBrowser::selection_changed, this, from(frames_));
  connect(tables_, &TableBrowser::selection_changed, this, from(tables_));
  connect(log_, &PipelineLogView::selection_changed, this, from(log_));
  connect(editor_, &EdgeEditor::changed, this, &DatabaseWorkspace::sync_banner);
  connect(editor_, &EdgeEditor::saved, this,
          [this] { tree_->rebuild(session_); }); // the edge count changed
  connect(save_, &QPushButton::clicked, this, [this] { save_edits(); });
  connect(discard_, &QPushButton::clicked, this,
          &DatabaseWorkspace::discard_edits);
  connect(&session_, &ProjectSession::state_changed, this,
          &DatabaseWorkspace::rebuild);
  connect(&theme(), &Theme::changed, this, [this] {
    tree_->rebuild(session_); // counts carry token colours and fonts
  });
  rebuild();
}

DatabaseWorkspace::~DatabaseWorkspace() = default;

int DatabaseWorkspace::pending_edits() const { return editor_->pending(); }

bool DatabaseWorkspace::save_edits() {
  const bool started = editor_->save();
  sync_banner();
  return started;
}

bool DatabaseWorkspace::save_edits_and_wait() {
  const bool ok = editor_->save_and_wait(this);
  sync_banner();
  return ok;
}

bool DatabaseWorkspace::is_saving() const { return editor_->is_saving(); }

void DatabaseWorkspace::discard_edits() { editor_->discard(); }

void DatabaseWorkspace::rebuild() {
  const bool open = session_.is_open();
  tree_->rebuild(session_);
  frames_->reload();
  log_->reload();
  selection_ = {};
  tree_pane_->setVisible(open);
  if (!open) {
    stack_->setCurrentWidget(empty_);
    const bool loading = session_.state() == ProjectSession::State::loading;
    empty_text_->setText(loading ? QString("Åbner projektet …")
                                 : QString("Åbn et projekt for at se dets "
                                           "billeder, tabeller og log."));
    empty_open_->setVisible(!loading);
    emit selection_changed(selection_);
  } else {
    open_item(ProjectTree::Kind::frames);
  }
  sync_banner();
}

void DatabaseWorkspace::open_item(ProjectTree::Kind kind, const QString &key) {
  tree_->select(kind, key);
  if (QTreeWidgetItem *it = tree_->currentItem())
    show_item(kind, it->data(0, ProjectTree::KeyRole).toString(),
              it->data(0, ProjectTree::TableRole).toString(),
              it->data(0, ProjectTree::CountRole).toLongLong());
  else
    show_item(kind, key, key, 0);
}

void DatabaseWorkspace::show_item(ProjectTree::Kind kind, const QString &key,
                                  const QString &table, qint64 count) {
  if (!session_.is_open())
    return;
  switch (kind) {
  case ProjectTree::Kind::none:
    return;
  case ProjectTree::Kind::frames:
    stack_->setCurrentWidget(frames_);
    set_selection(frames_->frame_selection(false));
    return;
  case ProjectTree::Kind::scan:
    stack_->setCurrentWidget(frames_);
    frames_->show_a_near(key.toInt());
    set_selection(frames_->frame_selection(false));
    return;
  case ProjectTree::Kind::log:
    stack_->setCurrentWidget(log_);
    log_->select_row(0);
    return;
  case ProjectTree::Kind::table:
  case ProjectTree::Kind::cloud:
  case ProjectTree::Kind::mesh: {
    qint64 rows = count;
    // A cloud/mesh item's count is its points/faces, not the table's rows.
    if (kind != ProjectTree::Kind::table || rows < 0) {
      rows = 0;
      try {
        for (const auto &ti : session_.db()->list_tables())
          if (qs(ti.name) == table)
            rows = ti.row_count;
      } catch (const std::exception &) {
      }
    }
    stack_->setCurrentWidget(tables_);
    tables_->show_table(table, rows);
    if (kind == ProjectTree::Kind::cloud) {
      tables_->select_where("name", key);
      set_selection(cloud_selection(key));
    } else if (kind == ProjectTree::Kind::mesh) {
      tables_->select_where("name", key);
      set_selection(mesh_selection(key));
    } else {
      Selection s;
      s.kind = "Tabel";
      s.title = table;
      s.subtitle = QString("%1 rækker").arg(count_text(rows));
      s.pill = "Kun læsning";
      SelectionSection cols{"Kolonner", {}, {}};
      try {
        for (const auto &c : session_.db()->table_columns(table.toStdString()))
          cols.rows.push_back(
              {qs(c.name),
               qs(c.declared_type) +
                   (c.primary_key ? QString(" · nøgle") : QString()),
               SelectionRow::Style::name});
      } catch (const std::exception &) {
      }
      s.sections.push_back(cols);
      set_selection(s);
    }
    return;
  }
  }
}

Selection DatabaseWorkspace::cloud_selection(const QString &name) const {
  Selection s;
  for (const auto &c : session_.summary().clouds) {
    if (qs(c.name) != name)
      continue;
    s.kind = "Punktsky";
    s.title = name;
    s.subtitle = cloud_type_da(c.type);
    SelectionSection p{"Sky", {}, {}};
    p.rows.push_back(
        {"Punkter", count_text(static_cast<qint64>(c.point_count))});
    p.rows.push_back({"Type", qs(c.type)});
    p.rows.push_back({"Organiseret",
                      c.organized ? QString("Ja") : QString("Nej"),
                      SelectionRow::Style::value});
    p.rows.push_back({"Rækkefølge", c.storage_order.empty()
                                        ? QString("Indsat")
                                        : qs(c.storage_order)});
    s.sections.push_back(p);
    if (!c.labels.empty()) {
      SelectionSection l{QString("Mærkater (%1)").arg(c.labels.size()), {}, {}};
      int i = 0;
      bool ok = false;
      int nslots = theme().value("--label-count").toInt(&ok);
      if (!ok || nslots <= 0)
        nslots = 1;
      for (const auto &[id, label] : c.labels) {
        if (i++ == 24) {
          l.rows.push_back({"…", QString("%1 flere").arg(c.labels.size() - 24),
                            SelectionRow::Style::value});
          break;
        }
        l.rows.push_back(
            {qs(label), QString::number(id), SelectionRow::Style::name,
             // The 3D view's and the web's slot rule.
             label_swatch_token(static_cast<std::uint32_t>(id), nslots)});
      }
      s.sections.push_back(l);
    }
  }
  return s;
}

Selection DatabaseWorkspace::mesh_selection(const QString &name) const {
  Selection s;
  for (const auto &m : session_.summary().meshes) {
    if (qs(m.name) != name)
      continue;
    s.kind = "Mesh";
    s.title = name;
    SelectionSection p{"Mesh", {}, {}};
    p.rows.push_back({"Hjørner", count_text(m.vertex_count)});
    p.rows.push_back({"Flader", count_text(m.face_count)});
    s.sections.push_back(p);
  }
  return s;
}

void DatabaseWorkspace::set_selection(const Selection &s) {
  selection_ = s;
  emit selection_changed(selection_);
}

void DatabaseWorkspace::sync_banner() {
  const int n = editor_->pending();
  const QString error = editor_->last_error();
  const bool saving = editor_->is_saving();
  banner_->setVisible(n > 0 || !error.isEmpty() || saving);
  banner_text_->setText(
      saving  ? QString("Gemmer ændringer i posegrafen …")
      : n > 0 ? QString("%1 i posegrafen venter på at blive gemt — %2")
                    .arg(n == 1 ? QString("1 ændring")
                                : QString("%1 ændringer").arg(n),
                         QString::fromStdString(
                             pending_summary_da(editor_->edits().ops())))
              : QString("Posegrafen"));
  banner_error_->setText(error);
  banner_error_->setToolTip(editor_->last_error_detail());
  banner_error_->setVisible(!error.isEmpty());
  // Read-only is a state, not a failure: the warning tone, like the title
  // bar's "Skrivebeskyttet" pill. A failed write (locked, other) is critical.
  const bool busy = !editor_->pipeline_busy().isEmpty();
  // Read-only, and a pipeline stage holding the project, are states: warn.
  const bool read_only_error =
      !error.isEmpty() && (!editor_->read_only_reason().isEmpty() || busy);
  banner_->setProperty("tone", error.isEmpty()   ? "pending"
                               : read_only_error ? "warn"
                                                 : "error");
  repolish(banner_);
  save_->setEnabled(n > 0 && editor_->can_save() && !saving && !busy);
  save_->setToolTip(
      !editor_->can_save() ? editor_->read_only_reason()
      : busy ? QString("Et pipeline-trin (%1) skriver til projektet — gem, "
                       "når det er færdigt.")
                   .arg(editor_->pipeline_busy())
             : QString("Skriv ændringerne til projektet (Ctrl+S)"));
  if (busy && n > 0 && error.isEmpty()) {
    banner_error_->setText(QString("Gem er sat på pause, mens %1 kører.")
                               .arg(editor_->pipeline_busy()));
    banner_error_->setVisible(true);
  }
  discard_->setEnabled(n > 0 && !saving);
}

} // namespace rux::qt
