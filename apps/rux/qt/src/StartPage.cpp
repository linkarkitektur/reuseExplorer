// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/RecentProjects.hpp>
#include <rux_qt/StartPage.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/palette_table.hpp>
#include <rux_qt/widgets.hpp>
#include <rux_qt/workspaces.hpp>

#include <QBoxLayout>
#include <QDir>
#include <QFileInfo>
#include <QGridLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>

#include <algorithm>

namespace rux::qt {
namespace {

QPushButton *button(const QString &text, const QString &kind) {
  auto *b = new QPushButton(NavItem::escape_mnemonic(text));
  b->setProperty("kind", kind);
  b->setCursor(Qt::PointingHandCursor);
  return b;
}

QLabel *label(const QString &text, const QString &object_name,
              bool wrap = false) {
  auto *l = new QLabel(text);
  l->setObjectName(object_name);
  l->setWordWrap(wrap);
  return l;
}

void clear_layout(QLayout *l) {
  while (QLayoutItem *it = l->takeAt(0)) {
    if (QWidget *w = it->widget()) {
      // Out of the layout but still a child until deleteLater runs: hide
      // it now or it paints at its old place over the new content.
      w->hide();
      w->deleteLater();
    } else if (QLayout *sub = it->layout())
      clear_layout(sub);
    delete it;
  }
}

QString recent_title(const QString &path) {
  return QString::fromStdString(recent_project_title(path.toStdString()));
}

QString pretty_dir(const QString &path) {
  QString d = QFileInfo(path).absolutePath();
  const QString home = QDir::homePath();
  if (d == home || d.startsWith(home + '/'))
    d = '~' + d.mid(home.size());
  return d;
}

} // namespace

StartPage::StartPage(ProjectSession &session, RecentProjects &recent,
                     QWidget *parent)
    : QWidget(parent), session_(session), recent_(recent) {
  setObjectName("startPage");
  const Theme &t = theme();
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  auto *scroll = new QScrollArea;
  scroll->setObjectName("startScroll");
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  outer->addWidget(scroll);

  auto *content = new QWidget;
  content->setObjectName("content");
  scroll->setWidget(content);
  auto *v = new QVBoxLayout(content);
  const int m = t.px("--space-6");
  v->setContentsMargins(m, t.px("--space-5"), m, m);
  v->setSpacing(t.px("--space-5"));

  const WorkspaceInfo &info = workspace_info(Workspace::start);
  auto *head = new QHBoxLayout;
  head->setSpacing(t.px("--space-3"));
  head->addWidget(new CapsLabel(info.name, "viewTitle", QString()), 0,
                  Qt::AlignBottom);
  head->addWidget(label(info.subtitle, "viewSub"), 0, Qt::AlignBottom);
  head->addStretch(1);
  v->addLayout(head);

  columns_ = new QBoxLayout(QBoxLayout::LeftToRight);
  columns_->setSpacing(t.px("--space-5"));
  main_box_ = new QWidget;
  main_col_ = new QVBoxLayout(main_box_);
  main_col_->setContentsMargins(0, 0, 0, 0);
  main_col_->setSpacing(t.px("--space-5"));
  recent_box_ = new QWidget;
  recent_col_ = new QVBoxLayout(recent_box_);
  recent_col_->setContentsMargins(0, 0, 0, 0);
  recent_col_->setSpacing(0);
  columns_->addWidget(main_box_, 3);
  columns_->addWidget(recent_box_, 2);
  v->addLayout(columns_);
  v->addStretch(1);

  connect(&session_, &ProjectSession::state_changed, this, [this] {
    rebuild_state();
    rebuild_recent(); // the "Åben" badge follows the open project
  });
  connect(&recent_, &RecentProjects::changed, this, &StartPage::rebuild_recent);
  rebuild_state();
  rebuild_recent();
}

void StartPage::set_drop_active(bool on) {
  drop_active_ = on;
  if (drop_zone_) {
    drop_zone_->setProperty("active", on);
    repolish(drop_zone_);
    for (QWidget *w : drop_zone_->findChildren<QWidget *>()) {
      w->setProperty("active", on);
      repolish(w);
    }
  }
}

void StartPage::resizeEvent(QResizeEvent *e) {
  QWidget::resizeEvent(e);
  update_columns();
}

void StartPage::update_columns() {
  // Two columns while both stay readable; one below that (a narrow window
  // with the inspector open).
  const int two_col_min =
      3 * theme().px("--layout-panel-width") - theme().px("--space-6");
  const auto dir = width() >= two_col_min ? QBoxLayout::LeftToRight
                                          : QBoxLayout::TopToBottom;
  if (columns_->direction() != dir)
    columns_->setDirection(dir);
}

void StartPage::rebuild_state() {
  clear_layout(main_col_);
  drop_zone_ = nullptr;
  switch (session_.state()) {
  case ProjectSession::State::empty:
    main_col_->addWidget(make_hero(false));
    break;
  case ProjectSession::State::loading:
    main_col_->addWidget(make_loading());
    break;
  case ProjectSession::State::failed:
    main_col_->addWidget(make_error());
    main_col_->addWidget(make_hero(true));
    break;
  case ProjectSession::State::open:
    main_col_->addWidget(make_summary());
    break;
  }
  main_col_->addStretch(1);
  set_drop_active(drop_active_);
}

QWidget *StartPage::make_hero(bool compact) {
  const Theme &t = theme();
  auto *card = new Panel();
  auto *b = card->body();
  b->setSpacing(t.px("--space-3"));
  if (!compact) {
    b->addWidget(
        new CapsLabel("Kom i gang", "eyebrowSurface", "--tracking-wide"));
    b->addWidget(label("Åbn et projekt", "heroTitle"));
    b->addWidget(
        label("En .rux-fil rummer en hel scanning: billeder med poser, "
              "punktskyer, mesh, panoramaer og kortlægningen. "
              "Oprettes med rux import.",
              "heroText", true));
  }
  auto *row = new QHBoxLayout;
  row->setSpacing(t.px("--space-3"));
  auto *open = button(compact ? "Vælg en anden fil…" : "Åbn projekt…",
                      compact ? "secondary" : "primary");
  connect(open, &QPushButton::clicked, this, &StartPage::browse_requested);
  row->addWidget(open);
  // The palette footer's inline key chip, not a button-height box.
  auto *kbd = label("Ctrl+O", "kbd");
  row->addWidget(kbd, 0, Qt::AlignVCenter);
  row->addStretch(1);
  b->addLayout(row);

  drop_zone_ = new QFrame;
  drop_zone_->setObjectName("dropZone");
  auto *d = new QVBoxLayout(drop_zone_);
  const int p = compact ? t.px("--space-4") : t.px("--space-6");
  d->setContentsMargins(p, p, p, p);
  d->setSpacing(t.px("--space-1"));
  auto *dt = label("Slip en .rux-fil her", "dropTitle");
  dt->setAlignment(Qt::AlignCenter);
  d->addWidget(dt);
  auto *dh =
      label("Træk projektfilen ind i vinduet for at åbne den", "dropHint");
  dh->setAlignment(Qt::AlignCenter);
  d->addWidget(dh);
  b->addWidget(drop_zone_);
  return card;
}

QWidget *StartPage::make_loading() {
  const Theme &t = theme();
  auto *card = new Panel();
  auto *b = card->body();
  b->setSpacing(t.px("--space-3"));
  b->addWidget(new CapsLabel("Åbner", "eyebrowSurface", "--tracking-wide"));
  b->addWidget(label(QFileInfo(session_.path()).fileName(), "heroTitle"));
  b->addWidget(new ElidedLabel(session_.path(), Qt::ElideMiddle));
  b->itemAt(b->count() - 1)->widget()->setObjectName("pathMono");
  auto *bar = new QProgressBar;
  bar->setRange(0, 0); // indeterminate: a ProjectDB open reports no progress
  bar->setTextVisible(false);
  b->addWidget(bar);
  b->addWidget(label("Store projekter migreres til det nyeste skema første "
                     "gang, de åbnes. Det kan tage et øjeblik.",
                     "heroText", true));
  return card;
}

QWidget *StartPage::make_error() {
  const Theme &t = theme();
  const auto &err = session_.error();
  auto *card = new QFrame;
  card->setObjectName("errorCard");
  auto *b = new QVBoxLayout(card);
  const int p = t.px("--space-4");
  b->setContentsMargins(p, p, p, p);
  b->setSpacing(t.px("--space-2"));
  auto *top = new QHBoxLayout;
  top->addWidget(new Pill("Fejl", "crit"));
  top->addStretch(1);
  b->addLayout(top);
  b->addWidget(label(err.title, "errorTitle", true));
  b->addWidget(label(err.hint, "errorText", true));
  auto *path = new ElidedLabel(session_.path(), Qt::ElideMiddle);
  path->setObjectName("pathMono");
  b->addWidget(path);
  if (!err.detail.isEmpty() && err.detail != session_.path()) {
    auto *detail = label(err.detail, "errorDetail", true);
    detail->setTextInteractionFlags(Qt::TextSelectableByMouse);
    b->addWidget(detail);
  }
  auto *row = new QHBoxLayout;
  row->setContentsMargins(0, t.px("--space-2"), 0, 0);
  row->setSpacing(t.px("--space-2"));
  const QString failed = session_.path();
  if (err.kind == OpenErrorKind::locked) {
    auto *ro = button("Åbn skrivebeskyttet", "primary");
    connect(ro, &QPushButton::clicked, this,
            [this, failed] { emit open_requested(failed, true); });
    row->addWidget(ro);
  }
  if (err.kind != OpenErrorKind::not_a_project &&
      err.kind != OpenErrorKind::not_a_file) {
    auto *retry = button("Prøv igen", "secondary");
    connect(retry, &QPushButton::clicked, this,
            [this, failed] { emit open_requested(failed, false); });
    row->addWidget(retry);
  }
  row->addStretch(1);
  b->addLayout(row);
  return card;
}

QWidget *StartPage::make_summary() {
  const Theme &t = theme();
  const auto &s = session_.summary();
  auto *card = new Panel();
  auto *b = card->body();
  b->setSpacing(t.px("--space-3"));

  auto *eyebrow_row = new QHBoxLayout;
  eyebrow_row->setSpacing(t.px("--space-2"));
  eyebrow_row->addWidget(
      new CapsLabel("Åbent projekt", "eyebrowSurface", "--tracking-wide"));
  eyebrow_row->addStretch(1);
  const int latest = reusex::ProjectDB::latest_schema_version();
  if (s.schema_version < latest)
    eyebrow_row->addWidget(
        new Pill(QString("Skema v%1 · ældre").arg(s.schema_version), "warn"));
  else
    eyebrow_row->addWidget(
        new Pill(QString("Skema v%1").arg(s.schema_version), "good"));
  eyebrow_row->addWidget(session_.is_read_only()
                             ? new Pill("Skrivebeskyttet", "warn")
                             : new Pill("Læs og skriv", "outline"));
  b->addLayout(eyebrow_row);

  b->addWidget(label(session_.display_name(), "heroTitle"));
  auto *path = new ElidedLabel(session_.path(), Qt::ElideMiddle);
  path->setObjectName("pathMono");
  b->addWidget(path);

  if (!s.projects.empty()) {
    const auto &pi = s.projects.front();
    QStringList bits;
    if (!pi.building_address.empty())
      bits << QString::fromStdString(pi.building_address);
    if (pi.year_of_construction > 0)
      bits << QString("opført %1").arg(pi.year_of_construction);
    if (!pi.survey_date.empty())
      bits
          << QString("kortlagt %1").arg(QString::fromStdString(pi.survey_date));
    if (!bits.isEmpty())
      b->addWidget(label(bits.join(" · "), "heroText", true));
  }
  if (session_.is_read_only())
    b->addWidget(label(session_.read_only_reason() +
                           " Du kan se alt, men ikke gemme ændringer.",
                       "warnText", true));

  // KPI grid: counts straight from ProjectDB::project_summary().
  std::size_t biggest = 0;
  for (const auto &c : s.clouds)
    biggest = std::max(biggest, c.point_count);
  const int scans = static_cast<int>(s.sensor_frames.scans.size());
  struct Kpi {
    QString label, value, hint;
  };
  const QVector<Kpi> kpis = {
      {"Billeder",
       format_count(static_cast<qulonglong>(s.sensor_frames.total_count)),
       scans > 1 ? QString("i %1 scanninger").arg(scans)
                 : QString("%1 segmenterede")
                       .arg(format_count(static_cast<qulonglong>(
                           s.sensor_frames.segmented_count)))},
      {"Punktskyer", QString::number(s.clouds.size()),
       biggest ? QString("største %1 punkter").arg(format_count(biggest))
               : QString("kør rux create clouds")},
      {"Mesh", QString::number(s.meshes.size()),
       s.meshes.empty() ? QString("kør rux create mesh")
                        : QString("%1 flader")
                              .arg(format_count(static_cast<qulonglong>(
                                  s.meshes.front().face_count)))},
      {"Panoramaer", QString::number(s.panoramic_images.total_count),
       QString("%1 koblet til billeder").arg(s.panoramic_images.matched_count)},
      {"Ressourcer", QString::number(s.materials.size()), "materialepas"},
      {"Komponenter", QString::number(s.components.total_count),
       QString("%1 typer").arg(s.components.count_by_type.size())},
  };
  auto *grid = new QGridLayout;
  grid->setSpacing(t.px("--space-3"));
  for (int i = 0; i < kpis.size(); ++i)
    grid->addWidget(
        new StatCard(kpis[i].label, kpis[i].value, {}, kpis[i].hint), i / 3,
        i % 3);
  b->addSpacing(t.px("--space-1"));
  b->addLayout(grid);

  auto *row = new QHBoxLayout;
  row->setContentsMargins(0, t.px("--space-2"), 0, 0);
  row->setSpacing(t.px("--space-2"));
  auto *go = button("Gå til Oversigt", "primary");
  connect(go, &QPushButton::clicked, this, [this] {
    emit navigate_requested(static_cast<int>(Workspace::database));
  });
  row->addWidget(go);
  auto *other = button("Åbn et andet projekt…", "secondary");
  connect(other, &QPushButton::clicked, this, &StartPage::browse_requested);
  row->addWidget(other);
  row->addStretch(1);
  row->addWidget(
      label(QString("Åbnet på %1 ms").arg(session_.load_ms()), "metaText"));
  b->addLayout(row);
  return card;
}

void StartPage::rebuild_recent() {
  clear_layout(recent_col_);
  const Theme &t = theme();
  auto *panel = new Panel("Seneste projekter");
  auto *b = panel->body();
  b->setContentsMargins(t.px("--space-2"), t.px("--space-2"), t.px("--space-2"),
                        t.px("--space-2"));
  b->setSpacing(0);
  const auto entries = recent_.entries();
  if (entries.isEmpty()) {
    auto *e = label("Ingen seneste projekter endnu. De projekter, du åbner, "
                    "vises her.",
                    "emptyText", true);
    e->setContentsMargins(t.px("--space-2"), t.px("--space-3"),
                          t.px("--space-2"), t.px("--space-3"));
    b->addWidget(e);
  }
  const QString current = session_.is_open() ? session_.path() : QString();
  int missing = 0;
  for (const auto &e : entries) {
    auto *row = new QPushButton;
    row->setObjectName("recentRow");
    row->setProperty("missing", e.missing);
    row->setCursor(e.missing ? Qt::ArrowCursor : Qt::PointingHandCursor);
    row->setToolTip(e.path);
    auto *l = new QHBoxLayout(row);
    l->setContentsMargins(t.px("--space-3"), t.px("--space-2"),
                          t.px("--space-3"), t.px("--space-2"));
    l->setSpacing(t.px("--space-3"));
    auto *text = new QVBoxLayout;
    text->setSpacing(t.px("--space-0"));
    auto *name = new ElidedLabel(recent_title(e.path), Qt::ElideRight);
    name->setObjectName("recentName");
    name->setProperty("missing", e.missing);
    auto *dir = new ElidedLabel(pretty_dir(e.path), Qt::ElideMiddle);
    dir->setObjectName("recentPath");
    text->addWidget(name);
    text->addWidget(dir);
    l->addLayout(text, 1);
    for (QWidget *w :
         {static_cast<QWidget *>(name), static_cast<QWidget *>(dir)})
      w->setAttribute(Qt::WA_TransparentForMouseEvents);
    if (e.missing) {
      ++missing;
      l->addWidget(new Pill("Mangler", "crit"), 0, Qt::AlignVCenter);
      auto *rm = button("Fjern", "ghost");
      rm->setObjectName("recentRemove");
      const QString p = e.path;
      connect(rm, &QPushButton::clicked, this,
              [this, p] { recent_.remove(p); });
      l->addWidget(rm, 0, Qt::AlignVCenter);
    } else {
      if (e.path == current)
        l->addWidget(new Pill("Åben", "accent"), 0, Qt::AlignVCenter);
      const QString p = e.path;
      connect(row, &QPushButton::clicked, this,
              [this, p] { emit open_requested(p, false); });
    }
    row->setMinimumHeight(l->sizeHint().height());
    b->addWidget(row);
  }
  if (!entries.isEmpty()) {
    auto *clear = button("Ryd", "ghost");
    connect(clear, &QPushButton::clicked, this, [this] { recent_.clear(); });
    panel->add_header_widget(label(
        missing ? QString("%1 af %2 mangler").arg(missing).arg(entries.size())
                : QString::number(entries.size()),
        "panelCount"));
    panel->add_header_widget(clear);
  }
  recent_col_->addWidget(panel);
  recent_col_->addStretch(1);
}

} // namespace rux::qt
