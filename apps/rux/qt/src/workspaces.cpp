// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/Theme.hpp>
#include <rux_qt/palette_table.hpp>
#include <rux_qt/widgets.hpp>
#include <rux_qt/workspaces.hpp>

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace rux::qt {

const QVector<WorkspaceInfo> &workspace_infos() {
  // Names and aliases come from the Qt-free palette table, the one list the
  // palette and its ranking tests use.
  auto name = [](int i) {
    return QString::fromStdString(
        page_entries()[static_cast<std::size_t>(i)].name);
  };
  auto keywords = [](int i) {
    return QString::fromStdString(
        page_entries()[static_cast<std::size_t>(i)].keywords);
  };
  static const QVector<WorkspaceInfo> infos = {
      {Workspace::start,
       name(0),
       keywords(0),
       "Åbn et projekt, eller fortsæt hvor du slap.",
       {},
       {},
       {}},
      {Workspace::database,
       name(1),
       keywords(1),
       "Billeder, panoramaer, skyer og alle tabeller — til at se på.",
       "Q2",
       {"Projekttræ med tabeller, skyer, mesh, billeder og panoramaer",
        "Billedbrowser med A/B-skydere: farve, dybde, konfidens og mærkater",
        "Kantstrimmel: ICP-forfining, tilføj og slet kanter, gem eksplicit",
        "Tabelvisning af hver sqlite-tabel og pipeline-loggen"},
       {"rux info", "rux get frames", "rux get clouds"}},
      {Workspace::viewer3d,
       name(2),
       keywords(2),
       "Punktskyer, mesh og kameraer i én scene.",
       "Q3",
       {"Lag: skyer, mesh, kamerafrustummer og panoramaer",
        "Farv efter en mærkatsky, med forklaring",
        "Visningsforudindstillinger og snitplan"},
       {"rux view", "rux render -o plan.png --view plan"}},
      {Workspace::edit,
       name(3),
       keywords(3),
       "Posegrafen: find de mistænkelige kanter, ret dem og bekræft dem.",
       "",
       {"Graf og kantliste, farvet efter type, status eller afvigelse",
        "Billedpar med 3D-overlay, manuel justering og ICP"},
       {"rux graph discover", "rux graph prune", "rux optimize"}},
      {Workspace::pipeline,
       name(4),
       keywords(4),
       "Kør trin med parametre, fremskridt og log.",
       "Q3",
       {"Parameterformular for hvert trin",
        "Kør i programmet med fremskridt og log", "Kopiér som rux-kommando"},
       {"rux create planes", "rux create rooms", "rux validate --stage mesh"}},
      {Workspace::log,
       name(5),
       keywords(5),
       "Hvad der er kørt på projektet, og hvornår.",
       "Q3",
       {"Pipeline-loggen med parametre og varighed",
        "Filtrér efter trin og status"},
       {"rux log"}},
  };
  return infos;
}

const WorkspaceInfo &workspace_info(Workspace w) {
  return workspace_infos()[static_cast<int>(w)];
}

WorkspacePlaceholder::WorkspacePlaceholder(const WorkspaceInfo &info,
                                           QWidget *parent)
    : QWidget(parent) {
  setObjectName("content");
  const Theme &t = theme();
  auto *outer = new QVBoxLayout(this);
  const int m = t.px("--space-6");
  outer->setContentsMargins(m, t.px("--space-5"), m, m);
  outer->setSpacing(t.px("--space-5"));

  // View head, as every workspace will have it.
  auto *head = new QHBoxLayout;
  head->setSpacing(t.px("--space-3"));
  head->addWidget(new CapsLabel(info.name, "viewTitle", QString()), 0,
                  Qt::AlignBottom);
  auto *sub = new QLabel(info.subtitle);
  sub->setObjectName("viewSub");
  head->addWidget(sub, 0, Qt::AlignBottom);
  head->addStretch(1);
  outer->addLayout(head);

  // The empty state, centred in the remaining space.
  auto *card = new QFrame;
  card->setObjectName("emptyCard");
  card->setMaximumWidth(2 * t.px("--layout-panel-width") + t.px("--space-7"));
  auto *c = new QVBoxLayout(card);
  const int p = t.px("--space-6");
  c->setContentsMargins(p, p, p, p);
  c->setSpacing(t.px("--space-3"));

  auto *pill_row = new QHBoxLayout;
  pill_row->addWidget(new Pill(QString("Kommer i %1").arg(info.phase), "wait"));
  pill_row->addStretch(1);
  c->addLayout(pill_row);

  auto *title = new QLabel("Under opbygning");
  title->setObjectName("emptyTitle");
  c->addWidget(title);
  auto *lead = new QLabel(
      QString("%1-arbejdsområdet bliver bygget i fase %2 af Qt-klienten. Det "
              "kommer til at rumme:")
          .arg(info.name, info.phase));
  lead->setObjectName("emptyText");
  lead->setWordWrap(true);
  c->addWidget(lead);

  auto *bullets = new QVBoxLayout;
  bullets->setSpacing(t.px("--space-2"));
  bullets->setContentsMargins(0, t.px("--space-1"), 0, t.px("--space-1"));
  for (const QString &b : info.coming) {
    auto *row = new QHBoxLayout;
    row->setSpacing(t.px("--space-3"));
    auto *dot = new QLabel;
    dot->setObjectName("bulletDot");
    row->addWidget(dot, 0, Qt::AlignTop);
    auto *txt = new QLabel(b);
    txt->setObjectName("bulletText");
    txt->setWordWrap(true);
    row->addWidget(txt, 1);
    bullets->addLayout(row);
  }
  c->addLayout(bullets);

  auto *rule = new QFrame;
  rule->setObjectName("rule");
  rule->setFixedHeight(1);
  c->addWidget(rule);

  c->addWidget(new CapsLabel("Indtil da i terminalen", "fieldLabel"));
  auto *chips = new QHBoxLayout;
  chips->setSpacing(t.px("--space-2"));
  for (const QString &cmd : info.cli) {
    auto *chip = new QLabel(cmd);
    chip->setObjectName("codeChip");
    chip->setTextInteractionFlags(Qt::TextSelectableByMouse);
    chips->addWidget(chip);
  }
  chips->addStretch(1);
  c->addLayout(chips);

  auto *foot = new QHBoxLayout;
  foot->setContentsMargins(0, t.px("--space-3"), 0, 0);
  foot->setSpacing(t.px("--space-3"));
  status_ = new QLabel;
  status_->setObjectName("emptyStatus");
  foot->addWidget(status_, 1);
  open_ = new QPushButton(NavItem::escape_mnemonic("Åbn projekt…"));
  open_->setProperty("kind", "primary");
  open_->setCursor(Qt::PointingHandCursor);
  connect(open_, &QPushButton::clicked, this,
          &WorkspacePlaceholder::browse_requested);
  foot->addWidget(open_);
  c->addLayout(foot);

  auto *centre = new QHBoxLayout;
  centre->addStretch(1);
  centre->addWidget(card, 4);
  centre->addStretch(1);
  outer->addStretch(1);
  outer->addLayout(centre);
  outer->addStretch(2);

  set_project_open(false);
}

void WorkspacePlaceholder::set_project_open(bool open, const QString &status) {
  open_->setVisible(!open);
  status_->setText(open ? status
                        : QString("Åbn et projekt for at komme i gang."));
}

} // namespace rux::qt
