// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "demo_pages.hpp"

#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

namespace rux::qt::gallery {
namespace {

QWidget *title_bar(const PageContext &ctx) {
  auto *bar = new QFrame;
  bar->setObjectName("titleBar");
  bar->setFixedHeight(theme().px("--layout-titlebar-height"));
  auto *l = new QHBoxLayout(bar);
  l->setContentsMargins(theme().px("--space-4"), 0, theme().px("--space-4"), 0);
  l->setSpacing(theme().px("--space-3"));

  // "ReUseX" with the X in the accent, as the web title bar. Rich text needs
  // a colour value, so it is taken from the token at run time.
  auto *product = new QLabel(QString("REUSE<span style=\"color:%1\">X</span>")
                                 .arg(theme().color("--color-accent").name()));
  product->setObjectName("product");
  product->setTextFormat(Qt::RichText);
  l->addWidget(product);

  auto *divider = new QFrame;
  divider->setObjectName("titleDivider");
  divider->setFixedSize(1, theme().px("--space-4"));
  l->addWidget(divider);

  auto *project = new QLabel(ctx.project_path.isEmpty()
                                 ? QString("Intet projekt")
                                 : QFileInfo(ctx.project_path).fileName());
  project->setObjectName("titleProject");
  l->addWidget(project);
  l->addStretch(1);

  if (ctx.db) {
    auto *meta = new QLabel(QString("Skema v%1").arg(ctx.db->schema_version()));
    meta->setObjectName("titleMeta");
    l->addWidget(meta);
  }
  auto *status =
      new Pill(ctx.db ? "Klar" : "Ingen data", ctx.db ? "good" : "wait");
  l->addWidget(status);
  return bar;
}

} // namespace

QWidget *demo_frame(const PageContext &ctx, int active, QWidget *content,
                    QWidget *inspector) {
  auto *root = new QWidget;
  root->setObjectName("appRoot");
  auto *v = new QVBoxLayout(root);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  v->addWidget(title_bar(ctx));

  auto *body = new QHBoxLayout;
  body->setSpacing(0);
  v->addLayout(body, 1);

  QString name = "Intet projekt";
  QString frames, clouds;
  if (ctx.db) {
    const auto s = ctx.db->project_summary();
    name = QFileInfo(ctx.project_path).completeBaseName();
    if (!s.projects.empty() && !s.projects.front().name.empty())
      name = QString::fromStdString(s.projects.front().name);
    frames = format_count(static_cast<qulonglong>(s.sensor_frames.total_count));
    clouds = QString::number(s.clouds.size());
  }

  auto *rail = new NavRail(name);
  rail->add_group("Data");
  rail->add_item("Oversigt", frames);
  rail->add_item("3D", clouds);
  rail->add_item("Rediger");
  rail->add_group("Behandling");
  rail->add_item("Pipeline");
  rail->add_item("Log");
  rail->add_item("Komponenter");
  // Mono for the key only; the words are prose.
  auto *hint_row = new QWidget;
  auto *hl = new QHBoxLayout(hint_row);
  hl->setContentsMargins(0, 0, 0, 0);
  hl->setSpacing(theme().px("--space-2"));
  auto *kbd = new QLabel("Ctrl+K");
  kbd->setObjectName("railKbd");
  auto *hint = new QLabel("Kommandopalet");
  hint->setObjectName("railHint");
  hl->addWidget(kbd);
  hl->addWidget(hint);
  hl->addStretch(1);
  rail->add_footer(hint_row);
  rail->set_current(active);
  body->addWidget(rail);

  content->setObjectName(
      content->objectName().isEmpty() ? "content" : content->objectName());
  body->addWidget(content, 1);
  if (inspector) {
    inspector->setFixedWidth(theme().px("--layout-panel-width"));
    body->addWidget(inspector);
  }
  return root;
}

} // namespace rux::qt::gallery
