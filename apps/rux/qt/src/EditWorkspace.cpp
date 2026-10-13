// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/EditWorkspace.hpp>
#include <rux_qt/PoseGraphWorkspace.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <QAction>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QPushButton>
#include <QSettings>
#include <QSignalBlocker>
#include <QSplitter>
#include <QVBoxLayout>

namespace rux::qt {
namespace {

QLabel *empty_state(const QString &text) {
  auto *l = new QLabel(text);
  l->setObjectName("emptyText");
  l->setAlignment(Qt::AlignCenter);
  l->setWordWrap(true);
  return l;
}

QString key_of(EditPane p) {
  const std::string_view k = edit_pane_key(p);
  return QString::fromLatin1(k.data(), static_cast<qsizetype>(k.size()));
}

} // namespace

EditWorkspace::EditWorkspace(ProjectSession &session, EdgeEditor &editor,
                             bool persist_layout, QWidget *parent)
    : QWidget(parent), session_(session), editor_(editor),
      persist_(persist_layout) {
  setObjectName("editWorkspace");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  v->addWidget(make_toolbar());

  graph_ = new PoseGraphWorkspace(session_, editor_);
  panes_[0] = new PaneFrame("Graf");
  panes_[0]->set_content(graph_);
  panes_[1] = new PaneFrame("Kanter");
  panes_[1]->set_content(empty_state("Kantlisten vises her."));
  panes_[2] = new PaneFrame("Billeder");
  panes_[2]->set_content(
      empty_state("Klik på en node for A, og en til for B."));
  panes_[3] = new PaneFrame("3D-overlay");
  panes_[3]->set_content(empty_state("Vælg A og B for at se deres skyer."));

  left_ = new QSplitter(Qt::Vertical);
  left_->addWidget(panes_[0]);
  left_->addWidget(panes_[1]);
  left_->setStretchFactor(0, 3);
  left_->setStretchFactor(1, 1);
  right_ = new QSplitter(Qt::Vertical);
  right_->addWidget(panes_[2]);
  right_->addWidget(panes_[3]);
  right_column_ = new QWidget;
  auto *rc = new QVBoxLayout(right_column_);
  rc->setContentsMargins(0, 0, 0, 0);
  rc->setSpacing(0);
  rc->addWidget(right_, 1);
  main_ = new QSplitter(Qt::Horizontal);
  main_->addWidget(left_);
  main_->addWidget(right_column_);
  main_->setStretchFactor(0, 3);
  main_->setStretchFactor(1, 2);
  for (QSplitter *s : {main_, left_, right_}) {
    s->setChildrenCollapsible(false);
    s->setHandleWidth(t.px("--space-1"));
  }
  v->addWidget(main_, 1);
  // The first layout gives each pane its size hint (a narrow right column);
  // stretch factors only share out later growth. Start from the intended
  // proportions instead — QSplitter scales them to the space it gets — and
  // let a stored layout override them.
  constexpr int kUnit = 1000;
  main_->setSizes({3 * kUnit, 2 * kUnit});
  left_->setSizes({3 * kUnit, kUnit});
  right_->setSizes({kUnit, kUnit});

  for (int i = 0; i < kEditPaneCount; ++i)
    connect(panes_[i], &PaneFrame::hide_requested, this,
            [this, i] { set_pane_visible(static_cast<EditPane>(i), false); });
  for (QSplitter *s : {main_, left_, right_})
    connect(s, &QSplitter::splitterMoved, this, [this] { save_layout(); });
  connect(graph_, &PoseGraphWorkspace::selection_changed, this,
          [this](const Selection &s) {
            selection_ = s;
            emit selection_changed(s);
          });
  restore_layout();
  sync_panes();
}

EditWorkspace::~EditWorkspace() = default;

QWidget *EditWorkspace::make_toolbar() {
  const Theme &t = theme();
  auto *bar = new QFrame;
  bar->setObjectName("dbToolbar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-2"),
                         t.px("--space-4"), t.px("--space-2"));
  bl->setSpacing(t.px("--space-3"));
  bl->addWidget(new CapsLabel("Rediger", "panelTitle"));
  meta_ = new QLabel;
  meta_->setObjectName("toolbarMeta");
  bl->addWidget(meta_, 1);
  auto *panes = new QPushButton(NavItem::escape_mnemonic("Paneler"));
  panes->setProperty("kind", "ghost");
  panes->setCursor(Qt::PointingHandCursor);
  auto *menu = new QMenu(panes);
  for (int i = 0; i < kEditPaneCount; ++i) {
    const auto p = static_cast<EditPane>(i);
    QAction *a = menu->addAction(QString::fromStdString(edit_pane_name_da(p)));
    a->setCheckable(true);
    a->setChecked(true);
    connect(a, &QAction::toggled, this,
            [this, p](bool on) { set_pane_visible(p, on); });
    pane_actions_[i] = a;
  }
  panes->setMenu(menu);
  panes->setMinimumHeight(panes->sizeHint().height());
  bl->addWidget(panes);
  return bar;
}

void EditWorkspace::activate() { graph_->activate(); }

void EditWorkspace::set_pane_visible(EditPane p, bool on) {
  const int i = static_cast<int>(p);
  wanted_[i] = on;
  if (pane_actions_[i] && pane_actions_[i]->isChecked() != on) {
    const QSignalBlocker b(pane_actions_[i]);
    pane_actions_[i]->setChecked(on);
  }
  sync_panes();
  save_layout();
}

void EditWorkspace::sync_panes() {
  for (int i = 0; i < kEditPaneCount; ++i) {
    const bool needs_pair = static_cast<EditPane>(i) == EditPane::overlay;
    panes_[i]->setVisible(wanted_[i] && (!needs_pair || pair_complete_));
  }
  // A splitter with nothing visible would leave an empty strip.
  left_->setVisible(wanted_[0] || wanted_[1]);
  const bool right_shown = !panes_[2]->isHidden() || !panes_[3]->isHidden();
  right_->setVisible(right_shown);
  right_column_->setVisible(right_shown);
}

void EditWorkspace::restore_layout() {
  if (!persist_)
    return;
  QSettings s;
  main_->restoreState(s.value("rediger/split/main").toByteArray());
  left_->restoreState(s.value("rediger/split/left").toByteArray());
  right_->restoreState(s.value("rediger/split/right").toByteArray());
  for (int i = 0; i < kEditPaneCount; ++i) {
    wanted_[i] =
        s.value("rediger/visible/" + key_of(static_cast<EditPane>(i)), true)
            .toBool();
    const QSignalBlocker b(pane_actions_[i]);
    pane_actions_[i]->setChecked(wanted_[i]);
  }
}

void EditWorkspace::save_layout() const {
  if (!persist_)
    return;
  QSettings s;
  s.setValue("rediger/split/main", main_->saveState());
  s.setValue("rediger/split/left", left_->saveState());
  s.setValue("rediger/split/right", right_->saveState());
  for (int i = 0; i < kEditPaneCount; ++i)
    s.setValue("rediger/visible/" + key_of(static_cast<EditPane>(i)),
               wanted_[i]);
}

} // namespace rux::qt
