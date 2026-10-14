// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/EdgeListPanel.hpp>
#include <rux_qt/EditWorkspace.hpp>
#include <rux_qt/GraphPanel.hpp>
#include <rux_qt/PairImages.hpp>
#include <rux_qt/PairOverlay.hpp>
#include <rux_qt/PairStrip.hpp>
#include <rux_qt/PendingBanner.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>
#include <rux_qt/workspace_logic.hpp>

#include <QAction>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QPushButton>
#include <QSettings>
#include <QSignalBlocker>
#include <QSplitter>
#include <QVBoxLayout>

#include <algorithm>
#include <optional>

namespace rux::qt {
namespace {

/// The Rediger button (and status) name of a stage it runs.
QString run_button_name(reusex::pipeline::JobStage stage) {
  return stage == reusex::pipeline::JobStage::optimize ? QString("Optimer")
                                                       : QString("Find kanter");
}

QString key_of(EditPane p) {
  const std::string_view k = edit_pane_key(p);
  return QString::fromLatin1(k.data(), static_cast<qsizetype>(k.size()));
}

} // namespace

EditWorkspace::EditWorkspace(ProjectSession &session, EdgeEditor &editor,
                             ViewSettings &view_settings, bool persist_layout,
                             bool interactive_3d, QWidget *parent)
    : QWidget(parent), session_(session), editor_(editor),
      persist_(persist_layout) {
  setObjectName("editWorkspace");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  v->addWidget(make_toolbar());
  banner_ = new PendingBanner(editor_);
  v->addWidget(banner_);

  graph_ = new GraphPanel(session_, editor_, view_settings, persist_);
  panes_[0] = new PaneFrame("Graf");
  panes_[0]->set_content(graph_);
  panes_[1] = new PaneFrame("Kanter");
  edges_ = new EdgeListPanel(editor_, persist_);
  panes_[1]->set_content(edges_);
  panes_[2] = new PaneFrame("Billeder");
  // Each side says which click fills it while its slot is empty.
  panes_[2]->set_content(images_ = new PairImages(session_));
  panes_[3] = new PaneFrame("3D-overlay");
  panes_[3]->set_content(overlay_ = new PairOverlay(session_, interactive_3d));
  auto *reset = new QPushButton(NavItem::escape_mnemonic("Nulstil"));
  reset->setProperty("kind", "ghost");
  reset->setCursor(Qt::PointingHandCursor);
  reset->setToolTip("Sæt B tilbage på sin gemte pose");
  reset->setMinimumHeight(reset->sizeHint().height());
  connect(reset, &QPushButton::clicked, overlay_, &PairOverlay::reset_offset);
  panes_[3]->add_header_widget(reset);

  left_ = new QSplitter(Qt::Vertical);
  left_->addWidget(panes_[0]);
  left_->addWidget(panes_[1]);
  left_->setStretchFactor(0, 3);
  left_->setStretchFactor(1, 2);
  right_ = new QSplitter(Qt::Vertical);
  right_->addWidget(panes_[2]);
  right_->addWidget(panes_[3]);
  // The images and the overlay (its canvas over six offset fields) share
  // the column evenly.
  right_->setStretchFactor(0, 1);
  right_->setStretchFactor(1, 1);
  right_column_ = new QWidget;
  auto *rc = new QVBoxLayout(right_column_);
  rc->setContentsMargins(0, 0, 0, 0);
  rc->setSpacing(0);
  auto *head = new QFrame;
  head->setObjectName("paneHeader");
  auto *hl = new QHBoxLayout(head);
  hl->setContentsMargins(t.px("--space-3"), t.px("--space-2"),
                         t.px("--space-3"), t.px("--space-2"));
  hl->setSpacing(t.px("--space-3"));
  hl->addWidget(new CapsLabel("Par", "panelTitle"));
  pair_a_ = new QLabel;
  pair_a_->setObjectName("pairIds");
  pair_b_ = new QLabel;
  pair_b_->setObjectName("pairIds");
  hl->addWidget(pair_a_);
  hl->addWidget(pair_b_);
  next_hint_ = new QLabel;
  next_hint_->setObjectName("toolbarMeta");
  next_hint_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  hl->addWidget(next_hint_, 1);
  auto *swap = new QPushButton(NavItem::escape_mnemonic("Byt A og B"));
  swap->setProperty("kind", "ghost");
  swap->setCursor(Qt::PointingHandCursor);
  connect(swap, &QPushButton::clicked, this, [this] { set_pair(b(), a()); });
  hl->addWidget(swap);
  rc->addWidget(head);
  rc->addWidget(right_, 1);
  strip_ = new PairStrip(session_, editor_);
  rc->addWidget(strip_);
  // Takes the column's spare height when both panes are hidden (a stretch
  // of 0 gets none while right_ is shown with 1).
  rc->addStretch(0);
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
  // The edge list gets room for a screenful of suspicious rows.
  left_->setSizes({3 * kUnit, 2 * kUnit});
  // The images over the overlay, half each: the overlay's fields take a
  // fixed share of its half, so a 3:2 split left the canvas a sliver.
  right_->setSizes({kUnit, kUnit});

  for (int i = 0; i < kEditPaneCount; ++i)
    connect(panes_[i], &PaneFrame::hide_requested, this,
            [this, i] { set_pane_visible(static_cast<EditPane>(i), false); });
  for (QSplitter *s : {main_, left_, right_})
    connect(s, &QSplitter::splitterMoved, this, [this] { save_layout(); });
  connect(graph_, &GraphPanel::selection_changed, this,
          [this](const Selection &s) {
            selection_ = s;
            emit selection_changed(s);
          });
  connect(graph_, &GraphPanel::frame_requested, this,
          &EditWorkspace::node_clicked);
  connect(graph_, &GraphPanel::pair_requested, this, &EditWorkspace::set_pair);
  // The list and the graph select each other's edge: a row highlights and
  // inspects the edge and makes its ends A and B (click_edge, which does not
  // emit edge_clicked); an edge clicked on the canvas selects its row
  // (select_edge, which does not emit edge_activated).
  connect(edges_, &EdgeListPanel::edge_activated, this,
          [this](const EdgeKey &k) { graph_->click_edge(k); });
  connect(graph_, &GraphPanel::edge_clicked, edges_,
          &EdgeListPanel::select_edge);
  connect(graph_, &GraphPanel::colour_mode_changed, edges_,
          [this](EdgeColourMode m) {
            edges_->set_sort(default_sort_column(m), true);
          });
  edges_->set_sort(default_sort_column(graph_->colour_mode()), true);
  connect(strip_, &PairStrip::edge_selected, this, [this](const Selection &s) {
    selection_ = s;
    emit selection_changed(s);
  });
  // ICP-forfin starts from B's manual offset and moves B to its result.
  strip_->set_delta_source([this]() -> std::optional<Mat4> {
    if (!overlay_->ready() || overlay_->offset().is_zero())
      return std::nullopt;
    return overlay_->delta();
  });
  connect(strip_, &PairStrip::icp_aligned, this,
          [this](int a, int b, const Mat4 &delta) {
            if (a == slots_.a() && b == slots_.b() && overlay_->ready())
              overlay_->set_offset(
                  offset_from_matrix(delta, overlay_->pivot()));
          });
  connect(&session_, &ProjectSession::state_changed, this,
          &EditWorkspace::on_session_state);
  restore_layout();
  apply_pair();
  sync_run_buttons();
}

void EditWorkspace::on_session_state() {
  sync_run_buttons();
  const QString path = session_.path();
  switch (session_.state()) {
  case ProjectSession::State::loading:
    // A reload of the same project (F5, the one after "Optimer") keeps the
    // pair to re-inspect; it is checked against the frames once open.
    if (path == pair_path_)
      return;
    slots_.clear();
    break;
  case ProjectSession::State::open: {
    if (path != pair_path_) {
      slots_.clear();
    } else if (const auto *db = session_.db()) {
      try {
        slots_.forget_missing(db->sensor_frame_ids());
      } catch (const std::exception &) {
        slots_.clear();
      }
    }
    pair_path_ = path;
    break;
  }
  case ProjectSession::State::empty:
  case ProjectSession::State::failed:
    slots_.clear();
    pair_path_.clear();
    break;
  }
  apply_pair();
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
  // A long status (a discover run's counts) is cut, never widens the window;
  // the tooltip holds all of it.
  meta_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
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

  discover_ = new QPushButton(NavItem::escape_mnemonic(
      run_button_name(reusex::pipeline::JobStage::graph_discover)));
  discover_->setProperty("kind", "secondary");
  discover_->setCursor(Qt::PointingHandCursor);
  discover_->setToolTip("rux graph discover: nye løkkelukninger fra de "
                        "oprindelige poser (afviste kanter springes over)");
  optimize_ = new QPushButton(NavItem::escape_mnemonic(
      run_button_name(reusex::pipeline::JobStage::optimize)));
  optimize_->setProperty("kind", "primary");
  optimize_->setCursor(Qt::PointingHandCursor);
  optimize_->setToolTip("rux optimize: løs den gemte posegraf");
  for (QPushButton *b : {discover_, optimize_}) {
    b->setMinimumHeight(b->sizeHint().height());
    bl->addWidget(b);
  }
  connect(discover_, &QPushButton::clicked, this, [this] {
    emit run_requested(reusex::pipeline::JobStage::graph_discover);
  });
  connect(optimize_, &QPushButton::clicked, this,
          [this] { emit run_requested(reusex::pipeline::JobStage::optimize); });
  bl->addWidget(panes);
  return bar;
}

void EditWorkspace::sync_run_buttons() {
  const bool can = session_.is_open() && running_.isEmpty();
  discover_->setEnabled(can);
  optimize_->setEnabled(can);
}

void EditWorkspace::set_job_running(const QString &stage_da) {
  running_ = stage_da;
  sync_run_buttons();
  meta_->setToolTip({});
  meta_->setText(
      QString("%1 kører … — grafen opdateres, når trinnet er færdigt.")
          .arg(stage_da));
}

void EditWorkspace::set_job_finished() {
  // show_job_result (just before) replaces the running text with the
  // outcome; any other stage's leaves nothing to say here.
  if (!running_.isEmpty())
    meta_->clear();
  running_.clear();
  sync_run_buttons();
}

void EditWorkspace::show_job_result(reusex::pipeline::JobStage stage, bool ok,
                                    const QString &message) {
  using reusex::pipeline::JobStage;
  if (stage != JobStage::graph_discover && stage != JobStage::optimize)
    return;
  running_.clear(); // set_job_finished keeps this text
  const QString name = run_button_name(stage);
  meta_->setToolTip(message);
  if (!ok) {
    meta_->setText(message.isEmpty()
                       ? QString("%1 blev ikke gennemført.").arg(name)
                       : QString("%1 fejlede: %2").arg(name, message));
    return;
  }
  // With edits pending the reload that shows the result was skipped ("Kør
  // uden dem"): the graph on screen is still the one before the run.
  const QString hidden =
      pending_edits() > 0
          ? QString(" — gem eller kassér ændringerne for at se resultatet")
          : QString();
  if (stage == JobStage::optimize) {
    meta_->setText(
        QString("%1 er færdig — poser og kantmålinger er opdateret%2.")
            .arg(name, hidden));
    return;
  }
  const auto c = parse_discover_summary(message.toStdString());
  if (!c) {
    meta_->setText(QString("%1 er færdig%2.").arg(name, hidden));
    return;
  }
  const auto n = [](int v) {
    return format_count(static_cast<qulonglong>(std::max(v, 0)));
  };
  // The line names what happened (new edges always, the rest when not 0);
  // the tooltip every count.
  const int skipped = c->skipped_existing + c->skipped_rejected;
  QStringList line{QString("%1 nye").arg(n(c->added))};
  if (skipped > 0)
    line << QString("%1 sprunget over").arg(n(skipped));
  if (c->pcm_rejected > 0)
    line << QString("%1 afvist af PCM").arg(n(c->pcm_rejected));
  if (c->pcm_capped > 0)
    line << QString("%1 over PCM-loftet").arg(n(c->pcm_capped));
  if (c->filled > 0)
    line << QString("%1 fik en måling").arg(n(c->filled));
  if (c->odometry_initialised > 0)
    line << QString("%1 odometrikanter oprettet først")
                .arg(n(c->odometry_initialised));
  const QString prefix = QString("%1%2").arg(
      c->dry_run ? QString("Prøvekørsel · ") : QString(), name);
  meta_->setText(QString("%1: %2%3").arg(prefix, line.join(" · "), hidden));
  QStringList all{
      QString("%1 foreslået, %2 matchet").arg(n(c->proposed), n(c->matched)),
      QString("%1 nye kanter").arg(n(c->added)),
      QString("%1 sprunget over: %2 findes, %3 afvist før")
          .arg(n(skipped), n(c->skipped_existing), n(c->skipped_rejected)),
      QString("%1 afvist af PCM, %2 over PCM-loftet")
          .arg(n(c->pcm_rejected), n(c->pcm_capped)),
      QString("%1 gemte kanter fik en måling").arg(n(c->filled))};
  if (c->odometry_initialised > 0)
    all << QString("%1 odometrikanter oprettet først (ingen graf)")
               .arg(n(c->odometry_initialised));
  meta_->setToolTip(prefix + "\n" + all.join("\n"));
}

void EditWorkspace::show_run_refused(reusex::pipeline::JobStage stage,
                                     const QString &reason) {
  if (reason.isEmpty())
    return; // e.g. the unsaved-edits prompt was answered "Annullér"
  const QString name = run_button_name(stage);
  meta_->setToolTip({});
  meta_->setText(QString("%1 kan ikke køre: %2").arg(name, reason));
}

void EditWorkspace::activate() { graph_->activate(); }

void EditWorkspace::node_clicked(int id, bool force_b) {
  if (slots_.click(id, force_b))
    apply_pair();
}

void EditWorkspace::set_pair(int a, int b) {
  slots_.set_pair(a, b);
  apply_pair();
}

void EditWorkspace::apply_pair() {
  graph_->set_pair(slots_.a(), slots_.b());
  strip_->set_pair(slots_.a(), slots_.b());
  images_->set_pair(slots_.a(), slots_.b(), slots_.next());
  overlay_->set_pair(slots_.a(), slots_.b());
  pair_complete_ = slots_.complete();
  sync_pair_header();
  sync_panes();
  emit pair_changed(slots_.a(), slots_.b());
}

void EditWorkspace::sync_pair_header() {
  auto id = [](int v) { return v >= 0 ? QString::number(v) : QString("—"); };
  pair_a_->setText("A  " + id(slots_.a()));
  pair_b_->setText("B  " + id(slots_.b()));
  next_hint_->setText(slots_.next() == Slot::a ? QString("Næste klik: A")
                                               : QString("Næste klik: B"));
}

int EditWorkspace::pending_edits() const { return editor_.pending(); }
bool EditWorkspace::save_edits() { return editor_.save(); }
bool EditWorkspace::save_edits_and_wait() {
  return editor_.save_and_wait(this);
}
bool EditWorkspace::is_saving() const { return editor_.is_saving(); }
void EditWorkspace::discard_edits() { editor_.discard(); }

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
  // The right column also holds the pair header and the pair strip, outside
  // the splitter and not hideable: it stays while the strip shows, so hiding
  // both panes leaves the pair and its edges on screen.
  right_column_->setVisible(right_shown || (strip_ && !strip_->isHidden()));
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
