// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The A<->B pair strip of Rediger: the pose-graph edges between the
// two frames (stored and pending), how far apart the frames are, and the
// edits — ICP refine (slam::refine_frame_pair_icp, on a detached thread
// with its own read-only connection, seeded with B's manual offset and
// moving B to its result), add a measured edge (pair_edge_to_add: ICP's
// B -> A edge while its fit is current, else T_A^-1 · ΔT · T_B), and per
// row confirm / un-confirm, reject / un-reject, revert.

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/PairStrip.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/edge_table.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/frame_pair_icp.hpp>

#include <QApplication>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPointer>
#include <QPushButton>
#include <QVBoxLayout>

#include <cmath>
#include <memory>
#include <thread>

namespace rux::qt {
namespace {

QString dec(double v, int decimals) {
  return QString::fromStdString(format_decimal_da(v, decimals));
}

QString residual_text(double r) {
  return std::isfinite(r) ? dec(r, 2) : QString("—");
}

QString status_da(const std::string &status) {
  return status == "confirmed"  ? QString("Bekræftet")
         : status == "rejected" ? QString("Afvist")
                                : QString("Auto");
}

QString status_tone(const std::string &status) {
  return status == "confirmed"  ? QString("good")
         : status == "rejected" ? QString("crit")
                                : QString("outline");
}

QString weight_text(double w) {
  if (!std::isfinite(w))
    return "—";
  return w >= 100 ? format_count(static_cast<qulonglong>(std::llround(w)))
                  : dec(w, 2);
}

/// A clickable edge row: shows the edge in the inspector when clicked.
class EdgeRow : public QFrame {
    public:
  explicit EdgeRow(std::function<void()> on_click)
      : on_click_(std::move(on_click)) {
    setObjectName("edgeRow");
    setCursor(Qt::PointingHandCursor);
  }

    protected:
  void mousePressEvent(QMouseEvent *e) override {
    if (on_click_)
      on_click_();
    QFrame::mousePressEvent(e);
  }

    private:
  std::function<void()> on_click_;
};

Selection edge_selection(const EdgeView &v, int a, int b) {
  Selection s;
  s.kind = "Kant";
  s.title = QString::fromStdString(edge_type_da(v.edge.key.type));
  s.subtitle = QString("%1 → %2").arg(v.edge.key.from).arg(v.edge.key.to);
  if (v.pending != EdgeView::Pending::none) {
    s.pill = "Ikke gemt";
    s.pill_tone = "accent";
  } else {
    s.pill = status_da(v.edge.status);
    s.pill_tone = status_tone(v.edge.status);
  }
  const auto add = [](SelectionSection &sec, const QString &key,
                      const QString &value,
                      SelectionRow::Style style = SelectionRow::Style::mono) {
    SelectionRow r;
    r.key = key;
    r.value = value;
    r.style = style;
    sec.rows.push_back(r);
  };
  SelectionSection e;
  e.title = "Kant";
  add(e, "Fra", QString::number(v.edge.key.from));
  add(e, "Til", QString::number(v.edge.key.to));
  add(e, "Type", QString::fromStdString(v.edge.key.type));
  add(e, "Vægt", weight_text(v.edge.weight));
  add(e, "Residual", residual_text(v.edge.residual));
  add(e, "Status", status_da(v.edge.status), SelectionRow::Style::value);
  if (!v.edge.source.empty())
    add(e, "Kilde", QString::fromStdString(v.edge.source));
  add(e, "Retning",
      v.reversed ? QString("B → A (%1 → %2)").arg(b).arg(a)
                 : QString("A → B (%1 → %2)").arg(a).arg(b),
      SelectionRow::Style::value);
  s.sections.push_back(e);
  SelectionSection about;
  about.title = "Om tallene";
  add(about, "Vægt", "Translationsinformation, 1/σ²",
      SelectionRow::Style::value);
  add(about, "Residual", "½ · hvidet kvadratfejl efter optimering",
      SelectionRow::Style::value);
  s.sections.push_back(about);
  return s;
}

} // namespace

PairStrip::PairStrip(ProjectSession &session, EdgeEditor &editor,
                     QWidget *parent)
    : QFrame(parent), session_(session), editor_(editor) {
  setObjectName("pairStrip");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  const int pad = t.px("--space-3");
  v->setContentsMargins(t.px("--space-4"), pad, t.px("--space-4"), pad);
  v->setSpacing(t.px("--space-2"));

  auto *head = new QHBoxLayout;
  head->setSpacing(t.px("--space-3"));
  // The ids are in Rediger's pair header just above.
  head->addWidget(new CapsLabel("Kant A ↔ B", "panelTitle"));
  delta_ = new QLabel;
  delta_->setObjectName("pairDelta");
  delta_->setWordWrap(true);
  head->addWidget(delta_, 1);
  v->addLayout(head);

  // The edits on a line of their own, so the strip fits a narrow column.
  // ("Nulstil" B is the 3D overlay's, beside the canvas it resets.)
  auto *actions = new QHBoxLayout;
  actions->setSpacing(t.px("--space-3"));
  icp_ = new QPushButton(NavItem::escape_mnemonic("ICP-forfin"));
  icp_->setProperty("kind", "secondary");
  icp_->setCursor(Qt::PointingHandCursor);
  icp_->setToolTip("Justér B's dybdesky mod A's med ICP og vis, hvor godt "
                   "de passer. Ændrer ikke projektet.");
  actions->addWidget(icp_);
  type_ = new QComboBox;
  type_->setObjectName("edgeType");
  for (const char *k : {"loop_closure", "odometry", "panorama"})
    type_->addItem(QString::fromStdString(edge_type_da(k)), QString(k));
  type_->setToolTip("Typen af den nye kant");
  actions->addWidget(type_);
  add_ = new QPushButton(NavItem::escape_mnemonic("Tilføj kant"));
  add_->setProperty("kind", "primary");
  add_->setCursor(Qt::PointingHandCursor);
  add_->setToolTip("Tilføj en kant med den nuværende relative pose (efter din "
                   "placering og ICP, hvis kørt). Bekræftet; gemmes med Gem "
                   "ændringer.");
  actions->addWidget(add_);
  actions->addStretch(1); // left-aligned, under the edge rows' start
  v->addLayout(actions);

  rows_ = new QWidget;
  rows_->setObjectName("edgeRows");
  rows_layout_ = new QVBoxLayout(rows_);
  rows_layout_->setContentsMargins(0, 0, 0, 0);
  rows_layout_->setSpacing(t.px("--space-1"));
  v->addWidget(rows_);

  icp_line_ = new QLabel;
  icp_line_->setObjectName("icpLine");
  icp_line_->setWordWrap(true);
  v->addWidget(icp_line_);
  note_ = new QLabel;
  note_->setObjectName("pairNote");
  note_->setWordWrap(true);
  v->addWidget(note_);

  connect(icp_, &QPushButton::clicked, this, &PairStrip::run_icp);
  connect(add_, &QPushButton::clicked, this, &PairStrip::add_edge);
  connect(type_, &QComboBox::currentIndexChanged, this,
          &PairStrip::refresh_add_label);
  connect(&editor_, &EdgeEditor::changed, this, &PairStrip::rebuild);
  connect(&session_, &ProjectSession::state_changed, this, [this] {
    icp_results_.clear();
    rebuild();
  });
  rebuild();
}

void PairStrip::set_pair(int a_id, int b_id) {
  if (a_id == a_ && b_id == b_)
    return;
  a_ = a_id;
  b_ = b_id;
  rebuild();
}

void PairStrip::rebuild() {
  const Theme &t = theme();
  // Clear the rows (hide first: a widget waiting for deleteLater paints).
  while (QLayoutItem *it = rows_layout_->takeAt(0)) {
    if (QWidget *w = it->widget()) {
      w->hide();
      w->deleteLater();
    }
    delete it;
  }

  const reusex::ProjectDB *db = session_.db();
  const bool pair_ok = db && a_ >= 0 && b_ >= 0;
  const bool same = pair_ok && a_ == b_;

  // How far apart: distance, angle and time between the stored poses.
  QString delta;
  if (pair_ok && !same) {
    try {
      QStringList parts;
      if (db->has_sensor_frame_pose(a_) && db->has_sensor_frame_pose(b_)) {
        const auto d =
            pose_delta(db->sensor_frame_pose(a_), db->sensor_frame_pose(b_));
        parts << QString("%1 m").arg(dec(d.distance_m, 2))
              << QString("%1°").arg(dec(d.angle_deg, 1));
      }
      const double ta = db->sensor_frame_timestamp(a_);
      const double tb = db->sensor_frame_timestamp(b_);
      if (ta >= 0 && tb >= 0)
        parts << QString("Δt %1 s").arg(dec(std::abs(tb - ta), 1));
      delta = parts.join("  ·  ");
    } catch (const std::exception &) {
    }
  }
  delta_->setText(delta);

  const auto views = pair_ok && !same ? editor_.edits().between(a_, b_)
                                      : std::vector<EdgeView>{};
  for (const EdgeView &v : views) {
    auto *row = new EdgeRow(
        [this, v] { emit edge_selected(edge_selection(v, a_, b_)); });
    row->setProperty("state", v.edge.status == "rejected" ? "rejected"
                              : v.pending != EdgeView::Pending::none
                                  ? "add"
                                  : "stored");
    // Two lines, so the row fits Rediger's right column: what the edge is
    // and its action, then its numbers and state.
    auto *rv = new QVBoxLayout(row);
    rv->setContentsMargins(t.px("--space-2"), t.px("--space-1"),
                           t.px("--space-1"), t.px("--space-1"));
    rv->setSpacing(t.px("--space-1"));
    auto *h = new QHBoxLayout;
    h->setSpacing(t.px("--space-3"));
    auto *h2 = new QHBoxLayout;
    h2->setSpacing(t.px("--space-3"));
    rv->addLayout(h);
    rv->addLayout(h2);
    h->addWidget(
        new Pill(QString::fromStdString(edge_type_da(v.edge.key.type)),
                 v.edge.key.type == "loop_closure" ? "accent" : "outline"));
    auto *dir =
        new QLabel(QString("%1 → %2").arg(v.edge.key.from).arg(v.edge.key.to));
    dir->setObjectName("edgeMono");
    h->addWidget(dir);
    h->addStretch(1);
    // One eliding label: a stored edge with a staged change carries two
    // pills, and fixed-width figures beside them widened the window. The
    // figures that judge an edge, the residual first, so a narrow column
    // cuts the source, not the number the colour modes and the list sort by.
    const auto cell = [&v](EdgeColumn c) {
      return QString::fromStdString(edge_cell_text(v, c));
    };
    const QString source =
        QString::fromStdString(edge_source_da(v.edge.source));
    const bool measured = std::isfinite(v.edge.residual) ||
                          v.edge.deviation_m || v.edge.deviation_deg;
    // A staged or never-optimized edge has no figures yet: say so instead
    // of a line of dashes.
    auto *r = new ElidedLabel(
        measured ? QString("Residual %1 · afvigelse %2 m / %3° · kilde %4")
                       .arg(cell(EdgeColumn::residual),
                            cell(EdgeColumn::deviation_m),
                            cell(EdgeColumn::deviation_deg), source)
                 : QString("Ikke optimeret endnu · kilde %1").arg(source),
        Qt::ElideRight);
    r->setObjectName("edgeMeta");
    r->setToolTip(r->full_text() +
                  "\nResidual: ½ · hvidet kvadratfejl efter seneste "
                  "optimering\nAfvigelse: kantens måling mod de optimerede "
                  "poser");
    h2->addWidget(r, 1);
    h2->addWidget(
        new Pill(status_da(v.edge.status), status_tone(v.edge.status)));
    if (v.pending != EdgeView::Pending::none)
      h2->addWidget(new Pill("Ikke gemt", "accent"));
    const EdgeKey key = v.edge.key;
    auto action = [&](const QString &text, auto fn) {
      auto *b = new QPushButton(NavItem::escape_mnemonic(text));
      b->setProperty("kind", "ghost");
      b->setObjectName("edgeAction");
      b->setCursor(Qt::PointingHandCursor);
      connect(b, &QPushButton::clicked, this,
              [this, key, fn] { fn(editor_, key); });
      b->setMinimumHeight(b->sizeHint().height());
      h->addWidget(b);
    };
    const bool stored = v.pending == EdgeView::Pending::none;
    if (stored && v.edge.status == "auto")
      action("Bekræft", [](EdgeEditor &e, const EdgeKey &k) { e.confirm(k); });
    if (stored && v.edge.status == "confirmed")
      action("Til auto",
             [](EdgeEditor &e, const EdgeKey &k) { e.unconfirm(k); });
    if (!stored)
      action("Fortryd", [](EdgeEditor &e, const EdgeKey &k) { e.revert(k); });
    else if (v.edge.status == "rejected")
      action("Fortryd afvisning",
             [](EdgeEditor &e, const EdgeKey &k) { e.unreject(k); });
    else
      action("Afvis", [](EdgeEditor &e, const EdgeKey &k) { e.reject(k); });
    row->setMinimumHeight(rv->sizeHint().height());
    rows_layout_->addWidget(row);
  }
  if (views.empty()) {
    auto *none = new QLabel;
    none->setObjectName("pairNone");
    none->setWordWrap(true);
    if (!pair_ok)
      none->setText("Vælg to billeder.");
    else if (same)
      none->setText("A og B er det samme billede — vælg et andet B "
                    "(Shift+← / Shift+→ eller Shift+klik i filmstrimlen).");
    else if (editor_.edits().base().empty() && editor_.edits().empty())
      none->setText("Posegrafen har ingen kanter endnu. Kør rux optimize, "
                    "eller tilføj en kant manuelt her.");
    else
      none->setText("Ingen kant mellem A og B.");
    rows_layout_->addWidget(none);
  }

  refresh_icp();

  const bool editable = pair_ok && !same;
  icp_->setEnabled(editable && icp_running_.first < 0);
  add_->setEnabled(editable);
  type_->setEnabled(editable);
  refresh_add_label();
  const QString ro = editor_.read_only_reason();
  note_->setText(ro.isEmpty() ? QString()
                              : QString("Skrivebeskyttet: kanter kan "
                                        "tilføjes og afvises her, men ikke "
                                        "gemmes."));
  note_->setVisible(!ro.isEmpty());
}

void PairStrip::refresh_add_label() {
  const bool pair_ok = session_.db() && a_ >= 0 && b_ >= 0 && a_ != b_;
  const bool replaces =
      pair_ok &&
      pair_add_replaces(editor_.edits().between(a_, b_),
                        type_->currentData().toString().toStdString());
  add_->setText(
      NavItem::escape_mnemonic(replaces ? "Erstat måling" : "Tilføj kant"));
}

std::optional<Mat4> PairStrip::current_delta() const {
  return delta_source_ ? delta_source_() : std::optional<Mat4>{};
}

IcpFitState PairStrip::fit_state(const IcpOutcome &o) const {
  return icp_fit_state(o.ok, o.stale, o.converged, o.applied, current_delta());
}

void PairStrip::refresh_icp() {
  // ICP result for this pair, if one was run.
  const auto key = std::make_pair(a_, b_);
  const bool pair_ok = session_.db() && a_ >= 0 && b_ >= 0;
  QString text, tone;
  if (pair_ok && icp_running_ == key) {
    text = "ICP kører …";
    tone = "wait";
  } else if (auto it = icp_results_.find(key); it != icp_results_.end()) {
    const IcpOutcome &o = it->second;
    // The fit's figures; how far B moved is the overlay's to say (one
    // measure, about B's centroid), so it is not repeated here.
    const QString fit =
        QString("RMS %1 cm · %2 % inden for 5 cm")
            .arg(dec(o.fitness * 100.0, 1), dec(o.inliers * 100.0, 0));
    const QString points =
        QString("%1 / %2 punkter")
            .arg(format_count(static_cast<qulonglong>(o.source_points)),
                 format_count(static_cast<qulonglong>(o.target_points)));
    const QString weight =
        QString("ny kant får vægt %1")
            .arg(weight_text(weight_from_icp_fitness(o.fitness)));
    const QString from =
        o.seeded ? QString("Fra din manuelle placering · ") : QString();
    switch (fit_state(o)) {
    case IcpFitState::failed:
      text = "ICP fejlede: " + o.error;
      tone = "crit";
      break;
    case IcpFitState::dropped:
      text = "ICP-resultatet er droppet: B blev flyttet, mens ICP kørte. "
             "Kør ICP-forfin igen.";
      tone = "warn";
      break;
    case IcpFitState::moved_since:
      text = "B er flyttet efter ICP — kør ICP-forfin igen. En ny kant måles "
             "fra B's nuværende placering.";
      tone = "warn";
      break;
    case IcpFitState::not_converged:
      text = QString("ICP konvergerede ikke · %1 — billederne deler for lidt "
                     "geometri. B bliver stående; en ny kant måles fra dens "
                     "placering.")
                 .arg(fit);
      tone = "warn";
      break;
    case IcpFitState::aligned:
      text = from + QString("ICP konvergerede · %1 · B er lagt på ICP's "
                            "placering · %2 · %3")
                        .arg(fit, points, weight);
      tone = "good";
      break;
    case IcpFitState::not_placed:
      text = from + QString("ICP konvergerede · %1 · %2 · %3. B er ikke "
                            "flyttet: 3D-overlayet var ikke klar.")
                        .arg(fit, points, weight);
      tone = "good";
      break;
    }
  }
  icp_line_->setText(text);
  icp_line_->setProperty("tone", tone.isEmpty() ? QVariant() : QVariant(tone));
  icp_line_->setVisible(!text.isEmpty());
  repolish(icp_line_);
}

void PairStrip::add_edge() {
  const reusex::ProjectDB *db = session_.db();
  if (!db || a_ < 0 || b_ < 0 || a_ == b_)
    return;
  std::optional<Mat4> pose_a, pose_b;
  try {
    if (db->has_sensor_frame_pose(a_) && db->has_sensor_frame_pose(b_)) {
      pose_a = db->sensor_frame_pose(a_);
      pose_b = db->sensor_frame_pose(b_);
    }
  } catch (const std::exception &e) {
    // A read error is not a missing pose: say what the project said.
    note_->setText(QString("Poserne kunne ikke læses: %1")
                       .arg(QString::fromUtf8(e.what())));
    note_->setVisible(true);
    return;
  }
  if (!pose_a || !pose_b) {
    note_->setText("Begge billeder skal have en pose, før en kant kan måles.");
    note_->setVisible(true);
    return;
  }
  // ICP's measurement while its fit still describes B's placement; else
  // B's placement itself (the stored relative pose when B is unmoved).
  std::optional<IcpEdgeMeasurement> icp;
  if (auto it = icp_results_.find({a_, b_});
      it != icp_results_.end() && icp_fit_usable(fit_state(it->second)))
    icp = IcpEdgeMeasurement{it->second.edge_transform, it->second.information};
  const EdgeRecord e =
      pair_edge_to_add(a_, b_, type_->currentData().toString().toStdString(),
                       *pose_a, *pose_b, current_delta(), icp);
  QString msg;
  switch (editor_.add(e)) {
  case PendingEdgeEdits::Result::added:
    break;
  case PendingEdgeEdits::Result::replaced: {
    // As the list and the rows show it: the store's direction.
    const EdgeKey k = canonical_key(e.key);
    msg = QString("Kanten %1 → %2 får den nye måling ved gem.")
              .arg(k.from)
              .arg(k.to);
    break;
  }
  case PendingEdgeEdits::Result::revived:
    msg = "Den afviste kant genoplives, og den nye måling erstatter den "
          "gemte ved gem.";
    break;
  default:
    msg = "Kanten kan ikke tilføjes (samme billede eller ukendt type).";
  }
  // add() emitted changed(): rebuild() has reset the note already.
  if (!msg.isEmpty()) {
    note_->setText(msg);
    note_->setVisible(true);
  }
}

void PairStrip::run_icp() {
  if (a_ < 0 || b_ < 0 || a_ == b_ || icp_running_.first >= 0 ||
      !session_.is_open())
    return;
  const int a = a_, b = b_;
  // Seeded with B's manual offset, if any; asked again when ICP finishes.
  std::optional<Mat4> guess;
  if (delta_source_)
    guess = delta_source_();
  icp_running_ = {a, b};
  rebuild();
  // A detached thread with its own read-only connection: ICP over two
  // depth clouds takes a second or two, and the GUI's connection is the
  // GUI thread's alone. Counted, so quitting meanwhile waits for it.
  const std::string path = session_.path().toStdString();
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PairStrip> guard(this);
  std::thread([path, a, b, guess, guard, work]() mutable {
    IcpOutcome o;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      reusex::slam::FramePairIcpOptions opts;
      opts.initial_guess = guess;
      // B onto A: "from" B, "to" A, as an edge A -> B constrains B.
      const auto r = reusex::slam::refine_frame_pair_icp(db, b, a, opts);
      o.ok = true;
      o.world_delta = r.world_delta;
      o.seeded = guess.has_value();
      o.information = reusex::slam::icp_edge_information(r);
      o.fitness = r.fitness;
      o.inliers = r.inlier_fraction;
      o.converged = r.converged;
      o.source_points = r.source_points;
      o.target_points = r.target_points;
      o.edge_transform = r.edge_transform;
    } catch (const std::exception &e) {
      o.error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, a, b, guess, o] {
          if (guard)
            guard->icp_finished(a, b, guess, o);
        },
        Qt::QueuedConnection);
    work.reset(); // last: counted until the thread has let go of everything
  }).detach();
}

void PairStrip::icp_finished(int a, int b, const std::optional<Mat4> &guess,
                             IcpOutcome outcome) {
  icp_running_ = {-1, -1};
  // Another pair meanwhile: the overlay started that pair unmoved, so the
  // result describes nothing on screen. Dropped.
  if (a != a_ || b != b_) {
    rebuild();
    return;
  }
  // B moved, reset or nudged while ICP ran: say so, never move B.
  outcome.stale = outcome.ok && !icp_seed_unchanged(guess, current_delta());
  const bool place = outcome.ok && outcome.converged && !outcome.stale;
  const Mat4 world_delta = outcome.world_delta;
  // Where B will sit — on the result, or still at the seed — is what the
  // fit describes; a later move outdates it. Set BEFORE the emit: placing B
  // re-enters refresh_icp() (offset_changed), which must already see the
  // fit as applied.
  outcome.applied = place ? std::optional<Mat4>(world_delta) : guess;
  icp_results_[{a, b}] = std::move(outcome);
  if (place) {
    emit icp_aligned(a, b, world_delta); // the overlay places B (normalised)
    // The overlay normalises the offset (angles wrapped, translation
    // clamped): the fit is applied at the ΔT it actually holds, if any.
    // Re-found: the emit may re-enter.
    if (auto it = icp_results_.find({a, b}); it != icp_results_.end())
      it->second.applied = current_delta();
  }
  rebuild();
}

} // namespace rux::qt
