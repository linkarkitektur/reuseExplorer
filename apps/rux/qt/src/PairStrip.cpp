// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The A<->B pair strip of Rediger: the pose-graph edges between the
// two frames (stored and pending), how far apart the frames are, and the
// three edits — ICP refine (slam::refine_frame_pair_icp, on a detached thread
// with its own read-only connection), add an edge, delete one.

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/PairStrip.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
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
    row->setProperty("state", v.edge.status == "rejected" ? "delete"
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
    // residual first, so a narrow column cuts the weight, not the number
    // the colour modes and the list sort by.
    auto *r = new ElidedLabel(QString("Residual %1   Vægt %2")
                                  .arg(residual_text(v.edge.residual))
                                  .arg(weight_text(v.edge.weight)),
                              Qt::ElideRight);
    r->setObjectName("edgeMeta");
    r->setToolTip(r->full_text() +
                  "\nResidual: ½ · hvidet kvadratfejl efter seneste "
                  "optimering");
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
    if (v.pending != EdgeView::Pending::none)
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

  // ICP result for this pair, if one was run.
  const auto key = std::make_pair(a_, b_);
  if (icp_running_ == key) {
    icp_line_->setText("ICP kører …");
    icp_line_->setProperty("tone", "wait");
  } else if (auto it = icp_results_.find(key); it != icp_results_.end()) {
    const IcpOutcome &o = it->second;
    if (!o.ok) {
      icp_line_->setText("ICP fejlede: " + o.error);
      icp_line_->setProperty("tone", "crit");
    } else {
      // B's camera centre moves by the shift; the angle is the rotation.
      icp_line_->setText(
          o.converged
              ? QString("ICP konvergerede · RMS %1 cm · %2 % inden for 5 cm "
                        "· B flyttes %3 cm og drejes %4° · %5 / %6 punkter · "
                        "ny kant får vægt %7")
                    .arg(dec(o.fitness * 100.0, 1), dec(o.inliers * 100.0, 0),
                         dec(o.center_shift_m * 100.0, 1),
                         dec(o.rotation_deg, 2),
                         format_count(static_cast<qulonglong>(o.source_points)),
                         format_count(static_cast<qulonglong>(o.target_points)),
                         weight_text(weight_from_icp_fitness(o.fitness)))
              : QString("ICP konvergerede ikke · RMS %1 cm · %2 % inden for "
                        "5 cm — billederne deler for lidt geometri. En ny kant "
                        "får vægt 1.")
                    .arg(dec(o.fitness * 100.0, 1), dec(o.inliers * 100.0, 0)));
      icp_line_->setProperty("tone", o.converged ? "good" : "warn");
    }
  } else {
    icp_line_->clear();
    icp_line_->setProperty("tone", QVariant());
  }
  icp_line_->setVisible(!icp_line_->text().isEmpty());
  repolish(icp_line_);

  const bool editable = pair_ok && !same;
  icp_->setEnabled(editable && icp_running_.first < 0);
  add_->setEnabled(editable);
  type_->setEnabled(editable);
  const QString ro = editor_.read_only_reason();
  note_->setText(ro.isEmpty() ? QString()
                              : QString("Skrivebeskyttet: kanter kan "
                                        "tilføjes og afvises her, men ikke "
                                        "gemmes."));
  note_->setVisible(!ro.isEmpty());
}

void PairStrip::add_edge() {
  if (a_ < 0 || b_ < 0 || a_ == b_)
    return;
  EdgeRecord e;
  e.key = {a_, b_, type_->currentData().toString().toStdString()};
  e.weight = 1.0;
  // An ICP fit of this pair makes the weight honest: 1/σ² from its RMS.
  if (auto it = icp_results_.find({a_, b_});
      it != icp_results_.end() && it->second.ok && it->second.converged)
    e.weight = weight_from_icp_fitness(it->second.fitness);
  const auto r = editor_.add(e);
  if (r == PendingEdgeEdits::Result::replaced) {
    note_->setText("Kanten findes — den nye måling erstatter den ved gem.");
    note_->setVisible(true);
  } else if (r == PendingEdgeEdits::Result::confirmed ||
             r == PendingEdgeEdits::Result::revived) {
    note_->setText("Kanten findes — den bekræftes ved gem, og dens måling "
                   "beholdes.");
    note_->setVisible(true);
  } else if (r == PendingEdgeEdits::Result::unchanged) {
    note_->setText("Kanten findes og er allerede bekræftet.");
    note_->setVisible(true);
  }
}

void PairStrip::run_icp() {
  if (a_ < 0 || b_ < 0 || a_ == b_ || icp_running_.first >= 0 ||
      !session_.is_open())
    return;
  const int a = a_, b = b_;
  icp_running_ = {a, b};
  rebuild();
  // A detached thread with its own read-only connection: ICP over two
  // depth clouds takes a second or two, and the GUI's connection is the
  // GUI thread's alone. Counted, so quitting meanwhile waits for it.
  const std::string path = session_.path().toStdString();
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PairStrip> guard(this);
  std::thread([path, a, b, guard, work]() mutable {
    IcpOutcome o;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      // B onto A: "from" B, "to" A, as an edge A -> B constrains B.
      const auto r = reusex::slam::refine_frame_pair_icp(db, b, a);
      o.ok = true;
      o.fitness = r.fitness;
      o.inliers = r.inlier_fraction;
      o.converged = r.converged;
      o.source_points = r.source_points;
      o.target_points = r.target_points;
      o.center_shift_m = r.source_center_shift_m;
      o.rotation_deg = r.rotation_deg;
    } catch (const std::exception &e) {
      o.error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, a, b, o] {
          if (guard)
            guard->icp_finished(a, b, o);
        },
        Qt::QueuedConnection);
    work.reset(); // last: counted until the thread has let go of everything
  }).detach();
}

void PairStrip::icp_finished(int a, int b, IcpOutcome outcome) {
  icp_results_[{a, b}] = std::move(outcome);
  icp_running_ = {-1, -1};
  rebuild();
}

} // namespace rux::qt
