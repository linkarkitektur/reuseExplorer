// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/MatchCanvas.hpp>
#include <rux_qt/PairImages.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/frame_pair_matches.hpp>

#include <opencv2/core/mat.hpp>

#include <QApplication>
#include <QButtonGroup>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QPointer>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

#include <optional>
#include <thread>

namespace rux::qt {
namespace slam = reusex::slam;

namespace {
/// Rediger's fit threshold: rigid_from_matches' default, named for the text.
constexpr double kInlierDistM = 0.05;

MatchPoint to_point(const slam::FrameKeypoint &k) {
  MatchPoint p;
  p.u = k.u;
  p.v = k.v;
  p.xyz = k.xyz; // Vec3 is the library's std::array<double, 3>
  return p;
}

std::optional<slam::PixelRect> to_rect(const std::optional<PixelBox> &b) {
  if (!b)
    return std::nullopt;
  return slam::PixelRect{b->x, b->y, b->w, b->h};
}

/// What would make the library refuse frame @p id, checked up front so the
/// status names it in Danish (R13). A node without a sensor frame (a
/// panorama) has no depth either.
struct Needs {
  bool colour = false, depth = false, pose = false;
};
std::optional<FrameProblem> frame_problem(const reusex::ProjectDB &db, int id,
                                          Needs need) {
  if (need.depth &&
      (!db.has_sensor_frame(id) || db.sensor_frame_depth(id).empty()))
    return FrameProblem::no_depth;
  if (need.pose && !db.has_sensor_frame_pose(id))
    return FrameProblem::no_pose;
  if (need.colour && db.sensor_frame_image(id).empty())
    return FrameProblem::no_colour;
  return std::nullopt;
}

QString problem_text(int id, FrameProblem p) {
  return QString::fromStdString(frame_problem_text(id, p));
}

QPushButton *segment(const QString &text, int position, int count) {
  auto *b = new QPushButton(NavItem::escape_mnemonic(text));
  b->setObjectName("segment");
  b->setCheckable(true);
  b->setCursor(Qt::PointingHandCursor);
  b->setProperty("position", position == 0           ? "first"
                             : position == count - 1 ? "last"
                                                     : "middle");
  return b;
}

/// Run @p work on a detached thread with its own read-only connection and
/// hand its result to @p done on the GUI thread, unless @p self is gone
/// meanwhile. Counted (BackgroundWork), so quitting waits for it. Nothing
/// escapes the thread: any throw becomes the result's detail.
template <class Result, class Work, class Done>
void run_detached(PairImages *self, const std::string &path, Work work,
                  Done done) {
  auto token = std::make_shared<BackgroundWork>();
  QPointer<PairImages> guard(self);
  std::thread([path, work, done, guard, token]() mutable {
    auto r = std::make_shared<Result>();
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      work(db, *r);
    } catch (const std::exception &e) {
      r->detail = QString::fromUtf8(e.what());
    } catch (...) {
      // Anything else escaping a detached thread would std::terminate the
      // app: report it like a failure, with no message of its own to show.
      r->detail = QStringLiteral("Ukendt fejl (ingen fejlbesked)");
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, done, r] {
          if (guard)
            done(guard.data(), r);
        },
        Qt::QueuedConnection);
    token.reset(); // last: counted until the thread has let go of everything
  }).detach();
}
} // namespace

// Each result: the generation it was started under, a Danish problem
// (frame_problem) or the library's English what() (detail), and its data.
struct PairImages::Found {
  unsigned generation = 0;
  QString problem, detail;
  std::vector<MatchPoint> a, b;
  std::vector<MatchLine> lines;
};
struct PairImages::Lifted {
  unsigned generation = 0;
  QString problem, detail;
  MatchSide side = MatchSide::a;
  std::optional<MatchPoint> point;
};
struct PairImages::Fitted {
  unsigned generation = 0;
  QString problem, detail;
  FitInput input;
  slam::RigidFit fit;
};

PairImages::PairImages(ProjectSession &session, QWidget *parent)
    : QWidget(parent), session_(session) {
  setObjectName("pairImages");
  const Theme &t = theme();
  loader_ = new FrameImageLoader(this);
  auto *g = new QGridLayout(this);
  g->setContentsMargins(t.px("--space-2"), t.px("--space-1"), t.px("--space-2"),
                        t.px("--space-1"));
  g->setHorizontalSpacing(t.px("--space-2"));
  g->setVerticalSpacing(t.px("--space-1"));

  // Farve / Dybde floats in the canvas' bottom-right corner, like the 3D
  // overlay's Tint | RGB; the mode is the pane header's (mode_switch()).
  // Only the toolbar takes a row of its own: at 1440x900 the canvas must
  // stay >= 200 px tall beside a >= 250 px overlay.
  auto *seg = new QFrame;
  seg->setObjectName("segmented");
  auto *sl = new QHBoxLayout(seg);
  sl->setContentsMargins(0, 0, 0, 0);
  sl->setSpacing(0);
  layers_ = new QButtonGroup(this);
  const char *layer_names[] = {"Farve", "Dybde"};
  const char *layer_tips[] = {"Vis farvebillederne",
                              "Vis dybdebillederne (match vises stadig)"};
  for (int i = 0; i < 2; ++i) {
    auto *b = segment(layer_names[i], i, 2);
    b->setToolTip(layer_tips[i]);
    layers_->addButton(b, i);
    sl->addWidget(b);
  }
  layers_->button(0)->setChecked(true);
  layer_switch_ = seg;

  auto *modes = new QFrame;
  modes->setObjectName("segmented");
  auto *ml = new QHBoxLayout(modes);
  ml->setContentsMargins(0, 0, 0, 0);
  ml->setSpacing(0);
  modes_ = new QButtonGroup(this);
  const char *mode_names[] = {"Vælg", "Tilføj", "Område"};
  const char *mode_tips[] = {
      "Klik på en linje for at udelade eller tage den med; træk for at "
      "panorere, hjul zoomer, dobbeltklik nulstiller",
      "Klik et punkt i A og så det samme i B (Esc fortryder)",
      "Træk et rektangel i A eller B: næste Find match søger kun der"};
  for (int i = 0; i < 3; ++i) {
    auto *b = segment(mode_names[i], i, 3);
    b->setToolTip(mode_tips[i]);
    modes_->addButton(b, i);
    ml->addWidget(b);
  }
  modes_->button(0)->setChecked(true);
  mode_switch_ = modes;

  // One row: Find match, Placér B fra match, Ryd område (with a region).
  auto *row = new QHBoxLayout;
  row->setSpacing(t.px("--space-2"));
  find_ = new QPushButton(NavItem::escape_mnemonic("Find match"));
  find_->setProperty("kind", "secondary");
  find_->setCursor(Qt::PointingHandCursor);
  find_->setToolTip("Find ORB-match mellem A og B (inden for områderne, "
                    "hvis du har tegnet nogen). Ændrer ikke projektet.");
  row->addWidget(find_);
  align_ = new QPushButton(NavItem::escape_mnemonic("Placér B fra match"));
  align_->setProperty("kind", "primary");
  align_->setCursor(Qt::PointingHandCursor);
  align_->setToolTip(
      "Flyt B i 3D-overlayet til den stive placering, de "
      "valgte match med dybde giver. ICP-forfin starter derfra.");
  row->addWidget(align_);
  clear_region_ = new QPushButton(NavItem::escape_mnemonic("Ryd område"));
  clear_region_->setObjectName("regionChip");
  clear_region_->setProperty("kind", "ghost");
  clear_region_->setCursor(Qt::PointingHandCursor);
  // Centred, not stretched: a Fixed-height chip with no alignment caps the
  // whole row at its own (smaller) height, which cut Find match and Placér
  // B fra match in half once the chip showed.
  row->addWidget(clear_region_, 0, Qt::AlignVCenter);
  row->addStretch(1);

  // No QSS rule of its own (R9): the name is for tests and the gallery.
  auto *bar = new QFrame;
  bar->setObjectName("matchToolbar");
  bar->setLayout(row);
  row->setContentsMargins(0, 0, 0, 0);
  g->addWidget(bar, 0, 0);

  canvas_ = new MatchCanvas;
  canvas_->set_data(&set_, &pick_);
  canvas_->set_corner_widget(layer_switch_);
  g->addWidget(canvas_, 1, 0);
  g->setRowStretch(1, 1);

  // The status floats in the canvas' bottom-left corner, beside Farve /
  // Dybde, like the 3D overlay's legend: a row of its own would cost the
  // canvas the height it needs.
  status_ = new ElidedLabel(QString(), Qt::ElideRight);
  status_->setObjectName("matchStatus");
  canvas_->set_status_widget(status_);

  // Esc reaches a half-made match even when focus has left the canvas (a
  // mode button); EditWorkspace widens it to the whole workspace.
  set_shortcut_scope(this);

  connect(layers_, &QButtonGroup::idClicked, this, [this](int id) {
    set_layer(id == 1 ? FrameLayer::depth : FrameLayer::color);
  });
  connect(modes_, &QButtonGroup::idClicked, this,
          [this](int id) { set_mode(static_cast<MatchMode>(id)); });
  connect(find_, &QPushButton::clicked, this, &PairImages::find_matches);
  connect(align_, &QPushButton::clicked, this, &PairImages::align_from_matches);
  connect(clear_region_, &QPushButton::clicked, this,
          &PairImages::clear_regions);
  connect(canvas_, &MatchCanvas::line_clicked, this, &PairImages::toggle_line);
  connect(canvas_, &MatchCanvas::pixel_clicked, this, &PairImages::pick);
  connect(canvas_, &MatchCanvas::region_dragged, this, &PairImages::set_region);
  connect(canvas_, &MatchCanvas::cancel_requested, this,
          &PairImages::cancel_pick);
  connect(loader_, &FrameImageLoader::frame_ready, this,
          &PairImages::frame_ready);
  connect(&session_, &ProjectSession::state_changed, this, &PairImages::reload);
  reload();
}

bool PairImages::pair_ok() const {
  return session_.is_open() && a_ >= 0 && b_ >= 0 && a_ != b_;
}

void PairImages::restart() {
  ++generation_; // anything still running describes the old state
  busy_ = Busy::none;
  pick_.cancel();
  placed_at_.reset();
  note_.clear();
  tone_.clear();
  detail_.clear();
  canvas_->reset_views();
  canvas_->set_data(&set_, &pick_); // the lines changed: no stale hover
}

void PairImages::reload() {
  loader_->set_project(session_.is_open() ? session_.path() : QString());
  // The same pair after a reload is still a fresh start: the project (and
  // so every keypoint's depth and pose) may have changed under it.
  set_.clear();
  set_.set_pair(-1, -1);
  set_.set_pair(a_, b_);
  restart();
  sync_images();
  refresh();
}

void PairImages::set_pair(int a, int b, Slot next) {
  const bool new_pair = a != a_ || b != b_;
  a_ = a;
  b_ = b;
  next_ = next;
  if (new_pair) {
    set_.set_pair(a, b); // a swap is a new pair too: clears
    restart();
  }
  sync_images();
  refresh();
}

void PairImages::set_layer(FrameLayer layer) {
  layer_ = layer;
  if (QAbstractButton *b = layers_->button(layer == FrameLayer::depth ? 1 : 0))
    b->setChecked(true);
  canvas_->set_layer(layer);
}

void PairImages::set_mode(MatchMode mode) {
  mode_ = mode;
  if (QAbstractButton *b = modes_->button(static_cast<int>(mode)))
    b->setChecked(true);
  if (mode != MatchMode::add && pick_.pending()) {
    pick_.cancel();
    note_.clear();
  }
  canvas_->set_mode(mode);
  refresh();
}

void PairImages::sync_images() {
  auto side = [this](MatchSide s, Slot slot, int id) {
    canvas_->set_caption(s,
                         QString::fromStdString(pair_image_caption(slot, id)));
    if (id < 0) {
      canvas_->set_message(
          s, QString::fromStdString(pair_image_hint(slot, a_, b_, next_)));
      return;
    }
    if (auto f = loader_->frame(id)) {
      canvas_->set_frame(s, f);
      return;
    }
    // Never the previous frame under the new caption: blank, "Henter …",
    // until this one is decoded (frame_ready() drops any other id).
    canvas_->set_message(s, {});
    canvas_->set_loading(s, true);
    loader_->request(id);
  };
  side(MatchSide::a, Slot::a, a_);
  side(MatchSide::b, Slot::b, b_);
}

void PairImages::frame_ready(int id) {
  // A decode for a frame no longer A or B (the pair moved on) is ignored.
  if (id != a_ && id != b_)
    return;
  auto f = loader_->frame(id);
  if (!f)
    return;
  if (id == a_)
    canvas_->set_frame(MatchSide::a, f);
  if (id == b_)
    canvas_->set_frame(MatchSide::b, f);
}

void PairImages::find_matches() {
  if (!pair_ok() || busy_ != Busy::none)
    return;
  slam::FrameMatchOptions o;
  o.region_a = to_rect(set_.region(MatchSide::a));
  o.region_b = to_rect(set_.region(MatchSide::b));
  busy_ = Busy::finding;
  note("Finder match …", "wait");
  const int a = a_, b = b_;
  const unsigned gen = generation_;
  run_detached<Found>(
      this, session_.path().toStdString(),
      [a, b, o](const reusex::ProjectDB &db, Found &r) {
        for (int id : {a, b})
          if (const auto p = frame_problem(db, id, {true, true, true})) {
            r.problem = problem_text(id, *p);
            return;
          }
        const auto m = slam::match_frame_pair(db, a, b, o);
        for (const auto &k : m.keypoints_a)
          r.a.push_back(to_point(k));
        for (const auto &k : m.keypoints_b)
          r.b.push_back(to_point(k));
        for (const auto &mm : m.matches) {
          MatchLine l;
          l.a = mm.a;
          l.b = mm.b;
          l.distance = mm.distance;
          r.lines.push_back(l);
        }
      },
      [gen](PairImages *self, std::shared_ptr<Found> r) {
        r->generation = gen; // set here, so also when the work threw
        self->found(std::move(r));
      });
}

void PairImages::found(std::shared_ptr<Found> r) {
  if (r->generation != generation_)
    return; // another pair (or a reload) meanwhile: dropped
  busy_ = Busy::none;
  if (!r->problem.isEmpty()) {
    note(r->problem, "crit");
    return;
  }
  if (!r->detail.isEmpty()) {
    note("Find match fejlede", "crit", r->detail);
    return;
  }
  set_.set_detected(std::move(r->a), std::move(r->b), std::move(r->lines));
  canvas_->set_data(&set_, &pick_); // new lines: no stale hover index
  note({}, {});
}

void PairImages::pick(MatchSide side, float u, float v) {
  if (!pair_ok() || refuse_busy())
    return;
  busy_ = Busy::lifting;
  refresh();
  const int id = side == MatchSide::a ? a_ : b_;
  const unsigned gen = generation_;
  run_detached<Lifted>(
      this, session_.path().toStdString(),
      [id, side, u, v](const reusex::ProjectDB &db, Lifted &r) {
        r.side = side;
        if (const auto p = frame_problem(db, id, {true, true, false})) {
          r.problem = problem_text(id, *p);
          return;
        }
        if (const auto k = slam::lift_pixel(db, id, u, v))
          r.point = to_point(*k);
      },
      [gen](PairImages *self, std::shared_ptr<Lifted> r) {
        r->generation = gen;
        self->lifted(std::move(r));
      });
}

void PairImages::lifted(std::shared_ptr<Lifted> r) {
  if (r->generation != generation_)
    return;
  busy_ = Busy::none;
  if (!r->problem.isEmpty()) {
    note(r->problem, "crit");
    return;
  }
  if (!r->detail.isEmpty()) {
    note("Punktet kunne ikke læses", "crit", r->detail);
    return;
  }
  if (mode_ != MatchMode::add) { // the mode changed while it lifted
    refresh();
    return;
  }
  if (!r->point) {
    note("ingen dybde her — klik tættere på en flade med dybde", "warn");
    return;
  }
  if (const auto done = pick_.pick(r->side, *r->point)) {
    set_.add_manual(done->first, done->second);
    canvas_->set_data(&set_, &pick_);
    note({}, {});
    return;
  }
  note(r->side == MatchSide::a ? "Klik nu det samme punkt i B (Esc fortryder)"
                               : "Klik nu det samme punkt i A (Esc fortryder)",
       {});
}

void PairImages::set_shortcut_scope(QWidget *scope) {
  // Only while a pick waits, and only with focus inside @p scope: never Esc
  // from the command palette or a dialog elsewhere in the window.
  delete escape_;
  escape_ = new QShortcut(QKeySequence(Qt::Key_Escape), scope);
  escape_->setContext(Qt::WidgetWithChildrenShortcut);
  escape_->setEnabled(pick_.pending());
  connect(escape_, &QShortcut::activated, this, &PairImages::cancel_pick);
}

bool PairImages::refuse_busy() {
  if (busy_ == Busy::none)
    return false;
  note(busy_ == Busy::finding   ? "Vent — finder match …"
       : busy_ == Busy::lifting ? "Vent — læser punktet …"
                                : "Vent — placerer B …",
       "wait");
  return true;
}

void PairImages::cancel_pick() {
  if (!pick_.pending())
    return;
  pick_.cancel();
  note({}, {});
}

void PairImages::toggle_line(int line) {
  // A fit running reads the lines it was started with: a toggle now would
  // be coloured by a fit that never saw it. A Find running replaces the
  // lines when it lands: a toggle now would be lost with the old ones.
  if (busy_ == Busy::aligning || busy_ == Busy::finding) {
    refuse_busy();
    return;
  }
  set_.toggle_excluded(line);
  note({}, {}); // refresh() repaints the canvas: the new colour shows now
}

void PairImages::set_region(MatchSide side, PixelBox box) {
  set_.set_region(side, box);
  note(side == MatchSide::a ? "Område i A — Find match søger kun der"
                            : "Område i B — Find match søger kun der",
       {});
}

void PairImages::clear_regions() {
  set_.clear_regions();
  note({}, {});
}

void PairImages::align_from_matches() {
  if (!pair_ok() || busy_ != Busy::none)
    return;
  const FitInput in = set_.fit_input();
  if (in.pairs.size() < 3) {
    note(QString::fromStdString(match_fit_failure_text(
             MatchFitFailure::too_few_pairs, static_cast<int>(in.pairs.size()),
             0, kInlierDistM)),
         "warn");
    return;
  }
  busy_ = Busy::aligning;
  note("Placerer B …", "wait");
  const int a = a_, b = b_;
  const unsigned gen = generation_;
  run_detached<Fitted>(
      this, session_.path().toStdString(),
      [a, b, in](const reusex::ProjectDB &db, Fitted &r) {
        r.input = in;
        for (int id : {a, b})
          if (const auto p = frame_problem(db, id, {false, false, true})) {
            r.problem = problem_text(id, *p);
            return;
          }
        // FitInput's pairs are (A xyz, B xyz) as std::array<double, 3>:
        // exactly slam::PointPair.
        r.fit = slam::rigid_from_matches(db.sensor_frame_pose(a),
                                         db.sensor_frame_pose(b), in.pairs,
                                         kInlierDistM);
      },
      [gen](PairImages *self, std::shared_ptr<Fitted> r) {
        r->generation = gen;
        self->fitted(std::move(r));
      });
}

void PairImages::fitted(std::shared_ptr<Fitted> r) {
  if (r->generation != generation_)
    return; // never moves B of a pair shown since
  busy_ = Busy::none;
  if (!r->problem.isEmpty()) {
    note(r->problem, "crit");
    return;
  }
  if (!r->detail.isEmpty()) {
    note("Placér B fejlede", "crit", r->detail);
    return;
  }
  const slam::RigidFit &f = r->fit;
  if (!f.ok) {
    const MatchFitFailure why =
        f.failure == slam::RigidFitFailure::too_few_pairs
            ? MatchFitFailure::too_few_pairs
        : f.failure == slam::RigidFitFailure::degenerate
            ? MatchFitFailure::degenerate
            : MatchFitFailure::no_consensus;
    note(QString::fromStdString(match_fit_failure_text(
             why, static_cast<int>(r->input.pairs.size()),
             static_cast<int>(f.inliers.size()), kInlierDistM)),
         "warn", QString::fromStdString(f.reason));
    return;
  }
  set_.apply_fit(r->input, f.inliers);
  last_inliers_ = static_cast<int>(f.inliers.size());
  last_rms_ = f.rms_m;
  refresh(); // the lines' new colours, before B moves
  emit align_ready(a_, b_, f.world_delta); // EditWorkspace: align_applied
}

void PairImages::align_applied(const std::optional<Mat4> &placed_at) {
  const QString fit =
      QString::fromStdString(match_fit_text(last_inliers_, last_rms_));
  placed_at_ = placed_at;
  note(placed_at ? fit
                 : fit + " — B er ikke flyttet: 3D-overlayet var ikke klart.",
       placed_at ? "good" : "warn");
}

void PairImages::b_moved(const std::optional<Mat4> &now) {
  if (!placed_at_ || icp_seed_unchanged(placed_at_, now))
    return;
  placed_at_.reset();
  // Neutral, not "good": the fit no longer describes where B is. The news
  // first: the status elides at its right end.
  note("B er flyttet siden · " +
           QString::fromStdString(match_fit_text(last_inliers_, last_rms_)),
       {});
}

void PairImages::note(const QString &text, const QString &tone,
                      const QString &detail) {
  note_ = text;
  tone_ = tone;
  detail_ = detail;
  refresh();
}

QString PairImages::status_text() const { return status_->full_text(); }

void PairImages::refresh() {
  const QString readout = QString::fromStdString(match_readout(set_.counts()));
  status_->set_full_text(note_.isEmpty() ? readout : note_);
  QString tip = note_.isEmpty() ? readout : note_ + "\n" + readout;
  if (!detail_.isEmpty())
    tip += "\n" + detail_;
  status_->setToolTip(tip);
  status_->setProperty("tone", tone_.isEmpty() ? QVariant() : QVariant(tone_));
  repolish(status_);
  const bool ok = pair_ok();
  const bool idle = busy_ == Busy::none;
  find_->setEnabled(ok && idle);
  align_->setEnabled(ok && idle && !set_.lines().empty());
  for (QAbstractButton *b : modes_->buttons())
    b->setEnabled(ok);
  clear_region_->setVisible(set_.has_region());
  escape_->setEnabled(pick_.pending());
  canvas_->update();
}

} // namespace rux::qt
