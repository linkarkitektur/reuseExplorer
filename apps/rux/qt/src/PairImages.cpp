// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/PairImages.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/graph_view.hpp>
#include <rux_qt/widgets.hpp>

#include <QButtonGroup>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

namespace rux::qt {

PairImages::PairImages(ProjectSession &session, QWidget *parent)
    : QWidget(parent), session_(session) {
  setObjectName("pairImages");
  const Theme &t = theme();
  loader_ = new FrameImageLoader(this);
  auto *g = new QGridLayout(this);
  g->setContentsMargins(t.px("--space-2"), t.px("--space-2"), t.px("--space-2"),
                        t.px("--space-2"));
  g->setHorizontalSpacing(t.px("--space-2"));
  g->setVerticalSpacing(t.px("--space-1"));

  auto *seg = new QFrame;
  seg->setObjectName("segmented");
  auto *sl = new QHBoxLayout(seg);
  sl->setContentsMargins(0, 0, 0, 0);
  sl->setSpacing(0);
  layers_ = new QButtonGroup(this);
  const char *names[] = {"Farve", "Dybde"};
  for (int i = 0; i < 2; ++i) {
    auto *b = new QPushButton(NavItem::escape_mnemonic(names[i]));
    b->setObjectName("segment");
    b->setCheckable(true);
    b->setCursor(Qt::PointingHandCursor);
    b->setProperty("position", i == 0 ? "first" : "last");
    layers_->addButton(b, i);
    sl->addWidget(b);
  }
  layers_->button(0)->setChecked(true);
  layer_switch_ = seg; // placed in the pane header by EditWorkspace

  caption_a_ = new QLabel;
  caption_a_->setObjectName("pairCaptionA");
  caption_b_ = new QLabel;
  caption_b_->setObjectName("pairCaptionB");
  g->addWidget(caption_a_, 0, 0);
  g->addWidget(caption_b_, 0, 1);
  pane_a_ = new ImagePane;
  pane_b_ = new ImagePane;
  for (ImagePane *p : {pane_a_, pane_b_}) {
    p->set_overlay(false, 0);
    // Small is fine here: the pane yields height to the 3D overlay, and
    // Oversigt has the full-size frames.
    p->setMinimumSize(t.px("--space-7"), t.px("--space-7"));
  }
  g->addWidget(pane_a_, 1, 0);
  g->addWidget(pane_b_, 1, 1);
  g->setRowStretch(1, 1);
  // A and B always get the same width, whatever their captions say.
  g->setColumnStretch(0, 1);
  g->setColumnStretch(1, 1);

  connect(layers_, &QButtonGroup::idClicked, this, [this](int id) {
    set_layer(id == 1 ? FrameLayer::depth : FrameLayer::color);
  });
  connect(loader_, &FrameImageLoader::frame_ready, this,
          &PairImages::frame_ready);
  connect(&session_, &ProjectSession::state_changed, this, &PairImages::reload);
  reload();
}

void PairImages::reload() {
  loader_->set_project(session_.is_open() ? session_.path() : QString());
  sync();
}

void PairImages::set_pair(int a, int b, Slot next) {
  if (a == a_ && b == b_ && next == next_)
    return;
  a_ = a;
  b_ = b;
  next_ = next;
  sync();
}

void PairImages::set_layer(FrameLayer layer) {
  layer_ = layer;
  if (QAbstractButton *b = layers_->button(layer == FrameLayer::depth ? 1 : 0))
    b->setChecked(true);
  pane_a_->set_layer(layer);
  pane_b_->set_layer(layer);
}

void PairImages::sync() {
  auto side = [this](ImagePane *pane, QLabel *caption, Slot slot, int id) {
    caption->setText(QString::fromStdString(pair_image_caption(slot, id)));
    if (id < 0) {
      pane->set_message(
          QString::fromStdString(pair_image_hint(slot, a_, b_, next_)));
      return;
    }
    if (auto f = loader_->frame(id)) {
      pane->set_frame(f);
      return;
    }
    // Never the previous frame under the new caption: blank, "Henter …",
    // until this one is decoded (frame_ready() drops any other id).
    pane->set_message({});
    pane->set_loading(true);
    loader_->request(id);
  };
  side(pane_a_, caption_a_, Slot::a, a_);
  side(pane_b_, caption_b_, Slot::b, b_);
}

void PairImages::frame_ready(int id) {
  // A decode for a frame no longer A or B (the pair moved on) is ignored.
  if (id != a_ && id != b_)
    return;
  auto f = loader_->frame(id);
  if (!f)
    return;
  if (id == a_)
    pane_a_->set_frame(f);
  if (id == b_)
    pane_b_->set_frame(f);
}

} // namespace rux::qt
