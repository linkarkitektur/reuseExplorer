// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/PendingBanner.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace rux::qt {

PendingBanner::PendingBanner(EdgeEditor &editor, QWidget *parent)
    : QFrame(parent), editor_(editor) {
  const Theme &t = theme();
  setObjectName("pendingBanner");
  auto *bl = new QHBoxLayout(this);
  // Thin: it takes its height from Rediger's graph and edge list.
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-1"),
                         t.px("--space-4"), t.px("--space-1"));
  bl->setSpacing(t.px("--space-3"));
  auto *dot = new QLabel;
  dot->setObjectName("bannerDot");
  bl->addWidget(dot, 0, Qt::AlignVCenter);
  auto *texts = new QVBoxLayout;
  texts->setSpacing(0);
  text_ = new QLabel;
  text_->setObjectName("bannerText");
  text_->setWordWrap(true); // it sits over Rediger's left column only
  texts->addWidget(text_);
  error_ = new QLabel;
  error_->setObjectName("bannerError");
  error_->setWordWrap(true);
  texts->addWidget(error_);
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
  setMinimumHeight(save_->sizeHint().height() + 2 * t.px("--space-1"));

  connect(save_, &QPushButton::clicked, this, [this] { editor_.save(); });
  connect(discard_, &QPushButton::clicked, this, [this] { editor_.discard(); });
  connect(&editor_, &EdgeEditor::changed, this, &PendingBanner::sync);
  sync();
}

void PendingBanner::sync() {
  const int n = editor_.pending();
  const QString error = editor_.last_error();
  const bool saving = editor_.is_saving();
  setVisible(n > 0 || !error.isEmpty() || saving);
  text_->setText(saving ? QString("Gemmer ændringer i posegrafen …")
                 : n > 0
                     ? QString("%1 i posegrafen venter på at blive gemt — %2")
                           .arg(n == 1 ? QString("1 ændring")
                                       : QString("%1 ændringer").arg(n),
                                QString::fromStdString(
                                    pending_summary_da(editor_.edits().ops())))
                     : QString("Posegrafen"));
  error_->setText(error);
  error_->setToolTip(editor_.last_error_detail());
  error_->setVisible(!error.isEmpty());
  // Read-only is a state, not a failure: the warning tone, like the title
  // bar's "Skrivebeskyttet" pill. A failed write (locked, other) is critical.
  const bool busy = !editor_.pipeline_busy().isEmpty();
  // Read-only, and a pipeline stage holding the project, are states: warn.
  const bool read_only_error =
      !error.isEmpty() && (!editor_.read_only_reason().isEmpty() || busy);
  setProperty("tone", error.isEmpty()   ? "pending"
                      : read_only_error ? "warn"
                                        : "error");
  repolish(this);
  save_->setEnabled(n > 0 && editor_.can_save() && !saving && !busy);
  save_->setToolTip(
      !editor_.can_save() ? editor_.read_only_reason()
      : busy ? QString("Et pipeline-trin (%1) skriver til projektet — gem, "
                       "når det er færdigt.")
                   .arg(editor_.pipeline_busy())
             : QString("Skriv ændringerne til projektet (Ctrl+S)"));
  if (busy && n > 0 && error.isEmpty()) {
    error_->setText(QString("Gem er sat på pause, mens %1 kører.")
                        .arg(editor_.pipeline_busy()));
    error_->setVisible(true);
  }
  discard_->setEnabled(n > 0 && !saving);
}

} // namespace rux::qt
