// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/Theme.hpp>
#include <rux_qt/cli_command.hpp>
#include <rux_qt/widgets.hpp>

#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QClipboard>
#include <QContextMenuEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLocale>
#include <QMenu>
#include <QPainter>
#include <QStyleOption>
#include <QVBoxLayout>

#include <algorithm>

namespace rux::qt {

// ---------------------------------------------------------------- CapsLabel --

CapsLabel::CapsLabel(const QString &text, const QString &role,
                     const QString &tracking_token, QWidget *parent)
    : QLabel(text, parent), tracking_(tracking_token) {
  setProperty("role", role);
  connect(&theme(), &Theme::changed, this, [this] {
    updateGeometry();
    update();
  });
}

QFont CapsLabel::tracked_font() const {
  QFont f = font(); // as resolved by the stylesheet (size, weight, family)
  f.setCapitalization(QFont::AllUppercase);
  const double em = tracking_.isEmpty() ? 0.0 : theme().em(tracking_);
  f.setLetterSpacing(QFont::AbsoluteSpacing, em * f.pixelSize());
  return f;
}

QSize CapsLabel::sizeHint() const {
  const QFontMetrics fm(tracked_font());
  const QMargins m = contentsMargins();
  return {fm.horizontalAdvance(text()) + m.left() + m.right() + 1,
          fm.height() + m.top() + m.bottom()};
}

QSize CapsLabel::minimumSizeHint() const { return {0, sizeHint().height()}; }

void CapsLabel::paintEvent(QPaintEvent *) {
  QPainter p(this);
  // Let the stylesheet draw background/border/padding first.
  QStyleOption opt;
  opt.initFrom(this);
  style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
  p.setFont(tracked_font());
  p.setPen(palette().color(foregroundRole()));
  const QFontMetrics fm(p.font());
  const QString shown =
      fm.elidedText(text(), Qt::ElideRight, contentsRect().width());
  p.drawText(contentsRect(), static_cast<int>(alignment()) | Qt::TextSingleLine,
             shown);
}

// --------------------------------------------------------------------- Pill --

Pill::Pill(const QString &text, const QString &tone, QWidget *parent)
    : QLabel(text, parent) {
  setObjectName("pill");
  setProperty("tone", tone);
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

// -------------------------------------------------------------------- Panel --

Panel::Panel(const QString &title, QWidget *parent, const QString &kind)
    : QFrame(parent) {
  setObjectName(kind);
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);
  if (!title.isEmpty()) {
    auto *head = new QWidget;
    head->setObjectName("panelHead");
    header_ = new QHBoxLayout(head);
    header_->setContentsMargins(
        theme().px("--space-4"), theme().px("--space-3"),
        theme().px("--space-4"), theme().px("--space-3"));
    header_->setSpacing(theme().px("--space-2"));
    header_->addWidget(new CapsLabel(title, "panelTitle"));
    header_->addStretch(1);
    outer->addWidget(head);
  }
  auto *bodyw = new QWidget;
  bodyw->setObjectName("panelBody");
  body_ = new QVBoxLayout(bodyw);
  const int pad = theme().px("--space-4");
  body_->setContentsMargins(
      pad, title.isEmpty() ? pad : theme().px("--space-3"), pad, pad);
  body_->setSpacing(theme().px("--space-3"));
  outer->addWidget(bodyw, 1);
}

PaneFrame::PaneFrame(const QString &title, QWidget *parent) : QFrame(parent) {
  setObjectName("paneFrame");
  const Theme &t = theme();
  body_ = new QVBoxLayout(this);
  body_->setContentsMargins(0, 0, 0, 0);
  body_->setSpacing(0);
  auto *head = new QFrame;
  head->setObjectName("paneHeader");
  // Never squeezed: a full graph pane gave its header ~5 px less than its
  // hint, clipping "Skjul".
  head->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  header_ = new QHBoxLayout(head);
  header_->setContentsMargins(t.px("--space-3"), t.px("--space-1"),
                              t.px("--space-2"), t.px("--space-1"));
  header_->setSpacing(t.px("--space-2"));
  header_->addWidget(new CapsLabel(title, "panelTitle"));
  header_->addStretch(1);
  auto *hide = new QPushButton(NavItem::escape_mnemonic("Skjul"));
  hide->setObjectName("paneHide");
  hide->setProperty("kind", "ghost");
  hide->setCursor(Qt::PointingHandCursor);
  hide->setToolTip(
      "Skjul panelet — vis det igen under Paneler eller med Ctrl+K");
  // Polished first: the unpolished hint lacks the QSS padding, and a
  // squeezed pane then clipped the label's descenders ("Skjul").
  hide->ensurePolished();
  hide->setMinimumHeight(hide->sizeHint().height());
  header_->addWidget(hide);
  body_->addWidget(head);
  connect(hide, &QPushButton::clicked, this, &PaneFrame::hide_requested);
}

void PaneFrame::set_content(QWidget *content) {
  if (content_) {
    content_->hide();
    body_->removeWidget(content_);
    content_->deleteLater();
  }
  content_ = content;
  body_->addWidget(content_, 1);
}

void PaneFrame::add_header_widget(QWidget *w, int stretch) {
  header_->insertWidget(header_->count() - 1, w, stretch); // before "Skjul"
}

void Panel::add_header_widget(QWidget *w) {
  if (header_)
    header_->addWidget(w);
}

// ----------------------------------------------------------------- StatCard --

StatCard::StatCard(const QString &label, const QString &value,
                   const QString &unit, const QString &hint, QWidget *parent)
    : QFrame(parent) {
  setObjectName("statCard");
  auto *l = new QVBoxLayout(this);
  const int pad = theme().px("--space-4");
  l->setContentsMargins(pad, pad, pad, pad);
  l->setSpacing(theme().px("--space-1"));
  l->addWidget(new CapsLabel(label, "statLabel"));
  auto *row = new QHBoxLayout;
  row->setSpacing(theme().px("--space-1"));
  auto *v = new QLabel(value);
  v->setObjectName("statValue");
  row->addWidget(v, 0, Qt::AlignBaseline);
  if (!unit.isEmpty()) {
    auto *u = new QLabel(unit);
    u->setObjectName("statUnit");
    row->addWidget(u, 0, Qt::AlignBaseline);
  }
  row->addStretch(1);
  l->addLayout(row);
  if (!hint.isEmpty()) {
    auto *h = new QLabel(hint);
    h->setObjectName("statHint");
    l->addWidget(h);
  }
}

// ------------------------------------------------------------------ NavItem --

QString NavItem::escape_mnemonic(QString s) { return s.replace("&", "&&"); }

NavItem::NavItem(const QString &label, const QString &count, QWidget *parent)
    : QPushButton(parent) {
  setObjectName("navItem");
  setCheckable(true);
  setCursor(Qt::PointingHandCursor);
  setFocusPolicy(Qt::StrongFocus);
  setAccessibleName(label);
  auto *l = new QHBoxLayout(this);
  l->setContentsMargins(theme().px("--space-4"), theme().px("--space-2"),
                        theme().px("--space-4"), theme().px("--space-2"));
  l->setSpacing(theme().px("--space-2"));
  dot_ = new QLabel;
  dot_->setObjectName("navDot");
  // A QLabel is not a button: its text is not a mnemonic, so '&' is safe
  // here. It is the QPushButton/QAction text that would need escape_mnemonic.
  text_ = new QLabel(label);
  text_->setObjectName("navText");
  l->addWidget(dot_, 0, Qt::AlignVCenter);
  l->addWidget(text_, 1);
  count_ = new QLabel(count);
  count_->setObjectName("navCount");
  count_->setVisible(!count.isEmpty());
  l->addWidget(count_, 0, Qt::AlignVCenter);
  for (QWidget *w :
       {static_cast<QWidget *>(dot_), static_cast<QWidget *>(text_),
        static_cast<QWidget *>(count_)})
    if (w)
      w->setAttribute(Qt::WA_TransparentForMouseEvents);
  connect(this, &QPushButton::toggled, this, &NavItem::sync_active);
  sync_active(false);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  setMinimumHeight(l->sizeHint().height());
}

void NavItem::set_count(const QString &count) {
  count_->setText(count);
  count_->setVisible(!count.isEmpty());
}

void NavItem::sync_active(bool on) {
  for (QWidget *w :
       {static_cast<QWidget *>(dot_), static_cast<QWidget *>(text_),
        static_cast<QWidget *>(count_)}) {
    if (!w)
      continue;
    w->setProperty("active", on);
    repolish(w);
  }
}

// ------------------------------------------------------------------ NavRail --

NavRail::NavRail(const QString &project_name, QWidget *parent)
    : QFrame(parent) {
  setObjectName("navRail");
  setFixedWidth(theme().px("--layout-nav-width"));
  auto *l = new QVBoxLayout(this);
  l->setContentsMargins(0, theme().px("--space-4"), 0, theme().px("--space-4"));
  l->setSpacing(0);

  auto *eyebrow = new CapsLabel("Projekt", "eyebrow", "--tracking-wide");
  eyebrow->setContentsMargins(theme().px("--space-4"), 0,
                              theme().px("--space-4"), 0);
  l->addWidget(eyebrow);
  name_ = new CapsLabel(project_name, "railProject", "--tracking-caps");
  name_->setContentsMargins(theme().px("--space-4"), theme().px("--space-1"),
                            theme().px("--space-4"), theme().px("--space-3"));
  name_->setToolTip(project_name);
  l->addWidget(name_);

  items_ = new QVBoxLayout;
  items_->setSpacing(0);
  l->addLayout(items_);
  l->addStretch(1);
  footer_ = new QVBoxLayout;
  footer_->setContentsMargins(theme().px("--space-4"), 0,
                              theme().px("--space-4"), 0);
  footer_->setSpacing(theme().px("--space-2"));
  l->addLayout(footer_);

  group_ = new QButtonGroup(this);
  group_->setExclusive(true);
  connect(group_, &QButtonGroup::idClicked, this, &NavRail::current_changed);
}

void NavRail::add_group(const QString &title) {
  auto *g = new CapsLabel(title, "navGroup", "--tracking-wide");
  g->setContentsMargins(theme().px("--space-4"), theme().px("--space-5"),
                        theme().px("--space-4"), theme().px("--space-1"));
  items_->addWidget(g);
}

NavItem *NavRail::add_item(const QString &label, const QString &count) {
  auto *item = new NavItem(label, count);
  group_->addButton(item, static_cast<int>(group_->buttons().size()));
  items_->addWidget(item);
  return item;
}

void NavRail::add_footer(QWidget *w) { footer_->addWidget(w); }

void NavRail::set_current(int index) {
  if (auto *b = group_->button(index))
    b->setChecked(true);
}

int NavRail::current() const { return group_->checkedId(); }

NavItem *NavRail::item(int index) const {
  return qobject_cast<NavItem *>(group_->button(index));
}

void NavRail::set_project_name(const QString &name) {
  name_->setText(name);
  name_->setToolTip(name);
  name_->updateGeometry();
  name_->update();
}

// -------------------------------------------------------------- ElidedLabel --

ElidedLabel::ElidedLabel(const QString &text, Qt::TextElideMode mode,
                         QWidget *parent)
    : QLabel(parent), mode_(mode) {
  set_full_text(text);
  setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
}

void ElidedLabel::set_full_text(const QString &text) {
  full_ = text;
  setText(text); // keeps sizeHint and accessibility honest
  setToolTip(text);
  updateGeometry();
  update();
}

QSize ElidedLabel::minimumSizeHint() const {
  return {0, QLabel::minimumSizeHint().height()};
}

QSize ElidedLabel::sizeHint() const { return QLabel::sizeHint(); }

void ElidedLabel::contextMenuEvent(QContextMenuEvent *e) {
  if (full_.isEmpty())
    return;
  QMenu menu(this);
  QAction *copy = menu.addAction("Kopiér værdi");
  connect(copy, &QAction::triggered, this,
          [text = full_] { QApplication::clipboard()->setText(text); });
  menu.exec(e->globalPos());
}

void ElidedLabel::paintEvent(QPaintEvent *) {
  QPainter p(this);
  QStyleOption opt;
  opt.initFrom(this);
  style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
  p.setPen(palette().color(foregroundRole()));
  p.setFont(font());
  const QRect r = contentsRect();
  p.drawText(r, static_cast<int>(alignment()) | Qt::TextSingleLine,
             fontMetrics().elidedText(full_, mode_, r.width()));
}

// ------------------------------------------------------------------- Swatch --

Swatch::Swatch(const QString &colour_token, QWidget *parent)
    : QWidget(parent), token_(colour_token) {
  const int s = theme().px("--space-3");
  setFixedSize(s, s);
  connect(&theme(), &Theme::changed, this, qOverload<>(&QWidget::update));
}

QSize Swatch::sizeHint() const {
  const int s = theme().px("--space-3");
  return {s, s};
}

void Swatch::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  p.setBrush(theme().color(token_));
  const qreal r = theme().px("--radius-sm");
  p.drawRoundedRect(QRectF(rect()), r, r);
}

// -------------------------------------------------------------- LabelLegend --

LabelLegend::LabelLegend(const QVector<LegendEntry> &entries, QWidget *parent)
    : QWidget(parent) {
  auto *g = new QGridLayout(this);
  g->setContentsMargins(0, 0, 0, 0);
  g->setHorizontalSpacing(theme().px("--space-2"));
  g->setVerticalSpacing(theme().px("--space-1"));
  int row = 0;
  for (const auto &e : entries) {
    g->addWidget(new Swatch(e.colour_token), row, 0, Qt::AlignVCenter);
    auto *name = new QLabel(e.name);
    name->setObjectName("legendName");
    g->addWidget(name, row, 1);
    auto *count = new QLabel(e.count);
    count->setObjectName("legendCount");
    count->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    g->addWidget(count, row, 2);
    ++row;
  }
  g->setColumnStretch(1, 1);
}

// ------------------------------------------------------------- PropertyList --

PropertyList::PropertyList(QWidget *parent) : QWidget(parent) {
  grid_ = new QGridLayout(this);
  grid_->setContentsMargins(0, 0, 0, 0);
  grid_->setHorizontalSpacing(theme().px("--space-3"));
  grid_->setVerticalSpacing(theme().px("--space-2"));
  grid_->setColumnStretch(1, 1);
}

void PropertyList::add(const QString &key, const QString &value, bool mono) {
  const int row = grid_->rowCount();
  auto *k = new CapsLabel(key, "propKey");
  grid_->addWidget(k, row, 0, Qt::AlignLeft | Qt::AlignVCenter);
  // Elided, never widening: a long value (a blob description, a path) must
  // not push the inspector wider than its panel and hide the other rows.
  auto *v = new ElidedLabel(value, Qt::ElideRight);
  v->setObjectName(mono ? "propValueMono" : "propValue");
  v->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  grid_->addWidget(v, row, 1);
}

void PropertyList::add_name(const QString &name, const QString &value) {
  const int row = grid_->rowCount();
  auto *k = new ElidedLabel(name, Qt::ElideMiddle);
  k->setObjectName("propName");
  k->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  k->setMaximumWidth(theme().px("--layout-panel-width") / 2);
  grid_->addWidget(k, row, 0, Qt::AlignLeft | Qt::AlignVCenter);
  auto *v = new ElidedLabel(value, Qt::ElideRight);
  v->setObjectName("propValueMono");
  v->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  grid_->addWidget(v, row, 1);
}

void PropertyList::add_swatch(const QString &colour_token, const QString &name,
                              const QString &value) {
  const int row = grid_->rowCount();
  auto *key = new QWidget;
  auto *h = new QHBoxLayout(key);
  h->setContentsMargins(0, 0, 0, 0);
  h->setSpacing(theme().px("--space-2"));
  h->addWidget(new Swatch(colour_token));
  auto *k = new QLabel(name);
  k->setObjectName("legendName");
  k->setToolTip(name);
  h->addWidget(k, 1);
  grid_->addWidget(key, row, 0, Qt::AlignLeft | Qt::AlignVCenter);
  auto *v = new ElidedLabel(value, Qt::ElideRight);
  v->setObjectName("propValueMono");
  v->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  grid_->addWidget(v, row, 1);
}

QString format_count(qulonglong n) {
  return QLocale(QLocale::Danish, QLocale::Denmark).toString(n);
}

// ------------------------------------------------------------- CommandBlock --

CommandBlock::CommandBlock(const QString &command, QWidget *parent)
    : QLabel(parent) {
  setObjectName("commandBlock");
  setTextInteractionFlags(Qt::TextSelectableByMouse);
  setWordWrap(false);
  setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  set_command(command);
}

void CommandBlock::set_command(const QString &command) {
  command_ = command;
  setText(wrapped(width()));
  updateGeometry();
}

QString CommandBlock::wrapped(int width) const {
  // rect() - contentsRect() includes the QSS padding and border, which
  // contentsMargins() alone does not.
  const int chrome_w = rect().width() - contentsRect().width();
  const int inner = width - chrome_w - 2 * margin();
  const int ch = std::max(1, fontMetrics().horizontalAdvance(QLatin1Char('m')));
  const std::size_t cols =
      inner > ch * 12 ? static_cast<std::size_t>(inner / ch) : 1000;
  return QString::fromStdString(
      wrap_shell_command(command_.toStdString(), cols));
}

int CommandBlock::heightForWidth(int width) const {
  const int lines = static_cast<int>(wrapped(width).count('\n')) + 1;
  const int chrome_h = rect().height() - contentsRect().height();
  return lines * fontMetrics().lineSpacing() + fontMetrics().descent() +
         chrome_h + 2 * margin();
}

QSize CommandBlock::sizeHint() const {
  return {0, heightForWidth(std::max(width(), 1))};
}

QSize CommandBlock::minimumSizeHint() const { return sizeHint(); }

void CommandBlock::resizeEvent(QResizeEvent *e) {
  QLabel::resizeEvent(e);
  const QString t = wrapped(width());
  if (t != text()) {
    setText(t);
    updateGeometry();
  }
}

} // namespace rux::qt
