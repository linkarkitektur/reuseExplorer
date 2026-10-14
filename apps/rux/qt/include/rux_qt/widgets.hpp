// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Shared building blocks of the Qt client — the Qt counterparts of the web
// GUI's Sidebar, StatCard, Pill, LabelLegend and panel/card frames.
//
// Styling lives in styles/app.qss, keyed on objectName and dynamic properties
// (`role`, `tone`, `active`). These classes only add what QSS cannot do —
// uppercase + letter-spacing, painted swatches — and read those values from
// the Theme, never from literals.

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QVector>

class QButtonGroup;
class QContextMenuEvent;
class QGridLayout;
class QHBoxLayout;
class QVBoxLayout;

namespace rux::qt {

/// A label drawn in capitals with token letter-spacing — QSS has neither
/// `text-transform` nor `letter-spacing`. Its QSS `role` property picks the
/// look (eyebrow, caps, fieldLabel, …); @p tracking_token the spacing
/// (empty = none).
class CapsLabel : public QLabel {
  Q_OBJECT
    public:
  CapsLabel(const QString &text, const QString &role,
            const QString &tracking_token = "--tracking-caps",
            QWidget *parent = nullptr);
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

    protected:
  void paintEvent(QPaintEvent *) override;

    private:
  QFont tracked_font() const;
  QString tracking_;
};

/// A small status pill: tone = good | warn | wait | crit | accent | outline.
class Pill : public QLabel {
  Q_OBJECT
    public:
  Pill(const QString &text, const QString &tone, QWidget *parent = nullptr);
};

/// A bordered surface with an optional caps header and a body layout —
/// `objectName` "panel" (raised card) or "well" (sunken inset).
class Panel : public QFrame {
  Q_OBJECT
    public:
  explicit Panel(const QString &title = {}, QWidget *parent = nullptr,
                 const QString &kind = "panel");
  QVBoxLayout *body() const { return body_; }
  /// Put a widget on the right of the header row (an action, a count).
  void add_header_widget(QWidget *w);

    private:
  QVBoxLayout *body_ = nullptr;
  QHBoxLayout *header_ = nullptr;
};

/// A pane of a splitter layout (Rediger): a caps title, optional header
/// widgets and a "Skjul" button over the content. Hiding is the owner's
/// job (hide_requested); the header never scrolls with the content.
class PaneFrame : public QFrame {
  Q_OBJECT
    public:
  explicit PaneFrame(const QString &title, QWidget *parent = nullptr);
  void set_content(QWidget *content);
  QWidget *content() const { return content_; }
  /// Before "Skjul"; @p stretch > 0 shares the header's spare width with
  /// the gap after the title (an eliding status line).
  void add_header_widget(QWidget *w, int stretch = 0);

    signals:
  void hide_requested();

    private:
  QHBoxLayout *header_ = nullptr;
  QVBoxLayout *body_ = nullptr;
  QWidget *content_ = nullptr;
};

/// A KPI tile: caps label, a large display-face figure, an optional hint.
class StatCard : public QFrame {
  Q_OBJECT
    public:
  StatCard(const QString &label, const QString &value, const QString &unit = {},
           const QString &hint = {}, QWidget *parent = nullptr);
};

/// One row of the nav rail: status dot, label, optional count.
class NavItem : public QPushButton {
  Q_OBJECT
    public:
  NavItem(const QString &label, const QString &count = {},
          QWidget *parent = nullptr);
  /// Keyboard mnemonic safety: a raw '&' in a page name is shown literally.
  static QString escape_mnemonic(QString s);
  /// Replace the count chip's text; empty hides the chip.
  void set_count(const QString &count);

    private:
  void sync_active(bool on);
  QLabel *dot_ = nullptr;
  QLabel *text_ = nullptr;
  QLabel *count_ = nullptr;
};

/// The left navigation rail (the web Sidebar): product/project identity on
/// the navy chrome, grouped NavItems, a footer slot.
class NavRail : public QFrame {
  Q_OBJECT
    public:
  NavRail(const QString &project_name, QWidget *parent = nullptr);
  void add_group(const QString &title);
  NavItem *add_item(const QString &label, const QString &count = {});
  void add_footer(QWidget *w);
  void set_current(int index);
  int current() const;
  NavItem *item(int index) const;
  void set_project_name(const QString &name);

    signals:
  void current_changed(int index);

    private:
  QVBoxLayout *items_ = nullptr;
  QVBoxLayout *footer_ = nullptr;
  QButtonGroup *group_ = nullptr;
  CapsLabel *name_ = nullptr;
};

/// A one-line label that elides instead of growing its parent: Qt::ElideMiddle
/// for paths (the file name stays visible), ElideRight for prose. The full
/// text is the tooltip.
class ElidedLabel : public QLabel {
  Q_OBJECT
    public:
  ElidedLabel(const QString &text, Qt::TextElideMode mode = Qt::ElideMiddle,
              QWidget *parent = nullptr);
  void set_full_text(const QString &text);
  QString full_text() const { return full_; }
  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;

    protected:
  void paintEvent(QPaintEvent *) override;
  /// A manually painted elided label has no Qt text-selection cursor to drag
  /// (the paintEvent draws elided text directly; QLabel's own selectable-text
  /// machinery never sees it), so the full, unelided value is reachable
  /// through a one-item context menu instead ("Kopiér værdi" — the inspector
  /// and property-list values final review finding 10 asks for).
  void contextMenuEvent(QContextMenuEvent *) override;

    private:
  QString full_;
  Qt::TextElideMode mode_;
};

/// One categorical entry of a label legend.
struct LegendEntry {
  QString name;
  QString colour_token; ///< e.g. "--label-3"
  QString count;        ///< already formatted; may be empty
};

/// The --label-* legend: swatch, class name, point count.
class LabelLegend : public QWidget {
  Q_OBJECT
    public:
  explicit LabelLegend(const QVector<LegendEntry> &entries,
                       QWidget *parent = nullptr);
};

/// A painted colour swatch (QSS cannot take a per-instance token colour
/// without a per-widget stylesheet). Repaints on theme change.
class Swatch : public QWidget {
  Q_OBJECT
    public:
  explicit Swatch(const QString &colour_token, QWidget *parent = nullptr);
  QSize sizeHint() const override;

    protected:
  void paintEvent(QPaintEvent *) override;

    private:
  QString token_;
};

/// Key/value rows for the inspector: muted caps key, mono value.
class PropertyList : public QWidget {
  Q_OBJECT
    public:
  explicit PropertyList(QWidget *parent = nullptr);
  void add(const QString &key, const QString &value, bool mono = true);
  /// A row whose key is a name (a cloud, a table): shown as written, in
  /// mono, not in caps.
  void add_name(const QString &name, const QString &value);
  /// A legend row: a token swatch, the name (as written) and a value.
  void add_swatch(const QString &colour_token, const QString &name,
                  const QString &value);

    private:
  QGridLayout *grid_ = nullptr;
};

/// A shell command in mono that re-wraps to its width between words, with
/// ` \` continuations so the shown text still pastes as one command
/// (wrap_shell_command). Selectable.
class CommandBlock : public QLabel {
  Q_OBJECT
    public:
  explicit CommandBlock(const QString &command = {}, QWidget *parent = nullptr);
  void set_command(const QString &command);
  QString command() const { return command_; }
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;
  bool hasHeightForWidth() const override { return true; }
  int heightForWidth(int width) const override;

    protected:
  void resizeEvent(QResizeEvent *e) override;

    private:
  QString wrapped(int width) const;
  QString command_;
};

/// Format a count the Danish way (12.345), as the web GUI's da-DK figures.
QString format_count(qulonglong n);

} // namespace rux::qt
