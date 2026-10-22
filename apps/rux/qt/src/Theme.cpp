// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/Theme.hpp>
#include <rux_qt/fonts.hpp>

#include <QApplication>
#include <QFile>
#include <QFileInfo>
#include <QPainter>
#include <QStyle>
#include <QStyleFactory>
#include <QTemporaryDir>
#include <QTimer>
#include <QWidget>

#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

namespace rux::qt {
namespace {

std::optional<QString> read_file(const QString &path) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly))
    return std::nullopt;
  return QString::fromUtf8(f.readAll());
}

/// How long a hot reload waits for a deleted style file to come back (some
/// editors and tools delete and then recreate on save).
constexpr int kRetryIntervalMs = 100;
constexpr int kRetryAttempts = 50;

int weight_from(const QString &v) {
  bool ok = false;
  const int w = v.toInt(&ok);
  return ok ? w : 400;
}

/// Font families of a CSS stack, unquoted: "'Archivo', Arial, sans-serif".
QStringList families(const QString &stack) {
  QStringList out;
  for (QString f : stack.split(',', Qt::SkipEmptyParts)) {
    f = f.trimmed();
    if (f.size() >= 2 && (f.front() == '\'' || f.front() == '"'))
      f = f.mid(1, f.size() - 2);
    // CSS generic families have Qt equivalents only by style hint.
    if (f == "sans-serif" || f == "serif" || f == "monospace" ||
        f == "ui-monospace")
      continue;
    out << f;
  }
  return out;
}

} // namespace

Theme &Theme::instance() {
  static Theme *t = new Theme; // QObject: never destroyed after QApplication
  return *t;
}

Theme::Source Theme::dev_source(const QString &style_dir) {
  const QString dir = style_dir.isEmpty()
                          ? QStringLiteral(RUX_QT_SOURCE_DIR "/styles")
                          : style_dir;
  return {QStringLiteral(RUX_QT_TOKENS_CSS), dir + "/app.qss", true};
}

Theme::Source Theme::embedded_source() {
  return {QStringLiteral(":/rux_qt/tokens.css"),
          QStringLiteral(":/rux_qt/styles/app.qss"), false};
}

void Theme::apply(ThemeMode mode, const Source &source) {
  mode_ = mode;
  source_ = source;
  if (source_.watch && !watcher_) {
    watcher_ = new QFileSystemWatcher(this);
    connect(watcher_, &QFileSystemWatcher::fileChanged, this, [this] {
      std::fprintf(stderr, "rux-qt: style changed on disk, reloading\n");
      reload();
    });
  }
  reload();
}

void Theme::reload() {
  const std::optional<QString> css = read_file(source_.tokens_css);
  const std::optional<QString> tmpl = read_file(source_.qss_template);
  if (!css || !tmpl) {
    const QString absent = !css ? source_.tokens_css : source_.qss_template;
    if (applied_) {
      // Mid-save (deleted, not yet recreated): keep the theme on screen and
      // try again shortly, re-watching the path once it is back. Never apply
      // an empty token set — that would paint everything magenta.
      if (retries_ < kRetryAttempts) {
        ++retries_;
        QTimer::singleShot(kRetryIntervalMs, this, [this] { reload(); });
      } else {
        std::fprintf(stderr,
                     "rux-qt: ERROR %s is gone; keeping the previous theme "
                     "(save it again to reload)\n",
                     qPrintable(absent));
        retries_ = 0;
      }
      return;
    }
    std::fprintf(stderr, "rux-qt: ERROR cannot read %s\n", qPrintable(absent));
  }
  retries_ = 0;
  missing_.clear();
  ensure_bundled_fonts();

  tokens_ = parse_tokens_css(css.value_or(QString()).toStdString(), mode_);
  add_generated_tokens();
  ResolveResult r =
      resolve_vars(tmpl.value_or(QString()).toStdString(), tokens_);
  if (!css)
    note_missing(source_.tokens_css + " (unreadable)");
  if (!tmpl)
    note_missing(source_.qss_template + " (unreadable)");
  for (const auto &m : r.missing)
    note_missing(QString::fromStdString(m));
  stylesheet_ = QString::fromStdString(r.text);

  // Fusion everywhere: no platform theme leaks into a screenshot, and every
  // Fusion-drawn primitive reads the palette below.
  QApplication::setStyle(QStyleFactory::create("Fusion"));

  QPalette p;
  const QColor surface = color("--color-surface");
  const QColor raised = color("--color-surface-raised");
  const QColor sunken = color("--color-surface-sunken");
  const QColor text = color("--color-text");
  const QColor faint = color("--color-text-faint");
  p.setColor(QPalette::Window, surface);
  p.setColor(QPalette::WindowText, text);
  p.setColor(QPalette::Base, raised);
  p.setColor(QPalette::AlternateBase, sunken);
  p.setColor(QPalette::Text, text);
  p.setColor(QPalette::Button, raised);
  p.setColor(QPalette::ButtonText, text);
  p.setColor(QPalette::BrightText, color("--color-text-inverse"));
  p.setColor(QPalette::PlaceholderText, faint);
  p.setColor(QPalette::ToolTipBase, color("--color-surface-overlay"));
  p.setColor(QPalette::ToolTipText, text);
  p.setColor(QPalette::Highlight, color("--color-accent-muted"));
  p.setColor(QPalette::HighlightedText, text);
  p.setColor(QPalette::Link, color("--color-accent-deep"));
  p.setColor(QPalette::Accent, color("--color-accent"));
  p.setColor(QPalette::Light, raised);
  p.setColor(QPalette::Midlight, color("--color-border"));
  p.setColor(QPalette::Mid, color("--color-border-strong"));
  p.setColor(QPalette::Dark, color("--color-border-strong"));
  p.setColor(QPalette::Shadow, color("--color-scrim"));
  for (auto role : {QPalette::WindowText, QPalette::Text, QPalette::ButtonText})
    p.setColor(QPalette::Disabled, role, faint);
  QApplication::setPalette(p);
  QApplication::setFont(font(FontRole::sans, "--font-size-md"));

  qApp->setStyleSheet(stylesheet_);
  // Debug aid: RUX_QT_DUMP_QSS=<file> writes the resolved stylesheet.
  if (const QString dump = qEnvironmentVariable("RUX_QT_DUMP_QSS");
      !dump.isEmpty()) {
    QFile out(dump);
    if (out.open(QIODevice::WriteOnly | QIODevice::Truncate))
      out.write(stylesheet_.toUtf8());
  }

  if (watcher_) {
    // Editors replace files on save, which drops them from the watcher.
    for (const QString &f : {source_.tokens_css, source_.qss_template})
      if (!watcher_->files().contains(f) && QFileInfo::exists(f))
        watcher_->addPath(f);
  }

  for (const QString &m : missing_)
    std::fprintf(stderr, "rux-qt: ERROR MISSING token %s\n", qPrintable(m));
  applied_ = true;
  emit changed();
}

namespace {

/// Paint a two-stroke glyph (tick, chevron) in @p colour at 2x the size QSS
/// shows it and save it as a PNG.
QString paint_glyph(const QString &path, const QColor &colour,
                    const std::vector<QPointF> &unit_points, double stroke) {
  const int s = 32; // 2x a 16px glyph: crisp at --scale 2
  QImage img(s, s, QImage::Format_ARGB32_Premultiplied);
  img.fill(Qt::transparent);
  QPainter p(&img);
  p.setRenderHint(QPainter::Antialiasing);
  QPen pen(colour);
  pen.setWidthF(s * stroke);
  pen.setCapStyle(Qt::RoundCap);
  pen.setJoinStyle(Qt::RoundJoin);
  p.setPen(pen);
  std::vector<QPointF> pts;
  for (const QPointF &u : unit_points)
    pts.emplace_back(u.x() * s, u.y() * s);
  p.drawPolyline(pts.data(), static_cast<int>(pts.size()));
  p.end();
  img.save(path);
  return path;
}

} // namespace

void Theme::add_generated_tokens() {
  // Glyphs QSS can only take by url (the shell has no QtSvg to tint an SVG),
  // painted in token colours and exposed as `--qt-icon-*` tokens.
  if (!icon_dir_)
    icon_dir_ = std::make_unique<QTemporaryDir>();
  auto colour = [&](const char *token) {
    const auto it = tokens_.find(token);
    const auto c = it == tokens_.end() ? std::nullopt : parse_color(it->second);
    if (!c)
      note_missing(token);
    return c ? QColor(c->r, c->g, c->b)
             : QColor(QLatin1StringView(kMissingColour));
  };
  const QString tag = mode_ == ThemeMode::dark ? "dark" : "light";
  tokens_["--qt-icon-check"] =
      paint_glyph(icon_dir_->filePath("check-" + tag + ".png"),
                  colour("--color-on-accent"),
                  {{0.24, 0.52}, {0.42, 0.70}, {0.76, 0.32}}, 0.12)
          .toStdString();
  // The dash of a partly checked (tri-state) check box.
  tokens_["--qt-icon-dash"] =
      paint_glyph(icon_dir_->filePath("dash-" + tag + ".png"),
                  colour("--color-on-accent"), {{0.28, 0.50}, {0.72, 0.50}},
                  0.12)
          .toStdString();
  tokens_["--qt-icon-chevron"] =
      paint_glyph(icon_dir_->filePath("chevron-" + tag + ".png"),
                  colour("--color-text-muted"),
                  {{0.30, 0.40}, {0.50, 0.60}, {0.70, 0.40}}, 0.08)
          .toStdString();
}

void Theme::note_missing(const QString &token) const {
  if (!missing_.contains(token))
    missing_ << token;
}

QString Theme::value(const QString &token) const {
  const auto it = tokens_.find(token.toStdString());
  if (it == tokens_.end()) {
    note_missing(token);
    return {};
  }
  return QString::fromStdString(it->second);
}

QColor Theme::color(const QString &token) const {
  const QString v = value(token);
  if (const auto c = parse_color(v.toStdString()))
    return QColor(c->r, c->g, c->b, c->a);
  if (!v.isEmpty())
    note_missing(token + " (not a colour)");
  return QColor(QLatin1StringView(kMissingColour));
}

int Theme::px(const QString &token) const {
  const auto l = length_px(value(token).toStdString());
  return l ? static_cast<int>(std::lround(*l)) : 0;
}

double Theme::em(const QString &token) const {
  return length_em(value(token).toStdString()).value_or(0.0);
}

QFont Theme::font(FontRole role, const QString &size_token,
                  const QString &weight_token) const {
  const char *stack = role == FontRole::display ? "--font-display"
                      : role == FontRole::mono  ? "--font-mono"
                                                : "--font-sans";
  QFont f;
  f.setFamilies(families(value(stack)));
  f.setStyleHint(role == FontRole::mono ? QFont::Monospace : QFont::SansSerif);
  f.setPixelSize(std::max(1, px(size_token)));
  f.setWeight(static_cast<QFont::Weight>(weight_from(value(weight_token))));
  return f;
}

void repolish(QWidget *w) {
  w->style()->unpolish(w);
  w->style()->polish(w);
  w->update();
}

} // namespace rux::qt
