// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// rux-qt-gallery: render one page of the Qt client, headless to PNG or live
// in a window. The design-studio skill's qt_shot.sh drives it; see
// .claude/skills/design-studio/references/qt-client.md.
//
// Exit codes: 0 ok, 2 bad command line, 3 a token or a bundled font family
// was missing (the PNG is still written, with magenta where the token was), 4
// project or page could not be opened, 5 the PNG could not be written.

#include "demo_pages.hpp"

#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/fonts.hpp>
#include <rux_qt/gallery_args.hpp>
#include <rux_qt/pages.hpp>
#include <rux_qt/workspace_logic.hpp>

#include <reusex/core/logging.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QPointer>
#include <QSurfaceFormat>
#include <QThread>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>

#include <clocale>
#include <cstdio>
#include <memory>

using namespace rux::qt;

namespace {

void pump(int rounds, int sleep_ms) {
  for (int i = 0; i < rounds; ++i) {
    QApplication::processEvents(QEventLoop::AllEvents);
    if (sleep_ms > 0)
      QThread::msleep(static_cast<unsigned long>(sleep_ms));
  }
}

} // namespace

int main(int argc, char **argv) {
  QElapsedTimer clock;
  clock.start();

  const GalleryArgs args = parse_gallery_args(argc, argv);
  if (!args.error.empty()) {
    std::fprintf(stderr, "rux-qt-gallery: %s\n\n%s", args.error.c_str(),
                 gallery_usage().c_str());
    return 2;
  }
  if (args.help) {
    std::fputs(gallery_usage().c_str(), stdout);
    return 0;
  }
  gallery::register_demo_pages();
  if (args.list_pages) {
    for (const auto &p : pages())
      std::printf("%-12s %s\n", qPrintable(p.name), qPrintable(p.description));
    return 0;
  }
  const PageInfo *page = find_page(QString::fromStdString(args.page));
  if (!page) {
    std::fprintf(stderr, "rux-qt-gallery: no page '%s' (see --list-pages)\n",
                 args.page.c_str());
    return 4;
  }

  const bool shot = !args.screenshot.empty();
  // Both must be set before the QApplication exists.
  if (args.scale != 1.0)
    qputenv("QT_SCALE_FACTOR", QByteArray::number(args.scale));
  if (shot && !args.gl && qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM"))
    qputenv("QT_QPA_PLATFORM", "offscreen");
  if (args.gl)
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

  QApplication app(argc, argv);
  // QApplication just called setlocale(LC_ALL, ""). Under a comma-decimal
  // locale (da_DK) every strtod/stod/sscanf in the library and its
  // dependencies (sqlite text, JSON, OpenCV, PCL) would read "799.85" as 799.
  // Numbers shown to the user go through QLocale, so the C library's numeric
  // locale stays "C" (the Qt docs recommend exactly this).
  std::setlocale(LC_NUMERIC, "C");
  QApplication::setApplicationName("rux-qt-gallery");
  // What rux's own log handler does for the app: library lines reach the
  // log tap (the Pipeline workspace's tail). Warnings also go to stderr.
  reusex::core::set_log_level(reusex::core::LogLevel::info);
  reusex::core::set_log_handler(
      [](reusex::core::LogLevel level, std::string_view message) {
        publish_log(static_cast<int>(level), message);
        if (level >= reusex::core::LogLevel::warn)
          std::fprintf(stderr, "rux-qt-gallery: %.*s\n",
                       static_cast<int>(message.size()), message.data());
      });

  // Hot reload in --dev mode and in every Debug build (the spec's "debug or
  // --dev"); a Release build reads the embedded snapshot unless --dev.
#ifdef NDEBUG
  const bool dev = args.dev;
#else
  const bool dev = true;
#endif
  Theme::Source source =
      dev ? Theme::dev_source(QString::fromStdString(args.style_dir))
          : Theme::embedded_source();
  if (shot)
    source.watch = false;
  theme().apply(args.theme, source);

  std::unique_ptr<reusex::ProjectDB> db;
  if (!args.project.empty()) {
    try {
      db = std::make_unique<reusex::ProjectDB>(args.project);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "rux-qt-gallery: cannot open %s: %s\n",
                   args.project.c_str(), e.what());
      return 4;
    }
  }
  PageContext ctx;
  ctx.db = db.get();
  ctx.project_path = QString::fromStdString(args.project);
  ctx.interactive_3d = args.gl;

  QWidget host;
  host.setObjectName("galleryHost");
  host.setWindowTitle(QString("rux-qt-gallery — %1").arg(page->name));
  auto *layout = new QVBoxLayout(&host);
  layout->setContentsMargins(0, 0, 0, 0);
  QPointer<QWidget> current = page->make(ctx);
  layout->addWidget(current);
  host.resize(args.width, args.height);

  // Dev mode: a token or QSS edit re-applies the stylesheet; rebuilding the
  // page as well picks up the values code reads from tokens (layout spacing,
  // the canvas colour, painted swatches).
  QObject::connect(&theme(), &Theme::changed, &host, [&] {
    if (current)
      current->deleteLater();
    current = page->make(ctx);
    layout->addWidget(current);
  });

  if (!shot) {
    host.show();
    return app.exec();
  }

  if (!args.gl)
    host.setAttribute(Qt::WA_DontShowOnScreen);
  host.show();
  // Coalesced renders (the 3D pane) run on zero-length timers; a GL widget
  // under Xvfb needs real time to get its first frame.
  pump(args.gl ? 30 : 6, args.gl ? 50 : 0);
  // Off-thread work a page started (frame decodes, thumbnails, an ICP run)
  // must land before the shot, or it shows "Henter …". Bounded: a hung
  // decode still produces a PNG.
  {
    QElapsedTimer settle;
    settle.start();
    // Idle twice in a row: a delivered frame can queue more work (the
    // thumbnails a repaint asks for).
    int idle = 0;
    while (idle < 2 && settle.elapsed() < 30000) {
      pump(2, 10);
      idle = FrameImageLoader::busy() == 0 && background_work_in_flight() == 0
                 ? idle + 1
                 : 0;
    }
    pump(6, 0);
  }

  // The shell must fit the window it is shot at: a workspace whose minimum
  // width pushes the inspector off a 1440 px laptop screen fails the shot
  // (qt_shot.sh greps TOO WIDE), never passes silently as a wider PNG.
  const int min_w = current ? current->minimumSizeHint().width() : 0;
  const bool too_wide = min_w > args.width;
  if (too_wide)
    std::fprintf(stderr,
                 "rux-qt-gallery: ERROR TOO WIDE: the page needs %d px, the "
                 "window is %d px\n",
                 min_w, args.width);
  const QImage img = host.grab().toImage();
  const bool saved = img.save(QString::fromStdString(args.screenshot));
  const QStringList missing = theme().missing();
  const QStringList fonts = ensure_bundled_fonts();
  // QString::arg(double) is locale-independent (std::printf is not: the
  // QApplication set the user's locale, and da_DK would print "1,0").
  std::fprintf(
      stderr, "%s\n",
      qPrintable(QString("rux-qt-gallery: %1 %2x%3 (dpr %4) in %5 ms%6")
                     .arg(QString::fromStdString(args.screenshot))
                     .arg(img.width())
                     .arg(img.height())
                     .arg(img.devicePixelRatio(), 0, 'f', 1)
                     .arg(clock.elapsed())
                     .arg(missing.isEmpty() && fonts.isEmpty()
                              ? QString()
                              : QString(" — MISSING TOKENS/FONTS"))));
  if (!saved) {
    std::fprintf(stderr, "rux-qt-gallery: ERROR could not write %s\n",
                 args.screenshot.c_str());
    return 5;
  }
  if (too_wide)
    return 6;
  if (!missing.isEmpty() || !fonts.isEmpty()) {
    for (const QString &m : missing)
      std::fprintf(stderr, "rux-qt-gallery: ERROR MISSING token %s\n",
                   qPrintable(m));
    // A fallback face would make every critique of type meaningless.
    for (const QString &f : fonts)
      std::fprintf(stderr, "rux-qt-gallery: ERROR MISSING font family %s\n",
                   qPrintable(f));
    return 3;
  }
  return 0;
}
