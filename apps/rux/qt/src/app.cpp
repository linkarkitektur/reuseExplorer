// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/AppShell.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/RecentProjects.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/app.hpp>
#include <rux_qt/fonts.hpp>

#include <reusex/core/logging.hpp>

#include <QApplication>
#include <QCloseEvent>
#include <QFileInfo>
#include <QMainWindow>
#include <QScreen>
#include <QSettings>
#include <QSurfaceFormat>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>

#include <clocale>
#include <cstdio>

namespace rux::qt {
namespace {

constexpr const char *kThemeKey = "theme";
constexpr const char *kGeometryKey = "mainWindow/geometry";
constexpr const char *kInspectorKey = "mainWindow/inspector";

/// The window: the shell plus what only a top-level window has — geometry
/// and the settings it remembers between runs.
class MainWindow : public QMainWindow {
    public:
  MainWindow(ProjectSession &session, RecentProjects &recent,
             ShellOptions options) {
    setObjectName("mainWindow");
    shell_ = new AppShell(session, recent, std::move(options), this);
    setCentralWidget(shell_);
    setWindowTitle("ReUseX");
    setMinimumSize(theme().px("--layout-nav-width") * 4,
                   theme().px("--layout-panel-width") * 2);

    QSettings s;
    if (!restoreGeometry(s.value(kGeometryKey).toByteArray())) {
      QSize size(1440, 900); // the design reference size (qt_shot.sh)
      if (const QScreen *scr = QGuiApplication::primaryScreen())
        size = size.boundedTo(scr->availableSize() * 9 / 10);
      resize(size);
    }
    shell_->set_inspector_visible(s.value(kInspectorKey, true).toBool());
    connect(shell_, &AppShell::inspector_toggled, this,
            [](bool on) { QSettings().setValue(kInspectorKey, on); });
    connect(shell_, &AppShell::quit_requested, this, &QWidget::close);
  }

  AppShell *shell() const { return shell_; }

    protected:
  void closeEvent(QCloseEvent *e) override {
    // Unsaved pose-graph edits: save, discard or stay (RTABMap writes its
    // pending link edits on close too, but asks nothing).
    // A pipeline run: stop it and close when it has ended, never by joining
    // it here.
    if (!shell_->resolve_running_job("afslutte", [this] { close(); }) ||
        !shell_->resolve_pending_edits("afslutte")) {
      e->ignore();
      return;
    }
    QSettings().setValue(kGeometryKey, saveGeometry());
    QMainWindow::closeEvent(e);
  }

    private:
  AppShell *shell_ = nullptr;
};

const char *state_name(ProjectSession::State s) {
  switch (s) {
  case ProjectSession::State::empty:
    return "empty";
  case ProjectSession::State::loading:
    return "loading";
  case ProjectSession::State::open:
    return "open";
  case ProjectSession::State::failed:
    return "failed";
  }
  return "?";
}

} // namespace

int run_app(int argc, char **argv, const AppOptions &options) {
  // Before the QApplication: every QVTKOpenGLNativeWidget (the 3D
  // workspace, Q3) needs this default format, and it cannot be set later.
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

  // Also before the QApplication: QFileDialog::Options::DontUseNativeDialog
  // passed to a single call is too late to help, since QFileDialogPrivate::
  // init() decides whether to build the native (here: GTK3) helper before
  // the dialog's own setOptions(args.options) ever runs. This attribute is
  // consulted by canBeNativeDialog() from the very first dialog onward, so
  // native file dialogs are off for the whole process from the start. Needed
  // because the nix devshell/package never puts GTK3's own GSettings schemas
  // on XDG_DATA_DIRS, and constructing the native GTK3 file chooser aborts
  // the process (g_error) the instant it looks one up.
  QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs);

  int qt_argc = 1; // rux's own flags are not Qt's to parse
  char *qt_argv[] = {argv[0], nullptr};
  (void)argc;
  QApplication app(qt_argc, qt_argv);
  // QApplication just called setlocale(LC_ALL, ""). Under a comma-decimal
  // locale (da_DK) every strtod/stod/sscanf in the library and its
  // dependencies (sqlite text, JSON, OpenCV, PCL) would read "799.85" as 799.
  // Numbers shown to the user go through QLocale, so the C library's numeric
  // locale stays "C" (the Qt docs recommend exactly this).
  std::setlocale(LC_NUMERIC, "C");
  QApplication::setOrganizationName("ReUseX");
  QApplication::setApplicationName("rux");
  QApplication::setApplicationDisplayName("ReUseX");

  ensure_bundled_fonts();
  // The Pipeline workspace tails a running stage's info lines through the
  // log tap; rux starts the library at warn (its terminal level stays so).
  if (reusex::core::get_log_level() > reusex::core::LogLevel::info)
    reusex::core::set_log_level(reusex::core::LogLevel::info);
  QSettings settings;
  const ThemeMode mode = settings.value(kThemeKey, "dark").toString() == "light"
                             ? ThemeMode::light
                             : ThemeMode::dark;
  // Hot reload from the source tree in a Debug build or with RUX_QT_DEV=1;
  // otherwise the stylesheet snapshot compiled into the binary.
#ifdef NDEBUG
  const bool dev = qEnvironmentVariableIntValue("RUX_QT_DEV") == 1;
#else
  const bool dev = true;
#endif
  const Theme::Source source =
      dev ? Theme::dev_source() : Theme::embedded_source();
  theme().apply(mode, source);

  ProjectSession session;
  RecentProjects *recent = RecentProjects::from_settings(&app);
  ShellOptions shell_options;
  shell_options.interactive_3d = true;
  shell_options.stage_executor = options.stage_executor;
  MainWindow window(session, *recent, shell_options);

  QObject::connect(window.shell(), &AppShell::toggle_theme_requested, &app,
                   [source] {
                     const ThemeMode next = theme().mode() == ThemeMode::dark
                                                ? ThemeMode::light
                                                : ThemeMode::dark;
                     theme().apply(next, source);
                     QSettings().setValue(
                         kThemeKey, next == ThemeMode::dark ? "dark" : "light");
                   });

  const bool report = options.quit_after_ms >= 0;
  if (report) {
    QObject::connect(&session, &ProjectSession::opened, &app, [&session] {
      std::fprintf(stderr, "rux: project open: %s (schema v%d, %lld ms%s)\n",
                   qPrintable(session.path()), session.summary().schema_version,
                   static_cast<long long>(session.load_ms()),
                   session.is_read_only() ? ", read-only" : "");
    });
    QObject::connect(&session, &ProjectSession::open_failed, &app, [&session] {
      std::fprintf(stderr, "rux: project failed: %s: %s — %s\n",
                   qPrintable(session.path()),
                   qPrintable(session.error().title),
                   qPrintable(session.error().detail));
    });
  }

  window.show();
  if (!options.project.isEmpty())
    window.shell()->open_project(options.project);
  // Smoke tests: RUX_QT_PAGE=database|3d|posegraf|pipeline|log lands on
  // that workspace once the project is open, so `--quit-after-ms` exercises
  // it for real.
  const QString smoke_page = qEnvironmentVariable("RUX_QT_PAGE").toLower();
  const QStringList smoke_pages = {"start",    "database", "3d",
                                   "posegraf", "pipeline", "log"};
  if (const int page = static_cast<int>(smoke_pages.indexOf(smoke_page));
      page > 0)
    QObject::connect(&session, &ProjectSession::opened, &window,
                     [&window, page] {
                       window.shell()->show_page(static_cast<Workspace>(page));
                     });

  if (report) {
    std::fprintf(stderr, "rux: main window shown (%dx%d, platform %s)\n",
                 window.width(), window.height(),
                 qPrintable(QGuiApplication::platformName()));
    QTimer::singleShot(options.quit_after_ms, &app, [&] {
      std::fprintf(stderr,
                   "rux: quitting after %d ms; window visible=%d, page=%d, "
                   "project state=%s\n",
                   options.quit_after_ms, window.isVisible() ? 1 : 0,
                   static_cast<int>(window.shell()->current_page()),
                   state_name(session.state()));
      window.close();
      app.quit();
    });
  }
  const int rc = app.exec();
  // The window is gone by now. An open still waiting on a locked project
  // (ProjectSession never joins its worker) gets a bounded grace period so it
  // can close its connection and finish logging before rux tears down the
  // logger; the user sees why the process is still alive.
  if (ProjectSession::opens_in_flight() > 0) {
    std::fprintf(stderr, "rux: venter på at et projekt bliver færdigt med at "
                         "åbne eller en ICP-kørsel (højst 8 s) …\n");
    if (!ProjectSession::wait_for_opens(8000))
      std::fprintf(stderr, "rux: afslutter uden at vente længere\n");
  }
  return rc;
}

} // namespace rux::qt
