// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/AppShell.hpp>
#include <rux_qt/DatabaseWorkspace.hpp>
#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/EditWorkspace.hpp>
#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/PipelineWorkspace.hpp>
#include <rux_qt/PoseGraphWorkspace.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/RecentProjects.hpp>
#include <rux_qt/StartPage.hpp>
#include <rux_qt/TableBrowser.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/Viewer3DWorkspace.hpp>
#include <rux_qt/palette_table.hpp>
#include <rux_qt/widgets.hpp>

#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QDir>
#include <QDragEnterEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHash>
#include <QLabel>
#include <QLocale>
#include <QMessageBox>
#include <QMimeData>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QTextEdit>
#include <QVBoxLayout>

#include <cmath>

namespace rux::qt {
namespace {

/// The single local .rux file in a drag, or empty.
QString dropped_project(const QMimeData *mime) {
  if (!mime || !mime->hasUrls() || mime->urls().size() != 1)
    return {};
  const QUrl url = mime->urls().front();
  if (!url.isLocalFile())
    return {};
  const QString path = url.toLocalFile();
  return path.endsWith(".rux", Qt::CaseInsensitive) ? path : QString();
}

QString human_size(qint64 bytes) {
  return QLocale(QLocale::Danish, QLocale::Denmark)
      .formattedDataSize(bytes, 1, QLocale::DataSizeSIFormat);
}

/// A read-only mono text block that wraps ANYWHERE and never sets its
/// parent's width: compact JSON or a long path has no word boundary, and a
/// word-wrapped QLabel would widen the inspector to the longest unbroken run
/// (pushing every value off screen). Its height follows the wrapped text.
class TextBlock : public QTextEdit {
    public:
  explicit TextBlock(const QString &text) {
    setObjectName("inspectorBlock");
    setReadOnly(true);
    setPlainText(text);
    setLineWrapMode(QTextEdit::WidgetWidth);
    setWordWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setFrameShape(QFrame::NoFrame);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    setMinimumWidth(0);
    setTextInteractionFlags(Qt::TextSelectableByMouse |
                            Qt::TextSelectableByKeyboard);
  }
  QSize sizeHint() const override { return {0, height()}; }
  QSize minimumSizeHint() const override { return {0, height()}; }

    protected:
  void resizeEvent(QResizeEvent *e) override {
    QTextEdit::resizeEvent(e);
    fit();
  }
  void showEvent(QShowEvent *e) override {
    QTextEdit::showEvent(e);
    fit();
  }

    private:
  void fit() {
    document()->setTextWidth(viewport()->width());
    const int h = static_cast<int>(std::ceil(document()->size().height())) +
                  contentsMargins().top() + contentsMargins().bottom() +
                  2 * frameWidth();
    if (h != height())
      setFixedHeight(h);
  }
};

QPushButton *chrome_button(const QString &text, const QString &kbd = {}) {
  auto *b = new QPushButton;
  b->setObjectName("chromeButton");
  b->setCursor(Qt::PointingHandCursor);
  b->setFocusPolicy(Qt::TabFocus);
  auto *l = new QHBoxLayout(b);
  l->setContentsMargins(theme().px("--space-3"), theme().px("--space-1"),
                        theme().px("--space-3"), theme().px("--space-1"));
  l->setSpacing(theme().px("--space-2"));
  auto *t = new QLabel(text);
  t->setObjectName("chromeButtonText");
  t->setAttribute(Qt::WA_TransparentForMouseEvents);
  l->addWidget(t);
  if (!kbd.isEmpty()) {
    auto *k = new QLabel(kbd);
    k->setObjectName("railKbd");
    k->setAttribute(Qt::WA_TransparentForMouseEvents);
    l->addWidget(k);
  }
  b->setMinimumWidth(l->sizeHint().width());
  b->setMinimumHeight(l->sizeHint().height());
  return b;
}

} // namespace

// ---------------------------------------------------------------- AppShell --

AppShell::AppShell(ProjectSession &session, RecentProjects &recent,
                   ShellOptions options, QWidget *parent)
    : QWidget(parent), session_(session), recent_(recent) {
  setObjectName("appRoot");
  setAcceptDrops(true);
  const Theme &t = theme();

  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  v->addWidget(make_title_bar());

  auto *body = new QHBoxLayout;
  body->setSpacing(0);
  v->addLayout(body, 1);

  // ---- nav rail
  rail_ = new NavRail("Intet projekt");
  const auto &infos = workspace_infos();
  rail_->add_item(infos[0].name);
  rail_->add_group("Data");
  rail_->add_item(infos[1].name);
  rail_->add_item(infos[2].name);
  rail_->add_item(infos[3].name);
  rail_->add_group("Behandling");
  rail_->add_item(infos[4].name);
  rail_->add_item(infos[5].name);
  auto *hint = new QPushButton;
  hint->setObjectName("railPalette");
  hint->setCursor(Qt::PointingHandCursor);
  hint->setFocusPolicy(Qt::NoFocus);
  auto *hl = new QHBoxLayout(hint);
  hl->setContentsMargins(0, t.px("--space-1"), 0, t.px("--space-1"));
  hl->setSpacing(t.px("--space-2"));
  auto *kbd = new QLabel("Ctrl+K");
  kbd->setObjectName("railKbd");
  auto *ht = new QLabel("Kommandopalet");
  ht->setObjectName("railHint");
  for (QLabel *l : {kbd, ht}) {
    l->setAttribute(Qt::WA_TransparentForMouseEvents);
    hl->addWidget(l);
  }
  hl->addStretch(1);
  hint->setMinimumHeight(hl->sizeHint().height());
  connect(hint, &QPushButton::clicked, this, [this] { open_palette(); });
  rail_->add_footer(hint);
  body->addWidget(rail_);

  // ---- workspaces
  stack_ = new QStackedWidget;
  stack_->setObjectName("workspaceStack");
  start_ = new StartPage(session_, recent_);
  stack_->addWidget(start_);
  edges_ = new EdgeEditor(session_, this);
  database_ = new DatabaseWorkspace(session_, *edges_);
  connect(database_, &DatabaseWorkspace::browse_requested, this,
          &AppShell::browse);
  stack_->addWidget(database_);
  viewer_ = new Viewer3DWorkspace(session_, options.interactive_3d);
  stack_->addWidget(viewer_);
  edit_ = new EditWorkspace(session_, *edges_, options.persist_layout);
  stack_->addWidget(edit_);
  pipeline_ = new PipelineWorkspace(session_, options.stage_executor);
  stack_->addWidget(pipeline_);
  log_ = new PipelineLogView(session_, /*filters=*/true);
  log_->setObjectName("logWorkspace");
  stack_->addWidget(log_);
  body->addWidget(stack_, 1);

  // ---- inspector
  inspector_ = new Inspector(session_);
  inspector_->setFixedWidth(t.px("--layout-panel-width"));
  body->addWidget(inspector_);

  palette_ = new CommandPalette(this);
  palette_->set_provider([this] { return commands(); });

  connect(rail_, &NavRail::current_changed, this,
          [this](int i) { show_page(static_cast<Workspace>(i)); });
  // The inspector follows the selection of the workspace on screen.
  connect(database_, &DatabaseWorkspace::selection_changed, this,
          [this](const Selection &sel) {
            if (current_page() == Workspace::database)
              inspector_->set_selection(sel);
          });
  auto follow = [this](Workspace page) {
    return [this, page](const Selection &sel) {
      if (current_page() == page)
        inspector_->set_selection(sel);
    };
  };
  connect(viewer_, &Viewer3DWorkspace::selection_changed, this,
          follow(Workspace::viewer3d));
  connect(edit_, &EditWorkspace::selection_changed, this,
          follow(Workspace::edit));
  connect(pipeline_, &PipelineWorkspace::selection_changed, this,
          follow(Workspace::pipeline));
  connect(log_, &PipelineLogView::selection_changed, this,
          [this](const Selection &sel) {
            log_selection_ = sel;
            if (current_page() == Workspace::log)
              inspector_->set_selection(sel);
          });
  // A pipeline run changed the project: re-read it, unless that would drop
  // pose-graph edits nobody has saved (then F5 does it when they are).
  connect(pipeline_, &PipelineWorkspace::project_changed, this, [this] {
    if (edit_->pending_edits() == 0 && !edit_->is_saving())
      session_.reload();
  });
  connect(&session_, &ProjectSession::opened, log_, &PipelineLogView::reload);
  // While a stage runs, pose-graph saves wait (they take the runner's
  // writer lease too, so a stage started meanwhile still wins cleanly).
  edges_->set_runner_source([this] { return pipeline_->runner_handle(); });
  pipeline_->set_pending_edits_source(
      [this] { return edit_->pending_edits(); });
  connect(pipeline_, &PipelineWorkspace::job_started, this,
          [this](const QString &stage) { edges_->set_pipeline_busy(stage); });
  connect(pipeline_, &PipelineWorkspace::job_finished, this, [this] {
    edges_->set_pipeline_busy({});
    if (after_job_) {
      auto next = std::move(after_job_);
      after_job_ = nullptr;
      next();
    }
  });
  connect(&session_, &ProjectSession::closed, log_, &PipelineLogView::reload);
  connect(start_, &StartPage::browse_requested, this, &AppShell::browse);
  connect(start_, &StartPage::open_requested, this, &AppShell::open_project);
  connect(start_, &StartPage::navigate_requested, this,
          [this](int i) { show_page(static_cast<Workspace>(i)); });
  connect(inspector_, &Inspector::hide_requested, this,
          [this] { set_inspector_visible(false); });
  connect(&session_, &ProjectSession::state_changed, this,
          &AppShell::sync_project);
  connect(&theme(), &Theme::changed, this, &AppShell::refresh_product_label);

  build_actions();
  set_inspector_visible(true);
  show_page(Workspace::start);
  sync_project();
}

QWidget *AppShell::make_title_bar() {
  const Theme &t = theme();
  auto *bar = new QFrame;
  bar->setObjectName("titleBar");
  bar->setFixedHeight(t.px("--layout-titlebar-height"));
  auto *l = new QHBoxLayout(bar);
  l->setContentsMargins(t.px("--space-4"), 0, t.px("--space-3"), 0);
  l->setSpacing(t.px("--space-3"));

  product_ = new QLabel;
  product_->setObjectName("product");
  product_->setTextFormat(Qt::RichText);
  refresh_product_label();
  l->addWidget(product_);

  auto *divider = new QFrame;
  divider->setObjectName("titleDivider");
  divider->setFixedSize(1, t.px("--space-4"));
  l->addWidget(divider);

  title_project_ = new QLabel;
  title_project_->setObjectName("titleProject");
  l->addWidget(title_project_);
  title_path_ = new ElidedLabel({}, Qt::ElideMiddle);
  title_path_->setObjectName("titleMeta");
  l->addWidget(title_path_, 1);

  schema_pill_ = new Pill({}, "good");
  access_pill_ = new Pill({}, "warn");
  l->addWidget(schema_pill_);
  l->addWidget(access_pill_);

  auto *search = chrome_button("Søg", "Ctrl+K");
  connect(search, &QPushButton::clicked, this, [this] { open_palette(); });
  l->addWidget(search);
  inspector_button_ = chrome_button("Inspektør");
  inspector_button_->setCheckable(true);
  inspector_button_->setProperty("toggle", true); // muted text when off
  connect(inspector_button_, &QPushButton::toggled, this,
          [this](bool on) { set_inspector_visible(on); });
  l->addWidget(inspector_button_);
  return bar;
}

void AppShell::refresh_product_label() {
  // "ReUseX" with the X in the accent, as the web title bar. Rich text needs
  // the colour value itself, so it is read from the token on every theme.
  if (product_)
    product_->setText(QString("REUSE<span style=\"color:%1\">X</span>")
                          .arg(theme().color("--color-accent").name()));
}

void AppShell::build_actions() {
  auto make = [this](const QString &text, const QKeySequence &key,
                     auto &&slot) {
    auto *a = new QAction(NavItem::escape_mnemonic(text), this);
    a->setShortcut(key);
    a->setShortcutContext(Qt::WindowShortcut);
    connect(a, &QAction::triggered, this, slot);
    addAction(a);
    return a;
  };
  auto key = [](const char *id) {
    return QKeySequence(QString::fromStdString(action_shortcut(id)));
  };
  open_action_ = make("Åbn projekt…", key("open"), [this] { browse(); });
  close_action_ = make("Luk projekt", key("close"), [this] {
    if (resolve_running_job("lukke projektet",
                            [this] { close_action_->trigger(); }) &&
        resolve_pending_edits("lukke projektet"))
      session_.close();
  });
  reload_action_ = make("Genindlæs projekt", key("reload"), [this] {
    if (resolve_running_job("genindlæse projektet",
                            [this] { reload_action_->trigger(); }) &&
        resolve_pending_edits("genindlæse projektet"))
      session_.reload();
  });
  inspector_action_ =
      make("Vis eller skjul inspektør", key("inspector"),
           [this] { set_inspector_visible(!inspector_visible()); });
  palette_action_ = make("Kommandopalet", QKeySequence(Qt::CTRL | Qt::Key_K),
                         [this] { open_palette(); });
  quit_action_ = make("Afslut", key("quit"), [this] { emit quit_requested(); });
  theme_action_ = make("Skift tema", key("theme"),
                       [this] { emit toggle_theme_requested(); });
  // Alt+1 … Alt+6 jump to a page, in rail order.
  for (int i = 0; i < kWorkspaceCount; ++i)
    make(workspace_infos()[i].name, QKeySequence(Qt::ALT | (Qt::Key_1 + i)),
         [this, i] { show_page(static_cast<Workspace>(i)); });
}

QVector<Command> AppShell::commands() {
  // The table itself is Qt-free (rux_qt/palette_table.hpp) so the ranking
  // tests rank exactly what is shown; here each id gets its action.
  PaletteState st;
  st.project_open = session_.is_open();
  st.project_failed = session_.state() == ProjectSession::State::failed;
  st.project_path = session_.path().toStdString();
  st.current_page = static_cast<int>(current_page());
  st.inspector_visible = inspector_visible();
  st.dark_theme = theme().mode() == ThemeMode::dark;
  for (const auto &e : recent_.entries())
    st.recent.push_back({e.path.toStdString(), e.missing});
  // "Kopiér som rux-kommando" copies what is on screen: the Pipeline form,
  // or the selected run in the Log.
  if (current_page() == Workspace::pipeline && session_.is_open())
    st.cli_command = pipeline_->command().text;
  else if (current_page() == Workspace::log)
    for (const auto &sec : log_selection_.sections)
      if (sec.title == "Som rux-kommando")
        st.cli_command = sec.block.toStdString();

  const QHash<QString, QAction *> actions = {
      {"open", open_action_},   {"reload", reload_action_},
      {"close", close_action_}, {"inspector", inspector_action_},
      {"theme", theme_action_}, {"quit", quit_action_}};
  QVector<Command> out;
  for (const PaletteEntry &e : build_palette(st)) {
    Command c;
    c.group = QString::fromStdString(e.group);
    c.title = QString::fromStdString(e.title);
    c.keywords = QString::fromStdString(e.keywords);
    c.subtitle = QString::fromStdString(e.subtitle);
    c.shortcut = QString::fromStdString(e.shortcut);
    c.badge = QString::fromStdString(e.badge);
    c.enabled = e.enabled;
    const QString id = QString::fromStdString(e.id);
    if (id.startsWith("page:")) {
      const int i = id.mid(5).toInt();
      c.run = [this, i] { show_page(static_cast<Workspace>(i)); };
    } else if (id.startsWith("recent:")) {
      const QString p = id.mid(7);
      c.run = [this, p] { open_project(p); };
    } else if (id == "copy-path" || id == "copy-cli") {
      const QString text = c.subtitle;
      c.run = [text] { QApplication::clipboard()->setText(text); };
    } else if (id.startsWith("pane:")) {
      if (const auto p = parse_edit_pane(id.mid(5).toStdString()))
        c.run = [this, p = *p] {
          show_page(Workspace::edit);
          edit_->toggle_pane(p);
        };
    } else if (QAction *a = actions.value(id)) {
      c.run = [a] { a->trigger(); };
    }
    out << c;
  }
  return out;
}

void AppShell::open_palette(const QString &query) {
  palette_->open_palette(query);
}

void AppShell::show_page(Workspace w) {
  const int i = static_cast<int>(w);
  rail_->set_current(i);
  stack_->setCurrentIndex(i);
  // Pages load lazily: a million-point cloud is read when 3D is first shown.
  if (w == Workspace::viewer3d)
    viewer_->activate();
  else if (w == Workspace::edit)
    edit_->activate();
  inspector_->set_selection(page_selection(w));
}

Selection AppShell::page_selection(Workspace w) const {
  switch (w) {
  case Workspace::database:
    return database_->selection();
  case Workspace::viewer3d:
    return viewer_->selection();
  case Workspace::edit:
    return edit_->selection();
  case Workspace::pipeline:
    return pipeline_->selection();
  case Workspace::log:
    return log_selection_;
  case Workspace::start:
    break;
  }
  return {};
}

bool AppShell::resolve_pending_edits(const QString &action) {
  if (!edit_)
    return true;
  // A save already on its way: let it land first (and report a failure).
  if (edit_->is_saving() && !edit_->save_edits_and_wait()) {
    show_page(Workspace::edit);
    return false;
  }
  if (edit_->pending_edits() == 0)
    return true;
  const int n = edit_->pending_edits();
  QMessageBox box(this);
  box.setIcon(QMessageBox::Question);
  box.setWindowTitle("Ændringer er ikke gemt");
  box.setText(
      QString("%1 i posegrafen er ikke gemt.")
          .arg(n == 1 ? QString("1 ændring") : QString("%1 ændringer").arg(n)));
  box.setInformativeText(
      QString("Vil du gemme, før du vælger at %1?").arg(action));
  auto *save = box.addButton(NavItem::escape_mnemonic("Gem ændringer"),
                             QMessageBox::AcceptRole);
  auto *discard = box.addButton(NavItem::escape_mnemonic("Kassér"),
                                QMessageBox::DestructiveRole);
  auto *cancel = box.addButton(NavItem::escape_mnemonic("Annullér"),
                               QMessageBox::RejectRole);
  box.setDefaultButton(save);
  box.setEscapeButton(cancel);
  box.exec();
  if (box.clickedButton() == save) {
    if (edit_->save_edits_and_wait())
      return true;
    // The banner says why; stay so the user can act on it.
    show_page(Workspace::edit);
    return false;
  }
  if (box.clickedButton() == discard) {
    edit_->discard_edits();
    return true;
  }
  return false;
}

bool AppShell::resolve_running_job(const QString &action,
                                   std::function<void()> retry) {
  if (!pipeline_ || !pipeline_->is_running())
    return true;
  const QString stage = pipeline_->running_stage();
  QMessageBox box(this);
  box.setIcon(QMessageBox::Question);
  box.setWindowTitle("Et trin kører");
  box.setText(QString("%1 kører stadig på projektet.").arg(stage));
  box.setInformativeText(
      QString("Vil du stoppe kørslen og derefter %1? Trinnet stopper ved sit "
              "næste kontrolpunkt; en MIP-løsning kan tage tid om det. "
              "Vinduet kan bruges imens.")
          .arg(action));
  auto *stop = box.addButton(NavItem::escape_mnemonic("Stop kørslen"),
                             QMessageBox::DestructiveRole);
  auto *keep = box.addButton(NavItem::escape_mnemonic("Fortsæt kørslen"),
                             QMessageBox::RejectRole);
  box.setDefaultButton(keep);
  box.setEscapeButton(keep);
  box.exec();
  if (box.clickedButton() == stop) {
    after_job_ = std::move(retry);
    pipeline_->cancel_running();
    show_page(Workspace::pipeline);
  }
  return false;
}

Workspace AppShell::current_page() const {
  return static_cast<Workspace>(stack_->currentIndex());
}

void AppShell::set_inspector_visible(bool on) {
  const bool changed = inspector_->isHidden() == on;
  inspector_->setVisible(on);
  inspector_button_->blockSignals(true);
  inspector_button_->setChecked(on);
  inspector_button_->blockSignals(false);
  if (changed)
    emit inspector_toggled(on);
}

bool AppShell::inspector_visible() const { return !inspector_->isHidden(); }

void AppShell::open_project(const QString &path, bool read_only) {
  if (path.isEmpty() ||
      !resolve_running_job(
          "åbne et andet projekt",
          [this, path, read_only] { open_project(path, read_only); }) ||
      !resolve_pending_edits("åbne et andet projekt"))
    return;
  // Only an existing file goes into the recent list; one that fails to open
  // stays (it may be locked) — the start page shows why.
  if (QFileInfo(path).isFile())
    recent_.add(QFileInfo(path).absoluteFilePath());
  session_.open(path, read_only);
  if (current_page() != Workspace::start)
    show_page(Workspace::start);
}

void AppShell::browse() {
  QString dir;
  if (!session_.path().isEmpty())
    dir = QFileInfo(session_.path()).absolutePath();
  else if (!recent_.paths().isEmpty())
    dir = QFileInfo(recent_.paths().front()).absolutePath();
  const QString path = QFileDialog::getOpenFileName(
      this, "Åbn projekt", dir, "ReUseX-projekter (*.rux);;Alle filer (*)");
  if (!path.isEmpty())
    open_project(path);
}

void AppShell::sync_project() {
  const auto state = session_.state();
  const bool open = state == ProjectSession::State::open;
  const QString name = session_.display_name();

  switch (state) {
  case ProjectSession::State::empty:
    title_project_->setText("Intet projekt");
    rail_->set_project_name("Intet projekt");
    break;
  case ProjectSession::State::loading:
    title_project_->setText(QFileInfo(session_.path()).fileName());
    rail_->set_project_name("Åbner …");
    break;
  case ProjectSession::State::failed:
    title_project_->setText(QFileInfo(session_.path()).fileName());
    rail_->set_project_name("Intet projekt");
    break;
  case ProjectSession::State::open:
    title_project_->setText(QFileInfo(session_.path()).fileName());
    rail_->set_project_name(name);
    break;
  }
  QString dir;
  if (!session_.path().isEmpty())
    dir = QFileInfo(session_.path()).absolutePath();
  static_cast<ElidedLabel *>(title_path_)->set_full_text(dir);

  // Status pills: the schema and the access mode, or the load state.
  schema_pill_->setVisible(state != ProjectSession::State::empty);
  access_pill_->setVisible(open && session_.is_read_only());
  if (state == ProjectSession::State::loading) {
    schema_pill_->setText("Åbner …");
    schema_pill_->setProperty("tone", "wait");
  } else if (state == ProjectSession::State::failed) {
    schema_pill_->setText("Kunne ikke åbnes");
    schema_pill_->setProperty("tone", "crit");
  } else if (open) {
    const int v = session_.summary().schema_version;
    const bool old = v < reusex::ProjectDB::latest_schema_version();
    schema_pill_->setText(old ? QString("Skema v%1 · ældre").arg(v)
                              : QString("Skema v%1").arg(v));
    schema_pill_->setProperty("tone", old ? "warn" : "good");
    schema_pill_->setToolTip(
        old ? QString("Denne rux bruger skema v%1. Åbn projektet med "
                      "skriveadgang for at migrere det.")
                  .arg(reusex::ProjectDB::latest_schema_version())
            : QString("Projektet bruger det nyeste skema."));
  }
  access_pill_->setText("Skrivebeskyttet");
  access_pill_->setToolTip(session_.read_only_reason());
  repolish(schema_pill_);

  // Nav counts.
  const auto &s = session_.summary();
  rail_->item(static_cast<int>(Workspace::database))
      ->set_count(open ? format_count(static_cast<qulonglong>(
                             s.sensor_frames.total_count))
                       : QString());
  rail_->item(static_cast<int>(Workspace::viewer3d))
      ->set_count(open ? QString::number(s.clouds.size()) : QString());

  close_action_->setEnabled(open);
  reload_action_->setEnabled(open || state == ProjectSession::State::failed);
  inspector_->refresh();
  window()->setWindowTitle(open ? QString("%1 — ReUseX").arg(name)
                                : QString("ReUseX"));
}

void AppShell::dragEnterEvent(QDragEnterEvent *e) {
  if (dropped_project(e->mimeData()).isEmpty())
    return;
  e->acceptProposedAction();
  start_->set_drop_active(true);
}

void AppShell::dragLeaveEvent(QDragLeaveEvent *) {
  start_->set_drop_active(false);
}

void AppShell::dropEvent(QDropEvent *e) {
  start_->set_drop_active(false);
  const QString path = dropped_project(e->mimeData());
  if (path.isEmpty())
    return;
  e->acceptProposedAction();
  open_project(path);
}

// --------------------------------------------------------------- Inspector --

Inspector::Inspector(ProjectSession &session, QWidget *parent)
    : QFrame(parent), session_(session) {
  setObjectName("inspector");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);

  auto *head = new QWidget;
  head->setObjectName("inspectorHead");
  auto *h = new QHBoxLayout(head);
  h->setContentsMargins(t.px("--space-4"), t.px("--space-3"), t.px("--space-2"),
                        t.px("--space-3"));
  h->addWidget(new CapsLabel("Inspektør", "eyebrowSurface", "--tracking-wide"),
               1);
  auto *hide = new QPushButton("Skjul");
  hide->setProperty("kind", "ghost");
  hide->setObjectName("inspectorHide");
  hide->setCursor(Qt::PointingHandCursor);
  hide->setToolTip("Skjul inspektøren (Ctrl+I)");
  connect(hide, &QPushButton::clicked, this, &Inspector::hide_requested);
  h->addWidget(hide);
  v->addWidget(head);

  auto *scroll = new QScrollArea;
  scroll->setObjectName("inspectorScroll");
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  body_ = new QWidget;
  body_->setObjectName("inspectorBody");
  new QVBoxLayout(body_);
  scroll->setWidget(body_);
  v->addWidget(scroll, 1);
  refresh();
}

void Inspector::refresh() {
  const Theme &t = theme();
  auto *l = static_cast<QVBoxLayout *>(body_->layout());
  while (QLayoutItem *it = l->takeAt(0)) {
    if (QWidget *w = it->widget()) {
      w->hide(); // see StartPage: a pending deleteLater still paints
      w->deleteLater();
    }
    delete it;
  }
  l->setContentsMargins(t.px("--space-4"), t.px("--space-1"), t.px("--space-4"),
                        t.px("--space-4"));
  l->setSpacing(t.px("--space-3"));
  if (session_.is_open() && !selection_.empty())
    show_selection();
  else
    show_project();
}

void Inspector::set_selection(const Selection &selection) {
  selection_ = selection;
  refresh();
}

void Inspector::show_selection() {
  const Theme &t = theme();
  auto *l = static_cast<QVBoxLayout *>(body_->layout());
  const Selection &sel = selection_;

  auto *head = new QWidget;
  head->setObjectName("selectionHead");
  auto *hv = new QVBoxLayout(head);
  hv->setContentsMargins(0, t.px("--space-2"), 0, 0);
  hv->setSpacing(t.px("--space-1"));
  auto *top = new QHBoxLayout;
  top->setSpacing(t.px("--space-2"));
  if (!sel.kind.isEmpty())
    top->addWidget(new CapsLabel(sel.kind, "sectionLabel"));
  top->addStretch(1);
  if (!sel.pill.isEmpty())
    top->addWidget(new Pill(sel.pill, sel.pill_tone));
  hv->addLayout(top);
  auto *title = new QLabel(sel.title);
  title->setObjectName("selectionTitle");
  title->setWordWrap(true);
  title->setTextInteractionFlags(Qt::TextSelectableByMouse);
  hv->addWidget(title);
  if (!sel.subtitle.isEmpty()) {
    auto *sub = new QLabel(sel.subtitle);
    sub->setObjectName("selectionSub");
    sub->setWordWrap(true);
    hv->addWidget(sub);
  }
  l->addWidget(head);
  if (!sel.preview.isNull()) {
    auto *img = new QLabel;
    img->setObjectName("selectionPreview");
    const int w = t.px("--layout-panel-width") - 2 * t.px("--space-4");
    QPixmap pm = QPixmap::fromImage(sel.preview.scaledToWidth(
        static_cast<int>(w * devicePixelRatioF()), Qt::SmoothTransformation));
    pm.setDevicePixelRatio(devicePixelRatioF());
    img->setPixmap(pm);
    l->addWidget(img);
  }
  for (const SelectionSection &sec : sel.sections) {
    if (!sec.title.isEmpty()) {
      auto *c = new CapsLabel(sec.title, "sectionLabel");
      c->setContentsMargins(0, t.px("--space-2"), 0, 0);
      l->addWidget(c);
    }
    if (!sec.rows.isEmpty()) {
      auto *p = new PropertyList;
      for (const SelectionRow &r : sec.rows) {
        if (!r.swatch_token.isEmpty())
          p->add_swatch(r.swatch_token, r.key, r.value);
        else if (r.style == SelectionRow::Style::name)
          p->add_name(r.key, r.value);
        else
          p->add(r.key, r.value, r.style == SelectionRow::Style::mono);
      }
      l->addWidget(p);
    }
    if (!sec.block.isEmpty()) {
      if (sec.command)
        l->addWidget(new CommandBlock(sec.block));
      else
        l->addWidget(new TextBlock(sec.block));
    }
  }
  l->addStretch(1);
}

void Inspector::show_project() {
  const Theme &t = theme();
  auto *l = static_cast<QVBoxLayout *>(body_->layout());
  auto section = [&](const QString &title) {
    auto *c = new CapsLabel(title, "sectionLabel");
    c->setContentsMargins(0, t.px("--space-2"), 0, 0);
    l->addWidget(c);
  };

  if (!session_.is_open()) {
    auto *e = new QLabel(session_.state() == ProjectSession::State::loading
                             ? QString("Åbner projektet …")
                             : QString("Intet valgt. Åbn et projekt, eller "
                                       "vælg et element i et arbejdsområde "
                                       "for at se dets egenskaber her."));
    e->setObjectName("emptyText");
    e->setWordWrap(true);
    l->addWidget(e);
    l->addStretch(1);
    return;
  }

  const auto &s = session_.summary();
  const QFileInfo fi(session_.path());
  section("Projekt");
  auto *p = new PropertyList;
  p->add("Fil", fi.fileName(), false);
  p->add("Størrelse", human_size(fi.size()));
  p->add("Skema", QString("v%1").arg(s.schema_version));
  p->add("Adgang", session_.is_read_only() ? "Skrivebeskyttet" : "Læs og skriv",
         false);
  p->add("Åbningstid", QString("%1 ms").arg(session_.load_ms()));
  l->addWidget(p);

  section("Indhold");
  auto *c = new PropertyList;
  c->add("Billeder",
         format_count(static_cast<qulonglong>(s.sensor_frames.total_count)));
  if (s.sensor_frames.width > 0)
    c->add("Opløsning", QString("%1 × %2")
                            .arg(s.sensor_frames.width)
                            .arg(s.sensor_frames.height));
  c->add("Scanninger", QString::number(s.sensor_frames.scans.size()));
  c->add("Segmenteret", format_count(static_cast<qulonglong>(
                            s.sensor_frames.segmented_count)));
  c->add("Panoramaer", QString::number(s.panoramic_images.total_count));
  c->add("Mesh", QString::number(s.meshes.size()));
  c->add("Ressourcer", QString::number(s.materials.size()));
  l->addWidget(c);

  if (!s.clouds.empty()) {
    section("Punktskyer");
    auto *k = new PropertyList;
    for (const auto &cl : s.clouds)
      k->add_name(QString::fromStdString(cl.name),
                  format_count(static_cast<qulonglong>(cl.point_count)));
    l->addWidget(k);
  }
  l->addStretch(1);
}

} // namespace rux::qt
