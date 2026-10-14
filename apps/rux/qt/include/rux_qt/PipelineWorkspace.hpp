// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Pipeline workspace (Stream Q, Q3): run a stage in-process.
//
// Left, the runnable stages (pipeline::job_stage_names()) with whether their
// inputs are there (core::validate_stage) and how the last run went
// (pipeline_log). Centre, a parameter form generated from
// pipeline::stage_parameters() — only keys the user changed are sent, and a
// presence-sensitive threshold is sent only when pinned (#214) — a "Kør"
// button, progress, "Annullér" when the stage can stop mid-run, the job's
// log tail, and "Kopiér som rux-kommando": the exact CLI line for the same
// run.
//
// Jobs run on a pipeline::JobRunner (its own worker thread and read-write
// ProjectDB); its events and the log tap (rux_qt/workspace_logic.hpp) are
// posted to the GUI thread.

#include <rux_qt/cli_command.hpp>
#include <rux_qt/selection.hpp>

#include <reusex/pipeline/JobRunner.hpp>
#include <reusex/pipeline/stage_parameters.hpp>

#include <QJsonObject>
#include <QWidget>

#include <functional>
#include <memory>
#include <optional>
#include <vector>

class QCheckBox;
class QGridLayout;
class QLabel;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QScrollArea;
class QTimer;
class QVBoxLayout;

namespace rux::qt {

class BackgroundWork;
class CommandBlock;
class Pill;
class ProjectSession;

class PipelineWorkspace : public QWidget {
  Q_OBJECT
    public:
  PipelineWorkspace(ProjectSession &session,
                    reusex::pipeline::StageExecutor executor,
                    QWidget *parent = nullptr);
  ~PipelineWorkspace() override;

  void select_stage(reusex::pipeline::JobStage stage);
  reusex::pipeline::JobStage stage() const { return stage_; }
  /// The form's parameters as a JSON object (only what was changed).
  QJsonObject parameters() const;
  /// The equivalent `rux …` line of the form as it stands.
  CliCommand command() const;
  /// Set one form field (the gallery, tests): the key must exist.
  bool set_field(const QString &key, const QVariant &value);
  /// Start the selected stage. False when it cannot run (and why is shown).
  bool run();
  /// Select @p stage and start it with the form's values (Rediger's "Find
  /// kanter" / "Optimer"). False when it did not start.
  bool run_stage(reusex::pipeline::JobStage stage);
  /// Why the selected stage cannot start now (Danish), or empty when it can
  /// or a job is running.
  QString blocked_reason() const;
  bool is_running() const;
  /// Danish name of the running stage, or empty.
  QString running_stage() const;
  /// Ask the running job to stop (it ends at its next checkpoint).
  void cancel_running();
  /// The runner, for a writer lease (EdgeEditor); empty when none exists.
  std::weak_ptr<reusex::pipeline::JobRunner> runner_handle() const {
    return runner_;
  }
  /// Unsaved pose-graph edits a run of `optimize` would not see.
  void set_pending_edits_source(std::function<int()> source) {
    pending_edits_ = std::move(source);
  }
  const Selection &selection() const { return selection_; }

    signals:
  void selection_changed(const rux::qt::Selection &selection);
  /// A run wrote to the project: views should re-read it.
  void project_changed();
  void job_started(const QString &stage);
  /// The job ended (any outcome).
  void job_finished();
  /// Just before job_finished, for a job on the project still open: how it
  /// went, with the stage's result summary (ok) or its error.
  void job_result(reusex::pipeline::JobStage stage, bool ok,
                  const QString &message);

    private:
  struct Field;
  struct LogBuffer;
  void reset();
  void rebuild_stages();
  void rebuild_form();
  void refresh_command();
  void refresh_readiness();
  void on_event(const reusex::pipeline::JobEvent &event);
  void drain_log();
  reusex::pipeline::JobRunner &runner();

  ProjectSession &session_;
  reusex::pipeline::StageExecutor executor_;
  reusex::pipeline::JobStage stage_ = reusex::pipeline::JobStage::planes;
  std::shared_ptr<reusex::pipeline::JobRunner> runner_;
  std::function<int()> pending_edits_;
  /// The project the running (or last) job ran on: its result belongs to it,
  /// whatever is open now.
  QString job_path_;
  std::string runner_path_;
  QString form_path_; ///< the project the form was built for
  std::size_t log_token_ = 0;
  std::shared_ptr<LogBuffer> log_;
  QTimer *log_timer_ = nullptr;
  std::string job_id_;
  /// Counted while a job runs, so a screenshot waits for it to finish.
  std::unique_ptr<BackgroundWork> busy_;
  reusex::pipeline::JobStage job_stage_ = reusex::pipeline::JobStage::planes;

  QWidget *stages_ = nullptr;
  QScrollArea *scroll_ = nullptr;
  QWidget *job_panel_ = nullptr;
  QLabel *title_ = nullptr;
  QLabel *blurb_ = nullptr;
  QLabel *ready_ = nullptr;
  QWidget *form_ = nullptr;
  QGridLayout *form_grid_ = nullptr;
  std::vector<std::unique_ptr<Field>> fields_;
  QPushButton *run_ = nullptr;
  QPushButton *cancel_ = nullptr;
  QPushButton *defaults_ = nullptr;
  CommandBlock *command_ = nullptr;
  QLabel *command_note_ = nullptr;
  QPushButton *copy_ = nullptr;
  QProgressBar *progress_ = nullptr;
  Pill *state_ = nullptr;
  QLabel *progress_text_ = nullptr;
  QPlainTextEdit *tail_ = nullptr;
  Selection selection_;
};

} // namespace rux::qt
