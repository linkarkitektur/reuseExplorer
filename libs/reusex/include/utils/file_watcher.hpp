// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A polling file watcher for hot-reload loops (reusex-yf4.2: `rux render
// --watch`, `rux view --watch`).
//
// Polling rather than inotify: it is portable, needs no thread of its own, and
// sees a file an editor replaces by rename (vim, VS Code's atomic save) the
// same way as one written in place -- the path's stamp changes either way. A
// change is a difference in (exists, size, mtime) from the last reported
// stamp, so the result is a pure function of what is on disk at each poll.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace reusex::utils {

/// What the watcher compares: whether a path exists, its size and its mtime.
struct FileStamp {
  bool exists = false;
  std::uintmax_t size = 0;
  std::filesystem::file_time_type mtime{};

  bool operator==(const FileStamp &) const = default;
};

/// The stamp of @p path now; `exists == false` when it cannot be stat'ed.
FileStamp stamp_of(const std::filesystem::path &path) noexcept;

struct FileWatcherOptions {
  /// A changed stamp is reported only once it has stayed the same for this
  /// many further polls, so an editor that saves in several writes (or
  /// deletes and re-creates the file) is reported once, complete, rather
  /// than half-written. 0 reports a change on the poll that first sees it.
  int settle_polls = 1;
};

/// Watches a fixed list of paths for changes, by polling their stamps.
class FileWatcher {
    public:
  using Options = FileWatcherOptions;

  /// The paths' current stamps are the baseline: only later changes are
  /// reported. A path that does not exist yet is watched for its creation.
  explicit FileWatcher(std::vector<std::filesystem::path> paths,
                       Options options = {});

  /// Stat every path once.
  /// @return the paths whose change has settled since the last report, in
  ///         the order they were given; empty when nothing did.
  std::vector<std::filesystem::path> poll();

  /// poll() every @p interval (sleeping in between -- never spinning) until
  /// it reports a change, @p stop becomes true, or @p timeout has passed.
  /// Checks once before the first sleep.
  /// @return what poll() reported; empty on stop or timeout.
  std::vector<std::filesystem::path>
  wait(std::chrono::milliseconds interval, const std::atomic<bool> &stop,
       std::optional<std::chrono::milliseconds> timeout = std::nullopt);

  [[nodiscard]] const std::vector<std::filesystem::path> &paths() const {
    return paths_;
  }

    private:
  struct Entry {
    FileStamp reported; ///< the stamp last reported (or the baseline)
    std::optional<FileStamp> pending; ///< a changed stamp not yet settled
    int stable_polls = 0;             ///< polls @ref pending has held
  };

  std::vector<std::filesystem::path> paths_;
  std::vector<Entry> entries_;
  Options options_;
};

} // namespace reusex::utils
