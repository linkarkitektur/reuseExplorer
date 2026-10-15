// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/utils/file_watcher.hpp"

#include <algorithm>
#include <system_error>
#include <thread>
#include <utility>

namespace reusex::utils {

namespace fs = std::filesystem;

FileStamp stamp_of(const fs::path &path) noexcept {
  FileStamp s;
  std::error_code ec;
  const auto status = fs::status(path, ec);
  if (ec || !fs::exists(status))
    return s;
  s.exists = true;
  if (fs::is_regular_file(status)) {
    const auto size = fs::file_size(path, ec);
    if (!ec)
      s.size = size;
  }
  const auto mtime = fs::last_write_time(path, ec);
  if (!ec)
    s.mtime = mtime;
  return s;
}

FileWatcher::FileWatcher(std::vector<fs::path> paths, Options options)
    : paths_(std::move(paths)), options_(options) {
  options_.settle_polls = std::max(options_.settle_polls, 0);
  entries_.reserve(paths_.size());
  for (const auto &p : paths_)
    entries_.push_back(Entry{stamp_of(p), std::nullopt, 0});
}

std::vector<fs::path> FileWatcher::poll() {
  std::vector<fs::path> changed;
  for (std::size_t i = 0; i < paths_.size(); ++i) {
    Entry &e = entries_[i];
    const FileStamp now = stamp_of(paths_[i]);
    if (now == e.reported) {
      // Back where it was (or never moved): nothing to report.
      e.pending.reset();
      e.stable_polls = 0;
      continue;
    }
    if (e.pending && *e.pending == now) {
      ++e.stable_polls;
    } else {
      e.pending = now;
      e.stable_polls = 0;
    }
    if (e.stable_polls >= options_.settle_polls) {
      e.reported = now;
      e.pending.reset();
      e.stable_polls = 0;
      changed.push_back(paths_[i]);
    }
  }
  return changed;
}

std::vector<fs::path>
FileWatcher::wait(std::chrono::milliseconds interval,
                  const std::atomic<bool> &stop,
                  std::optional<std::chrono::milliseconds> timeout) {
  using clock = std::chrono::steady_clock;
  const auto deadline =
      timeout ? std::optional<clock::time_point>(clock::now() + *timeout)
              : std::nullopt;
  // Sleep in short slices so a stop request is honoured promptly even with a
  // long interval.
  constexpr auto kSlice = std::chrono::milliseconds(50);
  while (!stop.load(std::memory_order_acquire)) {
    auto changed = poll();
    if (!changed.empty())
      return changed;
    const auto wake = clock::now() + interval;
    while (!stop.load(std::memory_order_acquire)) {
      const auto now = clock::now();
      if (deadline && now >= *deadline)
        return {};
      if (now >= wake)
        break;
      auto until = std::min(wake, now + kSlice);
      if (deadline)
        until = std::min(until, *deadline);
      std::this_thread::sleep_until(until);
    }
  }
  return {};
}

} // namespace reusex::utils
