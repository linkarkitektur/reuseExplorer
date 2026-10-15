// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "../../support/temp_path.hpp"

#include <reusex/utils/file_watcher.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;
using reusex::utils::FileWatcher;
using reusex::utils::stamp_of;

namespace {

/// Write @p text and pin the mtime to @p seconds past a base time, so a
/// change is visible to the watcher without sleeping past the filesystem's
/// timestamp granularity.
void write_at(const fs::path &path, const std::string &text, int seconds) {
  static const auto base = std::chrono::floor<std::chrono::seconds>(
      fs::file_time_type::clock::now());
  {
    std::ofstream(path, std::ios::trunc) << text;
  }
  fs::last_write_time(path, base + std::chrono::seconds(seconds));
}

} // namespace

TEST_CASE("stamp_of reports existence, size and mtime",
          "[utils][file_watcher]") {
  reusex::test_support::TempDir dir("file_watcher");
  const fs::path p = dir.path / "a.yaml";
  CHECK_FALSE(stamp_of(p).exists);
  write_at(p, "abc", 1);
  const auto s = stamp_of(p);
  CHECK(s.exists);
  CHECK(s.size == 3);
  CHECK(s == stamp_of(p));
  write_at(p, "abc", 2); // same size, newer mtime
  CHECK_FALSE(s == stamp_of(p));
}

TEST_CASE("FileWatcher without settling reports each change once",
          "[utils][file_watcher]") {
  reusex::test_support::TempDir dir("file_watcher");
  const fs::path a = dir.path / "a.yaml";
  const fs::path b = dir.path / "b.rux";
  write_at(a, "one", 1);
  write_at(b, "db", 1);

  FileWatcher w({a, b}, {.settle_polls = 0});
  CHECK(w.poll().empty()); // the state at construction is the baseline

  write_at(a, "two", 2);
  const auto changed = w.poll();
  REQUIRE(changed.size() == 1);
  CHECK(changed.front() == a);
  CHECK(w.poll().empty()); // reported once

  write_at(a, "three", 3);
  write_at(b, "db2", 3);
  const auto both = w.poll();
  REQUIRE(both.size() == 2);
  CHECK(both[0] == a); // in watch order
  CHECK(both[1] == b);
}

TEST_CASE("FileWatcher reports a deletion and a re-creation",
          "[utils][file_watcher]") {
  reusex::test_support::TempDir dir("file_watcher");
  const fs::path a = dir.path / "a.yaml";
  write_at(a, "one", 1);
  FileWatcher w({a}, {.settle_polls = 0});

  fs::remove(a);
  CHECK(w.poll().size() == 1);
  CHECK(w.poll().empty());
  write_at(a, "one", 1); // even with the old content and mtime
  CHECK(w.poll().size() == 1);
}

TEST_CASE("FileWatcher waits for a change to settle before reporting it",
          "[utils][file_watcher]") {
  reusex::test_support::TempDir dir("file_watcher");
  const fs::path a = dir.path / "a.yaml";
  write_at(a, "one", 1);
  FileWatcher w({a}, {.settle_polls = 1});

  write_at(a, "tw", 2); // an editor's first write ...
  CHECK(w.poll().empty());
  write_at(a, "two", 3); // ... and its second: still moving
  CHECK(w.poll().empty());
  CHECK(w.poll().size() == 1); // unchanged for one poll: settled
  CHECK(w.poll().empty());

  // A change undone before it settled is no change at all.
  write_at(a, "xxx", 4);
  CHECK(w.poll().empty());
  write_at(a, "two", 3);
  CHECK(w.poll().empty());
  CHECK(w.poll().empty());
}

TEST_CASE("FileWatcher::wait returns at once when asked to stop",
          "[utils][file_watcher]") {
  reusex::test_support::TempDir dir("file_watcher");
  const fs::path a = dir.path / "a.yaml";
  write_at(a, "one", 1);
  FileWatcher w({a});
  std::atomic<bool> stop{true};
  const auto t0 = std::chrono::steady_clock::now();
  CHECK(w.wait(std::chrono::seconds(10), stop).empty());
  CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5));
}

TEST_CASE("FileWatcher::wait returns a change already pending",
          "[utils][file_watcher]") {
  reusex::test_support::TempDir dir("file_watcher");
  const fs::path a = dir.path / "a.yaml";
  write_at(a, "one", 1);
  FileWatcher w({a}, {.settle_polls = 0});
  write_at(a, "two", 2);
  std::atomic<bool> stop{false};
  const auto changed = w.wait(std::chrono::milliseconds(1), stop);
  REQUIRE(changed.size() == 1);
  CHECK(changed.front() == a);
}

TEST_CASE("FileWatcher::wait gives up after its timeout",
          "[utils][file_watcher]") {
  reusex::test_support::TempDir dir("file_watcher");
  const fs::path a = dir.path / "a.yaml";
  write_at(a, "one", 1);
  FileWatcher w({a});
  std::atomic<bool> stop{false};
  CHECK(w.wait(std::chrono::milliseconds(1), stop, std::chrono::milliseconds(5))
            .empty());
}
