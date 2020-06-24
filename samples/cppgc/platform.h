// Copyright 2020 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SAMPLES_CPPGC_PLATFORM_H_
#define SAMPLES_CPPGC_PLATFORM_H_

#include <memory>
#include <thread>
#include <vector>

#include <cppgc/platform.h>

namespace sample {

/**
 * Simple implementation of TaskRunner.
 */
class TaskRunner : public cppgc::TaskRunner {
 public:
  TaskRunner() = default;

  void PostTask(std::unique_ptr<cppgc::Task> task) override;
  void PostNonNestableTask(std::unique_ptr<cppgc::Task> task) override;
  void PostDelayedTask(std::unique_ptr<cppgc::Task> task, double) override;
  void PostNonNestableDelayedTask(std::unique_ptr<cppgc::Task> task,
                                  double) override;

  void PostIdleTask(std::unique_ptr<cppgc::IdleTask> task) override;
  bool IdleTasksEnabled() override { return true; }

  bool RunSingleTask();
  bool RunSingleIdleTask(double duration_in_seconds);

  void RunUntilIdle();

 private:
  std::vector<std::unique_ptr<cppgc::Task>> tasks_;
  std::vector<std::unique_ptr<cppgc::IdleTask>> idle_tasks_;
};

/**
 * Simple implementation of PageAllocator based on mmap.
 */
class PageAllocator final : public cppgc::PageAllocator {
 public:
  PageAllocator() = default;

  size_t AllocatePageSize() override;
  size_t CommitPageSize() override;

  void SetRandomMmapSeed(int64_t) override;
  void* GetRandomMmapAddr() override;

  void* AllocatePages(void* hint, size_t size, size_t alignment,
                      Permission permissions) override;
  bool FreePages(void* address, size_t length) override;

  bool ReleasePages(void* address, size_t length, size_t new_length) override;

  bool SetPermissions(void* address, size_t length,
                      Permission permissions) override;
};

/**
 * Simple implementation of JobTask based on std::thread.
 */
class JobHandle : public cppgc::JobHandle {
 public:
  explicit JobHandle(std::shared_ptr<std::thread> thread)
      : thread_(std::move(thread)) {}

  void NotifyConcurrencyIncrease() override {}
  void Join() override {
    if (thread_->joinable()) thread_->join();
  }
  void Cancel() override { Join(); }
  bool IsRunning() override { return thread_->joinable(); }

 private:
  std::shared_ptr<std::thread> thread_;
};

/**
 * Simple implementation of Platform.
 */
class Platform final : public cppgc::Platform {
 public:
  Platform();
  ~Platform() noexcept;

  cppgc::PageAllocator* GetPageAllocator() final;

  double MonotonicallyIncreasingTime() final;

  std::shared_ptr<cppgc::TaskRunner> GetForegroundTaskRunner() final;

  std::unique_ptr<cppgc::JobHandle> PostJob(
      cppgc::TaskPriority priority,
      std::unique_ptr<cppgc::JobTask> job_task) final;

  void WaitAllForegroundTasks();
  void WaitAllBackgroundTasks();

 private:
  PageAllocator page_allocator_;
  std::shared_ptr<TaskRunner> foreground_task_runner_;
  std::vector<std::shared_ptr<std::thread>> job_threads_;
};

}  // namespace sample

#endif  // SAMPLES_CPPGC_PLATFORM_H_
