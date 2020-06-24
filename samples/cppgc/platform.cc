// Copyright 2020 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "platform.h"

#include <sys/mman.h>
#include <unistd.h>

#include <cassert>

namespace sample {
namespace {

int TranslatePermissions(cppgc::PageAllocator::Permission perms) {
  switch (perms) {
    case PageAllocator::kNoAccess:
      return PROT_NONE;
    case PageAllocator::kRead:
      return PROT_READ;
    case PageAllocator::kReadWrite:
      return PROT_READ | PROT_WRITE;
    case PageAllocator::kReadWriteExecute:
      return PROT_READ | PROT_WRITE | PROT_EXEC;
    case PageAllocator::kReadExecute:
      return PROT_READ | PROT_EXEC;
  }
}

void* Allocate(void* hint, size_t size,
               cppgc::PageAllocator::Permission perms) {
  assert(!(size % ::getpagesize()));
  void* result = ::mmap(hint, size, TranslatePermissions(perms),
                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (result == MAP_FAILED) return nullptr;
  return result;
}

bool Free(void* address, size_t size) {
  assert(!(reinterpret_cast<uintptr_t>(address) % ::getpagesize()));
  assert(!(size % ::getpagesize()));
  return ::munmap(address, size) == 0;
}

void* AlignedAddress(void* address, size_t alignment) {
  assert(!(alignment & (alignment - 1)));
  return reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(address) &
                                 ~static_cast<uintptr_t>(alignment - 1));
}

template <typename T>
T RoundDown(T x, intptr_t m) {
  assert(m != 0 && ((m & (m - 1)) == 0));
  return x & -m;
}

template <typename T>
T RoundUp(T x, intptr_t m) {
  return RoundDown<T>(static_cast<T>(x + m - 1), m);
}

}  // namespace

/**
 * Task runner implementation functions.
 */
void TaskRunner::PostTask(std::unique_ptr<cppgc::Task> task) {
  tasks_.push_back(std::move(task));
}

void TaskRunner::PostNonNestableTask(std::unique_ptr<cppgc::Task> task) {
  PostTask(std::move(task));
}

void TaskRunner::PostDelayedTask(std::unique_ptr<cppgc::Task> task, double) {
  PostTask(std::move(task));
}

void TaskRunner::PostNonNestableDelayedTask(std::unique_ptr<cppgc::Task> task,
                                            double) {
  PostTask(std::move(task));
}

void TaskRunner::PostIdleTask(std::unique_ptr<cppgc::IdleTask> task) {
  idle_tasks_.push_back(std::move(task));
}

bool TaskRunner::RunSingleTask() {
  if (!tasks_.size()) return false;

  tasks_.back()->Run();
  tasks_.pop_back();

  return true;
}

bool TaskRunner::RunSingleIdleTask(double deadline_in_seconds) {
  if (!idle_tasks_.size()) return false;

  idle_tasks_.back()->Run(deadline_in_seconds);
  idle_tasks_.pop_back();

  return true;
}

void TaskRunner::RunUntilIdle() {
  for (auto& task : tasks_) {
    task->Run();
  }
  tasks_.clear();

  for (auto& task : idle_tasks_) {
    task->Run(std::numeric_limits<double>::infinity());
  }
  idle_tasks_.clear();
}

/**
 * PageAllocator implementation functions.
 */
size_t PageAllocator::AllocatePageSize() { return ::getpagesize(); }

size_t PageAllocator::CommitPageSize() { return ::getpagesize(); }

void PageAllocator::SetRandomMmapSeed(int64_t) {}

void* PageAllocator::GetRandomMmapAddr() { return nullptr; }

void* PageAllocator::AllocatePages(void* hint, size_t size, size_t alignment,
                                   Permission permissions) {
  assert(!(size % AllocatePageSize()));
  assert(!(alignment % AllocatePageSize()));

  hint = AlignedAddress(hint, alignment);
  size_t request_size = size + (alignment - AllocatePageSize());

  // Add the maximum misalignment so we are guaranteed an aligned base
  // address.
  request_size = RoundUp(request_size, AllocatePageSize());
  void* result = Allocate(hint, request_size, permissions);
  if (result == nullptr) return nullptr;

  // Unmap memory allocated before the aligned base address.
  uint8_t* base = static_cast<uint8_t*>(result);
  uint8_t* aligned_base = reinterpret_cast<uint8_t*>(
      RoundUp(reinterpret_cast<uintptr_t>(base), alignment));
  if (aligned_base != base) {
    size_t prefix_size = static_cast<size_t>(aligned_base - base);
    Free(base, prefix_size);
    request_size -= prefix_size;
  }

  // Unmap memory allocated after the potentially unaligned end.
  if (size != request_size) {
    size_t suffix_size = request_size - size;
    Free(aligned_base + size, suffix_size);
    request_size -= suffix_size;
  }

  return static_cast<void*>(aligned_base);
}

bool PageAllocator::FreePages(void* address, size_t length) {
  return Free(address, length);
}

bool PageAllocator::ReleasePages(void* address, size_t length,
                                 size_t new_length) {
  assert(new_length < length);
  return Free(static_cast<uint8_t*>(address) + new_length, length - new_length);
}

bool PageAllocator::SetPermissions(void* address, size_t size,
                                   Permission perms) {
  return mprotect(address, size, TranslatePermissions(perms)) == 0;
}

/**
 * Platform implementation functions.
 */

Platform::Platform()
    : foreground_task_runner_(std::make_shared<TaskRunner>()) {}

Platform::~Platform() noexcept { WaitAllBackgroundTasks(); }

cppgc::PageAllocator* Platform::GetPageAllocator() { return &page_allocator_; }

double Platform::MonotonicallyIncreasingTime() {
  return std::chrono::duration<double>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}

std::shared_ptr<cppgc::TaskRunner> Platform::GetForegroundTaskRunner() {
  return foreground_task_runner_;
}

std::unique_ptr<cppgc::JobHandle> Platform::PostJob(
    cppgc::TaskPriority priority, std::unique_ptr<cppgc::JobTask> job_task) {
  auto thread = std::make_shared<std::thread>([task = std::move(job_task)] {
    class JobDelegate : public v8::JobDelegate {
     public:
      bool ShouldYield() override { return false; }
      void NotifyConcurrencyIncrease() override {}
    } delegate;

    if (task) task->Run(&delegate);
  });
  job_threads_.push_back(thread);
  return std::make_unique<JobHandle>(std::move(thread));
}

void Platform::WaitAllForegroundTasks() {
  foreground_task_runner_->RunUntilIdle();
}

void Platform::WaitAllBackgroundTasks() {
  for (auto& thread : job_threads_) {
    thread->join();
  }
  job_threads_.clear();
}

}  // namespace sample
