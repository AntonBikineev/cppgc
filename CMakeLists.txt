# Copyright 2020 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

cmake_minimum_required(VERSION 3.10)

project(cppgc CXX)


#===============================================================================
# Global options.
#===============================================================================
option(CPPGC_OBJECT_NAMES "Enable object names in cppgc for debug purposes" OFF)
option(CPPGC_CAGED_HEAP "Enable heap reservation of size 4GB, only possible for 64bit archs" OFF)
option(CPPGC_YOUNG_GENERATION "Enable young generation in cppgc" OFF)
set(CPPGC_TARGET_ARCH "x64" CACHE STRING "Target architecture, possible options: \
    x64, x86, arm, arm64, ppc64, s390x, mipsel, mips64el")


#===============================================================================
# Option checks.
#===============================================================================
if("${CPPGC_TARGET_ARCH}" STREQUAL "x64" OR
   "${CPPGC_TARGET_ARCH}" STREQUAL "arm64" OR
   "${CPPGC_TARGET_ARCH}" STREQUAL "ppc64" OR
   "${CPPGC_TARGET_ARCH}" STREQUAL "mips64el")
 if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
   message(FATAL_ERROR "64-bit arch specified for 32-bit compiler")
 endif()
 set(CPPGC_64_BITS ON)
endif()

if(CPPGC_CAGED_HEAP AND NOT CPPGC_64_BITS)
  message(FATAL_ERROR "Caged heap is only supported for 64bit archs")
endif()

if(CPPGC_64_BITS)
 # Always enable caged heap for 64bits archs.
 set(CPPGC_CAGED_HEAP ON FORCE)
endif()

if(CPPGC_YOUNG_GENERATION AND NOT CPPGC_CAGED_HEAP)
  message(FATAL_ERROR "Young generation is only supported for caged heap configuration")
endif()


#===============================================================================
# Base library.
#===============================================================================
set(CPPGC_BASE_SOURCES "src/base/address-region.h"
                       "src/base/atomic-utils.h"
                       "src/base/atomicops.h"
                       "src/base/atomicops_internals_atomicword_compat.h"
                       "src/base/atomicops_internals_portable.h"
                       "src/base/atomicops_internals_std.h"
                       "src/base/base-export.h"
                       "src/base/bit-field.h"
                       "src/base/bits-iterator.h"
                       "src/base/bits.cc"
                       "src/base/bits.h"
                       "src/base/bounded-page-allocator.cc"
                       "src/base/bounded-page-allocator.h"
                       "src/base/bounds.h"
                       "src/base/build_config.h"
                       "src/base/compiler-specific.h"
                       "src/base/cpu.cc"
                       "src/base/cpu.h"
                       "src/base/debug/stack_trace.cc"
                       "src/base/debug/stack_trace.h"
                       "src/base/division-by-constant.cc"
                       "src/base/division-by-constant.h"
                       "src/base/enum-set.h"
                       "src/base/export-template.h"
                       "src/base/file-utils.cc"
                       "src/base/file-utils.h"
                       "src/base/flags.h"
                       "src/base/free_deleter.h"
                       "src/base/functional.cc"
                       "src/base/functional.h"
                       "src/base/hashmap-entry.h"
                       "src/base/hashmap.h"
                       "src/base/ieee754.cc"
                       "src/base/ieee754.h"
                       "src/base/iterator.h"
                       "src/base/lazy-instance.h"
                       "src/base/logging.cc"
                       "src/base/logging.h"
                       "src/base/lsan.h"
                       "src/base/macros.h"
                       "src/base/memory.h"
                       "src/base/once.cc"
                       "src/base/once.h"
                       "src/base/optional.h"
                       "src/base/overflowing-math.h"
                       "src/base/page-allocator.cc"
                       "src/base/page-allocator.h"
                       "src/base/platform/condition-variable.cc"
                       "src/base/platform/condition-variable.h"
                       "src/base/platform/elapsed-timer.h"
                       "src/base/platform/mutex.cc"
                       "src/base/platform/mutex.h"
                       "src/base/platform/platform.h"
                       "src/base/platform/semaphore.cc"
                       "src/base/platform/semaphore.h"
                       "src/base/platform/time.cc"
                       "src/base/platform/time.h"
                       "src/base/region-allocator.cc"
                       "src/base/region-allocator.h"
                       "src/base/ring-buffer.h"
                       "src/base/safe_conversions.h"
                       "src/base/safe_conversions_impl.h"
                       "src/base/small-vector.h"
                       "src/base/sys-info.cc"
                       "src/base/sys-info.h"
                       "src/base/template-utils.h"
                       "src/base/timezone-cache.h"
                       "src/base/type-traits.h"
                       "src/base/utils/random-number-generator.cc"
                       "src/base/utils/random-number-generator.h"
                       "src/base/vlq-base64.cc"
                       "src/base/vlq-base64.h")

if(UNIX)
  list(APPEND CPPGC_BASE_SOURCES
    "src/base/platform/platform-posix.cc"
    "src/base/platform/platform-posix.h"
    "src/base/platform/platform-posix-time.cc"
    "src/base/platform/platform-posix-time.h")
endif()

if("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
  list(APPEND CPPGC_BASE_SOURCES
    "src/base/debug/stack_trace_posix.cc"
    "src/base/platform/platform-linux.cc")
endif()

if(APPLE)
  list(APPEND CPPGC_BASE_SOURCES
    "src/base/debug/stack_trace_posix.cc"
    "src/base/platform/platform-macos.cc")
endif()

if(WIN32)
  list(APPEND CPPGC_BASE_SOURCES
    "src/base/debug/stack_trace_win.cc"
    "src/base/platform/platform-win32.cc"
    "src/base/win32-headers.h")
endif()


#===============================================================================
# Stack scanning support sources.
#===============================================================================
set(CPPGC_STACK_SCANNING_ARCH_DIR ${CPPGC_TARGET_ARCH})
if ("${CPPGC_TARGET_ARCH}" STREQUAL "x86")
  set(CPPGC_STACK_SCANNING_ARCH_DIR "ia32")
elseif ("${CPPGC_TARGET_ARCH}" STREQUAL "ppc64")
  set(CPPGC_STACK_SCANNING_ARCH_DIR "ppc")
elseif ("${CPPGC_TARGET_ARCH}" STREQUAL "s390x")
  set(CPPGC_STACK_SCANNING_ARCH_DIR "s390")
elseif ("${CPPGC_TARGET_ARCH}" STREQUAL "mipsel")
  set(CPPGC_STACK_SCANNING_ARCH_DIR "mips")
elseif ("${CPPGC_TARGET_ARCH}" STREQUAL "mips64el")
  set(CPPGC_STACK_SCANNING_ARCH_DIR "mips64")
endif()

set(CPPGC_STACK_SCANNING_SOURCES "src/heap/base/stack.cc"
                                 "src/heap/base/stack.h")

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  message(STATUS "Configuring libcppgc for gcc compatible compiler")

  list(APPEND CPPGC_STACK_SCANNING_SOURCES
    "src/heap/base/asm/${CPPGC_STACK_SCANNING_ARCH_DIR}/push_registers_asm.cc")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  message(STATUS "Configuring libcppgc for msvc")

  list(APPEND CPPGC_STACK_SCANNING_SOURCES
    "src/heap/base/asm/${CPPGC_STACK_SCANNING_ARCH_DIR}/push_registers_masm.cc")
else()
  message(FATAL_ERROR "Unsupported compiler")
endif()

#===============================================================================
# Core CppGC sources.
#===============================================================================
set(CPPGC_CORE_SOURCES "include/cppgc/allocation.h"
                       "include/cppgc/common.h"
                       "include/cppgc/custom-space.h"
                       "include/cppgc/garbage-collected.h"
                       "include/cppgc/heap.h"
                       "include/cppgc/internal/api-constants.h"
                       "include/cppgc/internal/atomic-entry-flag.h"
                       "include/cppgc/internal/compiler-specific.h"
                       "include/cppgc/internal/finalizer-trait.h"
                       "include/cppgc/internal/gc-info.h"
                       "include/cppgc/internal/persistent-node.h"
                       "include/cppgc/internal/pointer-policies.h"
                       "include/cppgc/internal/prefinalizer-handler.h"
                       "include/cppgc/internal/process-heap.h"
                       "include/cppgc/internal/write-barrier.h"
                       "include/cppgc/liveness-broker.h"
                       "include/cppgc/liveness-broker.h"
                       "include/cppgc/macros.h"
                       "include/cppgc/member.h"
                       "include/cppgc/persistent.h"
                       "include/cppgc/platform.h"
                       "include/cppgc/prefinalizer.h"
                       "include/cppgc/source-location.h"
                       "include/cppgc/trace-trait.h"
                       "include/cppgc/type-traits.h"
                       "include/cppgc/visitor.h"
                       "include/v8config.h"
                       "src/heap/cppgc/allocation.cc"
                       "src/heap/cppgc/free-list.cc"
                       "src/heap/cppgc/free-list.h"
                       "src/heap/cppgc/garbage-collector.h"
                       "src/heap/cppgc/gc-info-table.cc"
                       "src/heap/cppgc/gc-info-table.h"
                       "src/heap/cppgc/gc-info.cc"
                       "src/heap/cppgc/gc-invoker.cc"
                       "src/heap/cppgc/gc-invoker.h"
                       "src/heap/cppgc/heap-base.cc"
                       "src/heap/cppgc/heap-base.h"
                       "src/heap/cppgc/heap-growing.cc"
                       "src/heap/cppgc/heap-growing.h"
                       "src/heap/cppgc/heap-object-header-inl.h"
                       "src/heap/cppgc/heap-object-header.cc"
                       "src/heap/cppgc/heap-object-header.h"
                       "src/heap/cppgc/heap-page-inl.h"
                       "src/heap/cppgc/heap-page.cc"
                       "src/heap/cppgc/heap-page.h"
                       "src/heap/cppgc/heap-space.cc"
                       "src/heap/cppgc/heap-space.h"
                       "src/heap/cppgc/heap-visitor.h"
                       "src/heap/cppgc/heap.cc"
                       "src/heap/cppgc/heap.h"
                       "src/heap/cppgc/liveness-broker.cc"
                       "src/heap/cppgc/logging.cc"
                       "src/heap/cppgc/marker.cc"
                       "src/heap/cppgc/marker.h"
                       "src/heap/cppgc/marking-visitor.cc"
                       "src/heap/cppgc/marking-visitor.h"
                       "src/heap/cppgc/object-allocator-inl.h"
                       "src/heap/cppgc/object-allocator.cc"
                       "src/heap/cppgc/object-allocator.h"
                       "src/heap/cppgc/object-start-bitmap-inl.h"
                       "src/heap/cppgc/object-start-bitmap.h"
                       "src/heap/cppgc/page-memory-inl.h"
                       "src/heap/cppgc/page-memory.cc"
                       "src/heap/cppgc/page-memory.h"
                       "src/heap/cppgc/persistent-node.cc"
                       "src/heap/cppgc/platform.cc"
                       "src/heap/cppgc/pointer-policies.cc"
                       "src/heap/cppgc/prefinalizer-handler.cc"
                       "src/heap/cppgc/prefinalizer-handler.h"
                       "src/heap/cppgc/process-heap.cc"
                       "src/heap/cppgc/raw-heap.cc"
                       "src/heap/cppgc/raw-heap.h"
                       "src/heap/cppgc/sanitizers.h"
                       "src/heap/cppgc/source-location.cc"
                       "src/heap/cppgc/stats-collector.cc"
                       "src/heap/cppgc/stats-collector.h"
                       "src/heap/cppgc/sweeper.cc"
                       "src/heap/cppgc/sweeper.h"
                       "src/heap/cppgc/task-handle.h"
                       "src/heap/cppgc/virtual-memory.cc"
                       "src/heap/cppgc/virtual-memory.h"
                       "src/heap/cppgc/visitor.cc"
                       "src/heap/cppgc/worklist.h"
                       "src/heap/cppgc/write-barrier.cc")

if(CPPGC_CAGED_HEAP)
  list(APPEND CPPGC_BASE_SOURCES
    "include/cppgc/internal/caged-heap-local-data.h"
    "src/heap/cppgc/caged-heap-local-data.cc"
    "src/heap/cppgc/caged-heap.cc"
    "src/heap/cppgc/caged-heap.h")
endif()


#===============================================================================
# Main target.
#===============================================================================
add_library(cppgc ${CPPGC_CORE_SOURCES}
                  ${CPPGC_STACK_SCANNING_SOURCES}
                  ${CPPGC_BASE_SOURCES})

target_include_directories(cppgc PRIVATE "${CMAKE_SOURCE_DIR}"
                                 PRIVATE "${CMAKE_SOURCE_DIR}/include")

if(NOT CPPGC_64_BITS)
  if(NOT MSVC)
    set_target_properties(cppgc PROPERTIES COMPILE_FLAGS "-m32" LINK_FLAGS "-m32")
  else()
    # TODO.
  endif()
endif()

if(CPPGC_OBJECT_NAMES)
  target_compile_definitions(cppgc PRIVATE "-DCPPGC_SUPPORTS_OBJECT_NAMES")
endif()
if(CPPGC_CAGED_HEAP)
  target_compile_definitions(cppgc PRIVATE "-DCPPGC_CAGED_HEAP")
endif()
if(CPPGC_YOUNG_GENERATION)
  target_compile_definitions(cppgc PRIVATE "-DCPPGC_YOUNG_GENERATION")
endif()


#===============================================================================
# Tests.
#===============================================================================
add_subdirectory(third_party/googletest/src/)

set(CPPGC_UNITTEST_SOURCES "test/unittests/heap/cppgc/concurrent-sweeper-unittest.cc"
                           "test/unittests/heap/cppgc/custom-spaces-unittest.cc"
                           "test/unittests/heap/cppgc/finalizer-trait-unittest.cc"
                           "test/unittests/heap/cppgc/free-list-unittest.cc"
                           "test/unittests/heap/cppgc/garbage-collected-unittest.cc"
                           "test/unittests/heap/cppgc/gc-info-unittest.cc"
                           "test/unittests/heap/cppgc/gc-invoker-unittest.cc"
                           "test/unittests/heap/cppgc/heap-growing-unittest.cc"
                           "test/unittests/heap/cppgc/heap-object-header-unittest.cc"
                           "test/unittests/heap/cppgc/heap-page-unittest.cc"
                           "test/unittests/heap/cppgc/heap-unittest.cc"
                           "test/unittests/heap/cppgc/logging-unittest.cc"
                           "test/unittests/heap/cppgc/marker-unittest.cc"
                           "test/unittests/heap/cppgc/marking-visitor-unittest.cc"
                           "test/unittests/heap/cppgc/member-unittest.cc"
                           "test/unittests/heap/cppgc/minor-gc-unittest.cc"
                           "test/unittests/heap/cppgc/object-start-bitmap-unittest.cc"
                           "test/unittests/heap/cppgc/page-memory-unittest.cc"
                           "test/unittests/heap/cppgc/persistent-unittest.cc"
                           "test/unittests/heap/cppgc/prefinalizer-unittest.cc"
                           "test/unittests/heap/cppgc/run-all-unittests.cc"
                           "test/unittests/heap/cppgc/source-location-unittest.cc"
                           "test/unittests/heap/cppgc/stack-unittest.cc"
                           "test/unittests/heap/cppgc/stats-collector-unittest.cc"
                           "test/unittests/heap/cppgc/sweeper-unittest.cc"
                           "test/unittests/heap/cppgc/test-platform.cc"
                           "test/unittests/heap/cppgc/test-platform.h"
                           "test/unittests/heap/cppgc/tests.cc"
                           "test/unittests/heap/cppgc/tests.h"
                           "test/unittests/heap/cppgc/visitor-unittest.cc"
                           "test/unittests/heap/cppgc/worklist-unittest.cc"
                           "test/unittests/heap/cppgc/write-barrier-unittest.cc")

add_executable(cppgc_unittests ${CPPGC_UNITTEST_SOURCES})

target_link_libraries(cppgc_unittests cppgc gtest gmock)

target_include_directories(cppgc_unittests PRIVATE "${CMAKE_SOURCE_DIR}"
                                           PRIVATE "${CMAKE_SOURCE_DIR}/include")

if(NOT CPPGC_64_BITS)
  if(NOT MSVC)
    set_target_properties(cppgc_unittests PROPERTIES COMPILE_FLAGS "-m32" LINK_FLAGS "-m32")
  else()
    # TODO.
  endif()
endif()

if(CPPGC_OBJECT_NAMES)
  target_compile_definitions(cppgc_unittests PRIVATE "-DCPPGC_SUPPORTS_OBJECT_NAMES")
endif()
if(CPPGC_CAGED_HEAP)
  target_compile_definitions(cppgc_unittests PRIVATE "-DCPPGC_CAGED_HEAP")
endif()
if(CPPGC_YOUNG_GENERATION)
  target_compile_definitions(cppgc_unittests PRIVATE "-DCPPGC_YOUNG_GENERATION")
endif()
