/*
 * alloc_counter.cpp
 *
 * Strong definitions of the glibc malloc-family entry points that count
 * allocating calls and forward to the __libc_* implementations. Symbol
 * interposition is process-global, which is why the allocation tier is a
 * single dedicated binary (see CMakeLists.txt in this directory).
 *
 * Under ASan/TSan the sanitizer runtime owns these symbols, and on
 * non-glibc platforms __libc_* does not exist, so the counter compiles to
 * an inactive stub and the tests skip.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "alloc_counter.hpp"

#include <atomic>
#include <cstdlib>

#if defined(__GLIBC__) && !defined(__SANITIZE_ADDRESS__) && \
    !defined(__SANITIZE_THREAD__)
#define XMNAV_ALLOC_COUNTING_ACTIVE 1
#else
#define XMNAV_ALLOC_COUNTING_ACTIVE 0
#endif

namespace {
// constant-initialized: interposed calls arrive before any dynamic init
std::atomic<long> g_allocating_calls{0};
}  // namespace

#if XMNAV_ALLOC_COUNTING_ACTIVE

#include <cerrno>
#include <cstddef>

extern "C" {
// glibc's real allocator entry points (always exported by glibc)
void *__libc_malloc(std::size_t size);
void *__libc_calloc(std::size_t nmemb, std::size_t size);
void *__libc_realloc(void *ptr, std::size_t size);
void *__libc_memalign(std::size_t alignment, std::size_t size);
void __libc_free(void *ptr);
}

extern "C" void *malloc(std::size_t size) noexcept {
  g_allocating_calls.fetch_add(1, std::memory_order_relaxed);
  return __libc_malloc(size);
}

extern "C" void *calloc(std::size_t nmemb, std::size_t size) noexcept {
  g_allocating_calls.fetch_add(1, std::memory_order_relaxed);
  return __libc_calloc(nmemb, size);
}

extern "C" void *realloc(void *ptr, std::size_t size) noexcept {
  g_allocating_calls.fetch_add(1, std::memory_order_relaxed);
  return __libc_realloc(ptr, size);
}

extern "C" void *aligned_alloc(std::size_t alignment,
                               std::size_t size) noexcept {
  g_allocating_calls.fetch_add(1, std::memory_order_relaxed);
  return __libc_memalign(alignment, size);
}

extern "C" void *memalign(std::size_t alignment, std::size_t size) noexcept {
  g_allocating_calls.fetch_add(1, std::memory_order_relaxed);
  return __libc_memalign(alignment, size);
}

extern "C" int posix_memalign(void **memptr, std::size_t alignment,
                              std::size_t size) noexcept {
  // POSIX contract: alignment must be a power of two and a multiple of
  // sizeof(void *); on failure *memptr is left unmodified
  if (alignment == 0 || alignment % sizeof(void *) != 0 ||
      (alignment & (alignment - 1)) != 0) {
    return EINVAL;
  }
  g_allocating_calls.fetch_add(1, std::memory_order_relaxed);
  void *p = __libc_memalign(alignment, size);
  if (p == nullptr) return ENOMEM;
  *memptr = p;
  return 0;
}

extern "C" void free(void *ptr) noexcept { __libc_free(ptr); }

#endif  // XMNAV_ALLOC_COUNTING_ACTIVE

namespace xmotion {
namespace alloc_test {

bool AllocCountingActive() { return XMNAV_ALLOC_COUNTING_ACTIVE != 0; }

AllocationProbe::AllocationProbe()
    : start_(g_allocating_calls.load(std::memory_order_relaxed)) {}

long AllocationProbe::Count() const {
  return g_allocating_calls.load(std::memory_order_relaxed) - start_;
}

}  // namespace alloc_test
}  // namespace xmotion
