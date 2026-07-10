/*
 * alloc_counter.hpp
 *
 * Process-global heap-allocation counter for the allocation test tier.
 * The matching .cpp interposes the glibc malloc family (not operator new:
 * Eigen's dynamic-size allocations reach std::malloc directly through
 * Eigen::internal::aligned_malloc) and counts every allocating call.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_TESTS_ALLOCATION_ALLOC_COUNTER_HPP
#define XMNAV_TESTS_ALLOCATION_ALLOC_COUNTER_HPP

namespace xmotion {
namespace alloc_test {

// true when the malloc interposition is compiled in (glibc, no ASan/TSan);
// allocation tests skip when it is not
bool AllocCountingActive();

// Allocating calls (malloc/calloc/realloc/aligned_alloc/posix_memalign/
// memalign) observed process-wide since construction; free is not counted.
class AllocationProbe {
 public:
  AllocationProbe();
  long Count() const;

 private:
  long start_;
};

}  // namespace alloc_test
}  // namespace xmotion

#endif  // XMNAV_TESTS_ALLOCATION_ALLOC_COUNTER_HPP
