/*
 * @file test_portable_normal.cpp
 * @brief Pins the sampling stream so it cannot silently diverge per platform.
 *
 * MPPI used std::normal_distribution, whose sequence is not specified by the
 * standard: libstdc++ and libc++ produced different rollouts from the same
 * seed, so tolerance-based integration tests could not be tuned once. These
 * tests lock the replacement down.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>

#include "xmnav/mppi/portable_normal.hpp"

namespace {

using xmotion::PortableNormal;

// The transform is only portable if the engine underneath it is. The standard
// fixes mt19937_64's 10000th consecutive default-seeded draw, so assert it --
// if this ever fails, nothing downstream can be reproducible either.
TEST(PortableNormal, EngineIsStandardConforming) {
  std::mt19937_64 engine;
  for (int i = 0; i < 9999; ++i) engine();
  EXPECT_EQ(engine(), 9981545732273789042ULL);
}

// Golden stream. Any change to the transform, or any platform that computes it
// differently, breaks this.
//
// Compared with a tolerance rather than exactly: the values come out of libm's
// sqrt/log/sin/cos, which are not required to be correctly rounded, so two
// C libraries may differ in the last ulp. The bound is ~1e-12 -- tight enough
// that a genuinely different algorithm cannot slip through, loose enough to
// absorb sub-ulp disagreement.
TEST(PortableNormal, StreamIsPinned) {
  constexpr double kGolden[8] = {
      -0.48121769980184498, -0.57453687389830577, 0.49458385623521361,
      0.57012155220737415,  0.3745542688498138,   0.25135417655083503,
      -0.73445603504191925, 0.75421479838146943,
  };

  std::mt19937_64 engine(42);
  PortableNormal normal;
  for (double expected : kGolden) {
    EXPECT_NEAR(normal(engine), expected, 1e-12);
  }
}

// Same seed, same stream -- the property the integration tests rely on.
TEST(PortableNormal, IsRepeatableForAGivenSeed) {
  std::mt19937_64 a(7), b(7);
  PortableNormal na, nb;
  for (int i = 0; i < 64; ++i) EXPECT_DOUBLE_EQ(na(a), nb(b));
}

// reset() drops the cached second value of the Box-Muller pair, so a reseeded
// engine restarts the stream rather than emitting a stale spare first.
TEST(PortableNormal, ResetDropsTheCachedPair) {
  std::mt19937_64 engine(11);
  PortableNormal normal;
  const double first = normal(engine);
  normal(engine);  // consumes the cached spare

  std::mt19937_64 reseeded(11);
  normal.reset();
  EXPECT_DOUBLE_EQ(normal(reseeded), first);
}

// Sanity: it must actually be N(0, 1), not merely deterministic.
TEST(PortableNormal, MatchesStandardNormalMoments) {
  std::mt19937_64 engine(2024);
  PortableNormal normal;
  constexpr int kN = 200000;
  double sum = 0.0, sum_sq = 0.0, min_v = 1e9, max_v = -1e9;
  for (int i = 0; i < kN; ++i) {
    const double z = normal(engine);
    sum += z;
    sum_sq += z * z;
    min_v = std::min(min_v, z);
    max_v = std::max(max_v, z);
    ASSERT_TRUE(std::isfinite(z)) << "draw " << i;
  }
  const double mean = sum / kN;
  const double variance = sum_sq / kN - mean * mean;
  EXPECT_NEAR(mean, 0.0, 0.01);
  EXPECT_NEAR(variance, 1.0, 0.01);
  // Both tails are reached, so the transform is not collapsing to a half.
  EXPECT_LT(min_v, -3.5);
  EXPECT_GT(max_v, 3.5);
}

}  // namespace
