/*
 * @file portable_normal.hpp
 * @brief Standard-normal draws that do not depend on the standard library.
 *
 * std::normal_distribution is not specified to produce any particular
 * sequence: libstdc++ and libc++ turn the same std::mt19937_64 stream into
 * different normals. MPPI therefore explored a different rollout set per
 * platform, so results were not reproducible across Linux and macOS and a
 * tolerance-based integration test could not be tuned once and trusted
 * everywhere.
 *
 * The engine is already portable -- mt19937_64 is fully specified -- so only
 * the transform needed pinning. The top 53 bits of each 64-bit draw map onto
 * [0, 1) exactly, and Box-Muller turns two uniforms into two normals. The
 * resulting stream is identical on every platform up to libm's rounding of
 * sqrt/log/sin/cos, which is sub-ULP and far below any effect on control.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_PORTABLE_NORMAL_HPP
#define XMNAV_MPPI_PORTABLE_NORMAL_HPP

#include <cmath>
#include <cstdint>
#include <limits>

namespace xmotion {

class PortableNormal {
 public:
  // Draws N(0, 1). Two normals come out of each pair of uniforms; the spare is
  // cached, so the engine is consumed at the same rate as a typical
  // std::normal_distribution implementation.
  template <typename Engine>
  double operator()(Engine &engine) {
    static_assert(Engine::min() == 0u &&
                      Engine::max() == std::numeric_limits<std::uint64_t>::max(),
                  "PortableNormal expects a full-range 64-bit engine "
                  "(e.g. std::mt19937_64)");

    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    // u1 must be strictly positive for log(); the mapping can yield exactly 0.
    double u1 = Uniform01(engine);
    if (u1 <= 0.0) u1 = kSmallest;
    const double u2 = Uniform01(engine);

    const double radius = std::sqrt(-2.0 * std::log(u1));
    const double angle = kTwoPi * u2;
    spare_ = radius * std::sin(angle);
    has_spare_ = true;
    return radius * std::cos(angle);
  }

  // Drops the cached value, so a reseeded engine restarts a known stream.
  void reset() { has_spare_ = false; }

 private:
  // Top 53 bits -> [0, 1) with no rounding: exactly the representable
  // multiples of 2^-53. Written out rather than using
  // std::generate_canonical, which is also implementation-defined.
  template <typename Engine>
  static double Uniform01(Engine &engine) {
    return static_cast<double>(static_cast<std::uint64_t>(engine()) >> 11) *
           kTwoPowMinus53;
  }

  static constexpr double kTwoPowMinus53 = 1.0 / 9007199254740992.0;  // 2^-53
  static constexpr double kSmallest = kTwoPowMinus53;
  static constexpr double kTwoPi = 6.283185307179586476925286766559;

  double spare_ = 0.0;
  bool has_spare_ = false;
};

}  // namespace xmotion

#endif  // XMNAV_MPPI_PORTABLE_NORMAL_HPP
