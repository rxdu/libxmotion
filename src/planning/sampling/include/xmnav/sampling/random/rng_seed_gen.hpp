/*
 * rng_seed_gen.hpp
 *
 * Created on: Dec 29, 2018 10:15
 * Description: changed RNGSeedGenerator to singleton
 *              commented out OMPL messages
 *
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Jonathan Gammell */

#ifndef RNG_SEED_GEN_HPP
#define RNG_SEED_GEN_HPP

#include <mutex>
#include <random>

namespace xmotion {
class RNGSeedGenerator {
  RNGSeedGenerator()
      : firstSeed_(std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::system_clock::now() -
                       std::chrono::system_clock::time_point::min())
                       .count()),
        sGen_(firstSeed_),
        sDist_(1, 1000000000) {}

  // non-copyable
  RNGSeedGenerator(const RNGSeedGenerator &) = delete;
  RNGSeedGenerator &operator=(const RNGSeedGenerator &) = delete;

 public:
  static RNGSeedGenerator &getRNGSeedGenerator() {
    static RNGSeedGenerator gen;
    return gen;
  }

  std::uint_fast32_t firstSeed() {
    std::lock_guard<std::mutex> slock(rngMutex_);
    return firstSeed_;
  }

  void setSeed(std::uint_fast32_t seed) {
    std::lock_guard<std::mutex> slock(rngMutex_);
    if (seed > 0) {
      if (someSeedsGenerated_) {
        // OMPL_ERROR("Random number generation already started. Changing seed
        // now will not lead to "
        //            "deterministic sampling.");
      } else {
        // In this case, since no seeds have been generated yet, so we remember
        // this seed as the first one.
        firstSeed_ = seed;
      }
    } else {
      if (someSeedsGenerated_) {
        // OMPL_WARN("Random generator seed cannot be 0. Ignoring seed.");
        return;
      }
      // OMPL_WARN("Random generator seed cannot be 0. Using 1 instead.");
      seed = 1;
    }
    sGen_.seed(seed);
  }

  std::uint_fast32_t nextSeed() {
    std::lock_guard<std::mutex> slock(rngMutex_);
    someSeedsGenerated_ = true;
    return sDist_(sGen_);
  }

 private:
  bool someSeedsGenerated_{false};
  std::uint_fast32_t firstSeed_;
  std::mutex rngMutex_;
  std::ranlux24_base sGen_;
  std::uniform_int_distribution<> sDist_;
};
}  // namespace xmotion

#endif /* RNG_SEED_GEN_HPP */
