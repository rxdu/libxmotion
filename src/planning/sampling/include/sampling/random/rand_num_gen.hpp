/*
 * rand_num_gen.hpp
 *
 * Created on: Dec 29, 2018 08:32
 * Description: adapted from RandomNumbers in OMPL
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

#ifndef RAN_NUM_GEN_HPP
#define RAN_NUM_GEN_HPP

#include <cassert>
#include <random>
#include <memory>

#include "sampling/random/rng_seed_gen.hpp"

namespace robosw {
class RandNumGen {
 public:
  RandNumGen()
      : local_seed_(RNGSeedGenerator::getRNGSeedGenerator().nextSeed()),
        generator_(local_seed_){};
  explicit RandNumGen(std::uint_fast32_t localSeed)
      : local_seed_(localSeed), generator_(localSeed) {}

  // seeding functions
  static void SetSeed(std::uint_fast32_t seed) {
    RNGSeedGenerator::getRNGSeedGenerator().setSeed(seed);
  }
  static std::uint_fast32_t GetSeed() {
    return RNGSeedGenerator::getRNGSeedGenerator().firstSeed();
  }

  void SetLocalSeed(std::uint_fast32_t localSeed);
  std::uint_fast32_t GetLocalSeed() const { return local_seed_; }

  // sampling functions
  double Uniform();
  double UniformReal(double lower_bound, double upper_bound);
  int UniformInt(int lower_bound, int upper_bound);
  bool UniformBool();

  double Gaussian();
  double Gaussian(double mean, double stddev);

  double HalfNormalReal(double r_min, double r_max, double focus = 3.0);
  int HalfNormalInt(int r_min, int r_max, double focus = 3.0);

  void Quaternion(double value[4]);
  void EulerRPY(double value[3]);

 private:
  std::uint_fast32_t local_seed_;
  std::random_device rd_;
  std::mt19937 generator_;

  std::uniform_real_distribution<> uniform_rdist_{0, 1};
  std::normal_distribution<> normal_dist_{0, 1};
};
};  // namespace robosw

#endif /* RAN_NUM_GEN_HPP */
