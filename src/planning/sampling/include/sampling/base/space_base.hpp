/*
 * space_base.hpp
 *
 * Created on: Dec 29, 2018 05:34
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SPACE_BASE_HPP
#define SPACE_BASE_HPP

#include <cstdint>
#include <functional>

#include "eigen3/Eigen/Dense"

#include "sampling/random/rand_num_gen.hpp"

namespace xmotion {
class State {
 public:
  explicit State(int64_t id) : id_(id){};
  virtual ~State() = default;

  // non-copyable
  State(const State &other) = delete;
  State &operator=(const State &other) = delete;

  int64_t id_;

  /****************** To Be Implemented ******************/
  virtual double operator[](int32_t i) const = 0;
  virtual double &operator[](int32_t i) = 0;
  /*******************************************************/
};

class SpaceBase {
 public:
  SpaceBase() = default;
  virtual ~SpaceBase() = default;

  // non-copyable
  SpaceBase(const SpaceBase &other) = delete;
  SpaceBase &operator=(const SpaceBase &other) = delete;

  /****************** To Be Implemented ******************/
  // common interface for space
  virtual int32_t GetDimension() const = 0;
  virtual double EvaluateDistance(std::shared_ptr<State> sstate,
                                  std::shared_ptr<State> dstate) = 0;

  virtual std::shared_ptr<State> CreateState(
      std::initializer_list<double> elements) = 0;
  virtual std::shared_ptr<State> CreateState(
      const Eigen::MatrixXd &elements) = 0;

  virtual std::shared_ptr<State> SampleUniform() = 0;
  virtual std::shared_ptr<State> SampleUniformAround(
      const std::shared_ptr<State> center, double distance) = 0;
  virtual std::shared_ptr<State> SampleGaussian(
      const std::shared_ptr<State> mean, double stdDev) = 0;
  /*******************************************************/

 protected:
  RandNumGen rng_;
};
}  // namespace xmotion

#endif /* SPACE_BASE_HPP */
