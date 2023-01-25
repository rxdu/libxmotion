/*
 * realvector_space.hpp
 *
 * Created on: Dec 29, 2018 06:29
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SPACE_REALVECTOR_SPACE_HPP
#define SPACE_REALVECTOR_SPACE_HPP

#include <cstdint>
#include <cassert>
#include <atomic>
#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_map>

#include <eigen3/Eigen/Dense>

#include "sampling/interface/space_interface.hpp"
#include "sampling/space/realvector_bound.hpp"
#include "sampling/space/realvector_state.hpp"
#include "sampling/random/rand_num_gen.hpp"

namespace robosw {
template <int32_t N>
class RealVectorSpace : public SpaceInterface<RealVectorState<N>> {
  using BaseType = SpaceInterface<RealVectorState<N>>;

 public:
  using StateType = RealVectorState<N>;

 public:
  RealVectorSpace() { bounds_.resize(N); }

  RealVectorSpace(std::initializer_list<RealVectorBound> bounds) {
    assert(bounds.size() == N);
    bounds_.resize(N);
    int32_t i = 0;
    for (auto &bd : bounds) bounds_[i++] = bd;
  }

  ~RealVectorSpace() {
    // for (auto &state : all_states_) delete state.second;
  }

  RealVectorSpace(RealVectorSpace &&other) {
    bounds_ = std::move(other.bounds_);
    all_states_ = std::move(other.all_states_);
    // to avoid the destructor free memory multiple times
    for (auto &state : other.all_states_) state.second = nullptr;
  }

  RealVectorSpace &operator=(RealVectorSpace &&other) {
    for (auto &state : all_states_) delete state.second;
    this->bounds_ = std::move(other.bounds_);
    this->all_states_ = std::move(other.all_states_);
    for (auto &state : other.all_states_) state.second = nullptr;
    return *this;
  }

  void SetBound(int32_t dim, double min, double max) {
    assert(dim < N);
    bounds_[dim] = RealVectorBound(min, max);
  }

  static constexpr int32_t DimensionSize = N;
  int32_t GetDimension() const override { return N; }
  std::vector<RealVectorBound> GetBounds() const { return bounds_; }

  RealVectorBound GetBound(int32_t dim) const {
    assert(dim < N);
    return bounds_[dim];
  }

  double GetVolume() const {
    double vol = 1.0;
    for (auto &dim_bd : bounds_) vol *= dim_bd.GetDifference();
    return vol;
  }

  double EvaluateDistance(std::shared_ptr<StateType> first,
                          std::shared_ptr<StateType> second) override {
    // 2-norm distance
    double se_sum = 0;
    for (int i = 0; i < N; ++i) {
      double err = first->values_[i] - second->values_[i];
      se_sum += err * err;
    }
    return std::sqrt(se_sum);
  }

  std::shared_ptr<StateType> CreateState(
      std::initializer_list<double> elements) override {
    assert(elements.size() >= N);

    std::shared_ptr<StateType> new_state = std::make_shared<StateType>();
    int i = 0;
    for (auto &element : elements) new_state->values_[i++] = element;
    all_states_[new_state->id_] = new_state;
    return new_state;
  }

  std::shared_ptr<StateType> CreateState(
      const Eigen::MatrixXd &elements) override {
    assert(elements.size() >= N);

    std::shared_ptr<StateType> new_state = std::make_shared<StateType>();
    for (int i = 0; i < N; ++i) new_state->values_[i] = elements(i);
    all_states_[new_state->id_] = new_state;
    return new_state;
  }

  std::shared_ptr<StateType> SampleUniform() override {
    std::shared_ptr<StateType> new_state = std::make_shared<StateType>();
    for (int i = 0; i < N; ++i)
      new_state->values_[i] =
          rng_.UniformReal(bounds_[i].GetLow(), bounds_[i].GetHigh());
    all_states_[new_state->id_] = new_state;
    return new_state;
  };

  std::shared_ptr<StateType> SampleUniformAround(
      std::shared_ptr<StateType> center, double distance) override {
    std::shared_ptr<StateType> new_state = std::make_shared<StateType>();
    for (int i = 0; i < N; ++i)
      new_state->values_[i] = rng_.UniformReal(
          std::max(bounds_[i].GetLow(), center->values_[i] - distance),
          std::min(bounds_[i].GetHigh(), center->values_[i] + distance));
    all_states_[new_state->id_] = new_state;
    return new_state;
  };

  std::shared_ptr<StateType> SampleGaussian(std::shared_ptr<StateType> mean,
                                            double stdDev) override {
    std::shared_ptr<StateType> new_state = std::make_shared<StateType>();
    for (int i = 0; i < N; ++i) {
      double v = rng_.Gaussian(mean->values_[i], stdDev);
      if (v < bounds_[i].GetLow())
        v = bounds_[i].GetLow();
      else if (v > bounds_[i].GetHigh())
        v = bounds_[i].GetHigh();
      new_state->values_[i] = v;
    }
    all_states_[new_state->id_] = new_state;
    return new_state;
  };

  void PrintInfo() {
    std::cout << "dimension: " << GetDimension() << std::endl;
    std::cout << "bounds: " << std::endl;
    for (int32_t i = 0; i < N; ++i)
      std::cout << " - dim " << i << " : [ " << bounds_[i].GetLow() << " , "
                << bounds_[i].GetHigh() << " ]" << std::endl;
  }

 private:
  RandNumGen rng_;
  std::vector<RealVectorBound> bounds_;
  std::unordered_map<std::size_t, std::shared_ptr<StateType>> all_states_;
};
}  // namespace robosw

#endif /* SPACE_REALVECTOR_SPACE_HPP */
