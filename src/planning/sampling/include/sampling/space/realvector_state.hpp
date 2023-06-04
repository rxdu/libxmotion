/*
 * realvector_state.hpp
 *
 * Created on: Dec 29, 2018 06:29
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SPACE_REALVECTOR_STATE_HPP
#define SPACE_REALVECTOR_STATE_HPP

#include <cstdint>
#include <cassert>
#include <atomic>
#include <iostream>

#include "sampling/interface/state_interface.hpp"

namespace xmotion {
template <int32_t N>
class RealVectorSpace;

template <int32_t N>
class RealVectorState : public StateInterface {
 public:
  RealVectorState() : StateInterface(RealVectorState::count) {
    RealVectorState::count.fetch_add(1);
  };

  RealVectorState(std::initializer_list<double> vals)
      : StateInterface(RealVectorState::count) {
    int32_t i = 0;
    for (auto &val : vals) values_[i++] = val;
    RealVectorState::count.fetch_add(1);
  }

  // states can only be created from the space
  template <int32_t M>
  friend class RealVectorSpace;

 public:
  double values_[N];

  double operator[](int32_t i) const override {
    assert(i < N);
    return values_[i];
  }

  double &operator[](int32_t i) override {
    assert(i < N);
    return values_[i];
  }

  friend std::ostream &operator<<(std::ostream &os,
                                  const RealVectorState &other) {
    os << "[ ";
    for (int i = 0; i < N; ++i) os << other.values_[i] << " ";
    os << "]";
    return os;
  }

  // statistics
  static std::atomic<std::size_t> count;
};

template <int32_t N>
std::atomic<std::size_t> RealVectorState<N>::count = {0};
}  // namespace xmotion

#endif /* SPACE_REALVECTOR_STATE_HPP */
