/**
 * @file space_interface.hpp
 * @date 16-01-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef INTERFACE_SPACE_INTERFACE_HPP
#define INTERFACE_SPACE_INTERFACE_HPP

#include <memory>
#include <cstdint>

#include "eigen3/Eigen/Dense"

namespace xmotion {
template <typename T>
class SpaceInterface {
 public:
  SpaceInterface() = default;
  virtual ~SpaceInterface() = default;

  // non-copyable
  SpaceInterface(const SpaceInterface &other) = delete;
  SpaceInterface &operator=(const SpaceInterface &other) = delete;

  // public methods
  /****************** To Be Implemented ******************/
  // common interface for space
  virtual int32_t GetDimension() const = 0;
  virtual double EvaluateDistance(std::shared_ptr<T> first,
                                  std::shared_ptr<T> second) = 0;

  virtual std::shared_ptr<T> CreateState(
      std::initializer_list<double> elements) = 0;
  virtual std::shared_ptr<T> CreateState(const Eigen::MatrixXd &elements) = 0;

  // optional interface for space sampling
  virtual std::shared_ptr<T> SampleUniform() { return nullptr; }
  virtual std::shared_ptr<T> SampleUniformAround(std::shared_ptr<T> center,
                                                 double distance) {
    return nullptr;
  }
  virtual std::shared_ptr<T> SampleGaussian(std::shared_ptr<T> mean,
                                            double stdDev) {
    return nullptr;
  }
};
}  // namespace xmotion

#endif /* INTERFACE_SPACE_INTERFACE_HPP */
