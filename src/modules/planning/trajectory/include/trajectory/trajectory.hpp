/* 
 * trajectory.hpp
 * 
 * Created on: Oct 14, 2018 04:57
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <cstdint>

namespace autodrive
{
template <int32_t N>
class Trajectory
{
public:
  Trajectory() : dimension_(N){};

  std::size_t GetDimension() const { return dimension_; }

private:
  int32_t dimension_;
};
} // namespace autodrive

#endif /* TRAJECTORY_HPP */
