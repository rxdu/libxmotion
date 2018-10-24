/* 
 * motion_primitive.hpp
 * 
 * Created on: Aug 11, 2018 02:48
 * Description: A motion primitive is a cubic polynomial sprial 
 *              connecting two states. 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MOTION_PRIMITIVE_HPP
#define MOTION_PRIMITIVE_HPP

#include <string>
#include <vector>
#include <cstdint>

#include "geometry/polyline.hpp"

namespace librav
{
class MotionPrimitive
{
public:
  struct MotionState
  {
    MotionState() : x(0), y(0), theta(0), kappa(0) {}
    MotionState(double _x, double _y, double _theta, double _k) : x(_x), y(_y), theta(_theta), kappa(_k) {}

    double x;
    double y;
    double theta;
    double kappa;
  };

  using State = MotionState;

  /////////////////////////////////////////////////////////

public:
  MotionPrimitive() : id_(-1) {}
  explicit MotionPrimitive(int32_t mp_id) : id_(mp_id){};
  ~MotionPrimitive() = default;

  int32_t id_;
  double length_;

  MotionState GetStartState() const { return state_s_; }
  MotionState GetFinalState() const { return state_f_; }

private:
  MotionState state_s_;
  MotionState state_f_;
};
} // namespace librav

#endif /* MOTION_PRIMITIVE_HPP */
