/* 
 * motion_primitive.hpp
 * 
 * Created on: Aug 11, 2018 02:48
 * Description: A motion primitive is a cubic polynomial sprial 
 *              connecting two states. 
 * 
 * Reference: 
 *  [1] M. McNaughton and C. Urmson and J. M. Dolan and J. W. Lee. 2011. 
 *    “Motion Planning for Autonomous Driving with a Conformal Spatiotemporal Lattice.” 
 *    In 2011 IEEE International Conference on Robotics and Automation, 4889–95.
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MOTION_PRIMITIVE_HPP
#define MOTION_PRIMITIVE_HPP

#include <string>
#include <vector>
#include <cstdint>

#include "geometry/polyline.hpp"
#include "geometry/polynomial.hpp"

#include "state_lattice/details/motion_state.hpp"
#include "state_lattice/details/point_kinematics.hpp"

namespace librav
{
class MotionPrimitive
{
public:
  MotionPrimitive() = default;
  MotionPrimitive(MotionState ss, MotionState sf, PointKinematics::Param p);
  explicit MotionPrimitive(int32_t mp_id) : id_(mp_id){};
  ~MotionPrimitive() = default;

  int32_t id_ = -1;

  // characteristic parameters
  double sf_;
  Polynomial<4> kappa_s_;
  MotionState Evaluate(double s, double ds = 0.1);

  MotionState GetStartState() const { return state_s_; }
  MotionState GetFinalState() const { return state_f_; }

  // void SetParameters(PointKinematics::Param p);

private:
  MotionState state_s_;
  MotionState state_f_;

  PointKinematics::Param params_;
  PointKinematics model_;
};
} // namespace librav

#endif /* MOTION_PRIMITIVE_HPP */
