/* 
 * primitive_generator.hpp
 * 
 * Created on: Oct 21, 2018 23:46
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef PRIMITIVE_GENERATOR_HPP
#define PRIMITIVE_GENERATOR_HPP

#include <cstdint>

#include "state_lattice/motion_primitive.hpp"
#include "state_lattice/details/point_kinematics.hpp"

namespace librav
{
class PrimitiveGenerator
{
public:
  PrimitiveGenerator();

  MotionPrimitive Calculate(MotionState state_s, MotionState state_f, PointKinematics::Param init_p);

private:
  const int32_t max_iter_ = 100;
  const double cost_th_ = 0.1;
  std::vector<double> scalers_;

  StatePMatrix Je_;
  PointKinematics model_;

  StatePMatrix CalcDeltaX(StatePMatrix target, StatePMatrix xi);
  JacobianMatrix CalcJacobian(MotionState init, MotionState target, PointKinematics::Param p);
  double SelectParamScaler(StatePMatrix start, StatePMatrix target, PointKinematics::Param p, ParamPMatrix p_i, ParamPMatrix delta_pi);
};
} // namespace librav

#endif /* PRIMITIVE_GENERATOR_HPP */
