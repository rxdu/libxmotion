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

#include "state_lattice/motion_primitive.hpp"

namespace librav
{
class PrimitiveGenerator
{
public:
  PrimitiveGenerator() = default;

  using State = MotionPrimitive::MotionState;

  MotionPrimitive Calculate(State ss, State sf);
};
} // namespace librav

#endif /* PRIMITIVE_GENERATOR_HPP */
