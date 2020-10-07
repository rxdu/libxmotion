/* 
 * unicycle_model.hpp
 * 
 * Created on: Sep 28, 2018 00:10
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef UNICYCLE_MODEL_HPP
#define UNICYCLE_MODEL_HPP

#include "ascent/Ascent.h"
#include "ascent/Utility.h"

namespace autodrive
{
class UnicycleKinematics
{
public:
  using control_t = double;
  UnicycleKinematics(control_t u);

  // x1 = x, x2 = y, x3 = v, x4 = theta
  void operator()(const asc::state_t &x, asc::state_t &xd, const double);

private:
  control_t u_ = 0;
  static constexpr double L = 2.4;
};
} // namespace autodrive

#endif /* UNICYCLE_MODEL_HPP */
