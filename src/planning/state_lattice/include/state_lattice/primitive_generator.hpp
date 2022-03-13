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
#include "state_lattice/details/lookup_table.hpp"

namespace robosw
{
class PrimitiveGenerator
{
  public:
    PrimitiveGenerator();
    explicit PrimitiveGenerator(std::string lookup_file);

    // Optimize trajectory
    void LoadLookupTable(std::string lookup_file);
    bool Calculate(MotionState state_s, MotionState state_f, MotionPrimitive &mp);

    // Warning: optimize trajectory WITHOUT using lookup table
    MotionPrimitive Calculate(MotionState state_s, MotionState state_f, PointKinematics::Param init_p);
    bool Calculate(MotionState state_s, MotionState state_f, PointKinematics::Param init_p, MotionPrimitive &mp);

  private:
    const int32_t max_iter_ = 100;
    const double cost_th_ = 0.05;
    std::vector<double> scalers_ = {1.0, 2.0, 0.5};
    StatePMatrix Je_;

    PointKinematics model_;
    bool lookup_available_ = false;
    LookupTable lookup_table_;

    StatePMatrix CalcDeltaX(StatePMatrix target, StatePMatrix xi);
    JacobianMatrix CalcJacobian(MotionState init, MotionState target, PointKinematics::Param p);
    double SelectParamScaler(StatePMatrix start, StatePMatrix target, PointKinematics::Param p, ParamPMatrix p_i, ParamPMatrix delta_pi);
};
} // namespace robosw

#endif /* PRIMITIVE_GENERATOR_HPP */
