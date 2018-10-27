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
    MotionPrimitive(MotionState state_s, MotionState state_f);
    MotionPrimitive(MotionState state_s, MotionState state_f, PointKinematics::Param p);

    // Defaulted big five
    ~MotionPrimitive() = default;
    MotionPrimitive(const MotionPrimitive &other) = default;
    MotionPrimitive &operator=(const MotionPrimitive &other) = default;
    MotionPrimitive(MotionPrimitive &&other) = default;
    MotionPrimitive &operator=(MotionPrimitive &&other) = default;

    // Characteristic parameters of the primitive
    double GetLength() const { return sf_; }
    MotionState GetStartState() const { return state_s_; }
    MotionState GetFinalState() const { return state_f_; }
    PointKinematics::Param GetParameters() const { return params_; }

    void SetParameters(const PointKinematics::Param &p);
    virtual MotionState Evaluate(double s, double ds = 0.1);

  protected:
    double sf_;
    MotionState state_s_;
    MotionState state_f_;

    PointKinematics::Param params_;
    PointKinematics model_;
};
} // namespace librav

#endif /* MOTION_PRIMITIVE_HPP */
