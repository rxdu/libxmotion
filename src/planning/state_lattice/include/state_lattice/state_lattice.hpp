/* 
 * state_lattice.hpp
 * 
 * Created on: Oct 26, 2018 11:11
 * Description: Derived from MotionPrimitive, interally transforms 
 *              start/target state to a coordinate aligned with 
 *              (x,y,theta) of the start state for easier numerical
 *              optimization. All transformations are taken care of 
 *              internally, so outside classes could use this one
 *              just like the normal MotionPrimitive.                      
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef STATE_LATTICE_HPP
#define STATE_LATTICE_HPP

#include <string>
#include <vector>
#include <unordered_map>

#include "state_lattice/motion_primitive.hpp"
#include "state_lattice/primitive_generator.hpp"

namespace robosw
{
class StateLattice : public MotionPrimitive
{
  public:
    StateLattice() = default;
    StateLattice(MotionState state_s, MotionState state_f);

    // Defaulted big five
    ~StateLattice() = default;
    StateLattice(const StateLattice &other) = default;
    StateLattice &operator=(const StateLattice &other) = default;
    StateLattice(StateLattice &&other) = default;
    StateLattice &operator=(StateLattice &&other) = default;

    // Characteristic parameters of the primitive
    bool IsValid() const { return valid_; }
    MotionState Evaluate(double s, double ds = 0.1) override;

    /* Useful functions derived from parent class */
    // double GetLength() const { return sf_; }

    void GetPositionVector(double s, double &x, double &y);
    void GetTangentVector(double s, double &x, double &y);

  private:
    static PrimitiveGenerator generator;

    double dx_;
    double dy_;
    double dtheta_;

    bool valid_ = false;

    MotionState tstate_s_;
    MotionState tstate_f_;

    MotionState TransformToLocal(const MotionState &input);
    MotionState TransformToGlobal(const MotionState &input);
    // MotionPrimitive TransformPrimitive(const MotionPrimitive &input, double dx, double dy, double dtheta);
    // PrimitiveNode TransformNode(const PrimitiveNode &input, double dx, double dy, double dtheta);
};
} // namespace robosw

#endif /* STATE_LATTICE_HPP */
