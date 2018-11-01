/* 
 * tstate_transition_sim.hpp
 * 
 * Created on: Oct 30, 2018 05:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TSTATE_TRANSITION_SIM_HPP
#define TSTATE_TRANSITION_SIM_HPP

#include <memory>

#include <eigen3/Eigen/Dense>

#include "reachability/tstate_space.hpp"
#include "reachability/details/car_model_propagator.hpp"

namespace librav
{
class TStateTransitionSim
{
public:
  std::shared_ptr<TStateSpace> state_space_;

  void SetupStateSpace(std::shared_ptr<TStateSpace> space) { state_space_ = space; }
  void SetupStateSpace(double smin, double smax, double vmin, double vmax, int32_t ssize, int32_t vsize);
  void SetControlSet(Eigen::VectorXd set) { control_set_ = set; }

  Eigen::MatrixXd RunSim(double T);

private:
  Eigen::VectorXd control_set_;

  CarModelPropagator propagator_;
};
} // namespace librav

#endif /* TSTATE_TRANSITION_SIM_HPP */
