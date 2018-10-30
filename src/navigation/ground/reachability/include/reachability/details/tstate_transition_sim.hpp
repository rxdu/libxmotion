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

#include "model/system_propagator.hpp"
#include "model/car_longitudinal_model.hpp"

#include "reachability/details/tstate_space.hpp"

namespace librav
{
class TStateTransitionSim
{
public:
  void SetupStateSpace(double smin, double smax, double vmin, double vmax, int32_t ssize, int32_t vsize);
  void SetControlSet(Eigen::VectorXd set) { control_set_ = set; }

  void RunSim(double T);

private:
  std::unique_ptr<TStateSpace> state_space_;
  Eigen::VectorXd control_set_;

  SystemPropagator<CarLongitudinalModel, CarLongitudinalModel::control_t> propagator_;
};
} // namespace librav

#endif /* TSTATE_TRANSITION_SIM_HPP */
