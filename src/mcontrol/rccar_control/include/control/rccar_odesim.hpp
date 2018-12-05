/* 
 * rccar_odesim.hpp
 * 
 * Created on: Dec 03, 2018 08:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RCCAR_ODESIM_HPP
#define RCCAR_ODESIM_HPP

#include "model/bicycle_model.hpp"
#include "model/system_propagator.hpp"

namespace librav
{
class RCCarODESim
{
  public:
    RCCarODESim();

    void RunSim(double t0, double tf, double dt);

  private:
    SystemPropagator<BicycleKinematics, BicycleKinematics::control_t> propagator_;
};
} // namespace librav

#endif /* RCCAR_ODESIM_HPP */
