#ifndef PATH_REPAIR_SIM_VIRTUAL_QUAD_H_
#define PATH_REPAIR_SIM_VIRTUAL_QUAD_H_

#include <memory>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "quadrotor/path_repair/sim_path_repair.h"

namespace librav
{

class VirtualQuadrotor
{
public:
    VirtualQuadrotor(std::shared_ptr<lcm::LCM> lcm);

    std::shared_ptr<SimPathRepair> qplanner_;

public:    
    void Load_30by50_Config();
    bool IsReady(){ return qplanner_->config_complete_; };

    void Step();    
    
private:
    std::shared_ptr<lcm::LCM> lcm_;
    bool reached_goal_;

    bool MoveForward();
    // void SendTransformation
};

}

#endif /* PATH_REPAIR_SIM_VIRTUAL_QUAD_H_ */
