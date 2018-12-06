/* 
 * cav_motion_manager.hpp
 * 
 * Created on: Dec 06, 2018 03:19
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAV_MOTION_MANAGER_HPP
#define CAV_MOTION_MANAGER_HPP

#include <memory>

#include "datalink/lcm_link.hpp"

namespace librav
{
class CAVMotionManager
{
  public:
    CAVMotionManager();

    bool IsReady();
    void Run();

  private:
    std::shared_ptr<LCMLink> data_link_;
    bool data_link_ready_ = false;
};
} // namespace librav

#endif /* CAV_MOTION_MANAGER_HPP */
