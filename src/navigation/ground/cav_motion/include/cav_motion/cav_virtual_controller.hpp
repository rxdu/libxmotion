/* 
 * cav_virtual_controller.hpp
 * 
 * Created on: Dec 09, 2018 04:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAV_VIRTUAL_CONTROLLER_HPP
#define CAV_VIRTUAL_CONTROLLER_HPP

#include <memory>

#include "cav_common/cav_datalink.hpp"

namespace librav
{
class CAVVirtualController
{
  public:
    CAVVirtualController(std::shared_ptr<LCMLink> link);

  private:
    std::shared_ptr<LCMLink> data_link_;

    // void HandleLCMMessage_SyncTrigger(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::SimSyncTrigger *msg);
};
} // namespace librav

#endif /* CAV_VIRTUAL_CONTROLLER_HPP */
