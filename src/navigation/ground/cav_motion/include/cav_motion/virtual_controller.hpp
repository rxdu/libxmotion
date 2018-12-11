/* 
 * virtual_controller.hpp
 * 
 * Created on: Dec 09, 2018 04:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VIRTUAL_CONTROLLER_HPP
#define VIRTUAL_CONTROLLER_HPP

#include <memory>

#include "cav_common/cav_datalink.hpp"

namespace librav
{
class VirtualController
{
  public:
    VirtualController(std::shared_ptr<LCMLink> link);

  private:
    std::shared_ptr<LCMLink> data_link_;

    // void HandleLCMMessage_SyncTrigger(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::SimSyncTrigger *msg);
};
} // namespace librav

#endif /* VIRTUAL_CONTROLLER_HPP */
