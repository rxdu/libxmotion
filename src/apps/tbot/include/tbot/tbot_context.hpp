/* 
 * tbot_context.hpp
 *
 * Created on 4/4/22 9:38 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_CONTEXT_HPP
#define ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_CONTEXT_HPP

#include <memory>

#include "async_port/async_can.hpp"

namespace robosw {
struct TbotContext {
  std::shared_ptr<AsyncCAN> can_ = nullptr;
};
}

#endif //ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_CONTEXT_HPP
