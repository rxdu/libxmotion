/*
 * @file control_context.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_CONTROL_CONTEXT_HPP
#define XMOTION_CONTROL_CONTEXT_HPP

#include <memory>

#include "swervebot/sbot_config.hpp"
#include "swervebot/ws_sbot_base.hpp"

namespace xmotion {
struct ControlContext {
  SbotConfig config;
  std::shared_ptr<WsSbotBase> robot_base;
};
}  // namespace xmotion

#endif  // XMOTION_CONTROL_CONTEXT_HPP