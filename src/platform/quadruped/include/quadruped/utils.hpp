/*
 * @file utils.hpp
 * @date 7/13/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_UTILS_HPP
#define QUADRUPED_MOTION_UTILS_HPP

#include <optional>

#include "quadruped/system_config.hpp"
#include "quadruped/control_context.hpp"

namespace xmotion {
namespace Utils {
std::optional<HidSettings::KeyFunction> PollKeyFunction(
    ControlContext &context, HidEventHandler::KeyboardEventType type);
}
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_UTILS_HPP
