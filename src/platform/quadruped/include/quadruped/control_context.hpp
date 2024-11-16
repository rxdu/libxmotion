/*
 * @file control_context.hpp
 * @date 7/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_CONTROL_CONTEXT_HPP
#define QUADRUPED_MOTION_CONTROL_CONTEXT_HPP

#include <memory>

#include "quadruped/system_config.hpp"
#include "quadruped/robot_model/quadruped_model.hpp"
#include "quadruped/estimator/estimator_interface.hpp"
#include "quadruped/event_handler/hid_event_handler.hpp"

namespace xmotion {
struct ControlContext {
  SystemConfig system_config;
  std::shared_ptr<QuadrupedModel> robot_model;
  std::shared_ptr<EstimatorInterface> estimator;
  std::shared_ptr<HidEventHandler> hid_event_listener;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_CONTROL_CONTEXT_HPP
