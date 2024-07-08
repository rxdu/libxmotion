/*
 * @file quadruped_system.hpp
 * @date 7/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_QUADRUPED_SYSTEM_HPP
#define QUADRUPED_MOTION_QUADRUPED_SYSTEM_HPP

#include <memory>
#include <atomic>
#include <thread>

#include "quadruped/system_config.hpp"
#include "quadruped/robot_model/quadruped_model.hpp"
#include "quadruped/control_mode_fsm.hpp"
#include "quadruped/event_handler/hid_event_handler.hpp"

namespace xmotion {
class QuadrupedSystem {
 public:
  QuadrupedSystem(const SystemConfig& config,
                  std::shared_ptr<QuadrupedModel> model);
  ~QuadrupedSystem();

  // do not allow copy or move
  QuadrupedSystem(const QuadrupedSystem&) = delete;
  QuadrupedSystem& operator=(const QuadrupedSystem&) = delete;
  QuadrupedSystem(QuadrupedSystem&&) = delete;
  QuadrupedSystem& operator=(QuadrupedSystem&&) = delete;

  // public methods
  bool Initialize();

  void Run();
  void Stop();

 private:
  void ControlSubsystem();

  const SystemConfig config_;
  std::shared_ptr<QuadrupedModel> model_;
  std::atomic<bool> keep_running_{false};

  std::shared_ptr<HidEventHandler> hid_event_listener_;

  std::thread control_thread_;
  std::atomic<bool> keep_control_loop_{false};
  std::unique_ptr<ControlModeFsm> fsm_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_QUADRUPED_SYSTEM_HPP
