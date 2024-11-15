/*
 * @file sbot_system.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SBOT_SYSTEM_HPP
#define XMOTION_SBOT_SYSTEM_HPP

#include <atomic>
#include <memory>

#include "swervebot/sbot_fsm.hpp"

namespace xmotion {
class SbotSystem {
 public:
  SbotSystem(const SbotConfig &config);
  ~SbotSystem() = default;

  // Run the application
  bool Initialize();
  void Run();
  void Stop();

 private:
  SbotConfig config_;
  std::atomic<bool> keep_running_{false};
  std::shared_ptr<WsSbotBase> sbot_;
  std::unique_ptr<SbotFsm> fsm_;
};
}  // namespace xmotion

#endif  // XMOTION_SBOT_SYSTEM_HPP