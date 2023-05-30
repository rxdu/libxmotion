/*
 * messenger.hpp
 *
 * Created on 4/4/22 9:52 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_INCLUDE_MESSENGER_HPP
#define ROBOSW_SRC_APPS_TBOT_INCLUDE_MESSENGER_HPP

#include <memory>
#include <unordered_map>

#include "interface/types.hpp"
#include "interface/driver/can_adapter.hpp"
#include "imview/data_buffer.hpp"

namespace robosw {
class Messenger {
 public:
  enum class DataBufferIndex : RSEnumBaseType {
    kRawRpmLeft = 0,
    kRawRpmRight,
    kFilteredRpmLeft,
    kFilteredRpmRight,
    kTargetRpmLeft,
    kTargetRpmRight
  };

  typedef enum {
    kNonSupervised = 0,
    kSupervisedPwm = 1,
    kSupervisedRpm = 2
  } SupervisedMode;

  struct SupervisorCommand {
    SupervisedMode supervised_mode;
  };

  struct SupervisedState {
    SupervisedMode supervised_mode;
  };

 public:
  Messenger(RSTimePoint tp);

  bool Start(std::string can);
  void Stop();
  bool IsStarted();

  SupervisedState GetSupervisedState() const;
  void SendSupervisorCommand(SupervisorCommand cmd);

  void SendPwmCommand(float left, float right);
  void SendRpmCommand(int32_t left, int32_t right);
  void SendMotionCommand(float linear, float angular);

  swviz::DataBuffer &GetDataBuffer(DataBufferIndex idx);

 private:
  RSTimePoint t0_;
  std::shared_ptr<CanAdapter> can_ = nullptr;

  SupervisedState supervised_state_;
  std::unordered_map<DataBufferIndex, swviz::DataBuffer> rpm_buffers_;

  void HandleCanFrame(const can_frame *rx_frame);
};
}  // namespace robosw

#endif  // ROBOSW_SRC_APPS_TBOT_INCLUDE_MESSENGER_HPP
