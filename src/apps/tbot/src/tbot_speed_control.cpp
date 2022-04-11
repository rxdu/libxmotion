/* 
 * tbot_speed_control.cpp
 *
 * Created on 4/12/22 12:03 AM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include <thread>
#include <chrono>
#include <memory>
#include <iostream>

#include "interface/types.hpp"
#include "async_port/async_can.hpp"
#include "pid/pid_controller.hpp"

namespace robosw {
class TbotSpeedControl {
 public:
  TbotSpeedControl(std::string can) : can_dev_(can) {
    can_ = std::make_shared<AsyncCAN>(can);
    can_->SetReceiveCallback(std::bind(&TbotSpeedControl::HandleCanFrame,
                                       this, std::placeholders::_1));
  }

  void Run() {
    if (!can_->Open()) {
      std::cout << "Failed to connect to target: " << can_dev_ << std::endl;
      return;
    }

    while (can_->IsOpened()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  void HandleCanFrame(can_frame *rx_frame) {
    auto tc = RSClock::now();
    float t = std::chrono::duration_cast<std::chrono::milliseconds>(
        tc - tl_).count() / 1000.0f;
    tl_ = tc;

    std::cout << "Received frame: " << std::hex << rx_frame->can_id
              << " period: " << t << std::endl;

//    switch (rx_frame->can_id) {
//      case TBOT_ENCODER_RAW_CAN_ID: {
//        int32_t left = ToInt32(rx_frame->data[0], rx_frame->data[1],
//                               rx_frame->data[2], rx_frame->data[3]);
//        int32_t right = ToInt32(rx_frame->data[4], rx_frame->data[5],
//                                rx_frame->data[6], rx_frame->data[7]);
//        rpm_buffers_[RpmIndex::kLeft].AddPoint(t, left / 1.0f);
//        rpm_buffers_[RpmIndex::kRight].AddPoint(t, right / 1.0f);
//        break;
//      }
//    }
  }

 private:
  std::string can_dev_;
  std::shared_ptr<AsyncCAN> can_ = nullptr;

  RSTimePoint t0_{RSClock::now()};
  RSTimePoint tl_{RSClock::now()};
  PidController pid_controller_{20, 0, 0, 100, 0.02};
};
}

using namespace robosw;

int main(int argc, char *argv[]) {
  TbotSpeedControl tbot_speed_control("can0");
  tbot_speed_control.Run();
  return 0;
}