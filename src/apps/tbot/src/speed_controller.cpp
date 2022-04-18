/* 
 * speed_controller.cpp
 *
 * Created on 4/18/22 11:17 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/speed_controller.hpp"

namespace robosw {
namespace {
int32_t ToInt32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  int32_t ret = 0;
  ret = static_cast<int32_t>(static_cast<uint32_t>(b0) << 24 |
      static_cast<uint32_t>(b1) << 16 |
      static_cast<uint32_t>(b2) << 8 |
      static_cast<uint32_t>(b3) << 0);
  return ret;
}
}

void SpeedController::Run(std::string can) {
  can_dev_ = can;
  if (can_ == nullptr) {
    can_ = std::make_shared<AsyncCAN>(can);
  }
  can_->SetReceiveCallback(std::bind(&SpeedController::HandleCanFrame,
                                     this, std::placeholders::_1));

  if (!can_->Open()) {
    std::cout << "Failed to connect to target: " << can_dev_ << std::endl;
    return;
  }

  while (can_->IsOpened()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void SpeedController::Stop() {
  if (can_ && can_->IsOpened()) {
    can_->Close();
  }
}

void SpeedController::SendPwmCommand(float left, float right) {
  if (can_ == nullptr) return;
  struct can_frame frame;
  frame.can_id = 0x101;
  frame.can_dlc = 2;
  frame.data[0] = static_cast<uint8_t>(left);
  frame.data[1] = static_cast<uint8_t>(right);
  can_->SendFrame(frame);
}

void SpeedController::HandleCanFrame(can_frame *rx_frame) {
  auto tc = RSClock::now();
  float t = std::chrono::duration_cast<std::chrono::milliseconds>(
      tc - tl_).count() / 1000.0f;
  tl_ = tc;

  switch (rx_frame->can_id) {
    case 0x212: {
      int32_t left = ToInt32(rx_frame->data[0], rx_frame->data[1],
                             rx_frame->data[2], rx_frame->data[3]);
      int32_t right = ToInt32(rx_frame->data[4], rx_frame->data[5],
                              rx_frame->data[6], rx_frame->data[7]);
      float u_left = left_pid_controller_.Update(120, left);
      float u_right = left_pid_controller_.Update(80, right);

      std::cout << "left: " << left << " , u: " << u_left << std::endl;
      SendPwmCommand(u_left, u_right);
      break;
    }
  }

//    std::cout << "Received frame: " << std::hex << rx_frame->can_id
//              << " period: " << t << std::endl;

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
}