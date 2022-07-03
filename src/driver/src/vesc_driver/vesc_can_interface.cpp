/* 
 * vesc_can_interface.cpp
 *
 * Created on 6/30/22 10:21 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "vesc_driver/vesc_can_interface.hpp"

#include <iostream>
#include <functional>

#include "vesc_driver/vesc_cmd_packet.hpp"
#include "vesc_driver/vesc_status_packet.hpp"

namespace robosw {
bool VescCanInterface::Connect(const std::string &can, uint8_t vesc_id) {
  vesc_id_ = vesc_id;
  can_ = std::make_shared<AsyncCAN>(can);
  can_->SetReceiveCallback(std::bind(&VescCanInterface::HandleCanFrame, this, std::placeholders::_1));
  return can_->Open();
}

void VescCanInterface::Disconnect() {
  if (can_ && can_->IsOpened()) {
    can_->Close();
  }
}

void VescCanInterface::SetStateUpdatedCallback(StateUpdatedCallback cb) {
  state_updated_callback_ = cb;
}

StampedVescState VescCanInterface::GetLastState() const {
  std::lock_guard<std::mutex> lock(state_mtx_);
  return stamped_state_;
}

void VescCanInterface::HandleCanFrame(const struct can_frame *frame) {
  uint32_t can_id = frame->can_id & 0x1fffffff;
//  std::cout << "received: " << std::hex << can_id << " id: " << (VescFrame::VescStatus5FrameId | vesc_id_)
//            << std::endl;
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    stamped_state_.time = VescClock::now();
    if (can_id == (VescFrame::VescStatus1FrameId | vesc_id_)) {
      auto pkt = VescStatus1Packet(*frame);
      stamped_state_.field = VescStateUpdatedField::kStatus1;
      stamped_state_.state.speed = pkt.GetRpm();
      stamped_state_.state.current_motor = pkt.GetCurrent();
      stamped_state_.state.duty_cycle = pkt.GetDuty();
    } else if (can_id == (VescFrame::VescStatus2FrameId | vesc_id_)) {
      auto pkt = VescStatus2Packet(*frame);
      stamped_state_.field = VescStateUpdatedField::kStatus2;
      stamped_state_.state.charge_drawn = pkt.GetAmpHours();
      stamped_state_.state.charge_regen = pkt.GetAmpHoursCharged();
    } else if (can_id == (VescFrame::VescStatus3FrameId | vesc_id_)) {
      auto pkt = VescStatus3Packet(*frame);
      stamped_state_.field = VescStateUpdatedField::kStatus3;
      stamped_state_.state.energy_drawn = pkt.GetWattHours();
      stamped_state_.state.energy_regen = pkt.GetWattHoursCharged();
    } else if (can_id == (VescFrame::VescStatus4FrameId | vesc_id_)) {
      auto pkt = VescStatus4Packet(*frame);
      stamped_state_.field = VescStateUpdatedField::kStatus4;
      stamped_state_.state.temperature_pcb = pkt.GetTempFET();
      stamped_state_.state.current_input = pkt.GetCurrentIn();
    } else if (can_id == (VescFrame::VescStatus5FrameId | vesc_id_)) {
      auto pkt = VescStatus5Packet(*frame);
      stamped_state_.field = VescStateUpdatedField::kStatus5;
      stamped_state_.state.displacement = pkt.GetTachoValue();
      stamped_state_.state.voltage_input = pkt.GetVoltageIn();
    }
  }

  if (state_updated_callback_) {
    state_updated_callback_(stamped_state_);
  }
}

// TODO
void VescCanInterface::RequestFwVersion() {

}

// TODO
void VescCanInterface::RequestState() {

}

// TODO
void VescCanInterface::RequestImuData() {

}

void VescCanInterface::SetDutyCycle(double duty_cycle) {
  if (can_ == nullptr || !can_->IsOpened()) return;
  VescSetDutyCycleCmdPacket pkt(vesc_id_, duty_cycle);
  can_->SendFrame(pkt.GetCanFrame());
}

void VescCanInterface::SetCurrent(double current) {
  if (can_ == nullptr || !can_->IsOpened()) return;
  VescSetCurrentCmdPacket pkt(vesc_id_, current);
  can_->SendFrame(pkt.GetCanFrame());
}

void VescCanInterface::SetBrake(double brake) {
  if (can_ == nullptr || !can_->IsOpened()) return;
  VescSetCurrentBrakeCmdPacket pkt(vesc_id_, brake);
  can_->SendFrame(pkt.GetCanFrame());
}

void VescCanInterface::SetSpeed(double speed) {
  if (can_ == nullptr || !can_->IsOpened()) return;
  VescSetRpmCmdPacket pkt(vesc_id_, speed);
  can_->SendFrame(pkt.GetCanFrame());
}

void VescCanInterface::SetPosition(double position) {
  if (can_ == nullptr || !can_->IsOpened()) return;
  VescSetPositionCmdPacket pkt(vesc_id_, position);
  can_->SendFrame(pkt.GetCanFrame());
}

void VescCanInterface::SetServo(double servo) {
  if (can_ == nullptr || !can_->IsOpened()) return;
  VescSetServoPosCmdPacket pkt(vesc_id_, servo);
  can_->SendFrame(pkt.GetCanFrame());
}
}
