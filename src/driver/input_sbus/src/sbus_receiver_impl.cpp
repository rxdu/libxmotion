/*
 * @file sbus_receiver_impl.cpp
 * @date 11/23/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "input_sbus/sbus_receiver.hpp"

#include <string>
#include <iostream>
#include <thread>
#include <atomic>

#include "rpi_sbus/SBUS.h"

namespace xmotion {
class SbusReceiver::Impl {
 public:
  Impl(const std::string& port) : port_(port) {}

  ~Impl() {
    if (keep_running_) Close();
  }

  bool Open() {
    sbus_.onPacket(std::bind(&Impl::OnPacket, this, std::placeholders::_1));
    sbus_err_t err =
        sbus_.install(port_.c_str(), true);  // true for blocking mode
    if (err != SBUS_OK) {
      std::cerr << "SBus install error: " << err << std::endl;
      return false;
    }

    keep_running_ = true;
    sbus_thread_ = std::thread([this]() {
      while (keep_running_) {
        sbus_err_t err = sbus_.read();
        if (err != SBUS_OK) {
          std::cerr << "SBus read error: " << err << std::endl;
          break;
        }
      }
    });

    return true;
  }

  void Close() {
    keep_running_ = false;
    if (sbus_thread_.joinable()) sbus_thread_.join();
    sbus_.uninstall();
  }

  void SetOnSbusMessageReceivedCallback(OnSbusMessageReceivedCallback cb) {
    on_sbus_message_received_cb_ = cb;
  }

 private:
  void OnPacket(const sbus_packet_t& packet) {
    SbusMessage msg;
    for (int i = 0; i < 16; ++i) {
      msg.channels[i] = packet.channels[i];
    }
    msg.channels[16] = packet.ch17;
    msg.channels[17] = packet.ch18;
    msg.frame_loss = packet.frameLost;
    msg.fault_protection = packet.failsafe;

    if (on_sbus_message_received_cb_) on_sbus_message_received_cb_(msg);
  }

  std::string port_;
  SBUS sbus_;
  OnSbusMessageReceivedCallback on_sbus_message_received_cb_;

  std::atomic_bool keep_running_{false};
  std::thread sbus_thread_;
};
}  // namespace xmotion