/*
 * @file sbus_receiver.cpp
 * @date 11/23/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "input_sbus/sbus_receiver.hpp"

#include "sbus_receiver_impl.cpp"

namespace xmotion {
SbusReceiver::SbusReceiver(const std::string& port)
    : pimpl_(std::make_unique<Impl>(port)) {}

SbusReceiver::~SbusReceiver() = default;

bool SbusReceiver::Open() { return pimpl_->Open(); }

void SbusReceiver::Close() { pimpl_->Close(); }

void SbusReceiver::SetOnSbusMessageReceivedCallback(
    OnSbusMessageReceivedCallback cb) {
  pimpl_->SetOnSbusMessageReceivedCallback(cb);
}
}  // namespace xmotion