/*
 * @file test_sbus.cpp
 * @date 11/19/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "input_sbus/sbus_decoder.hpp"
#include "async_port/async_serial.hpp"

using namespace xmotion;

SbusDecoder sbus_decoder;

void HandleUart(uint8_t* data, const size_t bufsize, size_t len) {
  // std::cout << "Received " << len << " bytes" << std::endl;
  // for (int i = 0; i < len; ++i) {
  //   std::cout << "0x" << std::hex << (int)data[i] << " ";
  // }
  for (size_t i = 0; i < len; ++i) {
    SbusMessage sbus_msg;
    if (sbus_decoder.SbusDecodeMessage(data[i], &sbus_msg)) {
      std::cout << "---- New SBUS frame received" << std::endl;
      for (int i = 0; i < 16; ++i) {
        std::cout << "Channel " << i << ": " << sbus_msg.channels[i]
                  << std::endl;
      }
    }
  }
}

int main(int argc, char* argv[]) {
  auto serial = std::make_shared<AsyncSerial>("/dev/ttyUSB1");
  serial->SetParity(AsyncSerial::Parity::kEven);
  serial->SetStopBits(AsyncSerial::StopBits::kTwo);
  serial->SetReceiveCallback(HandleUart);

  if (!serial->Open()) {
    std::cout << "Failed to open serial port" << std::endl;
    return -1;
  }

  serial->ChangeBaudRate(100000);

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}