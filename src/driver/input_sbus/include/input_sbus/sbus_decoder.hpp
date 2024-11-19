/*
 * sbus.h
 *
 * Created on: Feb 10, 2020 17:54
 * Description:
 *
 * 1 Start, 8 Data Bits, Even Parity, 2 Stop Bits @ 100k bps
 * Frame interval: 7ms (high-speed mode), 11ms (emulated mode)
 * Frame structure:
 *
 * [0]      0x0f
 * [1-22]   channel data [CH1-16] (22 Bytes = 22 * 8 Bits = 16 * 11 Bits)
 * [23]     [D-CH17][D-CH18][SIG_LOST][FAULT][RSV3][RSV2][RSV1][RSV0]
 * [24]     0x00
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SBUS_SBUS_H
#define SBUS_SBUS_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

namespace xmotion {
// 16 analog + 2 digital (encoded in one byte)
#define SBUS_CHANNEL_NUMBER 18
#define SBUS_DMA_RX_BUFFER_SIZE 64
#define SBUS_FRAME_LENGTH 25

typedef struct {
  uint16_t channels[SBUS_CHANNEL_NUMBER] __attribute__((aligned(8)));
  bool frame_loss __attribute__((aligned(8)));
  bool fault_protection __attribute__((aligned(8)));
} SbusMessage;

class SbusDecoder {
 public:
  struct SbusDecodeBuffer {
    uint8_t decode_state;
    uint8_t data[SBUS_FRAME_LENGTH];
    size_t indexer;
  };

  enum SbusDecodeState { WAIT_FOR_SOF = 0, WAIT_FOR_PAYLOAD, WAIT_FOR_EOF };

 public:
  SbusDecoder();
  ~SbusDecoder() = default;

  void SbusDecoderInit();
  bool SbusDecodeMessage(uint8_t ch, SbusMessage *sbus_msg);
  bool ValidateSbusMessage(const SbusMessage *sbus_msg);

 private:
  static constexpr uint8_t sbus_frame_head = 0x0f;
  static constexpr uint8_t sbus_frame_tail = 0x00;

  void SbusBuildMessage(uint8_t *sframe_buf, SbusMessage *sbus_msg);

  SbusDecodeBuffer decode_buffer_;
};
}  // namespace xmotion

#endif /* SBUS_SBUS_H */
