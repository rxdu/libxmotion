/*
 * sbus_stm32f4.c
 *
 * Created on: Aug 16, 2019 10:33
 * Description:
 *
 * Reference:
 *  [1] https://blog.csdn.net/Brendon_Tan/article/details/89854751
 *  [2] https://blog.csdn.net/yjl_programmer/article/details/88876582
 *  [3] https://blog.csdn.net/qq_31232793/article/details/80244211
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "input_sbus/sbus_decoder.hpp"

#include <string.h>

namespace xmotion {
SbusDecoder::SbusDecoder() { SbusDecoderInit(); }

void SbusDecoder::SbusDecoderInit() {
  decode_buffer_.decode_state = WAIT_FOR_SOF;
}

bool SbusDecoder::SbusDecodeMessage(uint8_t ch, SbusMessage *sbus_msg) {
  bool new_frame_parsed = false;
  switch (decode_buffer_.decode_state) {
    case WAIT_FOR_SOF: {
      decode_buffer_.indexer = 0;
      memset(decode_buffer_.data, 0, SBUS_FRAME_LENGTH);
      //   DWriteString(0, "SBUS: WAIT_FOR_SOF\n");
      if (ch == sbus_frame_head) {
        decode_buffer_.decode_state = WAIT_FOR_PAYLOAD;
        decode_buffer_.data[decode_buffer_.indexer] = ch;
      }
      break;
    }
    case WAIT_FOR_PAYLOAD: {
      //   DWriteString(0, "SBUS: WAIT_FOR_PAYLOAD\n");
      decode_buffer_.data[++decode_buffer_.indexer] = ch;
      if (decode_buffer_.indexer == (SBUS_FRAME_LENGTH - 2))
        decode_buffer_.decode_state = WAIT_FOR_EOF;
      break;
    }
    case WAIT_FOR_EOF: {
      //   DWriteString(0, "SBUS: WAIT_FOR_EOF\n");
      if (ch == sbus_frame_tail) {
        // new frame found
        new_frame_parsed = true;
        decode_buffer_.data[++decode_buffer_.indexer] = ch;
        SbusBuildMessage(decode_buffer_.data, sbus_msg);
        // reset state
        decode_buffer_.decode_state = WAIT_FOR_SOF;
      } else {  // Invalid SBUS frame
        decode_buffer_.decode_state = WAIT_FOR_SOF;
      }
      break;
    }
  }
  return new_frame_parsed;
}

void SbusDecoder::SbusBuildMessage(uint8_t *sframe_buf, SbusMessage *sbus_msg) {
  sbus_msg->channels[0] =
      ((int16_t)sframe_buf[1] >> 0 | ((int16_t)sframe_buf[2] << 8)) & 0x07FF;
  sbus_msg->channels[1] =
      ((int16_t)sframe_buf[2] >> 3 | ((int16_t)sframe_buf[3] << 5)) & 0x07FF;
  sbus_msg->channels[2] =
      ((int16_t)sframe_buf[3] >> 6 | ((int16_t)sframe_buf[4] << 2) |
       (int16_t)sframe_buf[5] << 10) &
      0x07FF;
  sbus_msg->channels[3] =
      ((int16_t)sframe_buf[5] >> 1 | ((int16_t)sframe_buf[6] << 7)) & 0x07FF;
  sbus_msg->channels[4] =
      ((int16_t)sframe_buf[6] >> 4 | ((int16_t)sframe_buf[7] << 4)) & 0x07FF;
  sbus_msg->channels[5] =
      ((int16_t)sframe_buf[7] >> 7 | ((int16_t)sframe_buf[8] << 1) |
       (int16_t)sframe_buf[9] << 9) &
      0x07FF;
  sbus_msg->channels[6] =
      ((int16_t)sframe_buf[9] >> 2 | ((int16_t)sframe_buf[10] << 6)) & 0x07FF;
  sbus_msg->channels[7] =
      ((int16_t)sframe_buf[10] >> 5 | ((int16_t)sframe_buf[11] << 3)) & 0x07FF;

  sbus_msg->channels[8] =
      ((int16_t)sframe_buf[12] << 0 | ((int16_t)sframe_buf[13] << 8)) & 0x07FF;
  sbus_msg->channels[9] =
      ((int16_t)sframe_buf[13] >> 3 | ((int16_t)sframe_buf[14] << 5)) & 0x07FF;
  sbus_msg->channels[10] =
      ((int16_t)sframe_buf[14] >> 6 | ((int16_t)sframe_buf[15] << 2) |
       (int16_t)sframe_buf[16] << 10) &
      0x07FF;
  sbus_msg->channels[11] =
      ((int16_t)sframe_buf[16] >> 1 | ((int16_t)sframe_buf[17] << 7)) & 0x07FF;
  sbus_msg->channels[12] =
      ((int16_t)sframe_buf[17] >> 4 | ((int16_t)sframe_buf[18] << 4)) & 0x07FF;
  sbus_msg->channels[13] =
      ((int16_t)sframe_buf[18] >> 7 | ((int16_t)sframe_buf[19] << 1) |
       (int16_t)sframe_buf[20] << 9) &
      0x07FF;
  sbus_msg->channels[14] =
      ((int16_t)sframe_buf[20] >> 2 | ((int16_t)sframe_buf[21] << 6)) & 0x07FF;
  sbus_msg->channels[15] =
      ((int16_t)sframe_buf[21] >> 5 | ((int16_t)sframe_buf[22] << 3)) & 0x07FF;

  // sbus_msg->channels[16] = sframe_buf[23];
  sbus_msg->channels[16] = sframe_buf[23] & 0x80;
  sbus_msg->channels[17] = sframe_buf[23] & 0x40;

  sbus_msg->frame_loss = sframe_buf[23] & 0x20;
  sbus_msg->fault_protection = sframe_buf[23] & 0x10;
}

bool SbusDecoder::ValidateSbusMessage(const SbusMessage *sbus_msg) {
  if (sbus_msg->fault_protection || sbus_msg->frame_loss) return false;

  uint8_t zero_num = 0;
  for (int i = 0; i < 8; ++i) {
    if (sbus_msg->channels[i] == 0) zero_num++;
    if (zero_num > 4) return false;
  }

  return true;
}
}  // namespace xmotion