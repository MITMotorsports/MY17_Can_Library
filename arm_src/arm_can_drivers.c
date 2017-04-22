#include "can_raw.h"

#include <stdint.h>

#include "chip.h"

#include "hardware_can.c"

void Can_Init(uint32_t baudrate) {
  Hardware_CAN_Init(baudrate);
}

void Can_Pass() {
  Hardware_CAN_Pass();
}

void Can_RawWrite(Frame *frame) {
  const uint32_t can_out_id = (uint32_t) (frame->id);
  const uint8_t can_out_bytes = frame->len;
  uint8_t data[can_out_bytes];

  uint8_t i;
  for (i = 0; i < can_out_bytes; i++) {
    data[i] = frame->data[i];
  }

  uint32_t ret = Hardware_CAN_Transmit(can_out_id, data, can_out_bytes);
  if (ret != HARDWARE_NO_CAN_ERROR) {
    // TODO handle error
  }
}

bool Can_RawRead(Frame *frame) {
  CCAN_MSG_OBJ_T rx_msg;
  HARDWARE_CAN_ERROR_T ret = Hardware_CAN_Receive(&rx_msg);
  if (ret == HARDWARE_NO_CAN_ERROR) {
    frame->id = rx_msg.mode_id;
    frame->len = rx_msg.dlc;
    uint8_t i;
    for (i = 0; i < frame->len; i++) {
      frame->data[i] = rx_msg.data[i];
    }
    return true;
  } else if (ret != HARDWARE_NO_RX_CAN_MESSAGE) {
    // TODO handle error
    return false;
  } else {
    return false;
  }
}
