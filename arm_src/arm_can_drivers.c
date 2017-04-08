#include "can_raw.h"

#include <stdint.h>

#include "chip.h"
#include "can.h"
#include "ccand_11xx.h"

void Can_Init(uint32_t baudrate) {
  CAN_Init(baudrate);
}

void Can_RawWrite(Frame *frame) {
  const uint32_t can_out_id = (uint32_t) (frame->id);
  const uint8_t can_out_bytes = frame->len;
  uint8_t data[can_out_bytes];

  for (uint8_t i = 0; i < can_out_bytes; i++) {
    data[i] = frame->data[i];
  }

  uint32_t ret = CAN_Transmit(can_out_id, data, can_out_bytes);
  if (ret != NO_CAN_ERROR) {
    // TODO handle error
  }
}

bool Can_RawRead(Frame *frame) {
  CCAN_MSG_OBJ_T rx_msg;
  CAN_ERROR_T ret = CAN_Receive(&rx_msg);
  if (ret == NO_CAN_ERROR) {
    frame->id = rx_msg.mode_id;
    frame->len = rx_msg.dlc;
    for (uint8_t i = 0; i < frame->len; i++) {
      frame->data[i] = rx_msg.data[i];
    }
    return true;
  } else if (ret != NO_RX_CAN_MESSAGE) {
    // TODO handle error
    return false;
  } else {
    return false;
  }
}
