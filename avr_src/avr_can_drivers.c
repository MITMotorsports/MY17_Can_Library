#include "can_raw.h"

#include <mcp_can.h>
#include <stdint.h>

#define SS_PIN 53

static MCP_CAN delegate = MCP_CAN(SS_PIN);

void Can_Init(uint32_t baudrate) {
  if (baudrate != 500000) {
    // TODO handle error
  }
  uint8_t response = delegate.begin(CAN_500KBPS);
  if (response != CAN_OK) {
    // TODO handle error
  }
}

void Can_RawWrite(Frame *frame) {
  uint8_t response = delegate.sendMsgBuf(frame->id, 0, frame->len, frame->data);
  if (response != CAN_OK) {
    // TODO handle error
  }
  
}

bool Can_RawRead(Frame *frame) {
  if (delegate.checkReceive() != CAN_MSGAVAIL) {
    return false;
  }
  uint8_t response = delegate.readMsgBuf(&(frame->len), frame->data);
  if (frame->len == 0 || response != CAN_OK) {
    // TODO handle error
    return false;
  }
  frame->id = delegate.getCanId();
  return true;
}
