#include "MY17_Can_Library.h"

#include "evil_macros.h"

typedef enum {
  LITTLE,
  BIG,
} ENDIAN_T;

#ifdef CAN_ARCHITECTURE_ARM
#include "arm_src/arm_can_drivers.c"
#elif CAN_ARCHITECTURE_AVR
#include "avr_src/avr_can_drivers.c"
#elif CAN_ARCHITECTURE_TEST

void Can_Init(uint32_t baudrate) {
  // TODO test harness
}

void Can_RawWrite(Frame *frame) {
  // TODO test harness
}

bool Can_RawRead(Frame *frame) {
  // TODO test harness
  return false;
}

int main(void) {
  // TODO test harness
}

#else
  // Define nothing so that there is a linker error!
#endif

Can_MsgID_T Can_MsgType(void) {
  bool result = Can_RawRead(&lastMessage);
  if (!result) {
    unread = false;
    return Can_No_Msg;
  }
  unread = true;

  uint16_t id = lastMessage.id;
  switch(id) {
    case FRONT_CAN_NODE_DRIVER_OUTPUT__id:
      return Can_FrontCanNode_DriverOutput_Msg;
    case FRONT_CAN_NODE_RAW_VALUES__id:
      return Can_FrontCanNode_RawValues_Msg;
    default:
      return Can_No_Msg;
  }
}

TO_CAN(Can_FrontCanNode_DriverOutput) {
  can_out->id = FRONT_CAN_NODE_DRIVER_OUTPUT__id;
  can_out->len = 5;
  can_out->data[0] = type_in->torque & 0xFF;
  can_out->data[1] = (type_in->torque & 0xFF00) >> 8;
  can_out->data[2] = type_in->brake_pressure;
  can_out->data[3] = type_in->steering_position;
  can_out->data[4] = 0;
  TOGGLE(can_out->data[4], type_in->throttle_implausible, 0);
  TOGGLE(can_out->data[4], type_in->brake_throttle_conflict, 1);
}

FROM_CAN(Can_FrontCanNode_DriverOutput) {
  type_out->torque = can_in->data[0] + (can_in->data[1] << 8);
  type_out->brake_pressure = can_in->data[2];
  type_out->steering_position = can_in->data[3];
  type_out->throttle_implausible = CHECK(can_in->data[4], 0);
  type_out->brake_throttle_conflict = CHECK(can_in->data[4], 1);
}

TO_CAN(Can_FrontCanNode_RawValues) {
  can_out->id = FRONT_CAN_NODE_RAW_VALUES__id;
  can_out->len = 8;
  can_out->data[0] = type_in->accel_1_raw & 0xFF;
  can_out->data[1] = (type_in->accel_1_raw & 0xFF00) >> 8;
  can_out->data[2] = type_in->accel_2_raw & 0xFF;
  can_out->data[3] = (type_in->accel_2_raw & 0xFF00) >> 8;
  can_out->data[4] = type_in->brake_1_raw & 0xFF;
  can_out->data[5] = (type_in->brake_1_raw & 0xFF00) >> 8;
  can_out->data[6] = type_in->brake_2_raw & 0xFF;
  can_out->data[7] = (type_in->brake_2_raw & 0xFF00) >> 8;
}

FROM_CAN(Can_FrontCanNode_RawValues) {
  type_out->accel_1_raw =
    (can_in->data[0] & 0x00FF) | (can_in->data[1] << 8);
  type_out->accel_2_raw =
    (can_in->data[2] & 0x00FF) | (can_in->data[3] << 8);
  type_out->brake_1_raw =
    (can_in->data[4] & 0x00FF) | (can_in->data[5] << 8);
  type_out->brake_2_raw =
    (can_in->data[6] & 0x00FF) | (can_in->data[7] << 8);
}

// Needed for actual implementation of the evil macros
DEFINE(Can_FrontCanNode_DriverOutput)
DEFINE(Can_FrontCanNode_RawValues)
