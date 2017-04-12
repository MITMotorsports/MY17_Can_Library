#include "MY17_Can_Library.h"
#include "evil_macros.h"

static bool unread;
static Frame lastMessage;

#define DEFINE(name) \
  bool name ##_Read(name ## _T *type) { \
    if (unread) { \
      name ## _FromCan(&lastMessage, type); \
      unread = false; \
      return true; \
    } else { \
      return false; \
    } \
  } \
  void name ##_Write(name ## _T *type) { \
    Frame frame; \
    name ## _ToCan(type, &frame); \
    Can_RawWrite(&frame); \
  }

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
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->torque, bitstring, 0, 16);
  bitstring = INSERT(type_in->brake_pressure, bitstring, 16, 8);
  bitstring = INSERT(type_in->steering_position, bitstring, 24, 8);
  bitstring = INSERT(type_in->throttle_implausible, bitstring, 32, 1);
  bitstring = INSERT(type_in->brake_throttle_conflict, bitstring, 33, 1);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = FRONT_CAN_NODE_DRIVER_OUTPUT__id;
  can_out->len = 5;
}

FROM_CAN(Can_FrontCanNode_DriverOutput) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->torque = EXTRACT(bitstring, 0, 16);
  type_out->brake_pressure = EXTRACT(bitstring, 16, 8);
  type_out->steering_position = EXTRACT(bitstring, 24, 8);
  type_out->throttle_implausible = EXTRACT(bitstring, 32, 1);
  type_out->brake_throttle_conflict = EXTRACT(bitstring, 33, 1);
}

TO_CAN(Can_FrontCanNode_RawValues) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->accel_1_raw, bitstring, 0, 10);
  bitstring = INSERT(type_in->accel_2_raw, bitstring, 10, 10);
  bitstring = INSERT(type_in->brake_1_raw, bitstring, 20, 10);
  bitstring = INSERT(type_in->brake_2_raw, bitstring, 30, 10);
  bitstring = INSERT(type_in->steering_raw, bitstring, 40, 10);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = FRONT_CAN_NODE_RAW_VALUES__id;
  can_out->len = 8;
}

FROM_CAN(Can_FrontCanNode_RawValues) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->accel_1_raw = EXTRACT(bitstring, 0, 10);
  type_out->accel_2_raw = EXTRACT(bitstring, 10, 10);
  type_out->brake_1_raw = EXTRACT(bitstring, 20, 10);
  type_out->brake_2_raw = EXTRACT(bitstring, 30, 10);
  type_out->steering_raw = EXTRACT(bitstring, 40, 10);
}

// Needed for actual implementation of the evil macros
DEFINE(Can_FrontCanNode_DriverOutput)
DEFINE(Can_FrontCanNode_RawValues)
