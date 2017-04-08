#include "MY17_Can_Library.h"

#include "evil_macros.h"

typedef enum {
  LITTLE,
  BIG,
} ENDIAN_T;

void Can_Raw_Write(Frame *frame) {
  // TODO
}

void Can_Raw_Read(Frame *frame) {
  // TODO
}

TO_CAN(Can_FrontCanNode_DriverOutput) {
  can_out->id = FRONT_CAN_NODE_DRIVER_OUTPUT__id;
  can_out->len = 8;
  int16_t torque = type_in->torque;
}

FROM_CAN(Can_FrontCanNode_DriverOutput) {
  type_out->torque = 0;
}

// Needed for actual implementation of the evil macros
DEFINE(Can_FrontCanNode_DriverOutput)

int main() {
  return 0;
}
