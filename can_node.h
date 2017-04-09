#ifndef _MY17_CAN_LIBRARY_CAN_NODE_H
#define _MY17_CAN_LIBRARY_CAN_NODE_H

#include "can_validator/fsae_can_spec.h"

#include <stdint.h>
#include <stdbool.h>

#include "ids.h"

typedef struct {
  int16_t torque;
  uint8_t brake_pressure;
  uint8_t steering_position;
  bool throttle_implausible;
  bool brake_throttle_conflict;
} Can_FrontCanNode_DriverOutput_T;

typedef struct {
  uint16_t accel_1_raw;
  uint16_t accel_2_raw;
  uint16_t brake_1_raw;
  uint16_t brake_2_raw;
} Can_FrontCanNode_RawValues_T;

typedef struct {
  uint32_t front_right_wheel_speed;
  uint32_t front_left_wheel_speed;
} Can_FrontCanNode_WheelSpeed_T;

typedef struct {
  uint32_t rear_right_wheel_speed;
  uint32_t rear_left_wheel_speed;
} Can_RearCanNode_WheelSpeed_T;

#endif // _MY17_CAN_LIBRARY_CAN_NODE_H
