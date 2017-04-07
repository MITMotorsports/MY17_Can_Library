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
  bool brake_Throttle_conflict;
} Can_FrontCanNode_DriverOutput_T;

typedef struct {
  uint16_t right_Throttle_pot;
  uint16_t left_Throttle_pot;
  uint16_t steering_pot;
  uint16_t front_brake_pressure;
  uint16_t rear_brake_pressure;
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
