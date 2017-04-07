#ifndef _MY17_CAN_LIBRARY_CURRENT_SENSOR_H
#define _MY17_CAN_LIBRARY_CURRENT_SENSOR_H

#include "can_validator/fsae_can_spec.h"

#include <stdint.h>
#include <stdbool.h>

#include "ids.h"

typedef struct {
  int32_t current_mA;
} Can_CurrentSensor_Current_T;

typedef struct {
  int32_t voltage_mV;
} Can_CurrentSensor_Voltage_T;

typedef struct {
  int32_t power_W;
} Can_CurrentSensor_Power_T;

typedef struct {
  int32_t energy_Wh;
} Can_CurrentSensor_Energy_T;

#endif // _MY17_CAN_LIBRARY_CURRENT_SENSOR_H
