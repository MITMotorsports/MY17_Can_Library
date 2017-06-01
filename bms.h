#ifndef _MY17_CAN_LIBRARY_BMS_H
#define _MY17_CAN_LIBRARY_BMS_H

#include "can_validator/fsae_can_spec.h"

#include <stdint.h>
#include <stdbool.h>

#include "ids.h"

typedef struct {
  Can_Bms_StateID_T state;
  uint16_t soc;
  bool fan_enable;
  bool dcdc_enable;
  bool dcdc_fault;
} Can_Bms_Heartbeat_T;

typedef struct {
  int16_t avg_cell_temp;
  int16_t min_cell_temp;
  int16_t max_cell_temp;
  uint16_t id_min_cell_temp;
  uint16_t id_max_cell_temp;
} Can_Bms_CellTemps_T;

typedef struct {
  uint16_t pack_voltage;
  int16_t pack_current;
  uint16_t avg_cell_voltage;
  uint16_t min_cell_voltage;
  uint8_t id_min_cell_voltage;
  uint16_t max_cell_voltage;
  uint8_t id_max_cell_voltage;
} Can_Bms_PackStatus_T;

typedef struct {
  Can_Bms_ErrorID_T type;
} Can_Bms_Error_T;

#endif // _MY17_CAN_LIBRARY_BMS_H
