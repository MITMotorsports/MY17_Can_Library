#ifndef _MY17_CAN_LIBRARY_BMS_H
#define _MY17_CAN_LIBRARY_BMS_H

#include "can_validator/fsae_can_spec.h"

#include <stdint.h>
#include <stdbool.h>

#include "ids.h"

typedef struct {
  Can_Bms_StateID_T state;
  uint16_t soc;
} Can_Bms_Heartbeat_T;

typedef struct {
  uint8_t avg_cell_Temp;
  uint8_t min_cell_Temp;
  uint8_t max_cell_Temp;
  uint8_t id_min_cell_Temp;
  uint8_t id_max_cell_Temp;
} Can_Bms_CellTemps_T;

typedef struct {
  uint16_t pack_voltage;
  uint16_t pack_current;
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
