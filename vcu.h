#ifndef _MY17_CAN_LIBRARY_VCU_H
#define _MY17_CAN_LIBRARY_VCU_H

#include "can_validator/fsae_can_spec.h"

#include <stdint.h>
#include <stdbool.h>

#include "ids.h"

typedef struct {
  bool alwaysTrue;
} Can_Vcu_BmsHeartbeat_T;

typedef struct {
  bool rtd_light;
  bool ams_light;
  bool imd_light;
  bool hv_light;
  bool traction_control;
  bool limp_mode;
  bool lv_warning;
  bool active_aero;
  bool regen;
  bool shutdown_esd_drain;
  bool shutdown_bms;
  bool shutdown_imd;
  bool shutdown_bspd;
  bool shutdown_vcu;
  bool shutdown_precharge;
  bool shutdown_master_reset;
  bool shutdown_driver_reset;
  uint8_t lv_battery_voltage;
} Can_Vcu_DashHeartbeat_T;

typedef struct {
  Can_MC_RegID_T requestType;
  uint8_t period;
} Can_Vcu_MCRequest_T;

typedef struct {
  int16_t torque_cmd;
} Can_Vcu_MCTorque_T;

#endif // _MY17_CAN_LIBRARY_VCU_H