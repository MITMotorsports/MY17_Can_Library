#ifndef _MY17_CAN_LIBRARY_VCU_H
#define _MY17_CAN_LIBRARY_VCU_H

#include "can_validator/fsae_can_spec.h"

#include <stdint.h>
#include <stdbool.h>

#include "ids.h"

typedef struct {
  bool alwaysTrue;
} Can_Vcu_BmsHeartbeat_T;

typedef enum {
  CAN_LIMP_NORMAL = 0,
  CAN_LIMP_50,
  CAN_LIMP_33,
  CAN_LIMP_25
} Can_Vcu_LimpState_T;

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
  bool heartbeat_front_can_node_dead;
  bool heartbeat_rear_can_node_dead;
  bool heartbeat_bms_dead;
  bool heartbeat_dash_dead;
  bool heartbeat_mc_dead;
  bool heartbeat_current_sensor_dead;
  bool tsms_off;
  bool reset_latch_open;
  bool precharge_running;
  bool master_reset_not_initialized;
  bool driver_reset_not_initialized;
  uint16_t lv_battery_voltage;
  Can_Vcu_LimpState_T limp_state;
} Can_Vcu_DashHeartbeat_T;

typedef struct {
  Can_MC_RegID_T requestType;
  uint8_t period;
} Can_Vcu_MCRequest_T;

typedef struct {
  int16_t torque_cmd;
} Can_Vcu_MCTorque_T;

#endif // _MY17_CAN_LIBRARY_VCU_H
