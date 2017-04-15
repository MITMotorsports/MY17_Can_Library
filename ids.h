#ifndef _MY17_CAN_LIBRARY_IDS_H
#define _MY17_CAN_LIBRARY_IDS_H

#include "can_validator/fsae_can_spec.h"

typedef enum {
  CAN_BMS_STATE_INIT = ____BMS_HEARTBEAT__STATE__INIT,
  CAN_BMS_STATE_STANDBY = ____BMS_HEARTBEAT__STATE__STANDBY,
  CAN_BMS_STATE_CHARGE = ____BMS_HEARTBEAT__STATE__CHARGE,
  CAN_BMS_STATE_DISCHARGE = ____BMS_HEARTBEAT__STATE__DISCHARGE,
  CAN_BMS_STATE_BALANCE = ____BMS_HEARTBEAT__STATE__BALANCE,
  CAN_BMS_STATE_BATTERY_FAULT = ____BMS_HEARTBEAT__STATE__BATTERY_FAULT,
  CAN_BMS_STATE_BMS_FAULT = ____BMS_HEARTBEAT__STATE__BMS_FAULT
} Can_Bms_StateID_T;

typedef enum {
  CAN_BMS_ERROR_NONE = ____BMS_ERRORS__ERROR_TYPE__NONE,
  CAN_BMS_ERROR_LTC6804_PEC = ____BMS_ERRORS__ERROR_TYPE__LTC6804_PEC,
  CAN_BMS_ERROR_LTC6804_CVST = ____BMS_ERRORS__ERROR_TYPE__LTC6804_CVST,
  CAN_BMS_ERROR_LTC6804_OWT = ____BMS_ERRORS__ERROR_TYPE__LTC6804_OWT,
  CAN_BMS_ERROR_EEPROM = ____BMS_ERRORS__ERROR_TYPE__EEPROM,
  CAN_BMS_ERROR_CELL_UNDER_VOLTAGE = ____BMS_ERRORS__ERROR_TYPE__CELL_UNDER_VOLTAGE,
  CAN_BMS_ERROR_CELL_OVER_VOLTAGE = ____BMS_ERRORS__ERROR_TYPE__CELL_OVER_VOLTAGE,
  CAN_BMS_ERROR_CELL_UNDER_TEMP = ____BMS_ERRORS__ERROR_TYPE__CELL_UNDER_TEMP,
  CAN_BMS_ERROR_CELL_OVER_TEMP = ____BMS_ERRORS__ERROR_TYPE__CELL_OVER_TEMP,
  CAN_BMS_ERROR_OVER_CURRENT =____BMS_ERRORS__ERROR_TYPE__OVER_CURRENT,
  CAN_BMS_ERROR_CAN =____BMS_ERRORS__ERROR_TYPE__CAN,
  CAN_BMS_ERROR_CONFLICTING_MODE_REQUESTS =____BMS_ERRORS__ERROR_TYPE__CONFLICTING_MODE_REQUESTS,
  CAN_BMS_ERROR_VCU_DEAD =____BMS_ERRORS__ERROR_TYPE__VCU_DEAD,
} Can_Bms_ErrorID_T;

typedef enum {
  CAN_MC_REG_CURRENT_CMD = ____MC_RESPONSE__REG_ID__CURRENT_CMD,
  CAN_MC_REG_CURRENT_CMD_AFTER_RAMP = ____MC_RESPONSE__REG_ID__CURRENT_CMD_AFTER_RAMP,
  CAN_MC_REG_CURRENT_ACTUAL = ____MC_RESPONSE__REG_ID__CURRENT_ACTUAL,
  CAN_MC_REG_CURRENT_LIMIT_ACTUAL = ____MC_RESPONSE__REG_ID__CURRENT_LIMIT_ACTUAL,
  CAN_MC_REG_CURRENT_LIMIT_MOTOR_TEMP = ____MC_RESPONSE__REG_ID__CURRENT_LIMIT_MOTOR_TEMP,
  CAN_MC_REG_CURRENT_LIMIT_IGBT_TEMP = ____MC_RESPONSE__REG_ID__CURRENT_LIMIT_IGBT_TEMP,

  CAN_MC_REG_SPEED_ACTUAL_RPM = ____MC_RESPONSE__REG_ID__SPEED_ACTUAL_RPM,
  CAN_MC_REG_SPEED_MAX_RPM = ____MC_RESPONSE__REG_ID__SPEED_MAX_RPM,

  CAN_MC_REG_MOTOR_TEMP = ____MC_RESPONSE__REG_ID__MOTOR_TEMP,
  CAN_MC_REG_IGBT_TEMP = ____MC_RESPONSE__REG_ID__IGBT_TEMP,
  CAN_MC_REG_AIR_TEMP = ____MC_RESPONSE__REG_ID__AIR_TEMP,

  CAN_MC_REG_TORQUE_CMD = ____MC_RESPONSE__REG_ID__TORQUE_CMD,

  CAN_MC_REG_STATE = ____MC_RESPONSE__REG_ID__STATE,
  CAN_MC_REG_ERRORS_AND_WARNINGS = ____MC_RESPONSE__REG_ID__ERRORS_AND_WARNINGS,

  CAN_MC_REG_MSG_EVENT_REQUEST = ____VCU_MC_MESSAGE__REG_ID__MSG_EVENT_REQUEST,
  CAN_MC_REG_MSG_REQUEST = ____VCU_MC_MESSAGE__REG_ID__MSG_REQUEST,
} Can_MC_RegID_T;

typedef enum {
  CAN_DASH_REQUEST_NO_REQUEST = ____DASH_REQUEST__REQUEST_TYPE__NO_REQUEST,
  CAN_DASH_REQUEST_RTD_ENABLE = ____DASH_REQUEST__REQUEST_TYPE__RTD_ENABLE,
  CAN_DASH_REQUEST_RTD_DISABLE = ____DASH_REQUEST__REQUEST_TYPE__RTD_DISABLE,
  CAN_DASH_REQUEST_LIMP_MODE_ENABLE = ____DASH_REQUEST__REQUEST_TYPE__LIMP_MODE_ENABLE,
  CAN_DASH_REQUEST_LIMP_MODE_DISABLE = ____DASH_REQUEST__REQUEST_TYPE__LIMP_MODE_DISABLE,
  CAN_DASH_REQUEST_DATA_FLAG = ____DASH_REQUEST__REQUEST_TYPE__DATA_FLAG,
} Can_Dash_RequestID_T;

#endif // _MY17_CAN_LIBRARY_IDS_H
