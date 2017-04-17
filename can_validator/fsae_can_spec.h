#ifndef ____FSAE_CAN_SPEC_H
#define ____FSAE_CAN_SPEC_H

#define BMS_HEARTBEAT__id 96
#define BMS_HEARTBEAT__freq 1
#define __BMS_HEARTBEAT__STATE__start 0
#define __BMS_HEARTBEAT__STATE__end 2
#define ____BMS_HEARTBEAT__STATE__INIT 0
#define ____BMS_HEARTBEAT__STATE__STANDBY 1
#define ____BMS_HEARTBEAT__STATE__CHARGE 2
#define ____BMS_HEARTBEAT__STATE__BALANCE 3
#define ____BMS_HEARTBEAT__STATE__DISCHARGE 4
#define ____BMS_HEARTBEAT__STATE__BATTERY_FAULT 5
#define ____BMS_HEARTBEAT__STATE__BMS_FAULT 6
#define __BMS_HEARTBEAT__SOC_PERCENTAGE__start 3
#define __BMS_HEARTBEAT__SOC_PERCENTAGE__end 12
#define ____BMS_HEARTBEAT__SOC_PERCENTAGE__SOC_PERCENTAGE__FROM 0
#define ____BMS_HEARTBEAT__SOC_PERCENTAGE__SOC_PERCENTAGE__TO 1023

#define VCU_BMS_HEARTBEAT__id 80
#define VCU_BMS_HEARTBEAT__freq 1
#define __VCU_BMS_HEARTBEAT__STATE__start 0
#define __VCU_BMS_HEARTBEAT__STATE__end 0
#define ____VCU_BMS_HEARTBEAT__STATE__DISCHARGE 0

#define CURRENT_SENSOR_VOLTAGE__id 1314
#define CURRENT_SENSOR_VOLTAGE__freq 20
#define __CURRENT_SENSOR_VOLTAGE__PACK_VOLTAGE__start 16
#define __CURRENT_SENSOR_VOLTAGE__PACK_VOLTAGE__end 47
#define ____CURRENT_SENSOR_VOLTAGE__PACK_VOLTAGE__PACK_VOLTAGE__FROM 0
#define ____CURRENT_SENSOR_VOLTAGE__PACK_VOLTAGE__PACK_VOLTAGE__TO 4294967295

#define DASH_REQUEST__id 113
#define DASH_REQUEST__freq -1
#define __DASH_REQUEST__REQUEST_TYPE__start 0
#define __DASH_REQUEST__REQUEST_TYPE__end 7
#define ____DASH_REQUEST__REQUEST_TYPE__NO_REQUEST 0
#define ____DASH_REQUEST__REQUEST_TYPE__RTD_ENABLE 1
#define ____DASH_REQUEST__REQUEST_TYPE__RTD_DISABLE 2
#define ____DASH_REQUEST__REQUEST_TYPE__LIMP_MODE_ENABLE 3
#define ____DASH_REQUEST__REQUEST_TYPE__LIMP_MODE_DISABLE 4
#define ____DASH_REQUEST__REQUEST_TYPE__DATA_FLAG 5

#define CURRENT_SENSOR_POWER__id 1318
#define CURRENT_SENSOR_POWER__freq 20
#define __CURRENT_SENSOR_POWER__PACK_POWER__start 16
#define __CURRENT_SENSOR_POWER__PACK_POWER__end 47
#define ____CURRENT_SENSOR_POWER__PACK_POWER__PACK_POWER__FROM 0
#define ____CURRENT_SENSOR_POWER__PACK_POWER__PACK_POWER__TO 4294967295

#define VCU_DASH_HEARTBEAT__id 81
#define VCU_DASH_HEARTBEAT__freq 10
#define __VCU_DASH_HEARTBEAT__RTD_LIGHT__start 0
#define __VCU_DASH_HEARTBEAT__RTD_LIGHT__end 0
#define ____VCU_DASH_HEARTBEAT__RTD_LIGHT__OFF 0
#define ____VCU_DASH_HEARTBEAT__RTD_LIGHT__ON 1
#define __VCU_DASH_HEARTBEAT__AMS_LIGHT__start 1
#define __VCU_DASH_HEARTBEAT__AMS_LIGHT__end 1
#define ____VCU_DASH_HEARTBEAT__AMS_LIGHT__OFF 0
#define ____VCU_DASH_HEARTBEAT__AMS_LIGHT__ON 1
#define __VCU_DASH_HEARTBEAT__IMD_LIGHT__start 2
#define __VCU_DASH_HEARTBEAT__IMD_LIGHT__end 2
#define ____VCU_DASH_HEARTBEAT__IMD_LIGHT__OFF 0
#define ____VCU_DASH_HEARTBEAT__IMD_LIGHT__ON 1
#define __VCU_DASH_HEARTBEAT__HV_LIGHT__start 3
#define __VCU_DASH_HEARTBEAT__HV_LIGHT__end 3
#define ____VCU_DASH_HEARTBEAT__HV_LIGHT__OFF 0
#define ____VCU_DASH_HEARTBEAT__HV_LIGHT__ON 1
#define __VCU_DASH_HEARTBEAT__TRACTION_CONTROL__start 4
#define __VCU_DASH_HEARTBEAT__TRACTION_CONTROL__end 4
#define ____VCU_DASH_HEARTBEAT__TRACTION_CONTROL__OFF 0
#define ____VCU_DASH_HEARTBEAT__TRACTION_CONTROL__ON 1
#define __VCU_DASH_HEARTBEAT__LIMP_MODE__start 5
#define __VCU_DASH_HEARTBEAT__LIMP_MODE__end 5
#define ____VCU_DASH_HEARTBEAT__LIMP_MODE__OFF 0
#define ____VCU_DASH_HEARTBEAT__LIMP_MODE__ON 1
#define __VCU_DASH_HEARTBEAT__LV_WARNING__start 6
#define __VCU_DASH_HEARTBEAT__LV_WARNING__end 6
#define ____VCU_DASH_HEARTBEAT__LV_WARNING__OFF 0
#define ____VCU_DASH_HEARTBEAT__LV_WARNING__ON 1
#define __VCU_DASH_HEARTBEAT__ACTIVE_AERO__start 7
#define __VCU_DASH_HEARTBEAT__ACTIVE_AERO__end 7
#define ____VCU_DASH_HEARTBEAT__ACTIVE_AERO__OFF 0
#define ____VCU_DASH_HEARTBEAT__ACTIVE_AERO__ON 1
#define __VCU_DASH_HEARTBEAT__REGEN__start 8
#define __VCU_DASH_HEARTBEAT__REGEN__end 8
#define ____VCU_DASH_HEARTBEAT__REGEN__OFF 0
#define ____VCU_DASH_HEARTBEAT__REGEN__ON 1
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_ESD_DRAIN__start 9
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_ESD_DRAIN__end 9
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_ESD_DRAIN__CLOSED 0
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_ESD_DRAIN__OPEN 1
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_BMS__start 10
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_BMS__end 10
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_BMS__CLOSED 0
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_BMS__OPEN 1
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_IMD__start 11
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_IMD__end 11
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_IMD__CLOSED 0
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_IMD__OPEN 1
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_BSPD__start 12
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_BSPD__end 12
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_BSPD__CLOSED 0
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_BSPD__OPEN 1
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_VCU__start 13
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_VCU__end 13
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_VCU__CLOSED 0
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_VCU__OPEN 1
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_PRECHARGE__start 14
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_PRECHARGE__end 14
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_PRECHARGE__CLOSED 0
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_PRECHARGE__OPEN 1
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_MASTER_RESET__start 15
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_MASTER_RESET__end 15
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_MASTER_RESET__CLOSED 0
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_MASTER_RESET__OPEN 1
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_DRIVER_RESET__start 16
#define __VCU_DASH_HEARTBEAT__SHUTDOWN_DRIVER_RESET__end 16
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_DRIVER_RESET__CLOSED 0
#define ____VCU_DASH_HEARTBEAT__SHUTDOWN_DRIVER_RESET__OPEN 1
#define __VCU_DASH_HEARTBEAT__LV_BATTERY_VOLTAGE_TENTH_VOLT__start 17
#define __VCU_DASH_HEARTBEAT__LV_BATTERY_VOLTAGE_TENTH_VOLT__end 24
#define ____VCU_DASH_HEARTBEAT__LV_BATTERY_VOLTAGE_TENTH_VOLT__LV_BATTERY_VOLTAGE_TENTH_VOLT__FROM 0
#define ____VCU_DASH_HEARTBEAT__LV_BATTERY_VOLTAGE_TENTH_VOLT__LV_BATTERY_VOLTAGE_TENTH_VOLT__TO 255

#define CURRENT_SENSOR_ENERGY__id 1320
#define CURRENT_SENSOR_ENERGY__freq 20
#define __CURRENT_SENSOR_ENERGY__PACK_ENERGY__start 16
#define __CURRENT_SENSOR_ENERGY__PACK_ENERGY__end 47
#define ____CURRENT_SENSOR_ENERGY__PACK_ENERGY__PACK_ENERGY__FROM 0
#define ____CURRENT_SENSOR_ENERGY__PACK_ENERGY__PACK_ENERGY__TO 4294967295

#define DASH_HEARTBEAT__id 112
#define DASH_HEARTBEAT__freq 1
#define __DASH_HEARTBEAT__HEARTBEAT_ON__start 0
#define __DASH_HEARTBEAT__HEARTBEAT_ON__end 0
#define ____DASH_HEARTBEAT__HEARTBEAT_ON__OFF 0
#define ____DASH_HEARTBEAT__HEARTBEAT_ON__ON 1

#define CURRENT_SENSOR_CURRENT__id 1313
#define CURRENT_SENSOR_CURRENT__freq 20
#define __CURRENT_SENSOR_CURRENT__PACK_CURRENT__start 16
#define __CURRENT_SENSOR_CURRENT__PACK_CURRENT__end 47
#define ____CURRENT_SENSOR_CURRENT__PACK_CURRENT__PACK_CURRENT__FROM 0
#define ____CURRENT_SENSOR_CURRENT__PACK_CURRENT__PACK_CURRENT__TO 4294967295

#define BMS_PACK_STATUS__id 353
#define BMS_PACK_STATUS__freq 10
#define __BMS_PACK_STATUS__PACK_VOLTAGE__start 0
#define __BMS_PACK_STATUS__PACK_VOLTAGE__end 8
#define ____BMS_PACK_STATUS__PACK_VOLTAGE__PACK_VOLTAGE__FROM 0
#define ____BMS_PACK_STATUS__PACK_VOLTAGE__PACK_VOLTAGE__TO 511
#define __BMS_PACK_STATUS__PACK_CURRENT__start 9
#define __BMS_PACK_STATUS__PACK_CURRENT__end 19
#define ____BMS_PACK_STATUS__PACK_CURRENT__NEGATIVE_PACK_CURRENT__FROM 1024
#define ____BMS_PACK_STATUS__PACK_CURRENT__NEGATIVE_PACK_CURRENT__TO 2047
#define ____BMS_PACK_STATUS__PACK_CURRENT__POSITIVE_PACK_CURRENT__FROM 0
#define ____BMS_PACK_STATUS__PACK_CURRENT__POSITIVE_PACK_CURRENT__TO 1023
#define __BMS_PACK_STATUS__AVE_CELL_VOLTAGE__start 20
#define __BMS_PACK_STATUS__AVE_CELL_VOLTAGE__end 29
#define ____BMS_PACK_STATUS__AVE_CELL_VOLTAGE__AVE_CELL_VOLTAGE__FROM 0
#define ____BMS_PACK_STATUS__AVE_CELL_VOLTAGE__AVE_CELL_VOLTAGE__TO 1023
#define __BMS_PACK_STATUS__MIN_CELL_VOLTAGE__start 30
#define __BMS_PACK_STATUS__MIN_CELL_VOLTAGE__end 39
#define ____BMS_PACK_STATUS__MIN_CELL_VOLTAGE__MIN_CELL_VOLTAGE__FROM 0
#define ____BMS_PACK_STATUS__MIN_CELL_VOLTAGE__MIN_CELL_VOLTAGE__TO 1023
#define __BMS_PACK_STATUS__MIN_CELL_VOLTAGE_ID__start 40
#define __BMS_PACK_STATUS__MIN_CELL_VOLTAGE_ID__end 46
#define ____BMS_PACK_STATUS__MIN_CELL_VOLTAGE_ID__MIN_CELL_VOLTAGE_ID__FROM 0
#define ____BMS_PACK_STATUS__MIN_CELL_VOLTAGE_ID__MIN_CELL_VOLTAGE_ID__TO 127
#define __BMS_PACK_STATUS__MAX_CELL_VOLTAGE__start 47
#define __BMS_PACK_STATUS__MAX_CELL_VOLTAGE__end 56
#define ____BMS_PACK_STATUS__MAX_CELL_VOLTAGE__MAX_CELL_VOLTAGE__FROM 0
#define ____BMS_PACK_STATUS__MAX_CELL_VOLTAGE__MAX_CELL_VOLTAGE__TO 1023
#define __BMS_PACK_STATUS__MAX_CELL_VOLTAGE_ID__start 57
#define __BMS_PACK_STATUS__MAX_CELL_VOLTAGE_ID__end 63
#define ____BMS_PACK_STATUS__MAX_CELL_VOLTAGE_ID__MAX_CELL_VOLTAGE_ID__FROM 0
#define ____BMS_PACK_STATUS__MAX_CELL_VOLTAGE_ID__MAX_CELL_VOLTAGE_ID__TO 127

#define FRONT_CAN_NODE_DRIVER_OUTPUT__id 48
#define FRONT_CAN_NODE_DRIVER_OUTPUT__freq 50
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__REQUESTED_TORQUE__start 0
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__REQUESTED_TORQUE__end 15
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__REQUESTED_TORQUE__NEGATIVE_TORQUE__FROM 32768
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__REQUESTED_TORQUE__NEGATIVE_TORQUE__TO 65535
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__REQUESTED_TORQUE__POSITIVE_TORQUE__FROM 0
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__REQUESTED_TORQUE__POSITIVE_TORQUE__TO 32767
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__BRAKE_PRESSURE__start 16
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__BRAKE_PRESSURE__end 23
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__BRAKE_PRESSURE__BRAKE_PRESSURE__FROM 0
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__BRAKE_PRESSURE__BRAKE_PRESSURE__TO 255
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__STEERING_POSITION__start 24
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__STEERING_POSITION__end 31
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__STEERING_POSITION__STEERING_TRAVEL_LEFTWARDS__FROM 0
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__STEERING_POSITION__STEERING_TRAVEL_LEFTWARDS__TO 255
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__THROTTLE_IMPLAUSIBLE__start 32
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__THROTTLE_IMPLAUSIBLE__end 32
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__THROTTLE_IMPLAUSIBLE__PLAUSIBLE 0
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__THROTTLE_IMPLAUSIBLE__IMPLAUSIBLE 1
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__BRAKE_THROTTLE_CONFLICT__start 33
#define __FRONT_CAN_NODE_DRIVER_OUTPUT__BRAKE_THROTTLE_CONFLICT__end 33
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__BRAKE_THROTTLE_CONFLICT__NO_CONFLICT 0
#define ____FRONT_CAN_NODE_DRIVER_OUTPUT__BRAKE_THROTTLE_CONFLICT__CONFLICT 1

#define REAR_CAN_NODE_WHEEL_SPEED__id 49
#define REAR_CAN_NODE_WHEEL_SPEED__freq 50
#define __REAR_CAN_NODE_WHEEL_SPEED__REAR_RIGHT_WHEEL_SPEED__start 0
#define __REAR_CAN_NODE_WHEEL_SPEED__REAR_RIGHT_WHEEL_SPEED__end 31
#define ____REAR_CAN_NODE_WHEEL_SPEED__REAR_RIGHT_WHEEL_SPEED__REAR_RIGHT_WHEEL_SPEED__FROM 0
#define ____REAR_CAN_NODE_WHEEL_SPEED__REAR_RIGHT_WHEEL_SPEED__REAR_RIGHT_WHEEL_SPEED__TO 4294967295
#define __REAR_CAN_NODE_WHEEL_SPEED__REAR_LEFT_WHEEL_SPEED__start 32
#define __REAR_CAN_NODE_WHEEL_SPEED__REAR_LEFT_WHEEL_SPEED__end 63
#define ____REAR_CAN_NODE_WHEEL_SPEED__REAR_LEFT_WHEEL_SPEED__REAR_LEFT_WHEEL_SPEED__FROM 0
#define ____REAR_CAN_NODE_WHEEL_SPEED__REAR_LEFT_WHEEL_SPEED__REAR_LEFT_WHEEL_SPEED__TO 4294967295

#define VCU_MC_MESSAGE__id 82
#define VCU_MC_MESSAGE__freq -1
#define __VCU_MC_MESSAGE__REG_ID__start 0
#define __VCU_MC_MESSAGE__REG_ID__end 7
#define ____VCU_MC_MESSAGE__REG_ID__TORQUE_CMD 144
#define ____VCU_MC_MESSAGE__REG_ID__MSG_EVENT_REQUEST 81
#define ____VCU_MC_MESSAGE__REG_ID__MSG_REQUEST 61
#define __VCU_MC_MESSAGE__DATA_1__start 8
#define __VCU_MC_MESSAGE__DATA_1__end 15
#define ____VCU_MC_MESSAGE__DATA_1__DATA_LOW_BYTE__FROM 0
#define ____VCU_MC_MESSAGE__DATA_1__DATA_LOW_BYTE__TO 255
#define __VCU_MC_MESSAGE__DATA_2__start 16
#define __VCU_MC_MESSAGE__DATA_2__end 23
#define ____VCU_MC_MESSAGE__DATA_2__DATA_HIGH_BYTE__FROM 0
#define ____VCU_MC_MESSAGE__DATA_2__DATA_HIGH_BYTE__TO 255

#define FRONT_CAN_NODE_WHEEL_SPEED__id 561
#define FRONT_CAN_NODE_WHEEL_SPEED__freq 10
#define __FRONT_CAN_NODE_WHEEL_SPEED__FRONT_RIGHT_WHEEL_SPEED__start 0
#define __FRONT_CAN_NODE_WHEEL_SPEED__FRONT_RIGHT_WHEEL_SPEED__end 31
#define ____FRONT_CAN_NODE_WHEEL_SPEED__FRONT_RIGHT_WHEEL_SPEED__FRONT_RIGHT_WHEEL_SPEED__FROM 0
#define ____FRONT_CAN_NODE_WHEEL_SPEED__FRONT_RIGHT_WHEEL_SPEED__FRONT_RIGHT_WHEEL_SPEED__TO 4294967295
#define __FRONT_CAN_NODE_WHEEL_SPEED__FRONT_LEFT_WHEEL_SPEED__start 32
#define __FRONT_CAN_NODE_WHEEL_SPEED__FRONT_LEFT_WHEEL_SPEED__end 63
#define ____FRONT_CAN_NODE_WHEEL_SPEED__FRONT_LEFT_WHEEL_SPEED__FRONT_LEFT_WHEEL_SPEED__FROM 0
#define ____FRONT_CAN_NODE_WHEEL_SPEED__FRONT_LEFT_WHEEL_SPEED__FRONT_LEFT_WHEEL_SPEED__TO 4294967295

#define BMS_CELL_TEMPS__id 352
#define BMS_CELL_TEMPS__freq 0
#define __BMS_CELL_TEMPS__AVE_CELL_TEMP__start 0
#define __BMS_CELL_TEMPS__AVE_CELL_TEMP__end 15
#define ____BMS_CELL_TEMPS__AVE_CELL_TEMP__AVE_CELL_TEMP__FROM 0
#define ____BMS_CELL_TEMPS__AVE_CELL_TEMP__AVE_CELL_TEMP__TO 255
#define __BMS_CELL_TEMPS__MIN_CELL_TEMP__start 16
#define __BMS_CELL_TEMPS__MIN_CELL_TEMP__end 31
#define ____BMS_CELL_TEMPS__MIN_CELL_TEMP__MIN_CELL_TEMP__FROM 0
#define ____BMS_CELL_TEMPS__MIN_CELL_TEMP__MIN_CELL_TEMP__TO 255
#define __BMS_CELL_TEMPS__MIN_CELL_TEMP_ID__start 32
#define __BMS_CELL_TEMPS__MIN_CELL_TEMP_ID__end 39
#define ____BMS_CELL_TEMPS__MIN_CELL_TEMP_ID__MIN_CELL_TEMP_ID__FROM 0
#define ____BMS_CELL_TEMPS__MIN_CELL_TEMP_ID__MIN_CELL_TEMP_ID__TO 255
#define __BMS_CELL_TEMPS__MAX_CELL_TEMP__start 40
#define __BMS_CELL_TEMPS__MAX_CELL_TEMP__end 55
#define ____BMS_CELL_TEMPS__MAX_CELL_TEMP__MAX_CELL_TEMP__FROM 0
#define ____BMS_CELL_TEMPS__MAX_CELL_TEMP__MAX_CELL_TEMP__TO 255
#define __BMS_CELL_TEMPS__MAX_CELL_TEMP_ID__start 56
#define __BMS_CELL_TEMPS__MAX_CELL_TEMP_ID__end 63
#define ____BMS_CELL_TEMPS__MAX_CELL_TEMP_ID__MAX_CELL_TEMP_ID__FROM 0
#define ____BMS_CELL_TEMPS__MAX_CELL_TEMP_ID__MAX_CELL_TEMP_ID__TO 255

#define FRONT_CAN_NODE_RAW_VALUES__id 560
#define FRONT_CAN_NODE_RAW_VALUES__freq 10
#define __FRONT_CAN_NODE_RAW_VALUES__RIGHT_THROTTLE_POT__start 0
#define __FRONT_CAN_NODE_RAW_VALUES__RIGHT_THROTTLE_POT__end 9
#define ____FRONT_CAN_NODE_RAW_VALUES__RIGHT_THROTTLE_POT__RIGHT_THROTTLE_POT__FROM 0
#define ____FRONT_CAN_NODE_RAW_VALUES__RIGHT_THROTTLE_POT__RIGHT_THROTTLE_POT__TO 255
#define __FRONT_CAN_NODE_RAW_VALUES__LEFT_THROTTLE_POT__start 10
#define __FRONT_CAN_NODE_RAW_VALUES__LEFT_THROTTLE_POT__end 19
#define ____FRONT_CAN_NODE_RAW_VALUES__LEFT_THROTTLE_POT__LEFT_THROTTLE_POT__FROM 0
#define ____FRONT_CAN_NODE_RAW_VALUES__LEFT_THROTTLE_POT__LEFT_THROTTLE_POT__TO 255
#define __FRONT_CAN_NODE_RAW_VALUES__FRONT_BRAKE_PRESSURE__start 20
#define __FRONT_CAN_NODE_RAW_VALUES__FRONT_BRAKE_PRESSURE__end 29
#define ____FRONT_CAN_NODE_RAW_VALUES__FRONT_BRAKE_PRESSURE__FRONT_BRAKE_PRESSURE__FROM 0
#define ____FRONT_CAN_NODE_RAW_VALUES__FRONT_BRAKE_PRESSURE__FRONT_BRAKE_PRESSURE__TO 255
#define __FRONT_CAN_NODE_RAW_VALUES__REAR_BRAKE_PRESSURE__start 30
#define __FRONT_CAN_NODE_RAW_VALUES__REAR_BRAKE_PRESSURE__end 39
#define ____FRONT_CAN_NODE_RAW_VALUES__REAR_BRAKE_PRESSURE__REAR_BRAKE_PRESSURE__FROM 0
#define ____FRONT_CAN_NODE_RAW_VALUES__REAR_BRAKE_PRESSURE__REAR_BRAKE_PRESSURE__TO 255
#define __FRONT_CAN_NODE_RAW_VALUES__STEERING_POT__start 40
#define __FRONT_CAN_NODE_RAW_VALUES__STEERING_POT__end 49
#define ____FRONT_CAN_NODE_RAW_VALUES__STEERING_POT__STEERING_POT__FROM 0
#define ____FRONT_CAN_NODE_RAW_VALUES__STEERING_POT__STEERING_POT__TO 255

#define BMS_ERRORS__id 608
#define BMS_ERRORS__freq 0
#define __BMS_ERRORS__ERROR_TYPE__start 0
#define __BMS_ERRORS__ERROR_TYPE__end 3
#define ____BMS_ERRORS__ERROR_TYPE__NONE 0
#define ____BMS_ERRORS__ERROR_TYPE__LTC6804_PEC 1
#define ____BMS_ERRORS__ERROR_TYPE__LTC6804_CVST 2
#define ____BMS_ERRORS__ERROR_TYPE__LTC6804_OWT 3
#define ____BMS_ERRORS__ERROR_TYPE__EEPROM 4
#define ____BMS_ERRORS__ERROR_TYPE__CELL_UNDER_VOLTAGE 5
#define ____BMS_ERRORS__ERROR_TYPE__CELL_OVER_VOLTAGE 6
#define ____BMS_ERRORS__ERROR_TYPE__CELL_UNDER_TEMP 7
#define ____BMS_ERRORS__ERROR_TYPE__CELL_OVER_TEMP 8
#define ____BMS_ERRORS__ERROR_TYPE__OVER_CURRENT 9
#define ____BMS_ERRORS__ERROR_TYPE__CAN 10
#define ____BMS_ERRORS__ERROR_TYPE__CONFLICTING_MODE_REQUESTS 11
#define ____BMS_ERRORS__ERROR_TYPE__VCU_DEAD 12
#define ____BMS_ERRORS__ERROR_TYPE__CONTROL_FLOW 13
#define ____BMS_ERRORS__ERROR_TYPE__OTHER 14

#define MC_RESPONSE__id 784
#define MC_RESPONSE__freq -1
#define __MC_RESPONSE__REG_ID__start 0
#define __MC_RESPONSE__REG_ID__end 7
#define ____MC_RESPONSE__REG_ID__CURRENT_ACTUAL 32
#define ____MC_RESPONSE__REG_ID__SPEED_ACTUAL_RPM 48
#define ____MC_RESPONSE__REG_ID__CURRENT_CMD_AFTER_RAMP 34
#define ____MC_RESPONSE__REG_ID__CURRENT_CMD 38
#define ____MC_RESPONSE__REG_ID__CURRENT_LIMIT_ACTUAL 72
#define ____MC_RESPONSE__REG_ID__MOTOR_TEMP 73
#define ____MC_RESPONSE__REG_ID__IGBT_TEMP 74
#define ____MC_RESPONSE__REG_ID__AIR_TEMP 75
#define ____MC_RESPONSE__REG_ID__CURRENT_LIMIT_MOTOR_TEMP 162
#define ____MC_RESPONSE__REG_ID__STATE 64
#define ____MC_RESPONSE__REG_ID__ERRORS_AND_WARNINGS 143
#define ____MC_RESPONSE__REG_ID__TORQUE_CMD 144
#define ____MC_RESPONSE__REG_ID__SPEED_MAX_RPM 200
#define ____MC_RESPONSE__REG_ID__CURRENT_LIMIT_IGBT_TEMP 88
#define __MC_RESPONSE__DATA__start 8
#define __MC_RESPONSE__DATA__end 23
#define ____MC_RESPONSE__DATA__DATA__FROM 0
#define ____MC_RESPONSE__DATA__DATA__TO 65535

#endif // ____FSAE_CAN_SPEC_H
