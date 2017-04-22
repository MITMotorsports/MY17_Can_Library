#include "MY17_Can_Library.h"
#include "evil_macros.h"

static bool unread;
static Frame lastMessage;

#define DEFINE(name) \
  bool name ##_Read(name ## _T *type) { \
    if (unread) { \
      name ## _FromCan(&lastMessage, type); \
      unread = false; \
      return true; \
    } else { \
      return false; \
    } \
  } \
  void name ##_Write(name ## _T *type) { \
    Frame frame; \
    name ## _ToCan(type, &frame); \
    Can_RawWrite(&frame); \
  }

typedef enum {
  LITTLE,
  BIG,
} ENDIAN_T;

typedef union {
  uint8_t byte[8];
  uint64_t bitstring;
} DATA_T;

void data_transfer(DATA_T *in, DATA_T *out) {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    (*out).byte[7-i] = (*in).byte[i];
  }
}

void to_bitstring(uint8_t in[], uint64_t *out) {
  data_transfer((DATA_T*)in, (DATA_T*)out);
}

void from_bitstring(uint64_t *in, uint8_t out[]) {
  data_transfer((DATA_T*)in, (DATA_T*)out);
}

#ifdef CAN_ARCHITECTURE_ARM
#include "arm_src/arm_can_drivers.c"
#elif CAN_ARCHITECTURE_AVR
#include "avr_src/avr_can_drivers.c"
#elif CAN_ARCHITECTURE_TEST

void Can_Init(uint32_t baudrate) {
  // TODO test harness
}

void Can_RawWrite(Frame *frame) {
  // TODO test harness
}

bool Can_RawRead(Frame *frame) {
  // TODO test harness
  return false;
}

#else
  // Define nothing so that there is a linker error!
#endif

Can_MsgID_T Can_MsgType(void) {
  bool result = Can_RawRead(&lastMessage);
  if (!result) {
    unread = false;
    return Can_No_Msg;
  }
  unread = true;

  uint16_t id = lastMessage.id;
  uint16_t first_byte = lastMessage.data[0];
  switch(id) {
    case FRONT_CAN_NODE_DRIVER_OUTPUT__id:
      return Can_FrontCanNode_DriverOutput_Msg;
    case FRONT_CAN_NODE_RAW_VALUES__id:
      return Can_FrontCanNode_RawValues_Msg;
    case FRONT_CAN_NODE_WHEEL_SPEED__id:
      return Can_FrontCanNode_WheelSpeed_Msg;

    case VCU_BMS_HEARTBEAT__id:
      return Can_Vcu_BmsHeartbeat_Msg;
    case VCU_DASH_HEARTBEAT__id:
      return Can_Vcu_DashHeartbeat_Msg;
    case VCU_MC_MESSAGE__id:
      if (first_byte == CAN_MC_REG_MSG_REQUEST || first_byte == CAN_MC_REG_MSG_EVENT_REQUEST) {
        return Can_Vcu_MCRequest_Msg;
      } else if (first_byte == CAN_MC_REG_TORQUE_CMD) {
        return Can_Vcu_MCTorque_Msg;
      } else {
        return Can_Unknown_Msg;
      }

    case BMS_HEARTBEAT__id:
      return Can_Bms_Heartbeat_Msg;
    case BMS_CELL_TEMPS__id:
      return Can_Bms_CellTemps_Msg;
    case BMS_PACK_STATUS__id:
      return Can_Bms_PackStatus_Msg;
    case BMS_ERRORS__id:
      return Can_Bms_Error_Msg;

    default:
      return Can_Unknown_Msg;
  }
}

TO_CAN(Can_FrontCanNode_DriverOutput) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->torque, bitstring, 0, 16);
  bitstring = INSERT(type_in->brake_pressure, bitstring, 16, 8);
  bitstring = INSERT(type_in->steering_position, bitstring, 24, 8);
  bitstring = INSERT(type_in->throttle_implausible, bitstring, 32, 1);
  bitstring = INSERT(type_in->brake_throttle_conflict, bitstring, 33, 1);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = FRONT_CAN_NODE_DRIVER_OUTPUT__id;
  can_out->len = 5;
}

FROM_CAN(Can_FrontCanNode_DriverOutput) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->torque = SIGN(EXTRACT(bitstring, 0, 16), 16);
  type_out->brake_pressure = EXTRACT(bitstring, 16, 8);
  type_out->steering_position = EXTRACT(bitstring, 24, 8);
  type_out->throttle_implausible = EXTRACT(bitstring, 32, 1);
  type_out->brake_throttle_conflict = EXTRACT(bitstring, 33, 1);
}

TO_CAN(Can_FrontCanNode_RawValues) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->accel_1_raw, bitstring, 0, 10);
  bitstring = INSERT(type_in->accel_2_raw, bitstring, 10, 10);
  bitstring = INSERT(type_in->brake_1_raw, bitstring, 20, 10);
  bitstring = INSERT(type_in->brake_2_raw, bitstring, 30, 10);
  bitstring = INSERT(type_in->steering_raw, bitstring, 40, 10);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = FRONT_CAN_NODE_RAW_VALUES__id;
  can_out->len = 8;
}

FROM_CAN(Can_FrontCanNode_RawValues) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->accel_1_raw = EXTRACT(bitstring, 0, 10);
  type_out->accel_2_raw = EXTRACT(bitstring, 10, 10);
  type_out->brake_1_raw = EXTRACT(bitstring, 20, 10);
  type_out->brake_2_raw = EXTRACT(bitstring, 30, 10);
  type_out->steering_raw = EXTRACT(bitstring, 40, 10);
}

TO_CAN(Can_FrontCanNode_WheelSpeed) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->front_right_wheel_speed, bitstring, 0, 32);
  bitstring = INSERT(type_in->front_left_wheel_speed, bitstring, 32, 32);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = FRONT_CAN_NODE_WHEEL_SPEED__id;
  can_out->len = 8;
}

FROM_CAN(Can_FrontCanNode_WheelSpeed) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->front_right_wheel_speed = EXTRACT(bitstring, 0, 32);
  type_out->front_left_wheel_speed = EXTRACT(bitstring, 32, 32);
}

TO_CAN(Can_Vcu_BmsHeartbeat) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->alwaysTrue, bitstring, 0, 1);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = VCU_BMS_HEARTBEAT__id;
  can_out->len = 1;
}

FROM_CAN(Can_Vcu_BmsHeartbeat) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->alwaysTrue = EXTRACT(bitstring, 0, 1);
}

TO_CAN(Can_Vcu_DashHeartbeat) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->rtd_light, bitstring, 0, 1);
  bitstring = INSERT(type_in->ams_light, bitstring, 1, 1);
  bitstring = INSERT(type_in->imd_light, bitstring, 2, 1);
  bitstring = INSERT(type_in->hv_light, bitstring, 3, 1);
  bitstring = INSERT(type_in->traction_control, bitstring, 4, 1);
  bitstring = INSERT(type_in->limp_mode, bitstring, 5, 1);
  bitstring = INSERT(type_in->lv_warning, bitstring, 6, 1);
  bitstring = INSERT(type_in->active_aero, bitstring, 7, 1);
  bitstring = INSERT(type_in->regen, bitstring, 8, 1);
  bitstring = INSERT(type_in->shutdown_esd_drain, bitstring, 9, 1);
  bitstring = INSERT(type_in->shutdown_bms, bitstring, 10, 1);
  bitstring = INSERT(type_in->shutdown_imd, bitstring, 11, 1);
  bitstring = INSERT(type_in->shutdown_bspd, bitstring, 12, 1);
  bitstring = INSERT(type_in->shutdown_vcu, bitstring, 13, 1);
  bitstring = INSERT(type_in->shutdown_precharge, bitstring, 14, 1);
  bitstring = INSERT(type_in->shutdown_master_reset, bitstring, 15, 1);
  bitstring = INSERT(type_in->shutdown_driver_reset, bitstring, 16, 1);
  bitstring = INSERT(type_in->lv_battery_voltage, bitstring, 17, 8);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = VCU_DASH_HEARTBEAT__id;
  can_out->len = 4;
}

FROM_CAN(Can_Vcu_DashHeartbeat) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->rtd_light = EXTRACT(bitstring, 0, 1);
  type_out->ams_light = EXTRACT(bitstring, 1, 1);
  type_out->imd_light = EXTRACT(bitstring, 2, 1);
  type_out->hv_light = EXTRACT(bitstring, 3, 1);
  type_out->traction_control = EXTRACT(bitstring, 4, 1);
  type_out->limp_mode = EXTRACT(bitstring, 5, 1);
  type_out->lv_warning = EXTRACT(bitstring, 6, 1);
  type_out->active_aero = EXTRACT(bitstring, 7, 1);
  type_out->regen = EXTRACT(bitstring, 8, 1);
  type_out->shutdown_esd_drain = EXTRACT(bitstring, 9, 1);
  type_out->shutdown_bms = EXTRACT(bitstring, 10, 1);
  type_out->shutdown_imd = EXTRACT(bitstring, 11, 1);
  type_out->shutdown_bspd = EXTRACT(bitstring, 12, 1);
  type_out->shutdown_vcu = EXTRACT(bitstring, 13, 1);
  type_out->shutdown_precharge = EXTRACT(bitstring, 14, 1);
  type_out->shutdown_master_reset = EXTRACT(bitstring, 15, 1);
  type_out->shutdown_driver_reset = EXTRACT(bitstring, 16, 1);
  type_out->lv_battery_voltage = EXTRACT(bitstring, 17, 8);
}

TO_CAN(Can_Vcu_MCRequest) {
  uint64_t bitstring = 0;
  bitstring = INSERT(CAN_MC_REG_MSG_REQUEST, bitstring, 0, 8);
  bitstring = INSERT(type_in->requestType, bitstring, 8, 8);
  bitstring = INSERT(type_in->period, bitstring, 16, 8);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = VCU_MC_MESSAGE__id;
  can_out->len = 3;
}

FROM_CAN(Can_Vcu_MCRequest) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->requestType = (Can_MC_RegID_T) (EXTRACT(bitstring, 8, 8));
  type_out->period = EXTRACT(bitstring, 16, 8);
}

TO_CAN(Can_Vcu_MCTorque) {
  uint64_t bitstring = 0;
  bitstring = INSERT(CAN_MC_REG_TORQUE_CMD, bitstring, 0, 8);
  bitstring = INSERT(type_in->torque_cmd, bitstring, 8, 16);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = VCU_MC_MESSAGE__id;
  can_out->len = 3;
}

FROM_CAN(Can_Vcu_MCTorque) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->torque_cmd = SIGN((EXTRACT(bitstring, 8, 16)), 16);
}

TO_CAN(Can_Bms_Heartbeat) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->state, bitstring, 0, 3);
  bitstring = INSERT(type_in->soc, bitstring, 3, 10);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = BMS_HEARTBEAT__id;
  can_out->len = 8;
}

FROM_CAN(Can_Bms_Heartbeat) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->state = (Can_Bms_StateID_T)(EXTRACT(bitstring, 0, 3));
  type_out->soc = EXTRACT(bitstring, 3, 10);
}

TO_CAN(Can_Bms_CellTemps) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->avg_cell_temp, bitstring, 0, 15);
  bitstring = INSERT(type_in->min_cell_temp, bitstring, 15, 15);
  bitstring = INSERT(type_in->id_min_cell_temp, bitstring, 30, 9);
  bitstring = INSERT(type_in->max_cell_temp, bitstring, 39, 15);
  bitstring = INSERT(type_in->id_max_cell_temp, bitstring, 54, 9);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = BMS_CELL_TEMPS__id;
  can_out->len = 8;
}

FROM_CAN(Can_Bms_CellTemps) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->avg_cell_temp = SIGN(EXTRACT(bitstring, 0, 15), 15);
  type_out->min_cell_temp = SIGN(EXTRACT(bitstring, 15, 15), 15);
  type_out->id_min_cell_temp = EXTRACT(bitstring, 30, 9);
  type_out->max_cell_temp = SIGN(EXTRACT(bitstring, 39, 15), 15);
  type_out->id_max_cell_temp = EXTRACT(bitstring, 54, 9);
}

TO_CAN(Can_Bms_PackStatus) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->pack_voltage, bitstring, 0, 9);
  bitstring = INSERT(type_in->pack_current, bitstring, 9, 11);
  bitstring = INSERT(type_in->avg_cell_voltage, bitstring, 20, 10);
  bitstring = INSERT(type_in->min_cell_voltage, bitstring, 30, 10);
  bitstring = INSERT(type_in->id_min_cell_voltage, bitstring, 40, 7);
  bitstring = INSERT(type_in->max_cell_voltage, bitstring, 47, 10);
  bitstring = INSERT(type_in->id_max_cell_voltage, bitstring, 57, 7);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = BMS_PACK_STATUS__id;
  can_out->len = 8;
}

FROM_CAN(Can_Bms_PackStatus) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->pack_voltage = EXTRACT(bitstring, 0, 9);
  type_out->pack_current = SIGN(EXTRACT(bitstring, 9, 11), 11);
  type_out->avg_cell_voltage = EXTRACT(bitstring, 20, 10);
  type_out->min_cell_voltage = EXTRACT(bitstring, 30, 10);
  type_out->id_min_cell_voltage = EXTRACT(bitstring, 40, 7);
  type_out->max_cell_voltage = EXTRACT(bitstring, 47, 10);
  type_out->id_max_cell_voltage = EXTRACT(bitstring, 57, 7);
}

TO_CAN(Can_Bms_Error) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->type, bitstring, 0, 4);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = BMS_ERRORS__id;
  can_out->len = 1;
}

FROM_CAN(Can_Bms_Error) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->type = (Can_Bms_ErrorID_T)(EXTRACT(bitstring, 0, 4));
}

TO_CAN(Can_Dash_Heartbeat) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->ok, bitstring, 0, 1);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = DASH_HEARTBEAT__id;
  can_out->len = 1;
}

FROM_CAN(Can_Dash_Heartbeat) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->ok = EXTRACT(bitstring, 0, 1);
}

TO_CAN(Can_Dash_Request) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->type, bitstring, 0, 3);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = DASH_REQUEST__id;
  can_out->len = 1;
}

FROM_CAN(Can_Dash_Request) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->type = (Can_Dash_RequestID_T)(EXTRACT(bitstring, 0, 3));
}

TO_CAN(Can_MC_DataReading) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->type, bitstring, 0, 8);
  bitstring = INSERT(type_in->value, bitstring, 8, 16);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = MC_RESPONSE__id;
  can_out->len = 3;
}

FROM_CAN(Can_MC_DataReading) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->type = (Can_MC_RegID_T)(EXTRACT(bitstring, 0, 8));
  type_out->value = SIGN(EXTRACT(bitstring, 8, 16), 16);
}

TO_CAN(Can_CurrentSensor_Voltage) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->voltage_mV, bitstring, 16, 32);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = CURRENT_SENSOR_VOLTAGE__id;
  can_out->len = 6;
}

FROM_CAN(Can_CurrentSensor_Voltage) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->voltage_mV = SIGN((EXTRACT(bitstring, 16, 32)), 32);
}

// Needed for actual implementation of the evil macros
DEFINE(Can_FrontCanNode_DriverOutput)
DEFINE(Can_FrontCanNode_RawValues)
DEFINE(Can_FrontCanNode_WheelSpeed)

DEFINE(Can_Vcu_BmsHeartbeat)
DEFINE(Can_Vcu_DashHeartbeat)
DEFINE(Can_Vcu_MCRequest)
DEFINE(Can_Vcu_MCTorque)

DEFINE(Can_Bms_Heartbeat)
DEFINE(Can_Bms_CellTemps)
DEFINE(Can_Bms_PackStatus)
DEFINE(Can_Bms_Error)

DEFINE(Can_Dash_Heartbeat)
DEFINE(Can_Dash_Request)

DEFINE(Can_MC_DataReading)
//TODO errors, warnings, state for motor controller

/* DEFINE(Can_CurrentSensor_Current) */
DEFINE(Can_CurrentSensor_Voltage)
/* DEFINE(Can_CurrentSensor_Power) */
/* DEFINE(Can_CurrentSensor_Energy) */
