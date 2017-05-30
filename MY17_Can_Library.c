#include "MY17_Can_Library.h"
#include "evil_macros.h"

static Frame lastMessage;
static Can_ErrorID_T lastError = Can_Error_NO_RX;

#define DEFINE(name) \
  Can_ErrorID_T name ##_Read(name ## _T *type) { \
    if (lastError == Can_Error_NONE) { \
      name ## _FromCan(&lastMessage, type); \
      lastError = Can_Error_NO_RX; \
      return Can_Error_NONE; \
    } else { \
      return lastError; \
    } \
  } \
  Can_ErrorID_T name ##_Write(name ## _T *type) { \
    Frame frame; \
    name ## _ToCan(type, &frame); \
    return Can_RawWrite(&frame); \
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

// Shameless copypasta-ing from Stack Overflow for trivial endian swap.
// https://stackoverflow.com/a/2637138
uint16_t swap_uint16( uint16_t val ) {
    return (val << 8) | (val >> 8 );
}

int16_t swap_int16( int16_t val ) {
    return (val << 8) | ((val >> 8) & 0xFF);
}

uint32_t swap_uint32( uint32_t val ) {
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF ); 
    return (val << 16) | (val >> 16);
}

int32_t swap_int32( int32_t val ) {
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF ); 
    return (val << 16) | ((val >> 16) & 0xFFFF);
}

#ifdef CAN_ARCHITECTURE_ARM
#include "arm_src/arm_can_drivers.c"
#elif CAN_ARCHITECTURE_AVR
#include "avr_src/avr_can_drivers.c"
#elif CAN_ARCHITECTURE_TEST

void Can_Init(uint32_t baudrate) {
  // TODO test harness
}

Can_ErrorID_T Can_RawWrite(Frame *frame) {
  // TODO test harness
  return 0;
}

Can_ErrorID_T Can_RawRead(Frame *frame) {
  // TODO test harness
  return 0;
}

#else
  // Define nothing so that there is a linker error!
#endif

Can_MsgID_T Can_MsgType(void) {
  lastError = Can_RawRead(&lastMessage);
  if (lastError == Can_Error_NO_RX) {
    return Can_No_Msg;
  } else if (lastError != Can_Error_NONE) {
    return Can_Error_Msg;
  }

  uint16_t id = lastMessage.id;
  uint16_t first_byte = lastMessage.data[0];
  switch(id) {
    case FRONT_CAN_NODE_DRIVER_OUTPUT__id:
      return Can_FrontCanNode_DriverOutput_Msg;
    case FRONT_CAN_NODE_RAW_VALUES__id:
      return Can_FrontCanNode_RawValues_Msg;
    case FRONT_CAN_NODE_WHEEL_SPEED__id:
      return Can_FrontCanNode_WheelSpeed_Msg;
    case REAR_CAN_NODE_HEARTBEAT__id:
      return Can_RearCanNode_Heartbeat_Msg;
    case REAR_CAN_NODE_WHEEL_SPEED__id:
      return Can_RearCanNode_WheelSpeed_Msg;

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

    case DASH_HEARTBEAT__id:
      return Can_Dash_Heartbeat_Msg;
    case DASH_REQUEST__id:
      return Can_Dash_Request_Msg;

    case MC_RESPONSE__id:
      if (first_byte == CAN_MC_REG_ERRORS_AND_WARNINGS) {
        return Can_MC_ErrorAndWarning_Msg;
      } else if (first_byte == CAN_MC_REG_STATE) {
        return Can_MC_State_Msg;
      } else {
        return Can_MC_DataReading_Msg;
      }

    case CURRENT_SENSOR_VOLTAGE__id:
      return Can_CurrentSensor_Voltage_Msg;
    case CURRENT_SENSOR_CURRENT__id:
      return Can_CurrentSensor_Current_Msg;
    case CURRENT_SENSOR_POWER__id:
      return Can_CurrentSensor_Power_Msg;
    case CURRENT_SENSOR_ENERGY__id:
      return Can_CurrentSensor_Energy_Msg;

    default:
      return Can_Unknown_Msg;
  }
}

// TODO this is a bit of a hack...unknown reads should follow same as regular reads
// and use of Can_RawRead must be banned.
Can_ErrorID_T Can_Unknown_Read(Frame *frame) {
  if (lastError == Can_Error_NONE) {
    frame->id = lastMessage.id;
    frame->len = lastMessage.len;
    uint8_t i;
    for (i = 0; i < 8; i++) {
      frame->data[i] = lastMessage.data[i];
    }
    lastError = Can_Error_NO_RX;
    return Can_Error_NONE;
  } else {
    return lastError;
  }
}

Can_ErrorID_T Can_Error_Read(void) {
  Can_ErrorID_T cachedError = lastError;
  lastError = Can_Error_NO_RX;
  return cachedError;
}

TO_CAN(Can_FrontCanNode_DriverOutput) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->torque, bitstring, 0, 16);
  bitstring = INSERT(type_in->brake_pressure, bitstring, 16, 8);
  bitstring = INSERT(type_in->steering_position, bitstring, 24, 8);
  bitstring = INSERT(type_in->throttle_implausible, bitstring, 32, 1);
  bitstring = INSERT(type_in->brake_throttle_conflict, bitstring, 33, 1);
  bitstring = INSERT(type_in->brake_engaged, bitstring, 34, 1);
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
  type_out->brake_engaged = EXTRACT(bitstring, 34, 1);
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

TO_CAN(Can_RearCanNode_Heartbeat) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->is_alive, bitstring, 0, 1);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = REAR_CAN_NODE_HEARTBEAT__id;
  can_out->len = 1;
}

FROM_CAN(Can_RearCanNode_Heartbeat) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->is_alive = EXTRACT(bitstring, 0, 1);
}

TO_CAN(Can_RearCanNode_WheelSpeed) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->rear_right_wheel_speed, bitstring, 0, 32);
  bitstring = INSERT(type_in->rear_left_wheel_speed, bitstring, 32, 32);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = REAR_CAN_NODE_WHEEL_SPEED__id;
  can_out->len = 8;
}

FROM_CAN(Can_RearCanNode_WheelSpeed) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->rear_right_wheel_speed = EXTRACT(bitstring, 0, 32);
  type_out->rear_left_wheel_speed = EXTRACT(bitstring, 32, 32);
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
  bitstring = INSERT(type_in->heartbeat_front_can_node_dead, bitstring, 25, 1);
  bitstring = INSERT(type_in->heartbeat_rear_can_node_dead, bitstring, 26, 1);
  bitstring = INSERT(type_in->heartbeat_bms_dead, bitstring, 27, 1);
  bitstring = INSERT(type_in->heartbeat_dash_dead, bitstring, 28, 1);
  bitstring = INSERT(type_in->heartbeat_mc_dead, bitstring, 29, 1);
  bitstring = INSERT(type_in->heartbeat_current_sensor_dead, bitstring, 30, 1);
  bitstring = INSERT(type_in->tsms_off, bitstring, 31, 1);
  bitstring = INSERT(type_in->reset_latch_open, bitstring, 32, 1);
  bitstring = INSERT(type_in->precharge_running, bitstring, 33, 1);
  bitstring = INSERT(type_in->master_reset_not_initialized, bitstring, 34, 1);
  bitstring = INSERT(type_in->driver_reset_not_initialized, bitstring, 35, 1);
  bitstring = INSERT(type_in->lv_battery_voltage, bitstring, 40, 10);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = VCU_DASH_HEARTBEAT__id;
  can_out->len = 5;
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
  type_out->heartbeat_front_can_node_dead = EXTRACT(bitstring, 25, 1);
  type_out->heartbeat_rear_can_node_dead = EXTRACT(bitstring, 26, 1);
  type_out->heartbeat_bms_dead = EXTRACT(bitstring, 27, 1);
  type_out->heartbeat_dash_dead = EXTRACT(bitstring, 28, 1);
  type_out->heartbeat_mc_dead = EXTRACT(bitstring, 29, 1);
  type_out->heartbeat_current_sensor_dead = EXTRACT(bitstring, 30, 1);
  type_out->tsms_off = EXTRACT(bitstring, 31, 1);
  type_out->reset_latch_open = EXTRACT(bitstring, 32, 1);
  type_out->precharge_running = EXTRACT(bitstring, 33, 1);
  type_out->master_reset_not_initialized = EXTRACT(bitstring, 34, 1);
  type_out->driver_reset_not_initialized = EXTRACT(bitstring, 35, 1);
  type_out->lv_battery_voltage = EXTRACT(bitstring, 40, 10);
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
  // TODO all other devices are big endian except motor controllers,
  // so we want to reverse the endianness
  uint64_t temp_torque_cmd = type_in->torque_cmd;
  uint8_t high_byte = EXTRACT(temp_torque_cmd, 48, 8);
  uint8_t low_byte = EXTRACT(temp_torque_cmd, 56, 8);
  bitstring = INSERT(low_byte, bitstring, 8, 8);
  bitstring = INSERT(high_byte, bitstring, 16, 8);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = VCU_MC_MESSAGE__id;
  can_out->len = 3;
}

FROM_CAN(Can_Vcu_MCTorque) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  // TODO all other devices are big endian except motor controllers,
  // so we want to reverse the endianness
  uint8_t low_byte = EXTRACT(bitstring, 8, 8);
  uint8_t high_byte = EXTRACT(bitstring, 16, 8);
  int16_t low_word = (int16_t) low_byte;
  int16_t high_word = (int16_t) (high_byte << 8);
  type_out->torque_cmd = low_word | high_word;
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

  // Little Endian
  int16_t swap_value = swap_int16(type_in->value);

  bitstring = INSERT(swap_value, bitstring, 8, 16);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = MC_RESPONSE__id;
  can_out->len = 3;
}

FROM_CAN(Can_MC_DataReading) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->type = (Can_MC_RegID_T)(EXTRACT(bitstring, 0, 8));

  // Little Endian
  int16_t swap_value = (int16_t)(swap_uint16(EXTRACT(bitstring, 8, 16)));

  type_out->value = swap_value;
}

#define BIT_SET(input, bit_value, bit_idx) \
  if ((bit_value)) { \
    ((input) |= (1UL << (bit_idx))); \
  } \
  else { \
    ((input) &= ~(1UL << (bit_idx))); \
  }

#define BIT_GET(input, bit_idx) \
  (bool)((input) & (1UL << (bit_idx)))

TO_CAN(Can_MC_ErrorAndWarning) {
  uint64_t bitstring = 0;
  bitstring = INSERT(CAN_MC_REG_ERRORS_AND_WARNINGS, bitstring, 0, 8);

  // Start as big endian for now, swap later.
  // Also need to work as 0-as-least-significant-bit due to motor controller
  // reasons.
  uint32_t data_string = 0;
  BIT_SET(data_string, type_in->error_parameter_damaged, 0);
  BIT_SET(data_string, type_in->error_output_stage_fault, 1);
  BIT_SET(data_string, type_in->error_rfe_fault, 2);
  BIT_SET(data_string, type_in->error_bus_fault, 3);
  BIT_SET(data_string, type_in->error_faulty_encoder, 4);
  BIT_SET(data_string, type_in->error_power_voltage_missing, 5);
  BIT_SET(data_string, type_in->error_motor_temp_high, 6);
  BIT_SET(data_string, type_in->error_device_temp_high, 7);
  BIT_SET(data_string, type_in->error_over_voltage, 8);
  BIT_SET(data_string, type_in->error_over_current, 9);
  BIT_SET(data_string, type_in->error_raceaway, 10);
  BIT_SET(data_string, type_in->error_user_selected_fault, 11);
  BIT_SET(data_string, type_in->error_i2r_overload, 12);
  BIT_SET(data_string, type_in->error_incompatible_firmware, 14);
  BIT_SET(data_string, type_in->error_ballast_overload, 15);

  BIT_SET(data_string, type_in->warning_inconsistent_device, 16);
  BIT_SET(data_string, type_in->warning_illegal_status_emi, 17);
  BIT_SET(data_string, type_in->warning_rfe_signal_inactive, 18);
  BIT_SET(data_string, type_in->warning_power_voltage_low, 21);
  BIT_SET(data_string, type_in->warning_motor_temp_high, 22);
  BIT_SET(data_string, type_in->warning_device_temp_high, 23);
  BIT_SET(data_string, type_in->warning_over_voltage, 24);
  BIT_SET(data_string, type_in->warning_over_current, 25);
  BIT_SET(data_string, type_in->warning_i2r_overload, 28);
  BIT_SET(data_string, type_in->warning_ballast_overload, 31);

  uint32_t swapped_data_string = swap_uint32(data_string);
  bitstring = INSERT(swapped_data_string, bitstring, 8, 32);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = MC_RESPONSE__id;
  can_out->len = 5;
}

FROM_CAN(Can_MC_ErrorAndWarning) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);

  uint32_t swapped_data_string = EXTRACT(bitstring, 8, 32);
  uint32_t data_string = swap_uint32(swapped_data_string);

  type_out->error_parameter_damaged = BIT_GET(data_string, 0);
  type_out->error_output_stage_fault = BIT_GET(data_string, 1);
  type_out->error_rfe_fault = BIT_GET(data_string, 2);
  type_out->error_bus_fault = BIT_GET(data_string, 3);
  type_out->error_faulty_encoder = BIT_GET(data_string, 4);
  type_out->error_power_voltage_missing = BIT_GET(data_string, 5);
  type_out->error_motor_temp_high = BIT_GET(data_string, 6);
  type_out->error_device_temp_high = BIT_GET(data_string, 7);
  type_out->error_over_voltage = BIT_GET(data_string, 8);
  type_out->error_over_current = BIT_GET(data_string, 9);
  type_out->error_raceaway = BIT_GET(data_string, 10);
  type_out->error_user_selected_fault = BIT_GET(data_string, 11);
  type_out->error_i2r_overload = BIT_GET(data_string, 12);
  type_out->error_incompatible_firmware = BIT_GET(data_string, 14);
  type_out->error_ballast_overload = BIT_GET(data_string, 15);

  type_out->warning_inconsistent_device = BIT_GET(data_string, 16);
  type_out->warning_illegal_status_emi = BIT_GET(data_string, 17);
  type_out->warning_rfe_signal_inactive = BIT_GET(data_string, 18);
  type_out->warning_power_voltage_low = BIT_GET(data_string, 21);
  type_out->warning_motor_temp_high = BIT_GET(data_string, 22);
  type_out->warning_device_temp_high = BIT_GET(data_string, 23);
  type_out->warning_over_voltage = BIT_GET(data_string, 24);
  type_out->warning_over_current = BIT_GET(data_string, 25);
  type_out->warning_i2r_overload = BIT_GET(data_string, 28);
  type_out->warning_ballast_overload = BIT_GET(data_string, 31);
}

TO_CAN(Can_MC_State) {
  uint64_t bitstring = 0;
  bitstring = INSERT(CAN_MC_REG_ERRORS_AND_WARNINGS, bitstring, 0, 8);

  // Start as big endian for now, swap later.
  // Also need to work as 0-as-least-significant-bit due to motor controller
  // reasons.
  uint32_t data_string = 0;
  BIT_SET(data_string, type_in->hardware_enable, 0);
  BIT_SET(data_string, type_in->drive_stopped, 1);
  BIT_SET(data_string, type_in->lim_plus_assigned, 2);
  BIT_SET(data_string, type_in->lim_minus_assigned, 3);
  BIT_SET(data_string, type_in->drive_ok, 4);
  BIT_SET(data_string, type_in->current_limit_to_continuous, 5);
  BIT_SET(data_string, type_in->speed_limited_torque_mode, 6);
  BIT_SET(data_string, type_in->position_control_mode, 7);
  BIT_SET(data_string, type_in->speed_control_mode, 8);
  BIT_SET(data_string, type_in->low_speed, 9);
  BIT_SET(data_string, type_in->btb_rdy, 14);
  BIT_SET(data_string, type_in->regen_active, 15);
  BIT_SET(data_string, type_in->inverted_command, 16);
  BIT_SET(data_string, type_in->speed_limited_via_switch, 17);
  BIT_SET(data_string, type_in->current_limited_via_switch, 20);
  BIT_SET(data_string, type_in->active_current_reduction, 21);
  BIT_SET(data_string, type_in->current_limited_via_speed, 22);
  BIT_SET(data_string, type_in->current_limited_via_igbt_temp, 23);
  BIT_SET(data_string, type_in->current_reduction_low_frequency, 25);
  BIT_SET(data_string, type_in->current_reduction_via_motor_temp, 26);
  BIT_SET(data_string, type_in->current_reduction_via_analog_input, 27);
  BIT_SET(data_string, type_in->handwheel_input_selected, 31);

  uint32_t swapped_data_string = swap_uint32(data_string);
  bitstring = INSERT(swapped_data_string, bitstring, 8, 32);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = MC_RESPONSE__id;
  can_out->len = 5;
}

FROM_CAN(Can_MC_State) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);

  uint32_t swapped_data_string = EXTRACT(bitstring, 8, 32);
  uint32_t data_string = swap_uint32(swapped_data_string);

  type_out->hardware_enable = BIT_GET(data_string, 0);
  type_out->drive_stopped = BIT_GET(data_string, 1);
  type_out->lim_plus_assigned = BIT_GET(data_string, 2);
  type_out->lim_minus_assigned = BIT_GET(data_string, 3);
  type_out->drive_ok = BIT_GET(data_string, 4);
  type_out->current_limit_to_continuous = BIT_GET(data_string, 5);
  type_out->speed_limited_torque_mode = BIT_GET(data_string, 6);
  type_out->position_control_mode = BIT_GET(data_string, 7);
  type_out->speed_control_mode = BIT_GET(data_string, 8);
  type_out->low_speed = BIT_GET(data_string, 9);
  type_out->btb_rdy = BIT_GET(data_string, 14);
  type_out->regen_active = BIT_GET(data_string, 15);
  type_out->inverted_command = BIT_GET(data_string, 16);
  type_out->speed_limited_via_switch = BIT_GET(data_string, 17);
  type_out->current_limited_via_switch = BIT_GET(data_string, 20);
  type_out->active_current_reduction = BIT_GET(data_string, 21);
  type_out->current_limited_via_speed = BIT_GET(data_string, 22);
  type_out->current_limited_via_igbt_temp = BIT_GET(data_string, 23);
  type_out->current_reduction_low_frequency = BIT_GET(data_string, 25);
  type_out->current_reduction_via_motor_temp = BIT_GET(data_string, 26);
  type_out->current_reduction_via_analog_input = BIT_GET(data_string, 27);
  type_out->handwheel_input_selected = BIT_GET(data_string, 31);
}

TO_CAN(Can_CurrentSensor_Current) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->current_mA, bitstring, 16, 32);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = CURRENT_SENSOR_CURRENT__id;
  can_out->len = 6;
}

FROM_CAN(Can_CurrentSensor_Current) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->current_mA = SIGN((EXTRACT(bitstring, 16, 32)), 32);
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

TO_CAN(Can_CurrentSensor_Power) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->power_W, bitstring, 16, 32);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = CURRENT_SENSOR_POWER__id;
  can_out->len = 6;
}

FROM_CAN(Can_CurrentSensor_Power) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->power_W = SIGN((EXTRACT(bitstring, 16, 32)), 32);
}

TO_CAN(Can_CurrentSensor_Energy) {
  uint64_t bitstring = 0;
  bitstring = INSERT(type_in->energy_Wh, bitstring, 16, 32);
  from_bitstring(&bitstring, can_out->data);
  can_out->id = CURRENT_SENSOR_ENERGY__id;
  can_out->len = 6;
}

FROM_CAN(Can_CurrentSensor_Energy) {
  uint64_t bitstring = 0;
  to_bitstring(can_in->data, &bitstring);
  type_out->energy_Wh = SIGN((EXTRACT(bitstring, 16, 32)), 32);
}

// Needed for actual implementation of the evil macros
DEFINE(Can_FrontCanNode_DriverOutput)
DEFINE(Can_FrontCanNode_RawValues)
DEFINE(Can_FrontCanNode_WheelSpeed)
DEFINE(Can_RearCanNode_Heartbeat)
DEFINE(Can_RearCanNode_WheelSpeed)

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
DEFINE(Can_MC_ErrorAndWarning)
DEFINE(Can_MC_State)

DEFINE(Can_CurrentSensor_Current)
DEFINE(Can_CurrentSensor_Voltage)
DEFINE(Can_CurrentSensor_Power)
DEFINE(Can_CurrentSensor_Energy)
