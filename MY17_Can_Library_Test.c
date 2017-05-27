#include "MY17_Can_Library_Test.h"

#include "MY17_Can_Library.h"

typedef void (*PRINT)(const char *);

#define BOILERPLATE(typename) \
  Frame mid; \
  typename ## _ToCan(&begin, &mid); \
  uint64_t end_bitstring = 0; \
  to_bitstring(mid.data, &end_bitstring); \
  typename ## _T end; \
  typename ## _FromCan(&mid, &end)


void Can_BinaryPrint(PRINT print, uint64_t out, uint8_t len) {
  int8_t i;
  for (i = len - 1; i >= 0; i--) {
    bool result = (out >> i) & 0x1;
    print(result ? "1" : "0");
  }
}

void Can_FrontCanNode_DriverOutput_Test(PRINT print) {
  Can_FrontCanNode_DriverOutput_T begin;
  begin.torque = -10000;
  begin.brake_pressure = 200;
  begin.steering_position = 100;
  begin.throttle_implausible = true;
  begin.brake_throttle_conflict = false;
  begin.brake_engaged = true;

  BOILERPLATE(Can_FrontCanNode_DriverOutput);

  bool equal = (
      begin.torque == end.torque &&
      begin.brake_pressure == end.brake_pressure &&
      begin.steering_position == end.steering_position &&
      begin.throttle_implausible == end.throttle_implausible &&
      begin.brake_throttle_conflict == end.brake_throttle_conflict &&
      begin.brake_engaged == end.brake_engaged);
  print(equal ? "FrontCanNode_DriverOutput_PASS\r\n" : "FrontCanNode_DriverOutput_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_FrontCanNode_RawValues_Test(PRINT print) {
  Can_FrontCanNode_RawValues_T begin;
  begin.accel_1_raw = 100;
  begin.accel_2_raw = 200;
  begin.brake_1_raw = 300;
  begin.brake_2_raw = 400;
  begin.steering_raw = 500;

  BOILERPLATE(Can_FrontCanNode_RawValues);

  bool equal = (
      begin.accel_1_raw == end.accel_1_raw &&
      begin.accel_2_raw == end.accel_2_raw &&
      begin.brake_1_raw == end.brake_1_raw &&
      begin.brake_2_raw == end.brake_2_raw &&
      begin.steering_raw == end.steering_raw);
  print(equal ? "FrontCanNode_RawValues_PASS\r\n" : "FrontCanNode_RawValues_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_FrontCanNode_WheelSpeed_Test(PRINT print) {
  Can_FrontCanNode_WheelSpeed_T begin;
  begin.front_right_wheel_speed = 100000;
  begin.front_left_wheel_speed = 3000000000UL;

  BOILERPLATE(Can_FrontCanNode_WheelSpeed);

  bool equal = (
      begin.front_right_wheel_speed == end.front_right_wheel_speed &&
      begin.front_left_wheel_speed == end.front_left_wheel_speed);

  print(equal ? "FrontCanNode_WheelSpeed_PASS\r\n" : "FrontCanNode_WheelSpeed_FAIL!!!!!!!!!!!!!!!!\r\n");

}

void Can_RearCanNode_Heartbeat_Test(PRINT print) {
  Can_RearCanNode_Heartbeat_T begin;
  begin.is_alive = true;

  BOILERPLATE(Can_RearCanNode_Heartbeat);

  bool equal = begin.is_alive == end.is_alive;

  print(equal ? "RearCanNode_Heartbeat_PASS\r\n" : "RearCanNode_Heartbeat_FAIL!!!!!!!!!!!!!!!!\r\n");

}

void Can_RearCanNode_WheelSpeed_Test(PRINT print) {
  Can_RearCanNode_WheelSpeed_T begin;
  begin.rear_right_wheel_speed = 1000000;
  begin.rear_left_wheel_speed = 30000000UL;

  BOILERPLATE(Can_RearCanNode_WheelSpeed);

  bool equal = (
      begin.rear_right_wheel_speed == end.rear_right_wheel_speed &&
      begin.rear_left_wheel_speed == end.rear_left_wheel_speed);

  print(equal ? "RearCanNode_WheelSpeed_PASS\r\n" : "RearCanNode_WheelSpeed_FAIL!!!!!!!!!!!!!!!!\r\n");

}

void Can_Vcu_BmsHeartbeat_Test(PRINT print) {
  Can_Vcu_BmsHeartbeat_T begin;
  begin.alwaysTrue = true;

  BOILERPLATE(Can_Vcu_BmsHeartbeat);

  bool equal = begin.alwaysTrue == end.alwaysTrue;
  print(equal ? "Vcu_BmsHeartbeat_PASS\r\n" : "Vcu_BmsHeartbeat_FAIL!!!!!!!!!!!!!!!!\r\n");

}

void Can_Vcu_DashHeartbeat_Test(PRINT print) {
  Can_Vcu_DashHeartbeat_T begin;

  begin.rtd_light = true;
  begin.ams_light = false;
  begin.imd_light = false;
  begin.hv_light = true;

  begin.traction_control = true;
  begin.limp_mode = true;
  begin.lv_warning = false;
  begin.active_aero = true;

  begin.regen = true;
  begin.shutdown_esd_drain = false;
  begin.shutdown_bms = true;
  begin.shutdown_imd = true;

  begin.shutdown_bspd = false;
  begin.shutdown_vcu = false;
  begin.shutdown_precharge = false;
  begin.shutdown_master_reset = true;

  begin.shutdown_driver_reset = false;

  begin.lv_battery_voltage = 138;

  begin.heartbeat_front_can_node_dead = false;
  begin.heartbeat_rear_can_node_dead = false;
  begin.heartbeat_bms_dead = true;
  begin.heartbeat_dash_dead = false;
  begin.heartbeat_mc_dead = true;
  begin.heartbeat_current_sensor_dead = false;

  begin.tsms_off = false;
  begin.reset_latch_open = true;
  begin.precharge_running = true;

  begin.master_reset_not_initialized = false;
  begin.driver_reset_not_initialized = true;

  BOILERPLATE(Can_Vcu_DashHeartbeat);

  bool equal = (
      begin.rtd_light == end.rtd_light &&
      begin.ams_light == end.ams_light &&
      begin.imd_light == end.imd_light &&
      begin.hv_light == end.hv_light &&
      begin.traction_control == end.traction_control &&
      begin.limp_mode == end.limp_mode &&
      begin.lv_warning == end.lv_warning &&
      begin.active_aero == end.active_aero &&
      begin.regen == end.regen &&
      begin.shutdown_esd_drain == end.shutdown_esd_drain &&
      begin.shutdown_bms == end.shutdown_bms &&
      begin.shutdown_imd == end.shutdown_imd &&
      begin.shutdown_bspd == end.shutdown_bspd &&
      begin.shutdown_vcu == end.shutdown_vcu &&
      begin.shutdown_precharge == end.shutdown_precharge &&
      begin.shutdown_master_reset == end.shutdown_master_reset &&
      begin.shutdown_driver_reset == end.shutdown_driver_reset &&
      begin.lv_battery_voltage == end.lv_battery_voltage &&
      begin.heartbeat_front_can_node_dead == end.heartbeat_front_can_node_dead &&
      begin.heartbeat_rear_can_node_dead == end.heartbeat_rear_can_node_dead &&
      begin.heartbeat_bms_dead == end.heartbeat_bms_dead &&
      begin.heartbeat_dash_dead == end.heartbeat_dash_dead &&
      begin.heartbeat_mc_dead == end.heartbeat_mc_dead &&
      begin.heartbeat_current_sensor_dead == end.heartbeat_current_sensor_dead &&
      begin.tsms_off == end.tsms_off &&
      begin.reset_latch_open == end. reset_latch_open &&
      begin.precharge_running == end.precharge_running &&
      begin.master_reset_not_initialized == end.master_reset_not_initialized &&
      begin.driver_reset_not_initialized == end.driver_reset_not_initialized);

  print(equal ? "Vcu_DashHeartbeat_PASS\r\n" : "Vcu_DashHeartbeat_FAIL!!!!!!!!!!!!!!!!\r\n");

}

void Can_Vcu_MCRequest_Test(PRINT print) {
  Can_Vcu_MCRequest_T begin;
  begin.requestType = CAN_MC_REG_MSG_REQUEST;
  begin.period = 100;

  BOILERPLATE(Can_Vcu_MCRequest);

  bool equal = (
      begin.requestType == end.requestType &&
      begin.period == end.period);
  print(equal ? "Vcu_MCRequest_PASS\r\n" : "Vcu_MCRequest_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_Vcu_MCTorque_Test(PRINT print) {
  Can_Vcu_MCTorque_T begin;
  begin.torque_cmd = 5000;

  BOILERPLATE(Can_Vcu_MCTorque);

  bool equal = begin.torque_cmd == end.torque_cmd;
  print(equal ? "Vcu_MCTorque_PASS\r\n" : "Vcu_MCTorque_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_Bms_Heartbeat_Test(PRINT print) {
  Can_Bms_Heartbeat_T begin;
  begin.state = CAN_BMS_STATE_BATTERY_FAULT;
  begin.soc = 90;

  BOILERPLATE(Can_Bms_Heartbeat);

  bool equal = (
      begin.state == end.state &&
      begin.soc == end.soc);
  print(equal ? "Bms_Heartbeat_PASS\r\n" : "Bms_Heartbeat_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_Bms_CellTemps_Test(PRINT print) {
  Can_Bms_CellTemps_T begin;
  begin.avg_cell_temp = -7520;
  begin.min_cell_temp = 7000;
  begin.id_min_cell_temp = 300;
  begin.max_cell_temp = -200;
  begin.id_max_cell_temp = 10;

  BOILERPLATE(Can_Bms_CellTemps);

  bool equal = (
      begin.avg_cell_temp == end.avg_cell_temp &&
      begin.min_cell_temp == end.min_cell_temp &&
      begin.id_min_cell_temp == end.id_min_cell_temp &&
      begin.max_cell_temp == end.max_cell_temp &&
      begin.id_max_cell_temp == end.id_max_cell_temp);
  print(equal ? "Bms_CellTemps_PASS\r\n" : "Bms_CellTemps_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_Bms_PackStatus_Test(PRINT print) {
  Can_Bms_PackStatus_T begin;
  begin.pack_voltage = 260;
  begin.pack_current = -600;
  begin.avg_cell_voltage = 330;
  begin.min_cell_voltage = 150;
  begin.id_min_cell_voltage = 20;
  begin.max_cell_voltage = 400;
  begin.id_max_cell_voltage = 40;

  BOILERPLATE(Can_Bms_PackStatus);

  bool equal = (
      begin.pack_voltage == end.pack_voltage &&
      begin.pack_current == end.pack_current &&
      begin.avg_cell_voltage == end.avg_cell_voltage &&
      begin.min_cell_voltage == end.min_cell_voltage &&
      begin.id_min_cell_voltage == end.id_min_cell_voltage &&
      begin.max_cell_voltage == end.max_cell_voltage &&
      begin.id_max_cell_voltage == end.id_max_cell_voltage);
  print(equal ? "Bms_PackStatus_PASS\r\n" : "Bms_PackStatus_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_Bms_Error_Test(PRINT print) {
  Can_Bms_Error_T begin;
  begin.type = CAN_BMS_ERROR_CONFLICTING_MODE_REQUESTS;

  BOILERPLATE(Can_Bms_Error);

  bool equal = begin.type == end.type;
  print(equal ? "Bms_Error_PASS\r\n" : "Bms_Error_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_Dash_Heartbeat_Test(PRINT print) {
  Can_Dash_Heartbeat_T begin;
  begin.ok = true;

  BOILERPLATE(Can_Dash_Heartbeat);

  bool equal = begin.ok == end.ok;
  print(equal ? "Dash_Heartbeat_PASS\r\n" : "Dash_Heartbeat_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_Dash_Request_Test(PRINT print) {
  Can_Dash_Request_T begin;
  begin.type = CAN_DASH_REQUEST_DATA_FLAG;

  BOILERPLATE(Can_Dash_Request);

  bool equal = begin.type == end.type;
  print(equal ? "Dash_Request_PASS\r\n" : "Dash_Request_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_MC_DataReading_Test(PRINT print) {
  Can_MC_DataReading_T begin;
  begin.type = CAN_MC_REG_CURRENT_LIMIT_ACTUAL;
  begin.value = -6000;

  BOILERPLATE(Can_MC_DataReading);

  bool equal = (
      begin.type == end.type &&
      begin.value == end.value);
  print(equal ? "MC_DataReading_PASS\r\n" : "MC_DataReading_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_MC_ErrorAndWarning_Test(PRINT print) {
  Can_MC_ErrorAndWarning_T begin;

  begin.error_parameter_damaged = true;
  begin.error_output_stage_fault = true;
  begin.error_rfe_fault = true;
  begin.error_bus_fault = false;
  begin.error_faulty_encoder = false;
  begin.error_power_voltage_missing = true;
  begin.error_motor_temp_high = false;
  begin.error_device_temp_high = false;
  begin.error_over_voltage = true;
  begin.error_over_current = false;
  begin.error_raceaway = true;
  begin.error_user_selected_fault = false;
  begin.error_i2r_overload = false;
  begin.error_incompatible_firmware = true;
  begin.error_ballast_overload = false;

  begin.warning_inconsistent_device = true;
  begin.warning_illegal_status_emi = false;
  begin.warning_rfe_signal_inactive = true;
  begin.warning_power_voltage_low = true;
  begin.warning_device_temp_high = false;
  begin.warning_over_voltage = true;
  begin.warning_over_current = false;
  begin.warning_i2r_overload = false;
  begin.warning_ballast_overload = true;

  BOILERPLATE(Can_MC_ErrorAndWarning);

  bool equal = (
      begin.error_parameter_damaged == end.error_parameter_damaged &&
      begin.error_output_stage_fault == end.error_output_stage_fault &&
      begin.error_rfe_fault == end.error_rfe_fault &&
      begin.error_bus_fault == end.error_bus_fault &&
      begin.error_faulty_encoder == end.error_faulty_encoder &&
      begin.error_power_voltage_missing == end.error_power_voltage_missing &&
      begin.error_motor_temp_high == end.error_motor_temp_high &&
      begin.error_device_temp_high == end.error_device_temp_high &&
      begin.error_over_voltage == end.error_over_voltage &&
      begin.error_over_current == end.error_over_current &&
      begin.error_raceaway == end.error_raceaway &&
      begin.error_user_selected_fault == end.error_user_selected_fault &&
      begin.error_i2r_overload == end.error_i2r_overload &&
      begin.error_incompatible_firmware == end.error_incompatible_firmware &&
      begin.error_ballast_overload == end.error_ballast_overload &&

      begin.warning_inconsistent_device == end.warning_inconsistent_device &&
      begin.warning_illegal_status_emi == end.warning_illegal_status_emi &&
      begin.warning_rfe_signal_inactive == end.warning_rfe_signal_inactive &&
      begin.warning_power_voltage_low == end.warning_power_voltage_low &&
      begin.warning_device_temp_high == end.warning_device_temp_high &&
      begin.warning_over_voltage == end.warning_over_voltage &&
      begin.warning_over_current == end.warning_over_current &&
      begin.warning_i2r_overload == end.warning_i2r_overload &&
      begin.warning_ballast_overload == end.warning_ballast_overload);
  print(equal ? "MC_ErrorAndWarning_PASS\r\n" : "MC_ErrorAndWarning_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_MC_State_Test(PRINT print) {
  Can_MC_State_T begin;

      begin.hardware_enable = true;
      begin.drive_stopped = false;
      begin.lim_plus_assigned = true;
      begin.lim_minus_assigned = false;
      begin.drive_ok = false;
      begin.current_limit_to_continuous = true;
      begin.speed_limited_torque_mode = false;
      begin.position_control_mode = true;
      begin.speed_control_mode = true;
      begin.low_speed = false;
      begin.btb_rdy = true;
      begin.regen_active = false;
      begin.inverted_command = true;
      begin.speed_limited_via_switch = false;
      begin.current_limited_via_switch = false;
      begin.active_current_reduction = true;
      begin.current_limited_via_speed = false;
      begin.current_limited_via_igbt_temp = true;
      begin.current_reduction_low_frequency = true;
      begin.current_reduction_via_motor_temp = false;
      begin.current_reduction_via_analog_input = false;
      begin.handwheel_input_selected = true;

  BOILERPLATE(Can_MC_State);

  bool equal = (
      begin.hardware_enable == end.hardware_enable &&
      begin.drive_stopped == end.drive_stopped &&
      begin.lim_plus_assigned == end.lim_plus_assigned &&
      begin.lim_minus_assigned == end.lim_minus_assigned &&
      begin.drive_ok == end.drive_ok &&
      begin.current_limit_to_continuous == end.current_limit_to_continuous &&
      begin.speed_limited_torque_mode == end.speed_limited_torque_mode &&
      begin.position_control_mode == end.position_control_mode &&
      begin.speed_control_mode == end.speed_control_mode &&
      begin.low_speed == end.low_speed &&
      begin.btb_rdy == end.btb_rdy &&
      begin.regen_active == end.regen_active &&
      begin.inverted_command == end.inverted_command &&
      begin.speed_limited_via_switch == end.speed_limited_via_switch &&
      begin.current_limited_via_switch == end.current_limited_via_switch &&
      begin.active_current_reduction == end.active_current_reduction &&
      begin.current_limited_via_speed == end.current_limited_via_speed &&
      begin.current_limited_via_igbt_temp
        == end.current_limited_via_igbt_temp &&
      begin.current_reduction_low_frequency
        == end.current_reduction_low_frequency &&
      begin.current_reduction_via_motor_temp
        == end.current_reduction_via_motor_temp &&
      begin.current_reduction_via_analog_input
        == end.current_reduction_via_analog_input &&
      begin.handwheel_input_selected == end.handwheel_input_selected);

  print(equal ? "MC_State_PASS\r\n" : "MC_State_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_CurrentSensor_Current_Test(PRINT print) {
  Can_CurrentSensor_Current_T begin;
  begin.current_mA = -15000;

  BOILERPLATE(Can_CurrentSensor_Current);

  bool equal = begin.current_mA == end.current_mA;
  print(equal ? "CurrentSensor_Current_PASS\r\n" : "CurrentSensor_Current_FAIL!!!!!!!!!!!!!!!!\r\n");
}


void Can_CurrentSensor_Voltage_Test(PRINT print) {
  Can_CurrentSensor_Voltage_T begin;
  begin.voltage_mV = 300000;

  BOILERPLATE(Can_CurrentSensor_Voltage);

  bool equal = begin.voltage_mV == end.voltage_mV;
  print(equal ? "CurrentSensor_Voltage_PASS\r\n" : "CurrentSensor_Voltage_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_CurrentSensor_Power_Test(PRINT print) {
  Can_CurrentSensor_Power_T begin;
  begin.power_W = 80000;

  BOILERPLATE(Can_CurrentSensor_Power);

  bool equal = begin.power_W == end.power_W;
  print(equal ? "CurrentSensor_Power_PASS\r\n" : "CurrentSensor_Power_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_CurrentSensor_Energy_Test(PRINT print) {
  Can_CurrentSensor_Energy_T begin;
  begin.energy_Wh = 10000;

  BOILERPLATE(Can_CurrentSensor_Energy);

  bool equal = begin.energy_Wh == end.energy_Wh;
  print(equal ? "CurrentSensor_Energy_PASS\r\n" : "CurrentSensor_Energy_FAIL!!!!!!!!!!!!!!!!\r\n");
}

void Can_All_Tests(PRINT print) {
  print("\n*********TEST RESULTS **************\n\n");

  Can_FrontCanNode_DriverOutput_Test(print);
  Can_FrontCanNode_RawValues_Test(print);
  Can_FrontCanNode_WheelSpeed_Test(print);
  Can_RearCanNode_Heartbeat_Test(print);
  Can_RearCanNode_WheelSpeed_Test(print);

  Can_Vcu_BmsHeartbeat_Test(print);
  Can_Vcu_DashHeartbeat_Test(print);
  Can_Vcu_MCRequest_Test(print);
  Can_Vcu_MCTorque_Test(print);

  Can_Bms_Heartbeat_Test(print);
  Can_Bms_CellTemps_Test(print);
  Can_Bms_PackStatus_Test(print);
  Can_Bms_Error_Test(print);

  Can_Dash_Heartbeat_Test(print);
  Can_Dash_Request_Test(print);

  Can_MC_DataReading_Test(print);
  Can_MC_ErrorAndWarning_Test(print);
  Can_MC_State_Test(print);

  Can_CurrentSensor_Current_Test(print);
  Can_CurrentSensor_Voltage_Test(print);
  Can_CurrentSensor_Power_Test(print);
  Can_CurrentSensor_Energy_Test(print);
}

