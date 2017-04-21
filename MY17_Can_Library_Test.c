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

  BOILERPLATE(Can_FrontCanNode_DriverOutput);

  bool equal = (
      begin.torque == end.torque &&
      begin.brake_pressure == end.brake_pressure &&
      begin.steering_position == end.steering_position &&
      begin.throttle_implausible == end.throttle_implausible &&
      begin.brake_throttle_conflict == end.brake_throttle_conflict);
  print(equal ? "FrontCanNode_DriverOutput_PASS\r\n" : "FrontCanNode_DriverOutput_FAIL\r\n");
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
  print(equal ? "FrontCanNode_RawValues_PASS\r\n" : "FrontCanNode_RawValues_FAIL\r\n");
}

void Can_FrontCanNode_WheelSpeed_Test(PRINT print) {
  Can_FrontCanNode_WheelSpeed_T begin;
  begin.front_right_wheel_speed = 100000;
  begin.front_left_wheel_speed = 3000000000UL;

  BOILERPLATE(Can_FrontCanNode_WheelSpeed);

  bool equal = (
      begin.front_right_wheel_speed == end.front_right_wheel_speed &&
      begin.front_left_wheel_speed == end.front_left_wheel_speed);

  print(equal ? "FrontCanNode_WheelSpeed_PASS\r\n" : "FrontCanNode_WheelSpeed_FAIL\r\n");

}

void Can_Vcu_BmsHeartbeat_Test(PRINT print) {
  Can_Vcu_BmsHeartbeat_T begin;
  begin.alwaysTrue = true;

  BOILERPLATE(Can_Vcu_BmsHeartbeat);

  bool equal = begin.alwaysTrue == end.alwaysTrue;
  print(equal ? "Vcu_BmsHeartbeat_PASS\r\n" : "Vcu_BmsHeartbeat_FAIL\r\n");

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
      begin.lv_battery_voltage == end.lv_battery_voltage);
  print(equal ? "Vcu_DashHeartbeat_PASS\r\n" : "Vcu_DashHeartbeat_FAIL\r\n");

}

void Can_Vcu_MCRequest_Test(PRINT print) {
  Can_Vcu_MCRequest_T begin;
  begin.requestType = CAN_MC_REG_MSG_REQUEST;
  begin.period = 100;

  BOILERPLATE(Can_Vcu_MCRequest);

  bool equal = (
      begin.requestType == end.requestType &&
      begin.period == end.period);
  print(equal ? "Vcu_MCRequest_PASS\r\n" : "Vcu_MCRequest_FAIL\r\n");
}

void Can_Vcu_MCTorque_Test(PRINT print) {
  Can_Vcu_MCTorque_T begin;
  begin.torque_cmd = 5000;

  BOILERPLATE(Can_Vcu_MCTorque);

  bool equal = begin.torque_cmd == end.torque_cmd;
  print(equal ? "Vcu_MCTorque_PASS\r\n" : "Vcu_MCTorque_FAIL\r\n");
}

void Can_Bms_Heartbeat_Test(PRINT print) {
  Can_Bms_Heartbeat_T begin;
  begin.state = CAN_BMS_STATE_BATTERY_FAULT;
  begin.soc = 90;

  BOILERPLATE(Can_Bms_Heartbeat);

  bool equal = (
      begin.state == end.state &&
      begin.soc == end.soc);
  print(equal ? "Bms_Heartbeat_PASS\r\n" : "Bms_Heartbeat_FAIL\r\n");
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
  print(equal ? "Bms_CellTemps_PASS\r\n" : "Bms_CellTemps_FAIL\r\n");
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
  print(equal ? "Bms_PackStatus_PASS\r\n" : "Bms_PackStatus_FAIL\r\n");
}

void Can_Bms_Error_Test(PRINT print) {
  Can_Bms_Error_T begin;
  begin.type = CAN_BMS_ERROR_CONFLICTING_MODE_REQUESTS;

  BOILERPLATE(Can_Bms_Error);

  bool equal = begin.type == end.type;
  print(equal ? "Bms_Error_PASS\r\n" : "Bms_Error_FAIL\r\n");
}

void Can_Dash_Heartbeat_Test(PRINT print) {
  Can_Dash_Heartbeat_T begin;
  begin.ok = true;

  BOILERPLATE(Can_Dash_Heartbeat);

  bool equal = begin.ok == end.ok;
  print(equal ? "Dash_Heartbeat_PASS\r\n" : "Dash_Heartbeat_FAIL\r\n");
}

void Can_Dash_Request_Test(PRINT print) {
  Can_Dash_Request_T begin;
  begin.type = CAN_DASH_REQUEST_DATA_FLAG;

  BOILERPLATE(Can_Dash_Request);

  bool equal = begin.type == end.type;
  print(equal ? "Dash_Request_PASS\r\n" : "Dash_Request_FAIL\r\n");
}

void Can_MC_DataReading_Test(PRINT print) {
  Can_MC_DataReading_T begin;
  begin.type = CAN_MC_REG_CURRENT_LIMIT_ACTUAL;
  begin.value = -6000;

  BOILERPLATE(Can_MC_DataReading);

  bool equal = (
      begin.type == end.type &&
      begin.value == end.value);
  print(equal ? "MC_DataReading_PASS\r\n" : "MC_DataReading_FAIL\r\n");
}

void Can_CurrentSensor_Voltage_Test(PRINT print) {
  Can_CurrentSensor_Voltage_T begin;
  begin.voltage_mV = 300000;

  BOILERPLATE(Can_CurrentSensor_Voltage);

  bool equal = begin.voltage_mV == end.voltage_mV;
  print(equal ? "CurrentSensor_Voltage_PASS\r\n" : "CurrentSensor_Voltage_FAIL\r\n");
}

void Can_All_Tests(PRINT print) {
  print("\n*********TEST RESULTS **************\n\n");

  Can_FrontCanNode_DriverOutput_Test(print);
  Can_FrontCanNode_RawValues_Test(print);
  Can_FrontCanNode_WheelSpeed_Test(print);

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

  Can_CurrentSensor_Voltage_Test(print);
}

