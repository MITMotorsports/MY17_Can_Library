#include "MY17_Can_Library_Test.h"

#include "MY17_Can_Library.h"

typedef void (*PRINT)(const char *);

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

  Frame mid;
  Can_FrontCanNode_DriverOutput_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_FrontCanNode_DriverOutput_T end;
  Can_FrontCanNode_DriverOutput_FromCan(&mid, &end);

  bool equal = (
      begin.torque == end.torque &&
      begin.brake_pressure == end.brake_pressure &&
      begin.steering_position == end.steering_position &&
      begin.throttle_implausible == end.throttle_implausible &&
      begin.brake_throttle_conflict == end.brake_throttle_conflict);
  print(equal ? "FrontCanNode_DriverOutput_PASS\n" : "FrontCanNode_DriverOutput_FAIL\n");
}

void Can_FrontCanNode_RawValues_Test(PRINT print) {
  Can_FrontCanNode_RawValues_T begin;
  begin.accel_1_raw = 100;
  begin.accel_2_raw = 200;
  begin.brake_1_raw = 300;
  begin.brake_2_raw = 400;
  begin.steering_raw = 500;

  Frame mid;
  Can_FrontCanNode_RawValues_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_FrontCanNode_RawValues_T end;
  Can_FrontCanNode_RawValues_FromCan(&mid, &end);
  bool equal = (
      begin.accel_1_raw == end.accel_1_raw &&
      begin.accel_2_raw == end.accel_2_raw &&
      begin.brake_1_raw == end.brake_1_raw &&
      begin.brake_2_raw == end.brake_2_raw &&
      begin.steering_raw == end.steering_raw);
  print(equal ? "FrontCanNode_RawValues_PASS\n" : "FrontCanNode_RawValues_FAIL\n");
}

void Can_FrontCanNode_WheelSpeed_Test(PRINT print) {
  Can_FrontCanNode_WheelSpeed_T begin;
  begin.front_right_wheel_speed = 100000;
  begin.front_left_wheel_speed = 3000000000UL;

  Frame mid;
  Can_FrontCanNode_WheelSpeed_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_FrontCanNode_WheelSpeed_T end;
  Can_FrontCanNode_WheelSpeed_FromCan(&mid, &end);

  bool equal = (
      begin.front_right_wheel_speed == end.front_right_wheel_speed &&
      begin.front_left_wheel_speed == end.front_left_wheel_speed);

  print(equal ? "FrontCanNode_WheelSpeed_PASS\n" : "FrontCanNode_WheelSpeed_FAIL\n");

}

void Can_Vcu_BmsHeartbeat_Test(PRINT print) {
  Can_Vcu_BmsHeartbeat_T begin;
  begin.alwaysTrue = true;

  Frame mid;
  Can_Vcu_BmsHeartbeat_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_Vcu_BmsHeartbeat_T end;
  Can_Vcu_BmsHeartbeat_FromCan(&mid, &end);

  bool equal = begin.alwaysTrue == end.alwaysTrue;
  print(equal ? "Vcu_BmsHeartbeat_PASS\n" : "Vcu_BmsHeartbeat_FAIL\n");

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

  Frame mid;
  Can_Vcu_DashHeartbeat_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_Vcu_DashHeartbeat_T end;
  Can_Vcu_DashHeartbeat_FromCan(&mid, &end);

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
  print(equal ? "Vcu_DashHeartbeat_PASS\n" : "Vcu_DashHeartbeat_FAIL\n");

}

void Can_Vcu_MCRequest_Test(PRINT print) {
  Can_Vcu_MCRequest_T begin;
  begin.requestType = CAN_MC_REG_MSG_REQUEST;
  begin.period = 100;

  Frame mid;
  Can_Vcu_MCRequest_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_Vcu_MCRequest_T end;
  Can_Vcu_MCRequest_FromCan(&mid, &end);

  bool equal = (
      begin.requestType == end.requestType &&
      begin.period == end.period);
  print(equal ? "Vcu_MCRequest_PASS\n" : "Vcu_MCRequest_FAIL\n");
}

void Can_Bms_Heartbeat_Test(PRINT print) {
  Can_Bms_Heartbeat_T begin;
  begin.state = CAN_BMS_STATE_BATTERY_FAULT;
  begin.soc = 90;

  Frame mid;
  Can_Bms_Heartbeat_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_Bms_Heartbeat_T end;
  Can_Bms_Heartbeat_FromCan(&mid, &end);

  bool equal = (
      begin.state == end.state &&
      begin.soc == end.soc);
  print(equal ? "Bms_Heartbeat_PASS\n" : "Bms_Heartbeat_FAIL\n");
}

void Can_Bms_CellTemps_Test(PRINT print) {
  Can_Bms_CellTemps_T begin;
  begin.avg_cell_temp = 45;
  begin.min_cell_temp = 30;
  begin.id_min_cell_temp = 10;
  begin.max_cell_temp = 60;
  begin.id_max_cell_temp = 60;

  Frame mid;
  Can_Bms_CellTemps_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_Bms_CellTemps_T end;
  Can_Bms_CellTemps_FromCan(&mid, &end);

  bool equal = (
      begin.avg_cell_temp == end.avg_cell_temp &&
      begin.min_cell_temp == end.min_cell_temp &&
      begin.id_min_cell_temp == end.id_min_cell_temp &&
      begin.max_cell_temp == end.max_cell_temp &&
      begin.id_max_cell_temp == end.id_max_cell_temp);
  print(equal ? "Bms_CellTemps_PASS\n" : "Bms_CellTemps_FAIL\n");
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

  Frame mid;
  Can_Bms_PackStatus_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_Bms_PackStatus_T end;
  Can_Bms_PackStatus_FromCan(&mid, &end);

  bool equal = (
      begin.pack_voltage == end.pack_voltage &&
      begin.pack_current == end.pack_current &&
      begin.avg_cell_voltage == end.avg_cell_voltage &&
      begin.min_cell_voltage == end.min_cell_voltage &&
      begin.id_min_cell_voltage == end.id_min_cell_voltage &&
      begin.max_cell_voltage == end.max_cell_voltage &&
      begin.id_max_cell_voltage == end.id_max_cell_voltage);
  print(equal ? "Bms_PackStatus_PASS\n" : "Bms_PackStatus_FAIL\n");
}

void Can_Bms_Error_Test(PRINT print) {
  Can_Bms_Error_T begin;
  begin.type = CAN_BMS_ERROR_CONFLICTING_MODE_REQUESTS;

  Frame mid;
  Can_Bms_Error_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_Bms_Error_T end;
  Can_Bms_Error_FromCan(&mid, &end);

  bool equal = begin.type == end.type;
  print(equal ? "Bms_Error_PASS\n" : "Bms_Error_FAIL\n");
}

