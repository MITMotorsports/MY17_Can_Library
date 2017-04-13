#include "../MY17_Can_Library.h"
#include "../evil_macros.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

void binaryPrint(uint64_t out, uint8_t len) {
  for (int i = len - 1; i >= 0; i--) {
    bool result = (out >> i) & 0x1;
    printf(result ? "1" : "0");
  }
}

void Driver_Output_Test(void) {
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
  printf("%s \n", equal ? "DRIVER_OUTPUT_PASS" : "DRIVER_OUTPUT_FAIL");
}

void Raw_Values_Test(void) {
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
  printf("%s \n", equal ? "RAW_VALUES_PASS" : "RAW_VALUES_FAIL");
}

void Bms_Heartbeat_Test(void) {
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
  printf("%s \n", equal ? "BMS_HEARTBEAT_PASS" : "BMS_HEARTBEAT_FAIL");
}

void Bms_CellTemps_Test() {
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
  printf("%s \n", equal ? "BMS_CELLTEMPS_PASS" : "BMS_CELLTEMPS_FAIL");
}

void Bms_PackStatus_Test() {
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
  printf("%s \n", equal ? "BMS_PACKSTATUS_PASS" : "BMS_PACKSTATUS_FAIL");
}

void Bms_Error_Test(void) {
  Can_Bms_Error_T begin;
  begin.type = CAN_BMS_ERROR_CONFLICTING_MODE_REQUESTS;

  Frame mid;
  Can_Bms_Error_ToCan(&begin, &mid);
  uint64_t end_bitstring = 0;
  to_bitstring(mid.data, &end_bitstring);

  Can_Bms_Error_T end;
  Can_Bms_Error_FromCan(&mid, &end);

  bool equal = begin.type == end.type;
  printf("%s \n", equal ? "BMS_ERROR_PASS" : "BMS_ERROR_FAIL");
}

int main(void) {
  Driver_Output_Test();
  Raw_Values_Test();
  Bms_Heartbeat_Test();
  Bms_CellTemps_Test();
  Bms_PackStatus_Test();
  Bms_Error_Test();
}

