#include "MY17_Can_Library.h"
#include "evil_macros.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

void binaryPrint(uint64_t out, uint8_t len) {
  for (int i = len - 1; i >= 0; i--) {
    bool result = (out >> i) & 0x1;
    printf(result ? "1" : "0");
  }
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

int main(void) {
  Driver_Output_Test();
  Raw_Values_Test();
}

