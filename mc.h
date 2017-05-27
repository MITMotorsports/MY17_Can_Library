#ifndef _MY17_CAN_LIBRARY_MC_H
#define _MY17_CAN_LIBRARY_MC_H

#include "can_validator/fsae_can_spec.h"

#include <stdint.h>
#include <stdbool.h>

#include "ids.h"

typedef struct {
  Can_MC_RegID_T type;
  int16_t value;
} Can_MC_DataReading_T;

typedef struct {
  bool error_parameter_damaged; // bit_0
  bool error_output_stage_fault; // bit_1
  bool error_rfe_fault; // bit_2
  bool error_bus_fault; // bit_3
  bool error_faulty_encoder; // bit_4
  bool error_power_voltage_missing; // bit_5
  bool error_motor_temp_high; // bit_6
  bool error_device_temp_high; // bit_7
  bool error_over_voltage; // bit_8
  bool error_over_current; // bit_9
  bool error_raceaway; // bit_10
  bool error_user_selected_fault; // bit_11
  bool error_i2r_overload; // bit_12
  bool error_incompatible_firmware; // bit_14
  bool error_ballast_overload; // bit_15

  bool warning_inconsistent_device; // bit_16
  bool warning_illegal_status_emi; // bit_17
  bool warning_rfe_signal_inactive; // bit_18
  bool warning_power_voltage_low; // bit_21
  bool warning_motor_temp_high; // bit_22
  bool warning_device_temp_high; // bit_23
  bool warning_over_voltage; // bit_24
  bool warning_over_current; // bit_25
  bool warning_i2r_overload; // bit_28
  bool warning_ballast_overload; // bit_31
} Can_MC_ErrorAndWarning_T;

typedef struct {
  bool hardware_enable; // bit_0
  bool drive_stopped; // bit_1
  bool lim_plus_assigned; // bit_2
  bool lim_minus_assigned; // bit_3
  bool drive_ok; // bit_4
  bool current_limit_to_continuous; // bit_5
  bool speed_limited_torque_mode; // bit_6
  bool position_control_mode; // bit_7
  bool speed_control_mode; // bit_8
  bool low_speed; // bit_9
  bool btb_rdy; // bit_14
  bool regen_active; // bit_15
  bool inverted_command; // bit_16
  bool speed_limited_via_switch; // bit_17
  bool current_limited_via_switch; // bit_20
  bool active_current_reduction; // bit_21
  bool current_limited_via_speed; // bit_22
  bool current_limited_via_igbt_temp; // bit_23
  bool current_reduction_low_frequency; // bit_25
  bool current_reduction_via_motor_temp; // bit_26
  bool current_reduction_via_analog_input; // bit_27
  bool handwheel_input_selected; // bit_31
} Can_MC_State_T;

#endif // _MY17_CAN_LIBRARY_MC_H
