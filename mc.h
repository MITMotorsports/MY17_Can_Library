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
  bool error_parameter_damaged;
  bool error_output_stage_fault;
  bool error_rfe_fault;
  bool error_bus_fault;
  bool error_faulty_encoder;
  bool error_power_voltage_missing;
  bool error_motor_temp_high;
  bool error_device_temp_high;
  bool error_over_voltage;
  bool error_over_current;
  bool error_raceaway;
  bool error_user_selected_fault;
  bool error_i2r_overload;
  bool error_incompatible_firmware;
  bool error_ballast_overload;

  bool warning_inconsistent_device;
  bool warning_emi;
  bool warning_rfe_signal_inactive;
  bool warning_power_voltage_low;
  bool warning_motor_temp_high;
  bool warning_device_temp_high;
  bool warning_over_voltage;
  bool warning_over_current;
  bool warning_i2r_overload;
  bool warning_ballast_overload;
} Can_MC_ErrorAndWarning_T;

typedef struct {
  bool hardware_enable;
  bool drive_stopped;
  bool lim_plus_assigned;
  bool lim_minus_assigned;
  bool drive_ok;
  bool current_limit_to_continuous;
  bool speed_limited_torque_mode;
  bool position_control_mode;
  bool speed_control_mode;
  bool low_speed;
  bool btb_rdy;
  bool regen_active;
  bool inverted_command;
  bool speed_limited_via_switch;
  bool current_limited_via_switch;
  bool current_limited_via_igbt_temp;
  bool current_reduction_via_motor_temp;
  bool current_reduction_active;
  bool current_reduction_via_speed;
  bool current_reduction_low_frequency;
  bool current_reduction_via_analog_input;
  bool handwheel_input_selected;
} Can_MC_State_T;

#endif // _MY17_CAN_LIBRARY_MC_H
