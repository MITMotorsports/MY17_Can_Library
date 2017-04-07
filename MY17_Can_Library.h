#ifndef _MY17_CAN_LIBRARY_H
#define _MY17_CAN_LIBRARY_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * Messages from CAN Node
******************************************************************************/

#include "can_node.h"

/******************************************************************************
 * Messages from VCU
******************************************************************************/

#include "vcu.h"

/******************************************************************************
 * Messages from BMS
******************************************************************************/

#include "bms.h"

/******************************************************************************
 * Messages from Dash
******************************************************************************/

#include "dash.h"

/******************************************************************************
 * Messages from Motor Controller
******************************************************************************/

#include "mc.h"

/******************************************************************************
 * Messages from Current Sensor
******************************************************************************/

#include "current_sensor.h"

// @see each individual header file for all message types.
// TODO make a super dank macro that generates this from the type names
typedef enum {
  Can_FrontCanNode_DriverOutput_Msg,
  Can_FrontCanNode_RawValues_Msg,
  Can_FrontCanNode_WheelSpeed_Msg,
  Can_RearCanNode_WheelSpeed_Msg,

  Can_Vcu_BmsHeartbeat_Msg,
  Can_Vcu_DashHeartbeat_Msg,
  Can_Vcu_MCRequest_Msg,
  Can_Vcu_MCTorque_Msg,

  Can_Bms_Heartbeat_Msg,
  Can_Bms_CellTemps_Msg,
  Can_Bms_PackStatus_Msg,
  Can_Bms_Error_Msg,

  Can_Dash_Heartbeat_Msg,
  Can_Dash_Request_Msg,

  Can_MC_DataReading_Msg,
  Can_MC_ErrorAndWarning_Msg,
  Can_MC_State_Msg,

  Can_CurrentSensor_Current_Msg,
  Can_CurrentSensor_Voltage_Msg,
  Can_CurrentSensor_Power_Msg,
  Can_CurrentSensor_Energy_Msg,
} Can_Msg_T;

#endif // _MY17_CAN_LIBRARY_H
