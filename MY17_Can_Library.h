#ifndef _MY17_CAN_LIBRARY_H
#define _MY17_CAN_LIBRARY_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * Enum of all message types!
 *
 * Remember to change this enum if any new message types are added,
 * and to add a DECLARE and implementation for them.
******************************************************************************/

// @see each individual header file for all message types.
typedef enum {
  Can_No_Msg,
  Can_Unknown_Msg,

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
} Can_MsgID_T;

Can_MsgID_T Can_MsgType(void);

#include "can_raw.h"

#define TO_CAN(name) \
  void name ## _ToCan(name ## _T *type_in, Frame *can_out)

#define FROM_CAN(name) \
  void name ## _FromCan(Frame *can_in, name ## _T *type_out)

#define DECLARE(name) \
  bool name ##_Read(name ## _T *type); \
  void name ##_Write(name ## _T *type); \
  TO_CAN(name); \
  FROM_CAN(name);

//ex. Can_FrontCanNode_DriverOutput_Read(Can_FrontCanNode_DriverOutput_T *data);


/******************************************************************************
 * Messages from CAN Node
******************************************************************************/

#include "can_node.h"

DECLARE(Can_FrontCanNode_DriverOutput)
DECLARE(Can_FrontCanNode_RawValues)
DECLARE(Can_FrontCanNode_WheelSpeed)
DECLARE(Can_RearCanNode_WheelSpeed)

/******************************************************************************
 * Messages from VCU
******************************************************************************/

#include "vcu.h"

DECLARE(Can_Vcu_BmsHeartbeat);
DECLARE(Can_Vcu_DashHeartbeat);
DECLARE(Can_Vcu_MCRequest);
DECLARE(Can_Vcu_MCTorque);

/******************************************************************************
 * Messages from BMS
******************************************************************************/

#include "bms.h"

DECLARE(Can_Bms_Heartbeat);
DECLARE(Can_Bms_CellTemps);
DECLARE(Can_Bms_PackStatus);
DECLARE(Can_Bms_Error);

/******************************************************************************
 * Messages from Dash
******************************************************************************/

#include "dash.h"

DECLARE(Can_Dash_Heartbeat)
DECLARE(Can_Dash_Request)

/******************************************************************************
 * Messages from Motor Controller
******************************************************************************/

#include "mc.h"

DECLARE(Can_MC_DataReading)
DECLARE(Can_MC_ErrorAndWarning)
DECLARE(Can_MC_State)

/******************************************************************************
 * Messages from Current Sensor
******************************************************************************/

#include "current_sensor.h"

DECLARE(Can_CurrentSensor_Energy)
DECLARE(Can_CurrentSensor_Voltage)
DECLARE(Can_CurrentSensor_Power)
DECLARE(Can_CurrentSensor_Energy)

#undef DECLARE

#endif // _MY17_CAN_LIBRARY_H
