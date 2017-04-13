#ifndef _MY17_CAN_LIBRARY_TEST_H
#define _MY17_CAN_LIBRARY_TEST_H

#include <stdint.h>

typedef void (*PRINT)(const char *);

void Can_BinaryPrint(PRINT print, uint64_t out, uint8_t len);
void Can_FrontCanNode_DriverOutput_Test(PRINT print);
void Can_FrontCanNode_RawValues_Test(PRINT print);
void Can_FrontCanNode_WheelSpeed_Test(PRINT print);
void Can_Vcu_BmsHeartbeat_Test(PRINT print);
void Can_Vcu_DashHeartbeat_Test(PRINT print);
void Can_Vcu_MCRequest_Test(PRINT print);
void Can_Bms_Heartbeat_Test(PRINT print);
void Can_Bms_CellTemps_Test(PRINT print);
void Can_Bms_PackStatus_Test(PRINT print);
void Can_Bms_Error_Test(PRINT print);

#endif // _MY17_CAN_LIBRARY_TEST_H
