#include "../MY17_Can_Library.h"
#include "../MY17_Can_Library_Test.h"
#include "../evil_macros.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


void test_print(const char * data) {
  printf("%s", data);
}

int main(void) {
  test_print("\n*********TEST RESULTS **************\n\n");
  Can_FrontCanNode_RawValues_Test(test_print);
  Can_FrontCanNode_WheelSpeed_Test(test_print);
  Can_Vcu_BmsHeartbeat_Test(test_print);
  Can_Vcu_DashHeartbeat_Test(test_print);
  Can_Vcu_MCRequest_Test(test_print);
  Can_Bms_Heartbeat_Test(test_print);
  Can_Bms_CellTemps_Test(test_print);
  Can_Bms_PackStatus_Test(test_print);
  Can_Bms_Error_Test(test_print);
}

