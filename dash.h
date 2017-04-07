#ifndef _MY17_CAN_LIBRARY_DASH_H
#define _MY17_CAN_LIBRARY_DASH_H

#include "can_validator/fsae_can_spec.h"

#include <stdint.h>
#include <stdbool.h>

#include "ids.h"

typedef struct {
  bool ok;
} Can_Dash_Heartbeat_T;

typedef struct {
  Can_Dash_RequestID_T type;
} Can_Dash_Request_T;

#endif // _MY17_CAN_LIBRARY_DASH_H
