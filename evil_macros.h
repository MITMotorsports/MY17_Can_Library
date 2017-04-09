#ifndef _MY17_CAN_LIBRARY_EVIL_MACROS_H
#define _MY17_CAN_LIBRARY_EVIL_MACROS_H

#include <stdint.h>
#include <stdbool.h>

#include "MY17_Can_Library.h"

static bool unread;
static Frame lastMessage;

#define TO_CAN(name) \
  void name ## _ToCan(name ## _T *type_in, Frame *can_out)

#define FROM_CAN(name) \
  void name ## _FromCan(name ## _T *type_out, Frame *can_in)

#define DEFINE(name) \
  void name ##_Write(name ## _T *type) { \
    Frame frame; \
    name ## _ToCan(type, &frame); \
    Can_RawWrite(&frame); \
  } \
  bool name ##_Read(name ## _T *type) { \
    if (unread) { \
      name ## _FromCan(type, &lastMessage); \
      unread = false; \
      return true; \
    } else { \
      return false; \
    } \
  }

#define TOGGLE(input, test, idx) \
  if (test) {\
    ((input) |= (1 << (7 - (idx)))); \
  } else { \
    ((input) &= ~(1 << (7 - (idx)))); \
  } \

#define CHECK(a,b) (((a) & (1<<(7- (b)))) != 0)

#endif // _MY17_CAN_LIBRARY_EVIL_MACROS_H
