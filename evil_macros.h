#ifndef _MY17_CAN_LIBRARY_EVIL_MACROS_H
#define _MY17_CAN_LIBRARY_EVIL_MACROS_H

#include <stdint.h>
#include <stdbool.h>

#include "can_raw.h"

static bool unread;
static Frame lastMessage;

typedef union {
  uint8_t byte[8];
  uint64_t bitstring;
} DATA_T;

#define DEFINE(name) \
  bool name ##_Read(name ## _T *type) { \
    if (unread) { \
      name ## _FromCan(&lastMessage, type); \
      unread = false; \
      return true; \
    } else { \
      return false; \
    } \
  } \
  void name ##_Write(name ## _T *type) { \
    Frame frame; \
    name ## _ToCan(type, &frame); \
    Can_RawWrite(&frame); \
  }

#define TOGGLE(input, test, idx) \
  if (test) {\
    ((input) |= (1 << (7 - (idx)))); \
  } else { \
    ((input) &= ~(1 << (7 - (idx)))); \
  } \

#define CHECK(a,b) (((a) & (1<<(7- (b)))) != 0)

#define ONES(len) \
  ((1ULL << (len)) - 1)

#define START_IDX(start, len) \
  (64 - (start) - (len))

#define ZEROES_MASK(start, len) \
  (~(ONES(len) << START_IDX(start, len)))

#define INPUT_MASK(input, start, len) \
  (((input) & ONES(len)) << START_IDX(start, len))

#define INSERT(input, output, start, len) \
  (((output) & (ZEROES_MASK(start, len))) | INPUT_MASK(input, start, len))

#define EXTRACT(input, start, len) \
  (((input) >> START_IDX(start, len)) & ONES(len))

void data_transfer(DATA_T *in, DATA_T *out);
void to_bitstring(uint8_t in[], uint64_t *out);
void from_bitstring(uint64_t *in, uint8_t out[]);

#endif // _MY17_CAN_LIBRARY_EVIL_MACROS_H
