#include <stdint.h>

typedef struct {
  uint16_t id;
  uint8_t len;
  uint8_t data[8];
} Frame;

void Can_Raw_Write(Frame *frame);
void Can_Raw_Read(Frame *frame);

#define TO_CAN(name) \
  void name ## _ToCan(name ## _T *type_in, Frame *can_out)

#define FROM_CAN(name) \
  void name ## _FromCan(name ## _T *type_out, Frame *can_in)

#define DEFINE(name) \
  void name ##_Write(name ## _T *type) { \
    Frame frame; \
    name ## _ToCan(type, &frame); \
    Can_Raw_Write(&frame); \
  } \
  void name ##_Read(name ## _T *type) { \
    Frame frame; \
    Can_Raw_Read(&frame); \
    name ## _FromCan(type, &frame); \
  }

