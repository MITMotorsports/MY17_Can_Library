#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint16_t id;
  uint8_t len;
  uint8_t data[8];
} Frame;

void Can_Init(uint32_t baudrate);
void Can_RawWrite(Frame *frame);
bool Can_RawRead(Frame *frame);


