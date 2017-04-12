#include "evil_macros.h"

void data_transfer(DATA_T *in, DATA_T *out) {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    (*out).byte[7-i] = (*in).byte[i];
  }
}

void to_bitstring(uint8_t in[], uint64_t *out) {
  data_transfer((DATA_T*)in, (DATA_T*)out);
}

void from_bitstring(uint64_t *in, uint8_t out[]) {
  data_transfer((DATA_T*)in, (DATA_T*)out);
}


