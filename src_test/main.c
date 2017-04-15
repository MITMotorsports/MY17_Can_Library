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
  Can_All_Tests(test_print);
}

