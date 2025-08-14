#include "test_utils.h"

void printHex(const char* info, const uint8_t* data, size_t len) {
    printf("%s: ", info);
    for (size_t i = 0; i < len; ++i) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    fflush(stdout);
}

