#pragma once
#include "stdio.h"

static inline void print_hex(uint8_t *data, size_t len, const char *preffix){
    if(preffix != NULL){
        printf(preffix);
    }
    for(int i =0;i<len - 1;i++){
        printf("%02x ", data[i]);
    }
    printf("%02x\n", data[len - 1]);
}