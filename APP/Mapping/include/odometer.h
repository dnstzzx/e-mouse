#pragma once
#include <stdint.h>

void odometer_reset();
static inline void odometer_init(){ odometer_reset();}
void odometer_update();
int16_t odometer_getmm();
uint16_t measure_back_distance_in_cell();