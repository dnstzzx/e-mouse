#pragma once

#include "vl53l0x.h"

#define BSP_VL53_COUNT (8)
extern vl53l0x_dev_t bsp_vl53_devs[BSP_VL53_COUNT];

bool bsp_vl53_init();
void bsp_vl53_start_continuous_all();
void bsp_vl53_read_all(uint16_t *rtn);