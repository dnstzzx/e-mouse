#pragma once

#include "vl53l0x.h"

typedef struct{
    float signal_rate_limit;
    uint8_t VcselPeriodPreRange, VcselPeriodFinalRange;
    uint32_t measurement_timing_budget_us;
} bsp_vl53_preset_t;

extern const bsp_vl53_preset_t BSP_VL53_PRESET_DEFAULT, 
                                BSP_VL53_PRESET_LONG_RANGE, 
                                BSP_VL53_PRESET_HIGH_SPEED, 
                                BSP_VL53_PRESET_HIGH_ACCURACY;


#define BSP_VL53_COUNT (4)
extern vl53l0x_dev_t bsp_vl53_devs[BSP_VL53_COUNT];

bool bsp_vl53_init();
void bsp_vl53_start_continuous_all();
bool bsp_vl53_read_all(uint16_t *rtn);
void bsp_vl53_config(vl53l0x_dev_t *dev,  bsp_vl53_preset_t preset);
