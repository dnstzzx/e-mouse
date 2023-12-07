#pragma once

#include "bsp_i2c.h"
#include <stdbool.h>

typedef struct{
    bool enable_hard_calibrate;
    bool enable_soft_calibrate;
    int16_t center[3];
    float scale[3];
} bsp_5883_calibrator_t;

extern int16_t bsp_5883l_raw_data[3];
extern int16_t bsp_5883_calibrated_data[3];
extern int16_t bsp_5883l_filtered_data[3];


bool bsp_5883_init();
void bsp_5883_interrupt_callback();
void bsp_5883_start_calibrate();
void bsp_5883_stop_calibrate();
void bsp_5883_get_calibrator(bsp_5883_calibrator_t *rtn);
void bsp_5883_set_calibrator(bsp_5883_calibrator_t *new);
void bsp_5883_calibrate_data(const int16_t raw_data[3], int16_t output[3]);
float bsp_5883_get_yaw_degree();