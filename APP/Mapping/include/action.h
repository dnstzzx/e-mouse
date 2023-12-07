#pragma once
#include "bsp_motor.h"
#include "direction.h"
#include "sensor.h"

// ------------- motor ---------------------

static inline void action_switch_motor_mode(bsp_motor_ctrl_mode_t mode){
    bsp_motor_set_ctrl_mode(BSP_MOTOR_L, mode);
    bsp_motor_set_ctrl_mode(BSP_MOTOR_R, mode);
}

static inline void action_set_speed(float l, float r){
    bsp_motor_set_speed(BSP_MOTOR_L, l);
    bsp_motor_set_speed(BSP_MOTOR_R, r);
}

#define do_with_motor_mode(mode, code) \
    do{ \
        bsp_motor_ctrl_mode_t origin_mode[2]; \
        for(int i = 0 ; i < 2; i++){ \
            origin_mode[i] = bsp_motors[i].ctrl_mode; \
            bsp_motor_set_ctrl_mode(&bsp_motors[i], mode); \
        } \
        do{code}while(0); \
        for(int i = 0 ; i < 2; i++){ \
            bsp_motor_set_ctrl_mode(&bsp_motors[i], origin_mode[i]); \
        } \
    }while(0);

void action_stop();

// ------------  keep  ---------------------
void action_keep_forward();
void action_keep_forward_reset(float keeping_azimuth);
void action_keep_forward_mm(uint16_t mm);

void action_keep_forward_cell(uint16_t cell_count);

// ------------  pad  ---------------------

// returns: azimuth before calibrate start, calibrated by new calibrator
float action_calibrate(bool stop);
void action_make_origin();
float action_wall_pad(uint8_t walls_mask);
float action_wall_pad_and_fix_origin(uint8_t walls_mark); // returns old origin azimuth

void action_pad_to(float target_azimuth);

static inline float action_pad(){
    float target = azimuth_from_direction(azimuth_to_direction(get_current_azimuth()));
    action_pad_to(target);
    return target;
}

static inline void action_pad_to_direction(direction_t direction){
    action_pad_to(azimuth_from_direction(direction));
}

// param direction: reletive direction, use action_pad_to_direction for abslote direction turn
void action_turn(direction_t direction);

static inline void action_calibrate_and_turn(direction_t direction){
    float start = action_calibrate(false);
    action_pad_to_direction(direction_relative_apply(azimuth_to_direction(start), direction));
}