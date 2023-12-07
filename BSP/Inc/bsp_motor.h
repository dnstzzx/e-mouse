#pragma once
#include "stdint.h"
#include "stddef.h"
#include "tim.h"
#include "pid.h"
#include "stdbool.h"

#define BSP_MOTOR_COUNT     (2)
#define BSP_MOTOR_RATE      (100 * 4 * 7)
#define BSP_MOTOR_TASK_HZ   (50)

typedef struct{
    uint32_t encoder_count;
    volatile int32_t encoder_ovf_count; // overflow count
} bsp_motor_encoder_state_t;

typedef enum{
    BSP_MOTOR_CTRL_MODE_NONE,
    BSP_MOTOR_CTRL_MODE_SPEED,
    BSP_MOTOR_CTRL_MODE_POS
} bsp_motor_ctrl_mode_t;

typedef struct{
    bool                        inverse;
    float                       target_speed;
    float                       target_pos;

    const uint8_t               pwm1_channel;
    const uint8_t               pwm2_channel;
    const TIM_HandleTypeDef*    pwm1_tim;
    const TIM_HandleTypeDef*    pwm2_tim;

    TIM_HandleTypeDef*          encoder_tim;
    bsp_motor_encoder_state_t   encoder_state;
    bsp_motor_encoder_state_t   last_encoder_state;

    float              current_speed;  // 编码器计数个数每ms,反转负数
    int32_t            current_pos;    // 开机以来编码器计数算术和(反转减小)

    bsp_motor_ctrl_mode_t       ctrl_mode;
    dn_pid_t                       speed_pid;
    dn_pid_t                       pos_pid;
} bsp_motor_t;

extern bsp_motor_t bsp_motors[BSP_MOTOR_COUNT];
#define BSP_MOTOR_L (&bsp_motors[0])
#define BSP_MOTOR_R (&bsp_motors[1])

/* 会自动调用，不要手动干预 */
//int32_t bsp_motor_update(bsp_motor_t *dev);

void bsp_motor_init();
void bsp_motor_set_pwm(bsp_motor_t *motor, float pwm);

static inline float bsp_motor_get_speed(bsp_motor_t *dev){
    return dev->current_speed;
}

static inline int32_t bsp_motor_get_pos(bsp_motor_t *dev){
    return dev->current_pos;
}

static inline void bsp_motor_reset_pos(bsp_motor_t *dev){
    dev->current_pos = 0;
}

static inline void bsp_motor_set_speed(bsp_motor_t *motor, float speed){
    motor->target_speed = speed;
}

static inline void bsp_motor_set_ctrl_mode(bsp_motor_t *motor, bsp_motor_ctrl_mode_t mode){
    motor->ctrl_mode = mode;
}

static inline void bsp_motor_set_pos(bsp_motor_t *motor, float pos){
    motor->target_pos = pos;
}

static inline void bsp_motor_config_speed_pid(bsp_motor_t *dev, dn_pid_param_t *param){
    pid_init(&dev->speed_pid, param);
}

static inline void bsp_motor_config_pos_pid(bsp_motor_t *dev, dn_pid_param_t *param){
    pid_init(&dev->pos_pid, param);
}