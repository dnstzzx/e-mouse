#include "bsp_motor.h"
#include "tim.h"
#include "string.h"
#include "bsp_task.h"

#define MOTOR1_PWM1_TTM     (&htim8)
#define MOTOR1_PWM1_CHANNEL (TIM_CHANNEL_2)
#define MOTOR1_PWM2_TTM     (&htim8)
#define MOTOR1_PWM2_CHANNEL (TIM_CHANNEL_1)
#define MOTOR2_PWM1_TTM     (&htim8)
#define MOTOR2_PWM1_CHANNEL (TIM_CHANNEL_4)
#define MOTOR2_PWM2_TTM     (&htim8)
#define MOTOR2_PWM2_CHANNEL (TIM_CHANNEL_3)

//TIMER PERIOD = ARR + 1
#define MOTOR_PWM_PERIOD        (1125)
#define MOTOR1_ENCODER_TIM      (&htim3)
#define MOTOR2_ENCODER_TIM      (&htim4)
#define MOTOR_ENCODER_PERIOD    (65536)

bsp_motor_t bsp_motors[BSP_MOTOR_COUNT] = {
    {
        .target_speed = 0.0f,
        .target_pos = 0,
        .pwm1_channel = MOTOR1_PWM1_CHANNEL,
        .pwm2_channel = MOTOR1_PWM2_CHANNEL,
        .pwm1_tim = MOTOR1_PWM1_TTM,
        .pwm2_tim = MOTOR1_PWM2_TTM,
        .encoder_tim = MOTOR1_ENCODER_TIM,
        .encoder_state = {},
        .last_encoder_state = {},
        .current_speed = 0,
        .current_pos = 0,
        .ctrl_mode = BSP_MOTOR_CTRL_MODE_NONE,
        .speed_pid = {}
    },
    {
        .target_speed = 0.0f,
        .target_pos = 0,
        .pwm1_channel = MOTOR2_PWM1_CHANNEL,
        .pwm2_channel = MOTOR2_PWM2_CHANNEL,
        .pwm1_tim = MOTOR2_PWM1_TTM,
        .pwm2_tim = MOTOR2_PWM2_TTM,
        .encoder_tim = MOTOR2_ENCODER_TIM,
        .encoder_state = {},
        .last_encoder_state = {},
        .current_speed = 0,
        .current_pos = 0,
        .ctrl_mode = BSP_MOTOR_CTRL_MODE_NONE,
        .speed_pid = {}
    },
};

#define _for_each_dev(opr) \
    for(int _i=0;_i<BSP_MOTOR_COUNT;_i++){ \
        bsp_motor_t *dev = &bsp_motors[_i]; opr; \
    }

inline static uint32_t bsp_motor_read_encoder(bsp_motor_t *dev){
    return __HAL_TIM_GET_COUNTER(dev->encoder_tim);
}

void bsp_motor_set_pwm(bsp_motor_t *motor, float pwm){
    if(pwm >= 0){
        __HAL_TIM_SET_COMPARE(motor->pwm1_tim, motor->pwm1_channel, pwm * MOTOR_PWM_PERIOD);
        __HAL_TIM_SET_COMPARE(motor->pwm2_tim, motor->pwm2_channel, 0);
    }else{
        pwm = 0 - pwm;
        __HAL_TIM_SET_COMPARE(motor->pwm1_tim, motor->pwm1_channel, 0);
        __HAL_TIM_SET_COMPARE(motor->pwm2_tim, motor->pwm2_channel, pwm * MOTOR_PWM_PERIOD);
    }
}

#include "stdio.h"
/* returns: delta of counter */
static int32_t bsp_motor_update(bsp_motor_t *dev){
    __disable_irq();
    dev->encoder_state.encoder_count = bsp_motor_read_encoder(dev);
    int32_t delta = 
        MOTOR_ENCODER_PERIOD * (dev->encoder_state.encoder_ovf_count - dev->last_encoder_state.encoder_ovf_count) 
        + dev->encoder_state.encoder_count
        - dev->last_encoder_state.encoder_count;
    memcpy(&dev->last_encoder_state, &dev->encoder_state, sizeof(bsp_motor_encoder_state_t));
    dev->current_pos += delta;
    dev->current_speed = delta * BSP_MOTOR_TASK_HZ / 1000.0f;
    __enable_irq();
    return delta;
}

static inline void motor_ctrl(bsp_motor_t *motor){
    bsp_motor_update(motor);
    switch(motor->ctrl_mode){
        case BSP_MOTOR_CTRL_MODE_POS:
            motor->target_speed = pid_update(&motor->pos_pid, motor->target_pos, motor->current_pos);
        // no break here
        case BSP_MOTOR_CTRL_MODE_SPEED:
            do{
                float pwm = pid_update(&motor->speed_pid, motor->target_speed, motor->current_speed);
                bsp_motor_set_pwm(motor, pwm);
            }while(0);
            break;
    }

}

void bsp_motor_task(void *param){
    _for_each_dev(motor_ctrl(dev));
}

void bsp_motor_init(){
    _for_each_dev(
        dev->target_speed = 0;
        dev->ctrl_mode = BSP_MOTOR_CTRL_MODE_NONE;
        dev->current_speed = 0;
        dev->current_pos = 0;

        HAL_TIM_PWM_Start(dev->pwm1_tim, dev->pwm1_channel);
        HAL_TIM_PWM_Start(dev->pwm2_tim, dev->pwm2_channel);

        memset(&dev->encoder_state, 0, sizeof(bsp_motor_encoder_state_t));
        memset(&dev->last_encoder_state, 0, sizeof(bsp_motor_encoder_state_t));
        TIM_HandleTypeDef *encoder_tim = dev->encoder_tim;
        __HAL_TIM_SET_COUNTER(encoder_tim, 0);
        __HAL_TIM_CLEAR_IT(encoder_tim,TIM_IT_UPDATE);
        __HAL_TIM_ENABLE_IT(encoder_tim,TIM_IT_UPDATE);
        __HAL_TIM_URS_ENABLE(encoder_tim);
        HAL_TIM_Encoder_Start(dev->encoder_tim, TIM_CHANNEL_ALL);

        bsp_motor_set_pwm(dev, 0.0f);
        dn_pid_param_t p = {};
        bsp_motor_config_speed_pid(dev, &p);
        bsp_motor_config_pos_pid(dev, &p);
    )

    bsp_motor_t *dev;
    osTimerAttr_t attr = {
        .name = "motor task"
    };
    osTimerId_t timer_id = osTimerNew(bsp_motor_task, osTimerPeriodic, NULL, &attr);
    osTimerStart(timer_id, 1000 / BSP_MOTOR_TASK_HZ);
    
}

void HAL_TIM_PeriodElapsedCallback_motor(TIM_HandleTypeDef *htim){
    _for_each_dev(
        if(htim == dev->encoder_tim){
            dev->encoder_state.encoder_ovf_count ++;
        }
    )
}
