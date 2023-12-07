#include "bsp_motor.h"
#include "bsp_uart.h"
#include "bsp_task.h"
#include "cmsis_os.h"
#include "math.h"
void report_task(void *param){
    while(1){
        
        printf("%d %d\n", BSP_MOTOR_L->current_pos, BSP_MOTOR_R->current_pos);
        osDelay(50);
    }
}

static dn_pid_param_t speed_pid = {
    .kp = 0.3f,
    .ki = 0.05f,
    .kd = 0.15f,
    .err_limit = 10,
    .integral_limit = 1,
    .out_limit = 0.8
};

static dn_pid_param_t pos_pid = {
    .kp = 8.0f,
    .ki = 0.002f,
    .kd = 0.2f,
    .err_limit = 2,
    .integral_limit = 1,
    .out_limit = 5
};

static void print_pid(void *cbk_param, float target, float measurement, float output){
    printf("%f,%f,%f\n", measurement, target - measurement, output);
}


static void speed_pid_test(){
    bsp_motor_t *motor = BSP_MOTOR_L;
    bsp_motor_init();
    rcfg:
    bsp_motor_config_speed_pid(motor, &speed_pid);
    bsp_motor_set_ctrl_mode(motor, BSP_MOTOR_CTRL_MODE_SPEED);
    motor->speed_pid.tracer_cbk_param = NULL;
    motor->speed_pid.tracer_callback = print_pid;
    motor->target_speed = 0;

    while(1){
        //printf("input new pwm\n");
        float s1;
        //scanf("%f %f", &s1, &s2);   
        char c;
        scanf("%c", &c);
        switch(c){
            case 'p':
                scanf("%f", &s1);
                speed_pid.kp = s1;
                bsp_motor_set_speed(motor, 0.0f);
                goto rcfg;
            case 'i':
                scanf("%f", &s1);
                speed_pid.ki = s1;
                bsp_motor_set_speed(motor, 0.0f);
                goto rcfg;
            case 'd':
                scanf("%f", &s1);
                speed_pid.kd = s1;
                bsp_motor_set_speed(motor, 0.0f);
                goto rcfg;
            case 'P':
                printf("pid is: %f, %f, %f\n", speed_pid.kp, speed_pid.ki, speed_pid.kd);
                break;
            case 's':
                scanf("%f", &s1);
                bsp_motor_set_speed(motor, s1);
                osDelay(50);
                break;
            case 'l':
                bsp_motor_set_speed(motor, 0.0f);
                bsp_motor_set_ctrl_mode(motor, BSP_MOTOR_CTRL_MODE_NONE);
                motor->speed_pid.tracer_callback = NULL;
                motor = BSP_MOTOR_L;
                goto rcfg;
                break;
            case 'r':
                bsp_motor_set_speed(motor, 0.0f);
                bsp_motor_set_ctrl_mode(motor, BSP_MOTOR_CTRL_MODE_NONE);
                motor->speed_pid.tracer_callback = NULL;
                motor = BSP_MOTOR_R;
                goto rcfg;
                break;
            case 'R':
                if(motor->speed_pid.tracer_callback == NULL){
                    motor->speed_pid.tracer_callback = print_pid;
                }else{
                    motor->speed_pid.tracer_callback = NULL;
                }
                break;
        }
        
    }
    count_up_forever();
}


static void pwm_test(){
    
    bsp_motor_init();
    //bsp_task_create("spd rpt", report_task, NULL, osPriorityAboveNormal1, 256);

    while(1){
        //printf("input new pwm\n");
        float s1, s2;
        //printf("input\n");
        scanf("%f %f", &s1, &s2);   
        //printf("set pwms to %f\n", pwm);
        bsp_motor_set_pwm(BSP_MOTOR_L, s1);
        bsp_motor_set_pwm(BSP_MOTOR_R, s2);
        if(s1 == 0.0f && s2 == 0.0f){
            BSP_MOTOR_L->current_pos = 0;
            BSP_MOTOR_R->current_pos = 0;
        }
        osDelay(50);
    }
    count_up_forever();
}

static void pos_pid_test(){
    bsp_motor_init();
    bsp_motor_config_speed_pid(BSP_MOTOR_L, &speed_pid);
    bsp_motor_config_pos_pid(BSP_MOTOR_L, &pos_pid);
    bsp_motor_set_ctrl_mode(BSP_MOTOR_L, BSP_MOTOR_CTRL_MODE_POS);
    bsp_motor_config_speed_pid(BSP_MOTOR_R, &speed_pid);
    bsp_motor_config_pos_pid(BSP_MOTOR_R, &pos_pid);
    bsp_motor_set_ctrl_mode(BSP_MOTOR_R, BSP_MOTOR_CTRL_MODE_POS);
    BSP_MOTOR_L->pos_pid.tracer_cbk_param = NULL;
    BSP_MOTOR_L->pos_pid.tracer_callback = print_pid;
    BSP_MOTOR_R->pos_pid.tracer_cbk_param = NULL;
    BSP_MOTOR_R->pos_pid.tracer_callback = print_pid;

    while(1){
        //printf("input new pwm\n");
        float s1, s2;
        static float last1 = 0.0f, last2 = 0.0f;
        scanf("%f %f", &s1, &s2);
        s1 += last1;
        s2 += last2;
    
        //printf("set pwms to %f\n", pwm);
        bsp_motor_set_pos(BSP_MOTOR_L, s1);
        bsp_motor_set_pos(BSP_MOTOR_R, s2);
        last1 = s1;last2 = s2;
        //bsp_motor_set_pwm(BSP_MOTOR_R, pwm);
        osDelay(50);
    }
    count_up_forever();
}

#define LPOS ( BSP_MOTOR_L->current_pos)
#define RPOS ( BSP_MOTOR_R->current_pos)
#define POS_D (LPOS - RPOS)
// 转90编码器理论变化值 待测
#define TURN_90_ENCODER_VAL (1976)
static move_test(){
    bsp_motor_init();
    bsp_motor_set_ctrl_mode(BSP_MOTOR_L, BSP_MOTOR_CTRL_MODE_SPEED);
    bsp_motor_set_ctrl_mode(BSP_MOTOR_R, BSP_MOTOR_CTRL_MODE_SPEED);
    bsp_task_create("spd rpt", report_task, NULL, osPriorityAboveNormal1, 256);
    while(1){
        char c;
        printf("input new command\n");
        scanf("%c", &c);
        
        int32_t pos;
        float spdl, spdr;
        switch(c){
            case 'L':
            case 'l':
                pos = -2 * TURN_90_ENCODER_VAL;
                spdl = -1;
                spdr = 1;
                break;
            case 'R':
            case 'r':
                pos = 2 * TURN_90_ENCODER_VAL;
                spdl = 1;
                spdr = -1;
                break;
            case 's':
            case 'S':
                bsp_motor_set_speed(BSP_MOTOR_L, 0);
                bsp_motor_set_speed(BSP_MOTOR_R, 0);
                BSP_MOTOR_L->current_pos = 0;
                BSP_MOTOR_R->current_pos = 0;
                continue;
                break;
            case 'f':
                bsp_motor_set_speed(BSP_MOTOR_L, 5);
                bsp_motor_set_speed(BSP_MOTOR_R, 5);
                continue;
                break;
            case 'b':
                bsp_motor_set_speed(BSP_MOTOR_L, -5);
                bsp_motor_set_speed(BSP_MOTOR_R, -5);
                continue;
                break;
            default:
                continue;
        }
        spdl *= 5;
        spdr *= 5;
        BSP_MOTOR_L->current_pos = 0;
        BSP_MOTOR_R->current_pos = 0;
        bsp_motor_set_speed(BSP_MOTOR_L, spdl);
        bsp_motor_set_speed(BSP_MOTOR_R, spdr);
        while(1){
            if(abs(POS_D - pos) <= 2000){
                spdl = spdl > 0 ? 2 : -2;
                spdr = spdr > 0 ? 2 : -2;
                bsp_motor_set_speed(BSP_MOTOR_L, spdl);
                bsp_motor_set_speed(BSP_MOTOR_R, spdr);
            }
            if(abs(POS_D) >= abs(pos)){
                bsp_motor_set_speed(BSP_MOTOR_L, 0);
                bsp_motor_set_speed(BSP_MOTOR_R, 0);
                break;
            }
            osDelay(2);
        }

    }
}


void app_idle_task(){
    //bsp_motor_init();
    speed_pid_test();
    //move_test();
}