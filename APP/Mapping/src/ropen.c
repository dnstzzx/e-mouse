#include "action.h"
#include "sensor.h"
#include "odometer.h"
#include "bsp_motor.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "bsp_vl53.h"
#include "map.h"
#include "bsp_uart.h"
#include "bsp_btn.h"

#define INTERACTIVE
#ifdef INTERACTIVE

//"pbprprpbplflprplflprps"
// f, l, r, b, s
static char cmds[256];
static size_t cmd_count = 0;
void input_cmd(){
    printf("input commands\n");
    cmd_count = 0;
    while(1){
        char c = getchar();
        if(c == 's'){
            break;
        }else{
           cmds[cmd_count] = c;
            cmd_count ++;
        }
        
        
    }

    printf("cmds are: ");
    for(int i=0;i<cmd_count;i++){
        printf("%c", cmds[i]);
    }
    printf("\n");
}
#else
// f, l, r, b, s
static char cmds[] = {'f',  's'};
static const size_t cmd_count = sizeof(cmds);
#endif

 void stop_motor(){
    bsp_motor_set_speed(BSP_MOTOR_L, 0.0f);
    bsp_motor_set_speed(BSP_MOTOR_R, 0.0f);
    osDelay(300);
    bsp_motor_set_ctrl_mode(BSP_MOTOR_L, BSP_MOTOR_CTRL_MODE_NONE);
    bsp_motor_set_ctrl_mode(BSP_MOTOR_R, BSP_MOTOR_CTRL_MODE_NONE);
    bsp_motor_set_pwm(BSP_MOTOR_L, 0.0f);
    bsp_motor_set_pwm(BSP_MOTOR_R, 0.0f);
    printf("stopped\n");
}



void ropen_main(){
    if(!sensor_init()){
        count_up_forever();
    }
    
    bsp_motor_init();
    action_switch_motor_mode(BSP_MOTOR_CTRL_MODE_SPEED);
    osDelay(50);
    update_sensor_data();
    odometer_init();
    #ifdef INTERACTIVE
        
    while(1){
        // if(!bsp_vl53_init()){
        //     count_up_forever();
        // }
        // bsp_vl53_start_continuous_all();
        input_cmd();

        
    #else
    printf("input anything to start\n");
    getchar();
    
    #endif
        for(int i=0;i<cmd_count;i++){
            char cmd = cmds[i];
            printf("exec cmd %d: %c\n\n", i, cmd);
            if(cmd == 'f'){
                float keeping_azimuth = action_pad();
                action_set_speed(0, 0);
                osDelay(200);
                action_keep_forward_cell(1);

                if(i+1 < cmd_count && cmds[i+1]== 'f')  // no need to stop
                    continue;
                //stop_motor();
            }else if(cmd == 's'){
                action_stop();
            }else if(cmd == 'l' || cmd == 'r' || cmd == 'b'){
                direction_t dir;
                switch(cmd){
                    case 'l': dir = DIRECTION_L; break;
                    case 'r': dir = DIRECTION_R; break;
                    case 'b': dir = DIRECTION_B; break;
                }
                action_calibrate_and_turn(dir);
                action_stop();
            }else if(cmd == 'p'){
                action_switch_motor_mode(BSP_MOTOR_CTRL_MODE_SPEED);
                float keeping_azimuth = action_pad();
                action_set_speed(0, 0);
                osDelay(200);
                update_sensor_data();
                action_keep_forward_reset(keeping_azimuth);
                do{
                    uint32_t st = HAL_GetTick();
                    action_keep_forward();
                    osDelay(25);
                    update_sensor_data();
                    //printf("time: %d\n", HAL_GetTick() - st);
                    //printf("odm: %d\n", odometer_getmm());
                } while(DIS_FRONT > 180);

            }else if(cmd == 'P'){
                action_switch_motor_mode(BSP_MOTOR_CTRL_MODE_SPEED);
                action_pad();
                action_stop();
            }else if(cmd == 'o'){
                float old_origin = origin_azimuth;
                action_make_origin();
                printf("old origin: %f, new origin: %f\n", old_origin, origin_azimuth);
            }else if(cmd == 'g'){
                start_build_map();
            }else if(cmd == 'w'){
                update_sensor_data();
                uint8_t walls_mask = 0;
                for(direction_t i=0;i<4;i++){
                    if(sensor_distances[i] < GRID_SIZE){
                        walls_mask |= (1 << i);
                    }
                }
                //walls_mask &= 0b1010;
                printf("walls mask: %d\n", walls_mask);
                osDelay(100);
                if(walls_mask !=0){
                    float old_origin = action_wall_pad_and_fix_origin(walls_mask);
                    printf("old origin: %f, new origin: %f\n", old_origin, origin_azimuth);
                }
            }else if(cmd == 'a'){
                printf("current azimuth: %f\n", get_current_azimuth());
            }else if(cmd == 'i'){
                uint32_t dtc = bsp_uart1_received_data_count();
                while(bsp_uart1_received_data_count() == dtc){
                    update_sensor_data();
                    printf("FRBL: %d, %d, %d, %d\n", DIS_FRONT_FILTERED, DIS_RIGHT_FILTERED, DIS_BACK_FILTERED, DIS_LEFT_FILTERED);
                    printf("azimuth: %f\n\n", get_current_azimuth());
                    osDelay(300);
                }
            }else if(cmd == 'G'){
                rush_map();
            }else if(cmd == 'B'){
                printf("waiting for button\n");
                bsp_btn_wait_until_pressed_and_released();
                osDelay(5000);
            }
            action_stop();
            osDelay(500);
        }
    #ifdef INTERACTIVE
    }
    #endif
    count_up_forever();
}