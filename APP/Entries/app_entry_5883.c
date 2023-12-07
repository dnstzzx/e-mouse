#include "bsp_5883.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "bsp_motor.h"
#include "action.h"
#include "direction.h"


static void calibrate_and_pad_test(){
    static bsp_5883_calibrator_t calibrator;
    bsp_motor_init();
    osDelay(10);
    while(1){
        char c = getchar();
        if(c == 'c'){
            int16_t origin_data[3];
            memcpy(origin_data, bsp_5883l_raw_data, sizeof(origin_data));
            bsp_5883_start_calibrate();
            action_switch_motor_mode(BSP_MOTOR_CTRL_MODE_SPEED);
            action_set_speed(7, -7);
            osDelay(1300);
            bsp_5883_stop_calibrate(&calibrator);
            action_set_speed(0, 0);
            printf("calibrate finished\n");
            
            int16_t calibrated_origin[3];
            bsp_5883_set_calibrator(&calibrator);
            bsp_5883_calibrate_data(origin_data, calibrated_origin);
            float heading = azimuth_from_5883_data(calibrated_origin);
            origin_azimuth = heading;
            printf("padding to origin yaw: %f\n", origin_azimuth);
            action_pad_to(origin_azimuth);
            action_set_speed(0,0);
            printf("finished\n");
        }else if(c == 'p'){
            action_pad_to(origin_azimuth);
            action_set_speed(0,0);
        }else if(c == 'r'){
            for(int i=1;i<=4;i++){
                osDelay(5000);
                
                action_pad_to(fmodf(origin_azimuth + 90 * i, 360.0f));
                action_set_speed(0, 0);
            }
        }else if(c=='P'){
            printf("calibrate center: %d, %d, %d\n", calibrator.center[0], calibrator.center[1], calibrator.center[2]);
        }
    }
    
}


void app_idle_task(){
    while(!bsp_5883_init()){
        printf("init failed\n");
        osDelay(1000);
    }

    //calibrate_test();
    calibrate_and_pad_test();
    
}