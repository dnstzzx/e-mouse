#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"

#include "bsp_motor.h"
#include "bsp_5883.h"
#include "bsp_vl53.h"

#include "basic_algs.h"
#include "pid.h"
#include "filters.h"

#include "sensor.h"
#include "direction.h"
#include "action.h"
#include "map.h"

const float speed_rate = 4;
const float pad_speed_rate = 2;

#define KEEP_FORWARD_SIDE_DIS_LEFT 190
#define KEEP_FORWARD_SIDE_DIS_RIGHT 200
#define PAD_THRESHOLD 0.5
#define PAD_CHECK_COUNT 5

#define WALL_PAD_RANGE 20

// 直行时用于位置修正的pid
static const dn_pid_param_t keep_forward_bias_pid_param = {
    .kp = 0.002f,
    .ki = 0.0003f,
    .kd = 0.0003f,
    .err_limit = 125,
    .integral_limit = 1,
    .out_limit = 0.5f
};

// 直行时用于姿态修正的pid
static const dn_pid_param_t keep_forward_fix_pid_param = {
    .kp = 0.025f,
    .ki = 0.0f,
    .kd = 0.0f,
    .err_limit = 90,
    .integral_limit = 1,
    .out_limit = 0.5f
};

// 转弯pid
static const dn_pid_param_t pad_param = {
    .kp = 0.125,
    .ki = 0.025,
    .kd = 0.02,
    .err_limit = 50,
    .out_limit = 3.5,
    .integral_limit = 0
};


static dn_pid_t keep_forward_bias_pid;  // 直行时用于位置修正的pid
static dn_pid_t keep_forward_fix_pid;   // 直行时用于姿态修正的pid
static float keep_forward_keeping_azimuth;  // 直行时姿态修正的目标角度

// 每段直行前调用
void action_keep_forward_reset(float keeping_azimuth){
    pid_init(&keep_forward_bias_pid, &keep_forward_bias_pid_param);
    pid_init(&keep_forward_fix_pid, &keep_forward_fix_pid_param);
    keep_forward_keeping_azimuth = keeping_azimuth;
}

// 直行一帧，非阻塞
#define DEBUG_KEEP_FORWARD 0
void action_keep_forward(){
    int delta = 0;
    float spdl=0, spdr=0;

    if(DIS_FRONT < 50){
        action_stop();
        printf("keep forward failed: met barrier\n");
        while(1) osDelay(1);
    }

    if(!sensor_data_valid(DIS_LEFT) && !sensor_data_valid(DIS_RIGHT)){
        printf("keep forward bias failed\n");
    }else if(sensor_data_valid(DIS_LEFT) && DIS_LEFT_FILTERED <= DIS_RIGHT_FILTERED){
        delta = KEEP_FORWARD_SIDE_DIS_LEFT - (DIS_LEFT_FILTERED % GRID_SIZE);
    }else{
        delta = (DIS_RIGHT_FILTERED % GRID_SIZE) - KEEP_FORWARD_SIDE_DIS_RIGHT;
    }

    float fix_delta =  azimuth_delta(keep_forward_keeping_azimuth, get_current_azimuth());
    float bias_fdbk = pid_update(&keep_forward_bias_pid, 0.0f, (float)delta);
    float fix_fdbk = pid_update(&keep_forward_fix_pid, 0, fix_delta);


    spdl = 1.0f - bias_fdbk + fix_fdbk;
    spdr = 1.0 + bias_fdbk - fix_fdbk;
    spdl *= speed_rate;
    spdr *= speed_rate;
    
    action_set_speed(spdl, spdr);
    
    if(DEBUG_KEEP_FORWARD){
        //printf("\tleft:%d,  right:%d\n", DIS_LEFT, DIS_RIGHT);
        printf("delta: %d, bias_fdbk: %f\n",  delta, bias_fdbk);
        printf("fix: %f,%f\n",  fix_delta, fix_fdbk);
    }
}

void action_stop(){
    bsp_motor_set_speed(BSP_MOTOR_L, 0);
    bsp_motor_set_speed(BSP_MOTOR_R, 0);  
}

#define DEBUG_PAD_TO 0
void action_pad_to(float target_azimuth){
    static dn_pid_t pad_pid;
    pid_init(&pad_pid, &pad_param);
    uint8_t ok_count = 0;
    while(ok_count < PAD_CHECK_COUNT){
        if(fabsf(azimuth_delta(get_current_azimuth(), target_azimuth)) <= PAD_THRESHOLD) {
            ok_count++;
        } else {
            ok_count = 0;
        }
        float d = pid_update(&pad_pid, 0, azimuth_delta(target_azimuth, get_current_azimuth()));
        d *= pad_speed_rate;
        if(DEBUG_PAD_TO) printf("pad to: %f, current: %f, delta: %f, d: %f\n", target_azimuth, get_current_azimuth(), azimuth_delta(target_azimuth, get_current_azimuth()), d);
        action_set_speed(d, -d);
        osDelay(20);
    }
    action_stop();
}

#define DEBUG_WALL_PAD 0
float action_wall_pad(uint8_t walls_mask){
    // for(direction_t d=0;d<4;d++){
    //     if(walls_mask >> d & 1 == 0){
    //         VL53L0X_stopContinuous(&bsp_vl53_devs[sensor_of_direction[d]]);
    //     }
    // }

    float origin_az = get_current_azimuth();
    float range = WALL_PAD_RANGE;
    float points[3] = {origin_az - range, origin_az + range , origin_az};

    uint32_t min_sum = 99999;
    float min_sum_az = 0;
    float seg_start_az = origin_az;
    update_sensor_data_full();
    vector_t *az_history = vector_new(float);
    for(int i=0;i<3;i++){
        points[i] = fmodf(points[i], 360.0f);
        float speed = 0.7;
        bool turn_right = azimuth_delta(seg_start_az ,points[i]) > 0;
        if(turn_right) speed *= -1;
        vector_clear(az_history);

        if(DEBUG_WALL_PAD) printf("\n\nturn %s\n", turn_right ? "right" : "left");
        if(DEBUG_WALL_PAD) printf("\nstart az: %f, target az: %f\n\n", seg_start_az, points[i]);
        action_set_speed(-speed, speed);

        uint32_t last_t = HAL_GetTick();
        float delta;
        do{
            update_sensor_data();
            float current_az = get_current_azimuth();
            vector_push_back(az_history, current_az);
            if(az_history->size > 2)
                _vector_pop_front(az_history, NULL);
            uint32_t sum = 0;
            for(int d=0;d<4;d++){
                uint16_t dis = filtered_sensor_distances[d];
                if(walls_mask >> d & 1){
                    if(!sensor_data_valid(dis)){
                        printf("wall pad failed: sensor data invalid\n");
                        // action_stop();
                        // while(1) osDelay(1);
                    }
                    sum +=  dis;
                }
            }
            if(sum < min_sum){
                min_sum = sum;
                min_sum_az = vector_front(az_history, float);
            }
            osDelayUntil(last_t + 33);
            last_t = HAL_GetTick();
            delta = azimuth_delta(get_current_azimuth(), points[i]);
            if(DEBUG_WALL_PAD) printf("az: %f, delta: %f, sum: %d\n", get_current_azimuth(), delta, sum);
        }while(turn_right ^ (delta < 0) );
        seg_start_az = points[i];
        action_stop();
    }
    vector_delete(az_history);
    // for(direction_t d=0;d<4;d++){
    //     if((walls_mask >> d & 1) == 0){
    //         VL53L0X_startContinuous(&bsp_vl53_devs[sensor_of_direction[d]], 0);
    //     }
    // }
    // osDelay(33);
    return min_sum_az;
}

float action_wall_pad_and_fix_origin(uint8_t walls_mark){
    float origin_p = get_padding_azimuth(), old_origin = origin_azimuth;
    float newp = action_wall_pad(walls_mark);
    action_pad_to(newp);
    origin_azimuth = fmodf(origin_azimuth + (newp - origin_p) + 720.0f, 360.0f);
    osDelay(300);
    return old_origin;
}


#define DEBUG_CALIBRATE 0
float action_calibrate(bool stop){
    int16_t origin_data[3];
    int16_t calibrated_origin[3];

    memcpy(origin_data, bsp_5883l_raw_data, sizeof(origin_data));
    bsp_5883_start_calibrate();
    action_set_speed(7, -7);
    osDelay(1300);
    bsp_5883_stop_calibrate();
    if(DEBUG_CALIBRATE) printf("calibrate finished\n");
    bsp_5883_calibrator_t calibrator;
    bsp_5883_get_calibrator(&calibrator);
    if(DEBUG_CALIBRATE) printf("calibrate center: %d, %d, %d\n", calibrator.center[0], calibrator.center[1], calibrator.center[2]);
    if(DEBUG_CALIBRATE && calibrator.enable_soft_calibrate) printf("calibrate scale: %f, %f, %f\n", calibrator.scale[0], calibrator.scale[1], calibrator.scale[2]);
    bsp_5883_calibrate_data(origin_data, calibrated_origin);
    if(stop)
        action_set_speed(0,0);
    return azimuth_from_5883_data(calibrated_origin);
}

void action_make_origin(){
    action_switch_motor_mode(BSP_MOTOR_CTRL_MODE_SPEED);
    origin_azimuth = action_calibrate(false);
    action_pad_to_direction(DIRECTION_F);
    action_set_speed(0, 0);
}

void action_turn(direction_t direction){
    float target = azimuth_from_direction(direction_relative_apply(get_padding_direction(), direction));
    action_pad_to(target);
}

#define DEBUG_KEEP_FORWARD_MM 0
void action_keep_forward_mm(uint16_t mm){
    update_sensor_data_full();
    update_sensor_data();
    float keeping_azimuth = get_padding_azimuth();
    odometer_reset();
    uint16_t stop = mm;
    action_keep_forward_reset(keeping_azimuth);
    do{
        action_keep_forward();
        osDelay(25);
        update_sensor_data();
        odometer_update();
        if(DEBUG_KEEP_FORWARD_MM)  printf("odm: %d\n", odometer_getmm());
    } while(odometer_getmm() < stop);
}

#define DEBUG_KEEP_FORWARD_CELL 0
void action_keep_forward_cell(uint16_t cell_count){
    update_sensor_data_full();
    float keeping_azimuth = get_padding_azimuth();
    odometer_reset();
    uint16_t stop = cell_count * GRID_SIZE - (measure_back_distance_in_cell() - (GRID_SIZE / 2));
    if(DEBUG_KEEP_FORWARD_CELL) printf("stop: %d\n", stop);
    action_keep_forward_reset(keeping_azimuth);
    do{
        action_keep_forward();
        osDelay(25);
        update_sensor_data();
        odometer_update();
        if(DEBUG_KEEP_FORWARD_CELL) printf("odm: %d\n", odometer_getmm());
    } while(odometer_getmm() < stop && DIS_FRONT > 140);
}


// void calibrate_5883_and_pad(){
//     bsp_motor_init();
//     osDelay(10);
//     while(1){
//         char c = getchar();
//         if(c == 'c'){
//             int16_t origin_data[3];
//             memcpy(origin_data, bsp_5883l_raw_data, sizeof(origin_data));
//             reset_calibration();
//             printf("start\n");
//             in_grogress = true;
//             first = true;
//             action_switch_motor_mode(BSP_MOTOR_CTRL_MODE_SPEED);
//             action_set_speed(5, -5);
//             while(getchar() != 's'){
//                 osDelay(50);
//             }

//             in_grogress = false;
//             action_set_speed(0, 0);
//             printf("calibrate finished\n");
//             print_calibrate_info();

//             bsp_5883_calibrator_t calibrator;
//             generate_calibrator(&calibrator);
//             int16_t calibrated_origin[3];
//             bsp_5883_set_calibrator(&calibrator);
//             bsp_5883_calibrate_data(origin_data, calibrated_origin);
//             float heading = atan2f(calibrated_origin[1], calibrated_origin[0]);
//             heading *= 180 / PI;
//             if(heading < 0)
//                 heading += 360;
//             origin_azimuth = heading;
//             printf("padding to origin yaw: %f\n", origin_azimuth);
//             // while(1){
//             //     float current = get_current_azimuth();
//             //     float delta = fmodf((current - origin_azimuth + 180), 360.0f ) - 180;
//             //     printf("origin: %f, current: %f, delta: %f\n", origin_azimuth, current, delta);
//             //     osDelay(50);
//             // }
//             action_pad_to(origin_azimuth);
//             action_set_speed(0,0);
//             printf("finished\n");
//         }
//     }
    
// }