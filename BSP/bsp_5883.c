#include <math.h>
#include "bsp_5883.h"
#include "hmc5883l.h"
#include "bsp_task.h"
#include "cmsis_os.h"
#include "filters.h"
#include <stdio.h>
#include <string.h>

#define PI 3.1415926f

static bsp_5883_calibrator_t calibrator = {
    .enable_hard_calibrate = false,
    .enable_soft_calibrate = false
};

#define MEAN_COUNT 3
int32_t mean_buffer[3][MEAN_COUNT];
static i32_sliding_mean_filter_t filters[3];

int16_t bsp_5883l_raw_data[3];
int16_t bsp_5883_calibrated_data[3];
int16_t bsp_5883l_filtered_data[3];


static bool inited = false;
static float yaw = 0.0;
static osThreadId_t taskid;

void bsp_5883_updt_callback(const int16_t data[3]);

void bsp_5883_calibrate_data(const int16_t raw_data[3], int16_t output[3]){
    for(int i=0;i<3;i++){
        output[i] = raw_data[i];
        if(calibrator.enable_hard_calibrate){
            output[i] -= calibrator.center[i];
        }
        if(calibrator.enable_soft_calibrate){
            output[i] *= calibrator.scale[i];
        }
    }
}

#define DEBUG_5883_TASK 0
static void bsp_5883_task(void *param){
    static int16_t *data = bsp_5883l_raw_data;
    while(1){
        osThreadFlagsWait(1 << 10, osFlagsWaitAny, osWaitForever);
        
        HMC5883L_getHeading(&data[0], &data[1], &data[2]);
        bsp_5883_calibrate_data(data, bsp_5883_calibrated_data);
        
        for(int i=0;i<3;i++){
            i32_sliding_mean_filter_input(&filters[i], bsp_5883_calibrated_data[i]);
            bsp_5883l_filtered_data[i] = filters[i].output;
        }


        
        float heading = atan2f(filters[1].output, filters[0].output);
        
        heading *= 180 / PI;
        if(heading < 0)
            heading += 360;

        if(DEBUG_5883_TASK) printf("heading: %f, raw: %d,%d\n", heading, data[0], data[1]);

        // float heading_curr = atan2f(data[1], data[0]);
        // heading_curr *= 180 / PI;
        // if(heading_curr < 0)
        //     heading_curr += 360;
        // printf("yaw: %f,%f\n", heading_curr, heading);
        yaw = heading;
        bsp_5883_updt_callback(data);
    }
}

bool bsp_5883_init(){
    if(!HMC5883L_testConnection()){
        return false;
    }

    calibrator.enable_hard_calibrate = false;
    calibrator.enable_soft_calibrate = false;
    for(int i=0;i<3;i++){
        i32_sliding_mean_filter_init(&filters[i], mean_buffer[i], MEAN_COUNT);
    }
    
    HMC5883L_initialize();
    HMC5883L_setSampleAveraging(HMC5883L_AVERAGING_8);
    HMC5883L_setDataRate(HMC5883L_RATE_75);
    HMC5883L_setMode(HMC5883L_MODE_CONTINUOUS);
    HMC5883L_setGain(HMC5883L_GAIN_1090);
    taskid = bsp_task_create("5883", bsp_5883_task, NULL, osPriorityBelowNormal4, 512);
    inited = true;
    osDelay(7 * MEAN_COUNT);
    return true;
}

void bsp_5883_interrupt_callback(){
    if(inited){
        osThreadFlagsSet(taskid, 1 << 10);
    }
}


float bsp_5883_get_yaw_degree(){
    return yaw;
}

static bool calibrating = false;
static bool calibrate_first = true;
static int16_t min_data[3], max_data[3];

void bsp_5883_updt_callback(const int16_t data[3]){
    if(!calibrating) return;
    if(calibrate_first){
        min_data[0] = max_data[0] = data[0];
        min_data[1] = max_data[1] = data[1];
        min_data[2] = max_data[2] = data[2];
        calibrate_first = false;
    }
    for(int i=0;i<3;i++){
        if(data[i] > max_data[i])
            max_data[i] = data[i];
        if(data[i] < min_data[i])
            min_data[i] = data[i];
    }
    //printf("%d,%d\n", data[0], data[1]);
}

void bsp_5883_start_calibrate(){
    memset(min_data, 0, sizeof(min_data));
    memset(max_data, 0, sizeof(max_data));
    calibrating = true;
    calibrate_first = true;
}

void bsp_5883_stop_calibrate(){
    calibrator.enable_hard_calibrate = true;
    calibrator.enable_soft_calibrate = true;
    int16_t delta[3], max_delta = 0;
    for(int i=0;i<3;i++){
        calibrator.center[i] = (min_data[i] + max_data[i]) / 2;
        delta[i] = max_data[i] - min_data[i];
        if(delta[i] > max_delta)
            max_delta = delta[i];
    }

    for(int i=0;i<3;i++){
        calibrator.scale[i] = (float)max_delta / delta[i];
    }
}

void bsp_5883_get_calibrator(bsp_5883_calibrator_t *rtn){
    memcpy(rtn, &calibrator, sizeof(bsp_5883_calibrator_t));
}

void bsp_5883_set_calibrator(bsp_5883_calibrator_t *new){
    memcpy(&calibrator, new, sizeof(calibrator));
}