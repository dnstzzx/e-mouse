#include "sensor.h"
#include "bsp_5883.h"
#include "bsp_vl53.h"
#include "filters.h"

// 距离传感器

#define SENSOR_FILTER_LENGTH 3

uint16_t sensor_distances[BSP_VL53_COUNT];    // index: direction FRBL
uint16_t filtered_sensor_distances[BSP_VL53_COUNT];
u32_sliding_mean_filter_t sensor_distance_filters[BSP_VL53_COUNT];
uint32_t sensor_distance_filter_buffers[BSP_VL53_COUNT][SENSOR_FILTER_LENGTH];

const uint8_t sensor_of_direction[4] = {3, 2, 1, 0};


void count_up_forever();
void stop_motor();
static void go_fail(){
    stop_motor();
    printf("\n\n\nvl53 failed\n");
    count_up_forever();
}

#define DEBGU_UPDATE_SENSOR_DATA 0
bool _update_sensor_data(){
    static uint16_t origin_data[4];
    if(!bsp_vl53_read_all(origin_data))
        return false;
    for(int i=0;i<4;i++){
        sensor_distances[i] = origin_data[sensor_of_direction[i]];
        u32_sliding_mean_filter_input(&sensor_distance_filters[i], sensor_distances[i]);
        filtered_sensor_distances[i] = sensor_distance_filters[i].output;
    }
    if(DEBGU_UPDATE_SENSOR_DATA) printf("sensor data FRBL: %d, %d, %d, %d\n", sensor_distances[0], sensor_distances[1], sensor_distances[2], sensor_distances[3]);
    return true;
}

static try_update_sensor_data(){
    if(!_update_sensor_data()){
        go_fail();
    }
}

bool update_sensor_data(){
    return try_update_sensor_data();
}

bool update_sensor_data_full(){
    for(int i=0;i<SENSOR_FILTER_LENGTH;i++){
        osDelay(33);
        try_update_sensor_data();
    }
    return true;
}



// 磁力计

float origin_azimuth;

float get_current_azimuth(){
    return bsp_5883_get_yaw_degree();
}


// all

bool sensor_init(){
    reset_origin_azimuth();
    if(!bsp_vl53_init()){
        printf("vl53l0x init failed\n");
        return false;
    }else if(!bsp_5883_init()){
        printf("5883 init failed\n");
        return false;
    }
    bsp_vl53_start_continuous_all();
    for(int i=0;i<BSP_VL53_COUNT;i++){
        u32_sliding_mean_filter_init(&sensor_distance_filters[i], sensor_distance_filter_buffers[i], SENSOR_FILTER_LENGTH);
        for(int j=0;j<SENSOR_FILTER_LENGTH;j++){
            osDelay(33);
            update_sensor_data();
        }
    }
    return true;
}

