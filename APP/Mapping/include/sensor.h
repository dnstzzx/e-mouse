#pragma once
#include <stdio.h>
#include <stdbool.h>
#include <direction.h>

#define SENSOR_DISTANCE_ENABLE_AZIMUTH_FIX (1)
#define SENSOR_MAX 8000

static inline bool sensor_data_valid(uint16_t mm){
    return mm <= SENSOR_MAX;
}

//  距离传感器
extern const uint8_t sensor_of_direction[4];
extern uint16_t sensor_distances[4];    // index: direction FRBL
extern uint16_t filtered_sensor_distances[4];
bool update_sensor_data();
bool update_sensor_data_full();

// param direction: 相对方向
static inline uint16_t distance_to(direction_t direction){
    return sensor_distances[3 - direction];
}

#define DIS_FRONT (sensor_distances[DIRECTION_F])
#define DIS_LEFT (sensor_distances[DIRECTION_L])
#define DIS_BACK (sensor_distances[DIRECTION_B])
#define DIS_RIGHT (sensor_distances[DIRECTION_R])

#define DIS_FRONT_FILTERED (filtered_sensor_distances[DIRECTION_F])
#define DIS_LEFT_FILTERED (filtered_sensor_distances[DIRECTION_L])
#define DIS_BACK_FILTERED (filtered_sensor_distances[DIRECTION_B])
#define DIS_RIGHT_FILTERED (filtered_sensor_distances[DIRECTION_R])

//  地磁计
float get_current_azimuth();

static inline direction_t get_padding_direction(){
    return azimuth_to_direction(get_current_azimuth());
}

static inline float get_padding_azimuth(){
    return azimuth_from_direction(get_padding_direction());
}

static inline void reset_origin_azimuth(){
    printf("origin azimuth reset\n");
    origin_azimuth = get_current_azimuth();
}

// all
bool sensor_init();

