#pragma once
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define PI (3.1415926f)

// ------------------ direction ------------------

typedef enum{
    DIRECTION_F=0,
    DIRECTION_R=1,
    DIRECTION_B=2,
    DIRECTION_L=3
} direction_t;
#define direction_t uint8_t

static inline direction_t direction_reverse(direction_t direction){
    return (direction + 2) % 4;
}

static inline direction_t direction_relative_apply(direction_t current, direction_t action){
    direction_t target = (current + action) % 4;
    return target;
}

static inline direction_t direction_relative_get(direction_t current, direction_t target){
    return (target - current + 4) % 4;
}


// ------------------ azimuth ------------------

extern float origin_azimuth;

static inline direction_t azimuth_to_direction(float azimuth){
    float delta = fmodf(azimuth - origin_azimuth + 360.0f, 360.0f);
    return (direction_t)((delta + 45) / 90);
}

static inline float azimuth_from_direction(direction_t direction){
    return fmodf(direction * 90 + origin_azimuth, 360.0f);
}

static inline float azimuth_from_5883_data(int16_t data[3]){
    float heading = atan2f(data[1], data[0]);
    heading *= 180 / PI;
    if(heading < 0)
        heading += 360;
    return heading;
}

static inline float azimuth_delta(float from, float to){
    float rst = fmodf((to - from + 360), 360.0f );
    if(rst > 180)
        rst -= 360;
    return rst;
}