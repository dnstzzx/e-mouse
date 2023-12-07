#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct{
    bool valid;
    uint16_t length;
    int32_t *data;
    int32_t *next;
    int32_t sum;
    float output;
} i32_sliding_mean_filter_t;

typedef struct{
    bool valid;
    uint16_t length;
    uint32_t *data;
    uint32_t *next;
    uint32_t sum;
    float output;
} u32_sliding_mean_filter_t;

void i32_sliding_mean_filter_init(i32_sliding_mean_filter_t *filter, int32_t *buffer, uint32_t length);
void i32_sliding_mean_filter_input(i32_sliding_mean_filter_t *filter, int32_t data);
static inline float i32_sliding_mean_filter_get_output(i32_sliding_mean_filter_t *filter){
    return filter->output;
}

void u32_sliding_mean_filter_init(u32_sliding_mean_filter_t *filter, uint32_t *buffer, uint32_t length);
void u32_sliding_mean_filter_input(u32_sliding_mean_filter_t *filter, uint32_t data);
static inline float u32_sliding_mean_filter_get_output(u32_sliding_mean_filter_t *filter){
    return filter->output;
}