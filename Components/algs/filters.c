#include "filters.h"
#include <string.h>

void i32_sliding_mean_filter_init(i32_sliding_mean_filter_t *filter, int32_t *buffer, uint32_t length){
    filter->data = buffer;
    filter->next = buffer;
    filter->length = length;
    filter->sum = 0;
    filter->output = 0;

    memset(buffer, 0, length * sizeof(int32_t));
}

void i32_sliding_mean_filter_input(i32_sliding_mean_filter_t *filter, int32_t data){
    filter->sum = filter->sum - *filter->next + data;
    filter->output = 1.0f * filter->sum / filter->length;
    *filter->next++ = data;
    if(filter->next == filter->data + filter->length)
        filter->next = filter->data;
}

void u32_sliding_mean_filter_init(u32_sliding_mean_filter_t *filter, uint32_t *buffer, uint32_t length){
    filter->data = buffer;
    filter->next = buffer;
    filter->length = length;
    filter->sum = 0;
    filter->output = 0;

    memset(buffer, 0, length * sizeof(uint32_t));
}

void u32_sliding_mean_filter_input(u32_sliding_mean_filter_t *filter, uint32_t data){
    filter->sum = filter->sum - *filter->next + data;
    filter->output = 1.0f * filter->sum / filter->length;
    *filter->next++ = data;
    if(filter->next == filter->data + filter->length)
        filter->next = filter->data;
}