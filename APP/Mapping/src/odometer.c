#include "odometer.h"
#include "stdio.h"
#include "basic_algs.h"
#include "stdlib.h"
#include "sensor.h"

static int count = 0;
typedef struct{
    uint16_t fmm;
    uint16_t bmm;
    unsigned fvalid:1;
    unsigned bvalid:1;
} odm_info;

static odm_info curr, last;


void odometer_reset(){
    odometer_update();
    count = 0;
}

#define DEBUG_ODOMETER 0
void odometer_update(){
    last = curr;
    curr.fmm = DIS_FRONT_FILTERED;
    curr.bmm = DIS_BACK_FILTERED;
    curr.fvalid = sensor_data_valid(DIS_FRONT);
    curr.bvalid = sensor_data_valid(DIS_BACK);
    if(DEBUG_ODOMETER){
        printf("\nodom data:\nfmm, bmm: %d, %d\n", curr.fmm, curr.bmm);
    }

    int deltaf = 0, deltab = 0;
    if(last.fvalid && curr.fvalid){
        deltaf = last.fmm - curr.fmm;
    }
    if(last.bvalid && curr.bvalid){
        deltab = curr.bmm - last.bmm;
    }

    if(deltaf == 0 && deltab == 0){
        printf("odometer failed: out of range\n");
    }else{
        int delta = deltaf + deltab;
        if(deltab != 0 && deltaf != 0){
            delta = curr.fmm < curr.bmm ? deltaf : deltab;
        }
        int fixed_delta = (int)(cosf(fabs(azimuth_delta(get_current_azimuth(), get_padding_azimuth())) * PI / 180) * delta + 0.5);
        count += fixed_delta;
        if(DEBUG_ODOMETER){
            printf("deltaf, deltab, delta, fixed_delta, count: %d, %d, %d, %d, %d\n", deltaf, deltab, delta, fixed_delta, count);
        }
    }
}

int16_t odometer_getmm(){
    return count;
}

uint16_t measure_back_distance_in_cell(){
    if(!curr.bvalid && !curr.fvalid){
        printf("odometer measure failed: out of range\n");
        return 200;
    }else{
        if(curr.bmm <= curr.fmm){
            return curr.bmm % 400;
        }else{
            return 400 - (curr.fmm % 400);
        }
    }
}