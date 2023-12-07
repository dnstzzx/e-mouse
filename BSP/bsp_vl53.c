#include "bsp_vl53.h"
#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "basic_algs.h"

typedef struct{
    GPIO_TypeDef *port;
    uint16_t      pin;
}bsp_vl53_shut_t;

vl53l0x_dev_t bsp_vl53_devs[BSP_VL53_COUNT];
uint8_t bsp_vl53_addrs[BSP_VL53_COUNT] = {0x54, 0x56, 0x58, 0x5A};
bsp_vl53_shut_t vl53_shut_array[BSP_VL53_COUNT] = {
    {VL53_1_SHUT_GPIO_Port, VL53_1_SHUT_Pin},
    {VL53_2_SHUT_GPIO_Port, VL53_2_SHUT_Pin},
    {VL53_3_SHUT_GPIO_Port, VL53_3_SHUT_Pin},
    {VL53_4_SHUT_GPIO_Port, VL53_4_SHUT_Pin}
};


#define _for_each_dev(opr) \
    for(int _i=0;_i<BSP_VL53_COUNT;_i++){ \
        vl53l0x_dev_t *dev = &bsp_vl53_devs[_i]; opr; \
    }

#define BSP_VL53_SET_SHUT(i, is_shut)  \
    HAL_GPIO_WritePin(vl53_shut_array[i].port, vl53_shut_array[i].pin, !is_shut);

static inline void bsp_vl53_set_shut_all(bool shut){
    _for_each_dev(
        BSP_VL53_SET_SHUT(_i, shut);
    );

}

bool bsp_vl53_init(){
    bsp_vl53_set_shut_all(true);
    vl53l0x_dev_t *dev;
    _for_each_dev(
        BSP_VL53_SET_SHUT(_i, false);
        memset(dev, 0, sizeof(vl53l0x_dev_t));
        dev->address = 0x52;
        dev->io_timeout = 500;
        dev->io_2v8 = true;
        dev->did_timeout = false;
        osDelay(2);
        if(!VL53L0X_init(dev)){
            printf("vl53 id %d init failed", _i);
            return false;
        }
        // VL53L0X_setSignalRateLimit(dev, 0.1);
        // VL53L0X_setVcselPulsePeriod(dev, VcselPeriodPreRange, 18);
        // VL53L0X_setVcselPulsePeriod(dev, VcselPeriodFinalRange, 14);
        //VL53L0X_setMeasurementTimingBudget(dev, 50 * 1000);
        //printf("timing budget: %d\n", dev->measurement_timing_budget_us);
        VL53L0X_setAddress(dev, bsp_vl53_addrs[_i]);
    );
    return true;
}

void bsp_vl53_start_continuous_all(){
    _for_each_dev(
        VL53L0X_startContinuous(dev, 0);
    );
}

bool bsp_vl53_read_all(uint16_t *rtn){
    _for_each_dev(
        int32_t rst = (int32_t)VL53L0X_readRangeContinuousMillimeters(dev);
        if(rst == 65535) return false;
        rtn[_i] = limit(rst, 0, 8191);
    );

    // _for_each_dev(
    //     int32_t rst = (int32_t)VL53L0X_readRangeContinuousMillimeters(dev);
    //     if(rst == 65535) return false;
    //     // if(rst <= 8000)
    //     //     rst = (int32_t)(fix_k[_i] * rst + fix_b[_i]);
    //     rtn[_i] = limit(rst, 0, 8191);
    //     //rtn[_i] = max(fix[_i] + rtn[_i], 0);
        
    //     //osDelay(1);
    // );
    return true;
}

void bsp_vl53_config(vl53l0x_dev_t *dev,  bsp_vl53_preset_t preset){
    VL53L0X_stopContinuous(dev);
    VL53L0X_setSignalRateLimit(dev, preset.signal_rate_limit);
    VL53L0X_setVcselPulsePeriod(dev, VcselPeriodPreRange, preset.VcselPeriodPreRange);
    VL53L0X_setVcselPulsePeriod(dev, VcselPeriodFinalRange, preset.VcselPeriodFinalRange);
    VL53L0X_setMeasurementTimingBudget(dev, preset.measurement_timing_budget_us);
    VL53L0X_startContinuous(dev, 0);
    osDelay(2 * preset.measurement_timing_budget_us / 1000);
}


const bsp_vl53_preset_t BSP_VL53_PRESET_DEFAULT = {
    .signal_rate_limit = 0.25f,
    .VcselPeriodPreRange = 14,
    .VcselPeriodFinalRange = 10,
    .measurement_timing_budget_us = 33 * 1000
};

const bsp_vl53_preset_t BSP_VL53_PRESET_LONG_RANGE = {
    .signal_rate_limit = 0.1f,
    .VcselPeriodPreRange = 18,
    .VcselPeriodFinalRange = 14,
    .measurement_timing_budget_us = 33 * 1000
};

const bsp_vl53_preset_t BSP_VL53_PRESET_HIGH_SPEED = {
    .signal_rate_limit = 0.25f,
    .VcselPeriodPreRange = 14,
    .VcselPeriodFinalRange = 10,
    .measurement_timing_budget_us = 20 * 1000
};

const bsp_vl53_preset_t BSP_VL53_PRESET_HIGH_ACCURACY = {
    .signal_rate_limit = 0.25f,
    .VcselPeriodPreRange = 14,
    .VcselPeriodFinalRange = 10,
    .measurement_timing_budget_us = 200 * 1000
};