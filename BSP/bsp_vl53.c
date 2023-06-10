#include "bsp_vl53.h"
#include "main.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdio.h"

typedef struct{
    GPIO_TypeDef *port;
    uint16_t      pin;
}bsp_vl53_shut_t;

vl53l0x_dev_t bsp_vl53_devs[BSP_VL53_COUNT];
static const uint8_t bsp_vl53_addrs[BSP_VL53_COUNT] = {0x54, 0x56, 0x58, 0x5A, 0x5C, 0x5E, 0x60, 0x62};
bsp_vl53_shut_t vl53_shut_array[BSP_VL53_COUNT] = {
    {VL53_1_SHUT_GPIO_Port, VL53_1_SHUT_Pin},
    {VL53_2_SHUT_GPIO_Port, VL53_2_SHUT_Pin},
    {VL53_3_SHUT_GPIO_Port, VL53_3_SHUT_Pin},
    {VL53_4_SHUT_GPIO_Port, VL53_4_SHUT_Pin},
    {VL53_5_SHUT_GPIO_Port, VL53_5_SHUT_Pin},
    {VL53_6_SHUT_GPIO_Port, VL53_6_SHUT_Pin},
    {VL53_7_SHUT_GPIO_Port, VL53_7_SHUT_Pin},
    {VL53_8_SHUT_GPIO_Port, VL53_8_SHUT_Pin}
};



#define _for_each_dev(opr) \
    for(int _i=0;_i<BSP_VL53_COUNT;_i++){ \
        vl53l0x_dev_t *dev = &bsp_vl53_devs[_i]; opr; \
    }

#define BSP_VL53_SET_SHUT(i, is_shut)  \
    HAL_GPIO_WritePin(vl53_shut_array[i].port, vl53_shut_array[i].pin, !is_shut);

static inline void bsp_vl53_set_shut_all(bool shut){
    for(int i=0;i<BSP_VL53_COUNT;i++){
        BSP_VL53_SET_SHUT(i, shut);
    }
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
        VL53L0X_setAddress(dev, bsp_vl53_addrs[_i]);
    );
    return true;
}

void bsp_vl53_start_continuous_all(){
    _for_each_dev(VL53L0X_startContinuous(dev, 0));
}
void bsp_vl53_read_all(uint16_t *rtn){
    _for_each_dev(rtn[_i] = VL53L0X_readRangeContinuousMillimeters(dev));
}