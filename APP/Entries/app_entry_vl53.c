#include "app_entry.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "bsp_vl53.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"

static inline void single_test(){

    vl53l0x_dev_t sensor = {
        .io_2v8 = true, 
        .address = 0x52, 
        .io_timeout = 500, 
        .did_timeout = false
    };

    printf("start measure...\n");
    bool success = VL53L0X_init(&sensor);

    if(success){
        printf("init successful\n");
    }else{
        printf("init failed\n");
        count_up_forever();
    }
    
    
    VL53L0X_startContinuous(&sensor, 0);
    while(1){
		uint16_t value = VL53L0X_readRangeContinuousMillimeters(&sensor);
		printf("\t%d\tmm\r\n", value);;
		if (VL53L0X_timeoutOccurred(&sensor) ) {
		    printf("TIMEOUT\r\n");
		}
	}
}

typedef struct{
    GPIO_TypeDef *port;
    uint16_t      pin;
}bsp_vl53_shut_t;

extern bsp_vl53_shut_t vl53_shut_array[BSP_VL53_COUNT];
extern uint8_t bsp_vl53_addrs[BSP_VL53_COUNT];

#define BSP_VL53_SET_SHUT(i, is_shut)  \
    HAL_GPIO_WritePin(vl53_shut_array[i].port, vl53_shut_array[i].pin, !is_shut);

static inline void bsp_vl53_set_shut_all(bool shut){
    for(int i=0;i<BSP_VL53_COUNT;i++){
        BSP_VL53_SET_SHUT(i, shut);
    }
}

static void each_test(){
    uint32_t start = HAL_GetTick();
    uint16_t rsts[8] = {};
        while(1){
                for(int i=0;i<8;i++){
                bsp_vl53_set_shut_all(true);
                BSP_VL53_SET_SHUT(i, false);
                osDelay(2);
                vl53l0x_dev_t *dev = &bsp_vl53_devs[i];
                memset(dev, 0, sizeof(vl53l0x_dev_t));
                dev->address = 0x52;
                dev->io_timeout = 500;
                dev->io_2v8 = true;
                dev->did_timeout = false;
                if(!VL53L0X_init(dev)){
                    printf("vl53 id %d init failed", i);
                }
                rsts[i] = VL53L0X_readRangeSingleMillimeters(dev);
                printf("no %d, data: %d, time: %d\n", i, rsts[i], HAL_GetTick() - start);
                start = HAL_GetTick();
            }
            
        }
    
    
    
}

void multi_test(void *param){
    if(!bsp_vl53_init()){
        count_up_forever();
    }
    bsp_vl53_start_continuous_all();
    uint16_t data[8];
    for(uint32_t i=0;;i++){    
        osDelay(50);   
        bsp_vl53_read_all(data);
        for(int j=0;j<8;j++){
            printf("%d ", data[j]);
        }
        printf("\n");

    }
}

static void try_measure(vl53l0x_dev_t *dev){
    VL53L0X_startContinuous(dev, 0);
    for(int i=0;i<5;i++){
        osDelay(300);
        uint16_t rst = VL53L0X_readRangeContinuousMillimeters(dev);
        printf("read data: %d mm\n", rst);
    }
}

static void multi_test1(uint8_t i){
    bsp_vl53_set_shut_all(true);
    printf("testing %d\n", i);
        BSP_VL53_SET_SHUT(i, false);
        osDelay(50);
        vl53l0x_dev_t *dev = &bsp_vl53_devs[i];
        memset(dev, 0, sizeof(vl53l0x_dev_t));
        dev->address = 0x52;
        dev->io_timeout = 500;
        dev->io_2v8 = true;
        dev->did_timeout = false;
        bool success = VL53L0X_init(dev);
        if(!success){
            printf("init fialed\n");
            return;
        }
        printf("init success, trying\n");
        try_measure(dev);
        printf("setting address to 0x%02x\n", bsp_vl53_addrs[i]);
        VL53L0X_setAddress(dev, bsp_vl53_addrs[i]);
        success = dev->last_status == HAL_OK;
        if(!success){
            printf("set address fialed\n");
            return;
        }
        try_measure(dev);
}

static void multi_test2(){
    bsp_vl53_set_shut_all(true);
    for(int i=0;i<8;i++){
        multi_test1(i);
        vl53l0x_dev_t *dev = &bsp_vl53_devs[i];
        while(bsp_uart1_received_data_count() == 0){
            uint16_t rst = VL53L0X_readRangeContinuousMillimeters(dev);
            printf("read data: %d mm\n", rst);
            osDelay(200);
        }
        while(bsp_uart1_received_data_count() != 0) bsp_uart1_read_char();
        printf("\n\n\n");
    }
}

static void precision_measure(){
    if(!bsp_vl53_init()){
        count_up_forever();
    }
    for(int i=0;i<8;i++){
        if(!VL53L0X_setMeasurementTimingBudget(&bsp_vl53_devs[i], 500 * 1000))
            count_up_forever();
    }
    bsp_vl53_start_continuous_all();

    while(1){
        printf("input s to star a new epoch\n");
        char c = getchar();
        while(bsp_uart1_received_data_count() != 0) c = getchar();
        const uint8_t times = 20;
        uint32_t sum[8] = {};
        uint16_t data[8];
        for(uint8_t i=0;i<times;i++){   
            osDelay(500);   
            bsp_vl53_read_all(data);
            printf("iter %d: ", i);
            for(int j=0;j<8;j++){
                printf("%d  ", data[j]);
                sum[j] += data[j];
            }
            printf("\n");
            
        }

        printf("\nresult: ");
        for(int j=0;j<8;j++){
            printf("%ld  ", sum[j]);
        }
        printf("\n\n");
    }
}

void app_idle_task(){
    //set_address();
    //single_test();
    //precision_measure();
    //each_test();
    multi_test(NULL);
    while(1) osDelay(100);
}