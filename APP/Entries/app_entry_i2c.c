#include "bsp_i2c.h"

#define scanning_i2c hi2c2

void i2c_scan(){
    while (1){
        osDelay(1000);
        printf("\n\n");
        uint8_t data[10];
        for (uint8_t i = 0;i < 255;i+=1){
            uint8_t re = HAL_I2C_Mem_Read(&scanning_i2c, i, 0, I2C_MEMADD_SIZE_8BIT, data, 1, 0xff);
            if (re == HAL_OK){
                printf("0x%02x ", i);
            }else{
                printf(".");
            }
        }
    }
}

void app_idle_task(){
    i2c_scan();

}