#include "stm32f4xx.h"
#include "bsp_5883.h"
#include "main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == HMC5883L_INT_Pin){
        bsp_5883_interrupt_callback();
    }
}