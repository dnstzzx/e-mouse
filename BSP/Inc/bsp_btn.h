#pragma once

#include "main.h"
#include "cmsis_os.h"

#define BSP_BTN_COUNT (1)

static inline bool bsp_btn_wkup_pressed(){
    return HAL_GPIO_ReadPin(BTN_WKUP_GPIO_Port, BTN_WKUP_Pin);
}

static inline void wait_until_pressed(){
    while(!bsp_btn_wkup_pressed())
        osDelay(1);
}

static inline void wait_until_released(){
    while(bsp_btn_wkup_pressed())
        osDelay(1);
}

static inline void wait_until_pressed_and_released(){
    wait_until_pressed();
    osDelay(10);
    wait_until_released();
}