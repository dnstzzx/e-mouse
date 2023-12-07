#pragma once

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"

#define BSP_BTN_COUNT (1)

static inline bool bsp_btn_wkup_pressed(){
    return HAL_GPIO_ReadPin(BTN_WKUP_GPIO_Port, BTN_WKUP_Pin);
}

static inline void bsp_btn_wait_until_pressed(){
    while(!bsp_btn_wkup_pressed())
        osDelay(1);
}

static inline void bsp_btn_wait_until_released(){
    while(bsp_btn_wkup_pressed())
        osDelay(1);
}

static inline void bsp_btn_wait_until_pressed_and_released(){
    bsp_btn_wait_until_pressed();
    osDelay(10);
    bsp_btn_wait_until_released();
}