#pragma once
#include "cmsis_os.h"

static inline osThreadId_t bsp_task_create(const char *name, osThreadFunc_t task_func, void *argument, osPriority_t priority, uint32_t stack_size){
    osThreadAttr_t attributes = {
        .name = name,
        .stack_size = stack_size * 4,
        .priority = priority,
    };
    return osThreadNew(task_func, argument, &attributes);
    
}