#pragma once

#define APP_ENTRY_ENABEL_PID_TRACER

#define APP_ENTRY_ENABLE_MOTOR
#define APP_ENTRY_ENABLE_MPU6050
#define APP_ENTRY_ENABLE_VL53L0X

#define APP_ENTRY_MPU6050_MODE BASIC

void count_up_forever();
void app_idle_task();