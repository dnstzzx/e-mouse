#pragma once

#include "i2c.h"
#include "cmsis_os.h"

#define BSP_I2C_IMU (&hi2c2)
#define BSP_I2C_VL53L0X (&hi2c1)

typedef struct{
    I2C_HandleTypeDef *hal_handle;
    osThreadId_t tx_thread;
    osThreadId_t rx_thread;
    osThreadId_t mem_tx_thread;
    osThreadId_t mem_rx_thread;
} i2c_dev_t;

extern i2c_dev_t bsp_i2c_dev_imu, bsp_i2c_dev_vl53l0x;

HAL_StatusTypeDef bsp_i2c_master_send(i2c_dev_t *dev, int16_t slave_addr, uint8_t *data, uint16_t size);
HAL_StatusTypeDef bsp_i2c_master_receive(i2c_dev_t *dev, int16_t slave_addr, uint8_t *data, uint16_t size);
HAL_StatusTypeDef bsp_i2c_master_write_reg(i2c_dev_t *dev, int16_t slave_addr, int16_t reg_addr, uint8_t *data, uint16_t size);
HAL_StatusTypeDef bsp_i2c_master_read_reg(i2c_dev_t *dev, int16_t slave_addr, int16_t reg_addr, uint8_t *data, uint16_t size);