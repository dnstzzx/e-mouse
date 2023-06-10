#ifndef __MPU6050_H
#define __MPU6050_H
#include "stdint.h"

void mpu6050_basic_read_temp(short *tempData);
void mpu6050_basic_read_gyro(short *gyroData);
void mpu6050_basic_read_acc(short *accData);
void mpu6050_basic_return_temp(float*Temperature);
void mpu6050_basic_init(void);
uint8_t mpu6050_basic_read_id(void);
void mpu6050_basic_read_data(uint8_t reg_add,unsigned char*Read,uint8_t num);
void mpu6050_basic_write_reg(uint8_t reg_add,uint8_t reg_dat);

void mpu6050_basic_pwr_mgmt_1_init(void);


#endif  /*__MPU6050*/
