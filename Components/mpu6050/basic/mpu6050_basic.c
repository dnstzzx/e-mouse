#include "./mpu6050_basic.h"
#include "./mpu6050_basic_register_map.h"
#include "bsp_i2c.h"
#include "cmsis_os.h"
#include "stdio.h"

#define MPU_ERROR 		printf
#define MPU_INFO 		printf
#define Delay       osDelay

/**
  * @brief   写数据到MPU6050寄存器
  * @param   reg_add:寄存器地址
  * @param	 reg_data:要写入的数据
  * @retval
  */
void mpu6050_basic_write_reg(uint8_t reg_add, uint8_t reg_dat){
    bsp_i2c_master_write_reg(&bsp_i2c_dev_imu, MPU6050_ADDRESS, reg_add, &reg_dat, 1);
}

/**
  * @brief   从MPU6050寄存器读取数据
  * @param   reg_add:寄存器地址
  * @param	 Read：存储数据的缓冲区
  * @param	 num：要读取的数据量
  * @retval
  */
void mpu6050_basic_read_data(uint8_t reg_add, unsigned char* Read, uint8_t num){
    bsp_i2c_master_read_reg(&bsp_i2c_dev_imu, MPU6050_ADDRESS, reg_add, Read, num);
}


/**
  * @brief   初始化MPU6050芯片
  * @param
  * @retval
  */
void mpu6050_basic_init(void){
    //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
    Delay(100);
    mpu6050_basic_write_reg(MPU6050_RA_PWR_MGMT_1, 0x00);	    //解除休眠状态
    mpu6050_basic_write_reg(MPU6050_RA_SMPLRT_DIV, 0x07);	    //陀螺仪采样率
    mpu6050_basic_write_reg(MPU6050_RA_CONFIG, 0x06);
    mpu6050_basic_write_reg(MPU6050_RA_ACCEL_CONFIG, 0x01);	  //配置加速度传感器工作在16G模式
    mpu6050_basic_write_reg(MPU6050_RA_GYRO_CONFIG, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
    Delay(200);
}

/**
  * @brief   读取MPU6050的ID
  * @param
  * @retval  正常返回1，异常返回0
  */
uint8_t mpu6050_basic_read_id(void){
    unsigned char Re = 0;
    mpu6050_basic_read_data(MPU6050_RA_WHO_AM_I, &Re, 1);    //读器件地址
    if (Re != 0x68){
        MPU_ERROR("MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
        return 0;
    }
    else{
        MPU_INFO("MPU6050 ID = %d\r\n", Re);
        return 1;
    }

}

/**
  * @brief   读取MPU6050的加速度数据
  * @param
  * @retval
  */
void mpu6050_basic_read_acc(short* accData){
    uint8_t buf[6];
    mpu6050_basic_read_data(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的角加速度数据
  * @param
  * @retval
  */
void mpu6050_basic_read_gyro(short* gyroData){
    uint8_t buf[6];
    mpu6050_basic_read_data(MPU6050_GYRO_OUT, buf, 6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的原始温度数据
  * @param
  * @retval
  */
void mpu6050_basic_read_temp(short* tempData){
    uint8_t buf[2];
    mpu6050_basic_read_data(MPU6050_RA_TEMP_OUT_H, buf, 2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}

/**
  * @brief   读取MPU6050的温度数据，转化成摄氏度
  * @param
  * @retval
  */
void mpu6050_basic_return_temp(float* Temperature){
    short temp3;
    uint8_t buf[2];

    mpu6050_basic_read_data(MPU6050_RA_TEMP_OUT_H, buf, 2);     //读取温度值
    temp3 = (buf[0] << 8) | buf[1];
    *Temperature = ((double)temp3 / 340.0) + 36.53;
}
