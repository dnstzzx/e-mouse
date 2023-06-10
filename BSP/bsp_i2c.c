#include "bsp_i2c.h"
#include "cmsis_os.h"
#include "stdio.h"

#define BSP_I2C_SEND_CPLT_NOTIFY (1 << 0)
#define BSP_I2C_RECV_CPLT_NOTIFY (1 << 1)
#define TX_TIMEOUT (400)
#define RX_TIMEOUT (400)


i2c_dev_t bsp_i2c_dev_imu = {.hal_handle=BSP_I2C_IMU}, bsp_i2c_dev_vl53l0x = {.hal_handle=BSP_I2C_VL53L0X};

static inline i2c_dev_t* get_dev(I2C_HandleTypeDef *hi2c){
    if(hi2c == BSP_I2C_IMU){
        return &bsp_i2c_dev_imu;
    }else if(hi2c == BSP_I2C_VL53L0X){
        return &bsp_i2c_dev_vl53l0x;
    }
    return NULL;
}

/* rtos 阻塞的i2c发送 */
HAL_StatusTypeDef bsp_i2c_master_send(i2c_dev_t *dev, int16_t addr, uint8_t *data, uint16_t size){
    dev->tx_thread = osThreadGetId();
    HAL_StatusTypeDef rst = HAL_I2C_Master_Transmit_DMA(dev->hal_handle, addr, data, size);
    if(rst == HAL_OK)
        osThreadFlagsWait(BSP_I2C_SEND_CPLT_NOTIFY, osFlagsWaitAny, TX_TIMEOUT);
    return rst;
}

/* rtos 阻塞的i2c接收 */
HAL_StatusTypeDef bsp_i2c_master_receive(i2c_dev_t *dev, int16_t addr, uint8_t *data, uint16_t size){
    dev->rx_thread = osThreadGetId();
    HAL_StatusTypeDef rst = HAL_I2C_Master_Receive_DMA(dev->hal_handle, addr, data, size);
    if(rst == HAL_OK)
        osThreadFlagsWait(BSP_I2C_RECV_CPLT_NOTIFY, osFlagsWaitAny, RX_TIMEOUT);
    return rst;
}

HAL_StatusTypeDef bsp_i2c_master_write_reg(i2c_dev_t *dev, int16_t slave_addr, int16_t reg_addr, uint8_t *data, uint16_t size){
    dev->mem_tx_thread = osThreadGetId();
    HAL_StatusTypeDef rst =  HAL_I2C_Mem_Write_DMA(dev->hal_handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,data, size);
    if(rst == HAL_OK){
        osThreadFlagsWait(BSP_I2C_SEND_CPLT_NOTIFY, osFlagsWaitAny, TX_TIMEOUT);
    }
    return rst;
}

HAL_StatusTypeDef bsp_i2c_master_read_reg(i2c_dev_t *dev, int16_t slave_addr, int16_t reg_addr, uint8_t *data, uint16_t size){
    dev->mem_rx_thread = osThreadGetId();
    HAL_StatusTypeDef rst = HAL_I2C_Mem_Read_DMA(dev->hal_handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size);
    if(rst == HAL_OK){
        osThreadFlagsWait(BSP_I2C_RECV_CPLT_NOTIFY, osFlagsWaitAny, RX_TIMEOUT);
    }
        
    return rst;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_dev_t *dev = get_dev(hi2c);
    if(dev != NULL){
        osThreadFlagsSet(dev->rx_thread, BSP_I2C_RECV_CPLT_NOTIFY);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_dev_t *dev = get_dev(hi2c);
    if(dev != NULL){
        osThreadFlagsSet(dev->tx_thread, BSP_I2C_SEND_CPLT_NOTIFY);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_dev_t *dev = get_dev(hi2c);
    if(dev != NULL){
        osThreadFlagsSet(dev->mem_rx_thread, BSP_I2C_RECV_CPLT_NOTIFY);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
    i2c_dev_t *dev = get_dev(hi2c);
    if(dev != NULL){
        osThreadFlagsSet(dev->mem_tx_thread, BSP_I2C_SEND_CPLT_NOTIFY);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle){
    printf("i2c error\r\n");
}