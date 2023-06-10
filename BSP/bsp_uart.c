#include "bsp_uart.h"
#include "usart.h"
#include "fifo.h"
#include "bsp_task.h"
#include "cmsis_os.h"
#include "basic_algs.h"

#define REDIRECT_STDIO (1)

#define BSP_UART_RX_BLK_SIZE (100)
#define BSP_UART_TX_BUFF_SIZE (1024)
#define BSP_UART_RX_BUFF_SIZE (1024)
#define BSP_UART_TX_NEW_DATA_NOTIFY (1)
#define BSP_UART_TX_CPLT_NOTIFY (2)
#define BSP_UART_RX_CPLT_NOTIFY (4)

typedef struct
{
    _fifo_t tx_fifo;	/* 发送fifo */
    _fifo_t rx_fifo;	/* 接收fifo */
    osThreadId_t tx_thread;
    osThreadId_t rx_thread;
}uart_device_t;

static uart_device_t uart1;
static uint8_t uart1_tx_buff[BSP_UART_TX_BUFF_SIZE];
static uint8_t uart1_tx_dma_buff[BSP_UART_TX_BUFF_SIZE];
static uint8_t uart1_rx_buff[BSP_UART_RX_BUFF_SIZE];
static uint8_t uart1_rx_dma_buff[BSP_UART_RX_BUFF_SIZE];

/* fifo上锁函数 */
static void fifo_lock(void){
    __disable_irq();
}

/* fifo解锁函数 */
static void fifo_unlock(void){
    __enable_irq();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart == &huart1){
        osThreadFlagsSet(uart1.tx_thread, BSP_UART_TX_CPLT_NOTIFY);
    }
}

static void uart_tx_task(void *param){
    while(1){
        osThreadFlagsWait(BSP_UART_TX_NEW_DATA_NOTIFY, osFlagsWaitAny, 1000);
        uint32_t len = fifo_read(&uart1.tx_fifo, uart1_tx_dma_buff, BSP_UART_TX_BUFF_SIZE);
        HAL_UART_Transmit_DMA(&huart1, uart1_tx_dma_buff, len);
    }
}

static void uart_rx_task(void *param){
    while(1){
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma_buff, BSP_UART_RX_BUFF_SIZE);
        osThreadFlagsWait(BSP_UART_RX_CPLT_NOTIFY, osFlagsWaitAny, 1000);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if(huart == &huart1){
        fifo_write(&uart1.rx_fifo, uart1_rx_dma_buff, Size);
        osThreadFlagsSet(uart1.rx_thread, BSP_UART_RX_CPLT_NOTIFY);
    }
}


void bsp_uart_init(){
    fifo_register(&uart1.tx_fifo , uart1_tx_buff , BSP_UART_TX_BUFF_SIZE , fifo_lock , fifo_unlock);
    fifo_register(&uart1.rx_fifo , uart1_rx_buff , BSP_UART_RX_BUFF_SIZE , fifo_lock , fifo_unlock);
    uart1.tx_thread = bsp_task_create("uart1_tx", uart_tx_task, &uart1, osPriorityLow1, 128);
    uart1.rx_thread = bsp_task_create("uart1_rx", uart_rx_task, &uart1, osPriorityLow2, 128);
}

uint32_t bsp_uart1_send_data(const uint8_t* data , uint16_t size){
    uint32_t rst = fifo_write(&uart1.tx_fifo, data, size);
    osThreadFlagsSet(uart1.tx_thread, BSP_UART_TX_NEW_DATA_NOTIFY);
    return rst;
}

uint32_t bsp_uart1_received_data_count(){
    return fifo_get_occupy_size(&uart1.rx_fifo);
}

uint32_t bsp_uart1_read_data(uint8_t *buff, uint32_t buff_size){
    return fifo_read(&uart1.rx_fifo, buff, buff_size);
}

char bsp_uart1_read_char(){
    char c;
    while(bsp_uart1_received_data_count() == 0) osDelay(1);
    bsp_uart1_read_data(&c, 1);
    return c;
}

uint32_t bsp_uart1_read_line(char *buf, uint32_t size){
    uint32_t count = 0;
    while(count < size){
        char c = bsp_uart1_read_char();
        if(c == '\r')   continue;
        if(c == '\n')   break;
        *buf = c;
        buf ++;
        count++;
    }
    *buf = '\0';
    return count;
}

#if REDIRECT_STDIO

#include "stdio.h"

int _write(int fd, char *str, int len){
    return bsp_uart1_send_data((uint8_t *)str, len);
}


int _read(int fd, char *ptr, int len){
    while(bsp_uart1_received_data_count() == 0)  osDelay(1);
    return bsp_uart1_read_data((uint8_t *)ptr, len);
}

#endif
