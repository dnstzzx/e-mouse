#pragma once
#include "stdint.h"


void bsp_uart_init();
uint32_t bsp_uart1_send_data(const uint8_t *data, uint16_t size);
uint32_t bsp_uart1_received_data_count();
uint32_t bsp_uart1_read_data(uint8_t *buff, uint32_t buff_size);
char bsp_uart1_read_char();
uint32_t bsp_uart1_read_line(char *buf, uint32_t size);