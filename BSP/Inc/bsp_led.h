#pragma once
#include "main.h"
#include "stdbool.h"
#include "cmsis_os.h"

typedef struct{
    uint8_t       id;
    GPIO_TypeDef *port;
    uint16_t      pin;
    osTimerId_t   blink_timer;
} bsp_led_dev_t;

#define BSP_LED_COUNT (4)

extern bsp_led_dev_t bsp_leds[BSP_LED_COUNT];

#define BSP_LED1 (&bsp_leds[0])
#define BSP_LED2 (&bsp_leds[1])
#define BSP_LED3 (&bsp_leds[2])
#define BSP_LED4 (&bsp_leds[3])

static inline bool bsp_led_get(bsp_led_dev_t *led){
    return led->port->ODR & led->pin != 0;
}

static inline void bsp_led_set(bsp_led_dev_t *led, bool on){
    HAL_GPIO_WritePin(led->port, led->pin, on);
}

static inline void bsp_led_toggle(bsp_led_dev_t *led){
    HAL_GPIO_TogglePin(led->port, led->pin);
}

/* interval: ms, 0 to stop blink */
void bsp_led_set_blink(bsp_led_dev_t *led, uint16_t interval);

