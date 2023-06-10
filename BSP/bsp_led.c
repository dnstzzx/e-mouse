#include "bsp_led.h"

bsp_led_dev_t bsp_leds[BSP_LED_COUNT] = {
    {.id = 0, .port = LED1_GPIO_Port, .pin = LED1_Pin, .blink_timer = NULL},
    {.id = 1, .port = LED2_GPIO_Port, .pin = LED2_Pin, .blink_timer = NULL},
    {.id = 2, .port = LED3_GPIO_Port, .pin = LED3_Pin, .blink_timer = NULL},
    {.id = 3, .port = LED4_GPIO_Port, .pin = LED4_Pin, .blink_timer = NULL},
};

static void bsp_led_blink_timer_task(void *param){
    bsp_led_dev_t *led = (bsp_led_dev_t *)param;
    bsp_led_toggle(led);
}

void bsp_led_set_blink(bsp_led_dev_t *led, uint16_t interval){
    if(led->blink_timer && osTimerIsRunning(led->blink_timer))
        osTimerStop(led->blink_timer);

    if(interval){
        if(led->blink_timer == NULL){
            char name[16] = "ledx blinker";
            name[3] = led->id + 1 + '0';
            osTimerAttr_t attr = {
                .name = name
            };
            osTimerNew(bsp_led_blink_timer_task, osTimerPeriodic, led, &attr);
        }
        osTimerStart(led->blink_timer, interval);
    }
}