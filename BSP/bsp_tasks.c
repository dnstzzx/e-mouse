#include "app_entry.h"
#include "cmsis_os.h"

void bsp_task_idle(){

    app_idle_task();
    /*
    static uint8_t data[120];
    sprintf(data, "Received:");
    while(1){
        if(bsp_uart1_received_data_count() != 0){
            uint32_t read_size = bsp_uart1_read_data(&data[9], 100 - 9);
            bsp_uart1_send_data(data, read_size + 9);
        }
        osDelay(1);
    }*/
}

void bsp_tasks_init(){

}